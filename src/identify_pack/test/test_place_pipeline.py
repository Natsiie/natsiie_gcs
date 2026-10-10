import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.time import Time
from std_msgs.msg import Header, String

from identify_pack.scripts.place_task import PlaceTask
from identify_pack.vision_identify.place_task_node import (
    PlaceTaskNode, placement_target, placement_target_homography)
from identify_pack.vision_identify.ring_centers import ring_centers_message
from identify_pack.scripts.ring_station_digits import DigitMappingTracker


class PlacePipelineTests(unittest.TestCase):
    def observation(self, stable=True, detected=True):
        result = SimpleNamespace(detected=detected, stable=stable, within_tolerance=stable,
                                 centers_px=((220., 240.), (320., 240.), (420., 240.)))
        return ring_centers_message(Header(stamp=Stamp(sec=42), frame_id='camera'),
                                    result, (640, 480), 'camera', {
                                        'complete_digit_mapping': detected,
                                        'digit_mapping_stable': stable,
                                        'centers_by_digit_px': {},
                                        'rings': [dict(digit=i + 1, center_px=list(center),
                                                       score=.95, score_margin=.4)
                                                  for i, center in enumerate(result.centers_px)]})

    def projection(self, message, ring=1):
        pose = np.diag([1., -1., -1., 1.])
        pose[2, 3] = 1.
        return placement_target(message, ring, (640, 480), 'camera',
                                np.array([[500., 0, 320.], [0, 500., 240.], [0, 0, 1.]]),
                                np.zeros(5), pose, -72.5, -42.5)

    def test_projection_uses_ring_plane_and_independent_release_height(self):
        np.testing.assert_allclose(self.projection(self.observation(), 1), [-214.5, 0., -42.5])
        np.testing.assert_allclose(self.projection(self.observation(), 3), [214.5, 0., -42.5])

    def test_homography_mapping_rotates_link_to_base(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(Path(directory) / 'progress.json')
            node.mapping_mode = 'homography'
            node.homography = np.array([[1., 0., -220.], [0., 1., -240.], [0., 0., 1.]])
            node.task.start(node.task.task_id, 'ring_station', 1)
            node.on_centers('ring_station', self.observation())
            node.tick()
            command = json.loads(node.command_pub.publish.call_args.args[0].data)
            self.assertEqual(command['ring'], 2)
            # 像素(320,240) -> link(100,0) mm；TF 四元数(x=1,w=0)=绕X转180°，
            # base_from_link[:2,:2]=diag(1,-1)，故 base 仍为 (100,0)。
            np.testing.assert_allclose([command['x_mm'], command['y_mm'], command['z_mm']],
                                       [100., 0., -42.5])

    def test_missing_unstable_wrong_resolution_and_invalid_metadata_are_rejected(self):
        for message in (self.observation(stable=False), self.observation(detected=False)):
            with self.assertRaises(ValueError):
                self.projection(message)
        message = self.observation()
        message.channels[0].values = [3., 2., 1.]
        with self.assertRaises(ValueError):
            self.projection(message)
        message = self.observation()
        message.channels[1].values = [1280.] * 3
        with self.assertRaises(ValueError):
            self.projection(message)

    def node(self, path):
        node = PlaceTaskNode.__new__(PlaceTaskNode)
        params = dict(enabled=True, base_frame='base', camera_frame='camera',
                      gimbal_frame='gimbal_link',
                      require_aligned=True, item_height_configured=True, item_height_mm=60.)
        for stage in ('ring_station', 'stacked_ring'):
            params.update({f'{stage}.height_configured': True, f'{stage}.plane_z_mm': -72.5,
                           f'{stage}.first_release_z_mm': -42.5})
        node.param = params.__getitem__
        node.path = path
        node.enabled = True
        node.mapping_mode = 'pinhole'
        node.homography = None
        node.max_age = .5
        node.task = PlaceTask()
        node.task.load('156+231+516+312')
        node.phase_started_ns = 41 * 10**9
        node.observations = {}
        node.command_pub = Mock()
        node.status_pub = Mock()
        clock = Mock()
        clock.clock_type = ClockType.ROS_TIME
        clock.now.return_value = Time(seconds=42.1, clock_type=ClockType.ROS_TIME)
        node.get_clock = lambda: clock
        node.get_logger = Mock()
        node.matrix = np.array([[500., 0, 320.], [0, 500., 240.], [0, 0, 1.]])
        node.distortion = np.zeros(5)
        node.image_size = (640, 480)
        node.tf_buffer = Mock()
        transform = TransformStamped()
        transform.transform.rotation.x = 1.
        transform.transform.rotation.w = 0.
        transform.transform.translation.z = 1.
        node.tf_buffer.lookup_transform.return_value = transform
        return node

    def test_command_is_sent_once_and_only_matching_result_advances(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(Path(directory) / 'progress.json')
            node.task.start(node.task.task_id, 'ring_station', 1)
            node.on_centers('ring_station', self.observation())
            node.tick()
            command = json.loads(node.command_pub.publish.call_args.args[0].data)
            self.assertEqual(command['ring'], 2)
            np.testing.assert_allclose([command['x_mm'], command['y_mm'], command['z_mm']],
                                       [0., 0., -42.5])
            restored = PlaceTask.restore(node.path)
            self.assertEqual(restored.pending, command)
            node.tick()
            node.command_pub.publish.assert_called_once()
            node.on_result(String(data=json.dumps(dict(task_id=node.task.task_id,
                                                       command_id=999, success=True))))
            self.assertEqual(node.task.progress['ring_station'], [0, 0])
            node.on_result(String(data=json.dumps(dict(task_id=node.task.task_id,
                                                       command_id=1, success=True))))
            self.assertEqual(node.task.expected.ring, 3)
            node.tick()
            node.command_pub.publish.assert_called_once()

    def test_printed_id_mapping_selects_pixels_in_task_code_order(self):
        result = SimpleNamespace(detected=True, stable=True, within_tolerance=True,
                                 centers_px=((220., 240.), (320., 240.), (420., 240.)))
        report = {'complete_digit_mapping': True, 'digit_mapping_stable': True,
                  'centers_by_digit_px': {}, 'rings': [
                      dict(digit=label, center_px=list(point), score=.9, score_margin=.3)
                      for label, point in zip((3, 1, 2), result.centers_px)]}
        message = ring_centers_message(Header(stamp=Stamp(sec=42), frame_id='camera'),
                                       result, (640, 480), 'camera', report)
        self.assertEqual([point.x for point in message.points], [320., 420., 220.])
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(Path(directory) / 'progress.json')
            node.task.start(node.task.task_id, 'ring_station', 1)
            node.on_centers('ring_station', message)
            node.tick()
            command = json.loads(node.command_pub.publish.call_args.args[0].data)
            self.assertEqual(command['ring'], 2)
            self.assertAlmostEqual(command['x_mm'], 214.5)

    def test_geometry_only_ids_cannot_send_ring_station_placement(self):
        message = self.observation()
        message.channels = [channel for channel in message.channels
                            if not channel.name.startswith('digit_')]
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(Path(directory) / 'progress.json')
            node.task.start(node.task.task_id, 'ring_station', 1)
            node.on_centers('ring_station', message)
            node.tick()
            node.command_pub.publish.assert_not_called()

    def test_fixed_numbering_projects_a_ring_with_covered_digit(self):
        result = SimpleNamespace(detected=True, stable=True, within_tolerance=True,
                                 centers_px=((220., 240.), (320., 240.), (420., 240.)))
        tracker = DigitMappingTracker(2, 4, expected_digits=(1, 2, 3))
        for _ in range(2):
            report = {'three_ring_geometry': True, 'complete_digit_mapping': False,
                      'rings': [dict(slot=i + 1, digit=digit, center_px=list(point),
                                     score=.9 if digit else 0., score_margin=.3 if digit else 0.)
                                for i, (digit, point) in enumerate(zip((1, None, 3), result.centers_px))]}
            tracker.update(report, (640, 480))
        message = ring_centers_message(Header(stamp=Stamp(sec=42), frame_id='camera'),
                                       result, (640, 480), 'camera', report)
        metadata = {channel.name: list(channel.values) for channel in message.channels}
        self.assertEqual(metadata['digit_observed'], [1., 0., 3.])
        self.assertEqual(metadata['ring_id'], [1., 2., 3.])
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(Path(directory) / 'progress.json')
            node.task.start(node.task.task_id, 'ring_station', 1)
            node.on_centers('ring_station', message)
            node.tick()
            command = json.loads(node.command_pub.publish.call_args.args[0].data)
            self.assertEqual(command['ring'], 2)
            self.assertAlmostEqual(command['x_mm'], 0.)

    def test_stale_unstable_and_checkpoint_failure_do_not_send(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(Path(directory) / 'progress.json')
            node.task.start(node.task.task_id, 'ring_station', 1)
            node.on_centers('ring_station', self.observation(stable=False))
            node.tick()
            node.command_pub.publish.assert_not_called()
            message = self.observation()
            message.header.stamp = Stamp(sec=40)
            node.on_centers('ring_station', message)
            node.tick()
            node.command_pub.publish.assert_not_called()
            node.on_centers('ring_station', self.observation())
            node.path = Path(directory)  # Cannot replace a directory with the checkpoint.
            node.tick()
            node.command_pub.publish.assert_not_called()
            self.assertEqual(node.task.state, 'WAIT_TARGET')


if __name__ == '__main__':
    unittest.main()
