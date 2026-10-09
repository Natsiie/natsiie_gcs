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
from identify_pack.vision_identify.place_task_node import PlaceTaskNode, placement_target
from identify_pack.vision_identify.ring_centers import ring_centers_message


class PlacePipelineTests(unittest.TestCase):
    def observation(self, stable=True, detected=True):
        result = SimpleNamespace(detected=detected, stable=stable, within_tolerance=stable,
                                 centers_px=((220., 240.), (320., 240.), (420., 240.)))
        return ring_centers_message(Header(stamp=Stamp(sec=42), frame_id='camera'),
                                    result, (640, 480), 'camera')

    def projection(self, message, ring=1):
        pose = np.diag([1., -1., -1., 1.])
        pose[2, 3] = 1.
        return placement_target(message, ring, (640, 480), 'camera',
                                np.array([[500., 0, 320.], [0, 500., 240.], [0, 0, 1.]]),
                                np.zeros(5), pose, -72.5, -42.5)

    def test_projection_uses_ring_plane_and_independent_release_height(self):
        np.testing.assert_allclose(self.projection(self.observation(), 1), [-214.5, 0., -42.5])
        np.testing.assert_allclose(self.projection(self.observation(), 3), [214.5, 0., -42.5])

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
                      require_aligned=True, item_height_configured=True, item_height_mm=60.)
        for stage in ('ring_station', 'stacked_ring'):
            params.update({f'{stage}.height_configured': True, f'{stage}.plane_z_mm': -72.5,
                           f'{stage}.first_release_z_mm': -42.5})
        node.param = params.__getitem__
        node.path = path
        node.enabled = True
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
