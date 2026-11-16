import json
import math
import time
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import cv2
import numpy as np
from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import Point32, TransformStamped
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import ChannelFloat32, PointCloud
from std_msgs.msg import String
from rclpy.time import Time
from tf2_ros import Buffer

from identify_pack.tf_transform.pixel_to_base_node import (
    ImageSample, PixelToBaseNode, transform_matrix,
)


class PixelToBaseTests(unittest.TestCase):
    def setUp(self):
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'gimbal_base'
        self.transform.child_frame_id = 'camera_optical_frame'
        self.transform.header.stamp = Stamp(sec=42)
        self.transform.transform.translation.x = 0.1
        self.transform.transform.translation.z = 0.4
        self.transform.transform.rotation.x = 1.0
        self.transform.transform.rotation.w = 0.0
        self.matrix = np.array([[308.0, 0, 288.0], [0, 305.0, 238.0], [0, 0, 1]])
        self.distortion = np.array([-0.018, -0.048, -0.00008, -0.03, 0.166])
        self.sample = ImageSample(np.zeros((480, 640, 3), dtype=np.uint8),
                                  Stamp(sec=42), time.monotonic(),
                                  transform_matrix(self.transform))
        self.node = SimpleNamespace(
            matrix=self.matrix, distortion=self.distortion, image_size=(640, 480),
            planes={'object': -72.5, 'ring': -20.0}, mode='object',
            base_frame='gimbal_base', camera_frame='camera_optical_frame',
            pair_anchor=None,
        )

    def test_distorted_pixel_projects_to_base_point_and_preserves_image_stamp(self):
        expected = np.array([0.15, 0.04, -0.0725])
        rotation = self.sample.base_from_camera[:3, :3].T
        rvec = cv2.Rodrigues(rotation)[0]
        tvec = -rotation @ self.sample.base_from_camera[:3, 3]
        pixel = cv2.projectPoints(expected.reshape(1, 3), rvec, tvec,
                                  self.matrix, self.distortion)[0].reshape(2)
        point = PixelToBaseNode.project_click(self.node, self.sample, pixel)
        np.testing.assert_allclose([point.point.x, point.point.y, point.point.z], expected, atol=1e-6)
        self.assertEqual(point.header.frame_id, 'gimbal_base')
        self.assertEqual(point.header.stamp, Stamp(sec=42))

    def test_each_plane_intersects_at_its_own_height(self):
        point = PixelToBaseNode.project_click(self.node, self.sample, (288, 238))
        self.assertAlmostEqual(point.point.z, -0.0725)
        self.node.mode = 'ring'
        point = PixelToBaseNode.project_click(self.node, self.sample, (288, 238))
        self.assertAlmostEqual(point.point.z, -0.02)

    def test_lookup_uses_image_time_and_caches_pose_before_later_rotation(self):
        self.node.tf_buffer = Buffer()
        self.node.tf_buffer.set_transform(self.transform, 'test')
        self.node.tf_wait = 0.5
        self.node.get_clock = lambda: Clock(clock_type=ClockType.ROS_TIME)
        self.node.get_logger = Mock()
        self.sample.base_from_camera = None
        PixelToBaseNode.resolve_tf(self.node, self.sample)
        first = PixelToBaseNode.project_click(self.node, self.sample, (288, 238))
        later = TransformStamped()
        later.header.frame_id = 'gimbal_base'
        later.child_frame_id = 'camera_optical_frame'
        later.header.stamp = Stamp(sec=43)
        later.transform.translation.y = 0.1
        later.transform.translation.z = 0.4
        later.transform.rotation.x = math.sqrt(0.5)
        later.transform.rotation.y = math.sqrt(0.5)
        later.transform.rotation.w = 0.0
        self.node.tf_buffer.set_transform(later, 'test')
        PixelToBaseNode.resolve_tf(self.node, self.sample)
        after = PixelToBaseNode.project_click(self.node, self.sample, (288, 238))
        self.assertEqual(first, after)
        self.assertAlmostEqual(after.point.x, 0.1)

    def test_missing_tf_rejects_click_without_using_latest(self):
        self.sample.base_from_camera = None
        self.node.tf_buffer = Buffer()
        self.node.tf_buffer.set_transform(self.transform, 'test')
        self.node.tf_wait = 0.5
        self.node.get_clock = lambda: Clock(clock_type=ClockType.ROS_TIME)
        self.node.get_logger = Mock()
        self.sample.stamp = Stamp(sec=43)
        PixelToBaseNode.resolve_tf(self.node, self.sample)
        self.assertIsNone(self.sample.base_from_camera)
        with self.assertRaisesRegex(ValueError, 'no matching TF'):
            PixelToBaseNode.project_click(self.node, self.sample, (288, 238))

    def test_click_publishes_displayed_frame_not_newest(self):
        self.node.displayed = self.sample
        self.node.latest = ImageSample(self.sample.image, Stamp(sec=43), time.monotonic())
        self.node.publisher = Mock()
        self.node.get_logger = Mock()
        self.node.project_click = lambda sample, pixel: PixelToBaseNode.project_click(self.node, sample, pixel)
        PixelToBaseNode.on_mouse(self.node, cv2.EVENT_LBUTTONDOWN, 288, 238, None, None)
        result = self.node.publisher.publish.call_args.args[0]
        self.assertEqual(result.header.stamp, Stamp(sec=42))

    def test_out_of_image_click_rejected(self):
        with self.assertRaisesRegex(ValueError, 'outside'):
            PixelToBaseNode.project_click(self.node, self.sample, (640, 238))

    def test_place_result_switches_ring_source_after_three_successes(self):
        node = SimpleNamespace(
            ring_centers_topic='/ring_station/centers_pixel',
            ring_centers_topic_after='/stacked_ring/centers_pixel',
            ring_source_switch_count=3,
            active_ring_source='/ring_station/centers_pixel',
            first_layer_placement_count=0,
            current_task_id=None,
            counted_place_commands=set(),
            pending_rings=deque(),
            last_ring_stamp=0,
            get_logger=lambda: Mock(),
            set_parameters=lambda params: None,
        )
        node._set_first_layer_placement_count = (
            lambda count: setattr(node, 'first_layer_placement_count', int(count)))
        node._set_active_ring_source = (
            lambda source, reason: setattr(node, 'active_ring_source', source))

        def result(task_id, command_id, success=True):
            PixelToBaseNode.on_place_result(node, SimpleNamespace(data=json.dumps({
                'task_id': task_id, 'command_id': command_id,
                'success': success})))

        result('task-1', 1)
        result('task-1', 2)
        self.assertEqual(node.first_layer_placement_count, 2)
        self.assertEqual(node.active_ring_source, '/ring_station/centers_pixel')
        result('task-1', 2)  # 重复回传不重复计数
        self.assertEqual(node.first_layer_placement_count, 2)
        result('task-1', 3)
        self.assertEqual(node.first_layer_placement_count, 3)
        self.assertEqual(node.active_ring_source, '/stacked_ring/centers_pixel')
        result('task-1', 4)  # 第一层已完成，后续叠料区不再累加
        self.assertEqual(node.first_layer_placement_count, 3)
        result('task-1', 5, success=False)  # 失败不计数
        self.assertEqual(node.first_layer_placement_count, 3)
        result('task-2', 1)  # 新任务重置回三圆环
        self.assertEqual(node.first_layer_placement_count, 1)
        self.assertEqual(node.active_ring_source, '/ring_station/centers_pixel')

    def test_empty_measure_window_retries_until_deadline(self):
        node = SimpleNamespace(
            ring_measure_window=0.5,
            ring_measure_retry_timeout=3.0,
            measure_buffer=deque(),
            measure_pending=True,
            measure_started=0.0,
            measure_retry_deadline=time.monotonic() + 3.0,
            measure_retry_count=0,
            get_logger=lambda: Mock(),
        )
        PixelToBaseNode._publish_averaged_rings(node)
        self.assertTrue(node.measure_pending)
        self.assertEqual(node.measure_retry_count, 1)
        self.assertGreater(node.measure_started, 0.0)

    def test_empty_measure_window_times_out(self):
        node = SimpleNamespace(
            ring_measure_window=0.5,
            ring_measure_retry_timeout=3.0,
            measure_buffer=deque(),
            measure_pending=True,
            measure_started=0.0,
            measure_retry_deadline=time.monotonic() - 1.0,
            measure_retry_count=2,
            get_logger=lambda: Mock(),
        )
        PixelToBaseNode._publish_averaged_rings(node)
        self.assertFalse(node.measure_pending)
        self.assertEqual(node.measure_retry_count, 2)

    def test_per_ring_radial_offsets(self):
        node = SimpleNamespace(
            ring_radial_offset_mm=10.0,
            ring_extra_radial_mm=(0.0, 20.0, -5.0),
            get_logger=lambda: Mock(),
        )
        # 节点内部坐标单位是米；10mm/20mm/-5mm 额外量分别为 0.01/0.02/-0.005 m。
        projected = [
            np.array([0.1, 0.0, 0.0]),
            np.array([0.0, 0.1, 0.0]),
            np.array([-0.1, 0.0, 0.0]),
        ]
        shifted = PixelToBaseNode._offset_ring_radius(node, projected)
        np.testing.assert_allclose(
            [p[0] for p in shifted], [0.11, 0.0, -0.105], atol=1e-9)
        np.testing.assert_allclose(
            [p[1] for p in shifted], [0.0, 0.13, 0.0], atol=1e-9)

    def test_region_ready_starts_ring_measurement(self):
        node = SimpleNamespace(
            ring_project_enabled=True, ring_measure_window=1.0,
            ring_measure_retry_timeout=3.0,
            ring_centers_topic='/ring_station/centers_pixel',
            ring_centers_topic_after='/stacked_ring/centers_pixel',
            active_ring_source='/ring_station/centers_pixel',
            measure_source=None, measure_pending=False,
            measure_started=0.0, measure_retry_deadline=0.0,
            measure_retry_count=0, measure_buffer=deque(['stale']),
            get_logger=lambda: Mock(),
        )
        PixelToBaseNode.on_region_ready(
            node, String(data=json.dumps({'stage': 3})))
        self.assertEqual(node.active_ring_source, '/stacked_ring/centers_pixel')
        self.assertEqual(node.measure_source, '/stacked_ring/centers_pixel')
        self.assertTrue(node.measure_pending)
        self.assertFalse(node.measure_buffer)
        self.assertGreater(node.measure_started, 0.0)

    def test_robust_ring_centers_rejects_outliers(self):
        messages = []
        for dx in (0.0, 1.0, 2.0, 3.0, 100.0):
            message = PointCloud()
            message.header.frame_id = 'camera'
            message.points = [
                Point32(x=100.0 + dx, y=200.0, z=0.0),
                Point32(x=300.0 + dx, y=200.0, z=0.0),
                Point32(x=500.0 + dx, y=200.0, z=0.0),
            ]
            message.channels = [
                ChannelFloat32(name='ring_id', values=[1.0, 2.0, 3.0])]
            messages.append(message)
        averaged, rejected, kept = PixelToBaseNode._robust_centers(
            messages, 20.0, 3)
        self.assertEqual(rejected, 3)   # 每环剔除 dx=100 的那一帧
        self.assertEqual(kept, 4)
        np.testing.assert_allclose(
            [point.x for point in averaged.points], [101.5, 301.5, 501.5])
        self.assertEqual(averaged.header.frame_id, 'camera')


if __name__ == '__main__':
    unittest.main()
