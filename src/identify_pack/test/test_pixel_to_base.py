import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import cv2
import numpy as np
from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
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


if __name__ == '__main__':
    unittest.main()
