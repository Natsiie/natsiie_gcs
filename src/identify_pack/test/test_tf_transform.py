import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer

from identify_pack.tf_transform.tf_transform_node import TFTransformNode
from identify_pack.tf_transform.transforms import load_camera_pose, normalize_quaternion, yaw_quaternion


class TFTransformTests(unittest.TestCase):
    def test_yaw_sign_offset_and_quaternion_order(self):
        self.assertEqual(yaw_quaternion(math.pi / 2, -1, math.pi / 2), (0, 0, 0, 1))
        self.assertAlmostEqual(yaw_quaternion(math.pi / 2)[2], math.sqrt(0.5))
        self.assertEqual(normalize_quaternion([0, 0, 0, 2]), (0, 0, 0, 1))
        for values in ([0, 0, 0, 0], [0, 0, math.nan, 1]):
            with self.assertRaises(ValueError):
                normalize_quaternion(values)
        with self.assertRaises(ValueError):
            yaw_quaternion(0, sign=0)

    def test_extrinsics_frame_validation(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'extrinsics.json'
            path.write_text(json.dumps({
                'parent_frame': 'gimbal_link', 'child_frame': 'camera_optical_frame',
                'translation_m': [0.1, 0.2, 0.3], 'quaternion_xyzw': [1, 0, 0, 0],
            }), encoding='utf-8')
            translation, quaternion = load_camera_pose(path, 'gimbal_link', 'camera_optical_frame')
            self.assertEqual(translation, (0.1, 0.2, 0.3))
            self.assertEqual(quaternion, (1, 0, 0, 0))
            with self.assertRaises(ValueError):
                load_camera_pose(path, 'gimbal_base', 'camera_optical_frame')

    def test_tf_chain_transforms_metric_point(self):
        stamp = Time(seconds=10).to_msg()
        buffer = Buffer()
        buffer.set_transform(TFTransformNode._make_transform(
            'gimbal_base', 'gimbal_link', stamp, (0, 0, 0), yaw_quaternion(math.pi / 2)), 'test')
        buffer.set_transform_static(TFTransformNode._make_transform(
            'gimbal_link', 'camera_optical_frame', stamp, (0.1, 0, 0.2), (1, 0, 0, 0)), 'test')
        point = PointStamped()
        point.header.frame_id = 'camera_optical_frame'
        point.header.stamp = stamp
        point.point.x, point.point.y, point.point.z = (0.05, 0.02, 0.1)
        node = SimpleNamespace(base_frame='gimbal_base', tf_buffer=buffer,
                               get_clock=lambda: Clock(clock_type=ClockType.SYSTEM_TIME))
        result = TFTransformNode.point_to_base(node, point)
        self.assertEqual(result.header.frame_id, 'gimbal_base')
        self.assertAlmostEqual(result.point.x, 0.02)
        self.assertAlmostEqual(result.point.y, 0.15)
        self.assertAlmostEqual(result.point.z, 0.1)

    def test_feedback_does_not_publish_stale_or_invalid_angles(self):
        clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        now = clock.now()
        logger = Mock()
        broadcaster = Mock()
        node = SimpleNamespace(
            yaw_joint_name='gimbal_yaw_joint', max_age=0.5, future_tolerance=0.05,
            last_stamp=None, sign=1.0, offset=0.0, base_frame='gimbal_base',
            gimbal_frame='gimbal_link', gimbal_translation=(0, 0, 0),
            dynamic_broadcaster=broadcaster, get_clock=lambda: clock,
            get_logger=lambda: logger, _make_transform=TFTransformNode._make_transform,
        )
        msg = JointState()
        msg.name = ['gimbal_yaw_joint']
        msg.position = [math.pi / 2]
        msg.header.stamp = Time(nanoseconds=now.nanoseconds - 2_000_000_000).to_msg()
        TFTransformNode.on_joint_state(node, msg)
        broadcaster.sendTransform.assert_not_called()
        msg.header.stamp = now.to_msg()
        TFTransformNode.on_joint_state(node, msg)
        broadcaster.sendTransform.assert_called_once()
        transform = broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(transform.header.stamp, msg.header.stamp)
        TFTransformNode.on_joint_state(node, msg)
        self.assertEqual(broadcaster.sendTransform.call_count, 1)
        msg.header.stamp = clock.now().to_msg()
        msg.position = [math.nan]
        TFTransformNode.on_joint_state(node, msg)
        self.assertEqual(broadcaster.sendTransform.call_count, 1)


if __name__ == '__main__':
    unittest.main()
