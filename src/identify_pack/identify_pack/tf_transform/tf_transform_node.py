#!/usr/bin/env python3
"""Publish a yaw-only gimbal TF and a fixed calibrated camera optical TF."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

from identify_pack.tf_transform.transforms import (
    finite_vector, load_camera_pose, normalize_quaternion, yaw_quaternion,
)


class TFTransformNode(Node):
    def __init__(self):
        super().__init__('tf_transform')
        defaults = {
            'base_frame': 'gimbal_base',
            'gimbal_frame': 'gimbal_link',
            'camera_frame': 'camera_optical_frame',
            'joint_states_topic': '/gimbal/joint_states',
            'yaw_joint_name': 'gimbal_yaw_joint',
            'angle_sign': 1.0,
            'angle_offset_rad': 0.0,
            'feedback_max_age_sec': 0.5,
            'future_tolerance_sec': 0.05,
            'base_to_gimbal_translation_m': [0.0, 0.0, 0.0],
            'camera_transform_configured': False,
            'camera_extrinsics_file': '',
            'camera_translation_m': [0.0, 0.0, 0.0],
            'camera_quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
        }
        # Changing static frame geometry requires restarting the node.
        descriptor = ParameterDescriptor(read_only=True)
        for name, value in defaults.items():
            self.declare_parameter(name, value, descriptor)
        self.base_frame = self._param('base_frame')
        self.gimbal_frame = self._param('gimbal_frame')
        self.camera_frame = self._param('camera_frame')
        frames = (self.base_frame, self.gimbal_frame, self.camera_frame)
        if (len(set(frames)) != 3
                or any(not frame or frame.startswith('/') or any(c.isspace() for c in frame)
                       for frame in frames)):
            raise ValueError('Frame names must be distinct, nonempty and have no leading slash/whitespace')
        self.yaw_joint_name = self._param('yaw_joint_name')
        if not self.yaw_joint_name:
            raise ValueError('yaw_joint_name cannot be empty')
        self.sign = self._param('angle_sign')
        self.offset = self._param('angle_offset_rad')
        yaw_quaternion(0.0, self.sign, self.offset)
        self.max_age = self._param('feedback_max_age_sec')
        self.future_tolerance = self._param('future_tolerance_sec')
        if (not math.isfinite(self.max_age) or self.max_age <= 0
                or not math.isfinite(self.future_tolerance) or self.future_tolerance < 0):
            raise ValueError('Invalid feedback time limits')
        self.gimbal_translation = finite_vector(
            self._param('base_to_gimbal_translation_m'), 3, 'base-to-gimbal translation')
        camera_pose = None
        path = self._param('camera_extrinsics_file')
        if path:
            camera_pose = load_camera_pose(path, self.gimbal_frame, self.camera_frame)
        elif self._param('camera_transform_configured'):
            camera_pose = (
                finite_vector(self._param('camera_translation_m'), 3, 'camera translation'),
                normalize_quaternion(self._param('camera_quaternion_xyzw')),
            )

        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_stamp = None
        self.last_feedback_time = None
        self.subscription = self.create_subscription(
            JointState, self._param('joint_states_topic'), self.on_joint_state, 10)
        self.health_timer = self.create_timer(1.0, self.check_feedback)
        if camera_pose:
            transform = self._make_transform(
                self.gimbal_frame, self.camera_frame,
                self.get_clock().now().to_msg(), *camera_pose)
            self.static_broadcaster.sendTransform(transform)
            self.get_logger().info(f'Static TF: {self.gimbal_frame} -> {self.camera_frame}')
        else:
            self.get_logger().warning(
                'Camera extrinsics not configured; camera static TF is disabled. '
                'Set camera_extrinsics_file or camera_transform_configured with measured values.')
        self.get_logger().info(
            f'Waiting for actual yaw (radians) on {self._param("joint_states_topic")}, '
            f'joint={self.yaw_joint_name}; no zero-angle TF is assumed.')

    def _param(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _make_transform(parent, child, stamp, translation, quaternion):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = (float(value) for value in translation)
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = (
             float(value) for value in quaternion)
        return transform

    def on_joint_state(self, msg):
        try:
            if msg.name.count(self.yaw_joint_name) != 1 or len(msg.name) != len(msg.position):
                raise ValueError('Expected one configured yaw joint and matching name/position lengths')
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                raise ValueError('JointState requires an acquisition timestamp in ROS clock time')
            stamp = Time.from_msg(msg.header.stamp, clock_type=self.get_clock().clock_type)
            now = self.get_clock().now()
            age = (now - stamp).nanoseconds / 1e9
            if age > self.max_age or age < -self.future_tolerance:
                raise ValueError(f'Angle feedback timestamp outside allowed range: age={age:.3f}s')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                raise ValueError('Angle timestamps must increase monotonically')
            quaternion = yaw_quaternion(msg.position[msg.name.index(self.yaw_joint_name)],
                                        self.sign, self.offset)
            self.dynamic_broadcaster.sendTransform(self._make_transform(
                self.base_frame, self.gimbal_frame, msg.header.stamp,
                self.gimbal_translation, quaternion))
            self.last_stamp = stamp
            self.last_feedback_time = now
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f'Ignoring yaw feedback: {error}', throttle_duration_sec=2.0)

    def check_feedback(self):
        if (self.last_feedback_time is None
                or (self.get_clock().now() - self.last_feedback_time).nanoseconds / 1e9 > self.max_age):
            self.get_logger().warning(
                'No fresh yaw feedback; dynamic TF is not extrapolated or restamped.',
                throttle_duration_sec=5.0)

    def point_to_base(self, point):
        """Convert a metric PointStamped at its acquisition time; TF errors reach caller.

        Pixel coordinates must first be projected into 3D using intrinsics and
        the selected target plane. This method does not accept pixel coordinates.
        """
        if not point.header.frame_id:
            raise ValueError('PointStamped requires a source frame')
        if point.header.stamp.sec == 0 and point.header.stamp.nanosec == 0:
            raise ValueError('PointStamped requires an acquisition timestamp')
        transform = self.tf_buffer.lookup_transform(
            self.base_frame, point.header.frame_id,
            Time.from_msg(point.header.stamp, clock_type=self.get_clock().clock_type))
        return do_transform_point(point, transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TFTransformNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
