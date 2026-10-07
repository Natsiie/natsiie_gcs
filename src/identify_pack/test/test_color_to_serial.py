import copy
import math
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from builtin_interfaces.msg import Time as Stamp
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, TransformStamped
from rclpy.clock import ClockType
from rclpy.time import Time
from sensor_msgs.msg import ChannelFloat32, PointCloud
from tf2_ros import Buffer

from identify_pack.scripts.color_config import build_default_color_config
from identify_pack.serial.protocol import COLOR_TARGET_TYPE, OUTGOING_TOPIC_TYPES, parse_frame
from identify_pack.serial.serial_bridge_node import SerialBridgeNode
from identify_pack.tf_transform.pixel_to_base_node import PixelToBaseNode
from identify_pack.vision_identify.color_detect_node import ColorDetectNode


class ColorToSerialTests(unittest.TestCase):
    def setUp(self):
        self.clock = Mock()
        self.clock.clock_type = ClockType.ROS_TIME
        self.clock.now.return_value = Time(seconds=42.05, clock_type=ClockType.ROS_TIME)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'gimbal_base'
        self.transform.child_frame_id = 'camera_optical_frame'
        self.transform.header.stamp = Stamp(sec=42)
        self.transform.transform.translation.x = 0.1
        self.transform.transform.translation.z = 0.4
        self.transform.transform.rotation.x = 1.0
        self.transform.transform.rotation.w = 0.0
        params = {'auto_project': True}
        self.projector = SimpleNamespace(
            planes={'object': -12.5}, camera_frame='camera_optical_frame',
            base_frame='gimbal_base', image_size=(640, 480),
            matrix=np.array([[308., 0, 288.], [0, 305., 238.], [0, 0, 1]]),
            distortion=np.zeros(5), tf_buffer=Buffer(),
            last_auto_publish=-math.inf, auto_interval=0.2, target_max_age=1.0,
            tf_wait=0.5, pending_centers=deque(maxlen=10), last_center_stamp=0,
            param=lambda name: params[name], target_publisher=Mock(),
            get_clock=lambda: self.clock, get_logger=Mock(),
            _channel_values=PixelToBaseNode._channel_values,
        )
        for name in ('validate_centers', 'project_pending_centers', 'project_centers'):
            setattr(self.projector, name,
                    getattr(PixelToBaseNode, name).__get__(self.projector))
        self.projector.tf_buffer.set_transform(self.transform, 'test')
        self.bridge = SimpleNamespace(
            target_frame='gimbal_base', target_max_age=1.0, target_max_hz=5.0,
            target_topic='/color/targets_base', target_type=COLOR_TARGET_TYPE,
            _last_target_send={}, _last_target_stamp={},
            task_mode=False,
            get_clock=lambda: self.clock, get_logger=Mock(), ser=Mock(),
        )
        self.bridge.ser.is_open = True
        self.bridge.ser.write.side_effect = lambda frame: len(frame)
        self.bridge._send = SerialBridgeNode._send.__get__(self.bridge)

    def centers(self):
        msg = PointCloud()
        msg.header.frame_id = 'camera_optical_frame'
        msg.header.stamp = Stamp(sec=42)
        msg.points = [Point32(x=288., y=238., z=0.)]
        msg.channels = [ChannelFloat32(name=name, values=[value]) for name, value in
                        [('color_id', 1.), ('area_px2', 1000.),
                         ('image_width', 640.), ('image_height', 480.)]]
        return msg

    def project(self, message):
        PixelToBaseNode.on_color_centers(self.projector, message)
        if self.projector.target_publisher.publish.called:
            return self.projector.target_publisher.publish.call_args.args[0]

    def test_real_color_detection_projects_and_sends_mm_frame(self):
        detector = SimpleNamespace(
            br=CvBridge(), color_cfg=build_default_color_config(), color_roi=None,
            lock_dominant=True, min_blob_area=300., min_blob_dimension=12.,
            show=False, camera_frame='camera_optical_frame', motion_detect=False,
            require_single_roi=False,
            pub_centers=Mock(), pub_color=Mock(), get_logger=Mock(),
        )
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        image[218:259, 268:309] = [0, 0, 255]
        message = detector.br.cv2_to_imgmsg(image, encoding='bgr8')
        message.header.stamp = Stamp(sec=42)
        ColorDetectNode.image_cb(detector, message)
        centers = detector.pub_centers.publish.call_args.args[0]
        self.assertEqual(centers.header.stamp, message.header.stamp)
        self.assertEqual(centers.header.frame_id, 'camera_optical_frame')
        self.assertEqual(detector.pub_color.publish.call_args.args[0].data, '1:288,238')
        target = self.project(centers)
        self.assertEqual(target.header.stamp, message.header.stamp)
        SerialBridgeNode._send_color_targets(self.bridge, target)
        frame = self.bridge.ser.write.call_args.args[0]
        self.assertEqual(parse_frame(frame), (0x08, '1,100.0,0.0,-12.5', b''))
        self.assertEqual(frame, bytes.fromhex(
            'ff 08 11 31 2c 31 30 30 2e 30 2c 30 2e 30 2c 2d 31 32 2e 35 30 fe'))

    def test_largest_blob_per_color_and_all_colors_sent(self):
        msg = self.centers()
        msg.points += [Point32(x=388., y=238., z=0.), Point32(x=288., y=338., z=0.)]
        for channel in msg.channels:
            channel.values = {
                'color_id': [1., 1., 3.], 'area_px2': [1000., 2000., 500.],
                'image_width': [640.] * 3, 'image_height': [480.] * 3,
            }[channel.name]
        targets = self.project(msg)
        self.assertEqual(len(targets.points), 2)
        self.assertEqual(list(targets.channels[0].values), [1., 3.])
        self.assertGreater(targets.points[0].x, 0.2)
        SerialBridgeNode._send_color_targets(self.bridge, targets)
        self.assertEqual(self.bridge.ser.write.call_count, 2)
        payloads = [parse_frame(call.args[0])[1] for call in self.bridge.ser.write.call_args_list]
        self.assertTrue(payloads[0].startswith('1,'))
        self.assertTrue(payloads[1].startswith('3,'))

    def test_missing_image_time_tf_waits_for_matching_transform(self):
        self.projector.tf_buffer.clear()
        msg = self.centers()
        self.assertIsNone(self.project(msg))
        self.projector.tf_buffer.set_transform(self.transform, 'test')
        self.projector.project_pending_centers()
        self.projector.target_publisher.publish.assert_called_once()

    def test_future_tf_does_not_replace_image_time(self):
        self.projector.tf_buffer.clear()
        self.transform.header.stamp = Stamp(sec=43)
        self.projector.tf_buffer.set_transform(self.transform, 'test')
        self.assertIsNone(self.project(self.centers()))

    def test_bad_pixels_metadata_resolution_and_timestamps_rejected(self):
        invalid = []
        msg = self.centers()
        msg.points[0].x = float('nan')
        invalid.append(msg)
        msg = self.centers()
        msg.points[0].x = 640.
        invalid.append(msg)
        msg = self.centers()
        msg.channels[0].values = []
        invalid.append(msg)
        msg = self.centers()
        msg.channels[0].values = [1.5]
        invalid.append(msg)
        msg = self.centers()
        msg.channels[2].values = [320.]
        invalid.append(msg)
        msg = self.centers()
        msg.header.frame_id = 'other_camera'
        invalid.append(msg)
        msg = self.centers()
        msg.header.stamp = Stamp()
        invalid.append(msg)
        for msg in invalid:
            with self.subTest(msg=msg):
                self.assertIsNone(self.project(msg))
        self.projector.target_publisher.publish.assert_not_called()

    def test_serial_rejects_stale_future_nan_and_wrong_frame(self):
        valid = self.project(self.centers())
        for stamp in (Stamp(), Stamp(sec=40), Stamp(sec=43)):
            msg = copy.deepcopy(valid)
            msg.header.stamp = stamp
            SerialBridgeNode._send_color_targets(self.bridge, msg)
        msg = copy.deepcopy(valid)
        msg.header.frame_id = 'camera_optical_frame'
        SerialBridgeNode._send_color_targets(self.bridge, msg)
        msg = copy.deepcopy(valid)
        msg.points[0].x = float('nan')
        SerialBridgeNode._send_color_targets(self.bridge, msg)
        self.bridge.ser.write.assert_not_called()

    def test_per_color_rate_limit_and_duplicate_stamp(self):
        msg = self.project(self.centers())
        with patch('identify_pack.serial.serial_bridge_node.time.monotonic', return_value=10.):
            SerialBridgeNode._send_color_targets(self.bridge, msg)
            msg.header.stamp.nanosec = 10_000_000
            SerialBridgeNode._send_color_targets(self.bridge, msg)
        self.assertEqual(self.bridge.ser.write.call_count, 1)
        with patch('identify_pack.serial.serial_bridge_node.time.monotonic', return_value=10.21):
            SerialBridgeNode._send_color_targets(self.bridge, msg)
            SerialBridgeNode._send_color_targets(self.bridge, msg)
        self.assertEqual(self.bridge.ser.write.call_count, 2)

    def test_failed_or_partial_write_is_not_recorded_as_sent(self):
        msg = self.project(self.centers())
        self.bridge.ser.write.side_effect = IOError('port disconnected')
        SerialBridgeNode._send_color_targets(self.bridge, msg)
        self.assertEqual(self.bridge._last_target_stamp, {})
        self.bridge.ser.write.side_effect = lambda frame: len(frame) - 1
        SerialBridgeNode._send_color_targets(self.bridge, msg)
        self.assertEqual(self.bridge._last_target_stamp, {})
        self.bridge.ser.write.side_effect = lambda frame: len(frame)
        SerialBridgeNode._send_color_targets(self.bridge, msg)
        self.assertEqual(self.bridge._last_target_stamp, {1: 42_000_000_000})

    def test_target_cannot_bypass_typed_subscription_as_string(self):
        self.assertNotIn('ColorTarget', OUTGOING_TOPIC_TYPES)


if __name__ == '__main__':
    unittest.main()
