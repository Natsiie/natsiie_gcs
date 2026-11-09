import copy
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
from builtin_interfaces.msg import Time as Stamp
from cv_bridge import CvBridge
from rclpy.time import Time
from rclpy.clock import ClockType
from sensor_msgs.msg import ChannelFloat32, PointCloud
from std_msgs.msg import Bool, UInt32, String

from identify_pack.scripts.color_config import build_default_color_config
from identify_pack.scripts.pick_task import PickTask
from identify_pack.scripts.turntable_motion import OpticalFlowMotion
from identify_pack.serial.serial_bridge_node import SerialBridgeNode
from identify_pack.serial.protocol import GRAB_CMD_TYPE, make_frame, parse_frame
from identify_pack.tf_transform.pixel_to_base_node import PixelToBaseNode
from identify_pack.vision_identify.color_detect_node import ColorDetectNode
import test_color_to_serial as pipeline_fixture


class TaskPipelineTests(unittest.TestCase):
    def setUp(self):
        fixture = pipeline_fixture.ColorToSerialTests()
        fixture.setUp()
        self.fixture = fixture
        self.bridge = fixture.bridge
        self.bridge.task_mode = True
        self.bridge.camera_frame = 'camera_optical_frame'
        self.bridge.task = PickTask()
        self.bridge.task_code_sent = False
        self.bridge.pub_task_status = Mock()
        self.bridge._publish_task_status = SerialBridgeNode._publish_task_status.__get__(self.bridge)
        self.bridge.motion_stamp_ns = 0
        self.bridge.grab_cmd_sent = False
        self.bridge.grab_cmd_pending = False
        self.bridge.require_turntable_align = False
        self.bridge.turntable_within = True
        self.bridge.turntable_within_time = 0.0
        self.bridge.turntable_align_max_age = 1.0
        for name in ('_try_send_grab_cmd', '_turntable_ready',
                     '_update_turntable_align', '_reset_turntable_gate'):
            setattr(self.bridge, name,
                    getattr(SerialBridgeNode, name).__get__(self.bridge))
        self.bridge.ready_since_ns = 0
        self.bridge.motion_ready = False
        self.bridge.motion_receive_time = -float('inf')

    def centers(self, ready, nanosec=100_000_000):
        msg = self.fixture.centers()
        msg.header.stamp = Stamp(sec=42, nanosec=nanosec)
        msg.channels.append(ChannelFloat32(name='motion_ready', values=[float(ready)]))
        return msg

    def target(self, center):
        self.fixture.projector.validate_centers(center)
        return self.fixture.projector.project_centers(
            center, np.array([[1., 0., 0., .1], [0., -1., 0., 0.],
                              [0., 0., -1., .4], [0., 0., 0., 1.]]))

    def test_match_once_then_confirm_and_wait_for_next_color(self):
        SerialBridgeNode._on_task(self.bridge, String(data='156+123+516+231'))
        self.bridge.ser.write.reset_mock()
        self.fixture.clock.now.return_value = Time(seconds=42.2, clock_type=ClockType.ROS_TIME)
        center = self.centers(True)
        target = self.target(center)
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.bridge.ser.write.assert_not_called()  # No current motion state.
        SerialBridgeNode._on_motion(self.bridge, center)
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.bridge.ser.write.assert_called_once()
        self.assertEqual(self.bridge.task.state, 'WAIT_PICK_DONE')
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.bridge.ser.write.assert_called_once()
        SerialBridgeNode._on_pick_done(self.bridge, UInt32(data=1))
        self.assertEqual(self.bridge.task.expected.color, 5)
        SerialBridgeNode._on_task(self.bridge, String(data='156+123+516+231'))
        self.assertEqual(self.bridge.task.expected.color, 5)
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.bridge.ser.write.assert_called_once()

    def test_arrived_after_matching_color_sends_grab_command(self):
        SerialBridgeNode._on_task(self.bridge, String(data='156+123+516+231'))
        self.fixture.clock.now.return_value = Time(seconds=42.2, clock_type=ClockType.ROS_TIME)
        center = self.centers(True)
        SerialBridgeNode._on_motion(self.bridge, center)
        target = self.target(center)
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.assertEqual(self.bridge.task.state, 'WAIT_PICK_DONE')
        self.bridge.ser.write.reset_mock()
        SerialBridgeNode._on_arrived(self.bridge, Bool(data=True))
        self.bridge.ser.write.assert_called_once_with(make_frame(GRAB_CMD_TYPE, '1'))

    def test_moving_or_unknown_blocks_delayed_target_and_clears_projection_queue(self):
        self.bridge.task.load('156+123+516+231', 1)
        self.bridge.task_code_sent = True
        self.fixture.clock.now.return_value = Time(seconds=42.3, clock_type=ClockType.ROS_TIME)
        ready = self.centers(True)
        SerialBridgeNode._on_motion(self.bridge, ready)
        target = self.target(ready)
        moving = self.centers(False, 200_000_000)
        moving.points = []
        SerialBridgeNode._on_motion(self.bridge, moving)
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.bridge.ser.write.assert_not_called()
        self.fixture.projector.pending_centers.append((ready, time.monotonic()))
        PixelToBaseNode.on_color_centers(self.fixture.projector, moving)
        self.assertFalse(self.fixture.projector.pending_centers)
        # Once ready again, reject a delayed target from before that ready interval.
        SerialBridgeNode._on_motion(self.bridge, self.centers(True, 250_000_000))
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.bridge.ser.write.assert_not_called()

    def test_failed_task_write_blocks_coordinates_until_task_retry_succeeds(self):
        self.bridge.ser.write.side_effect = IOError('disconnected')
        task = String(data='156+123+516+231')
        SerialBridgeNode._on_task(self.bridge, task)
        self.assertFalse(self.bridge.task_code_sent)
        self.bridge.ser.write.reset_mock()
        self.fixture.clock.now.return_value = Time(seconds=42.2, clock_type=ClockType.ROS_TIME)
        center = self.centers(True)
        SerialBridgeNode._on_motion(self.bridge, center)
        SerialBridgeNode._send_color_targets(self.bridge, self.target(center))
        self.bridge.ser.write.assert_not_called()
        self.bridge.ser.write.side_effect = lambda frame: len(frame)
        SerialBridgeNode._on_task(self.bridge, task)
        self.assertTrue(self.bridge.task_code_sent)
        SerialBridgeNode._send_color_targets(self.bridge, self.target(center))
        self.assertEqual(self.bridge.task.state, 'WAIT_PICK_DONE')

    def test_failed_coordinate_write_does_not_lock_or_advance(self):
        SerialBridgeNode._on_task(self.bridge, String(data='156+123+516+231'))
        self.fixture.clock.now.return_value = Time(seconds=42.2, clock_type=ClockType.ROS_TIME)
        center = self.centers(True)
        SerialBridgeNode._on_motion(self.bridge, center)
        target = self.target(center)
        self.bridge.ser.write.side_effect = IOError('disconnected')
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.assertEqual(self.bridge.task.state, 'WAIT_MATCH')
        SerialBridgeNode._on_pick_done(self.bridge, UInt32(data=1))
        self.assertEqual(self.bridge.task.expected.sequence, 1)
        self.bridge.ser.write.side_effect = lambda frame: len(frame)
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.assertEqual(self.bridge.task.state, 'WAIT_PICK_DONE')

    def test_expired_motion_blocks_fresh_target(self):
        self.bridge.task.load('156+123+516+231', 1)
        self.bridge.task_code_sent = True
        center = self.centers(True)
        SerialBridgeNode._on_motion(self.bridge, center)
        self.bridge.motion_receive_time = time.monotonic() - 2.
        SerialBridgeNode._send_color_targets(self.bridge, self.target(center))
        self.bridge.ser.write.assert_not_called()

    @patch('identify_pack.serial.serial_bridge_node.time.monotonic')
    def test_all_six_targets_require_matching_confirmations_and_batch_notification(self, monotonic):
        SerialBridgeNode._on_task(self.bridge, String(data='156+123+516+231'))
        self.bridge.ser.write.reset_mock()
        for sequence, color in enumerate((1, 5, 6, 5, 1, 6), start=1):
            monotonic.return_value = 100. + sequence
            self.fixture.clock.now.return_value = Time(
                seconds=42. + sequence, clock_type=ClockType.ROS_TIME)
            center = self.centers(True)
            center.header.stamp = Stamp(sec=42 + sequence)
            center.channels[0].values = [float(color)]
            SerialBridgeNode._on_motion(self.bridge, center)
            target = self.target(center)
            SerialBridgeNode._send_color_targets(self.bridge, target)
            self.assertEqual(self.bridge.ser.write.call_count, sequence)
            self.assertEqual(parse_frame(self.bridge.ser.write.call_args.args[0])[1].split(',')[0],
                             str(color))
            SerialBridgeNode._on_pick_done(self.bridge, UInt32(data=sequence + 1))
            self.assertEqual(self.bridge.task.state, 'WAIT_PICK_DONE')
            SerialBridgeNode._send_color_targets(self.bridge, target)
            self.assertEqual(self.bridge.ser.write.call_count, sequence)
            SerialBridgeNode._on_pick_done(self.bridge, UInt32(data=sequence))
            if sequence == 3:
                self.assertEqual(self.bridge.task.state, 'WAIT_NEXT_BATCH')
                SerialBridgeNode._on_next_batch(self.bridge, UInt32(data=2))
                self.assertEqual(self.bridge.task.state, 'WAIT_NEXT_BATCH')
                SerialBridgeNode._on_next_batch(self.bridge, UInt32(data=1))
                self.assertEqual(self.bridge.task.state, 'WAIT_MATCH')
        self.assertEqual(self.bridge.task.state, 'DONE')
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.assertEqual(self.bridge.ser.write.call_count, 6)

    def test_wrong_color_and_missing_task_cannot_send(self):
        center = self.centers(True, 0)
        SerialBridgeNode._on_motion(self.bridge, center)
        target = self.target(center)
        self.bridge.task_code_sent = True
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.bridge.ser.write.assert_not_called()
        self.bridge.task.load('516+123+156+231', 1)
        SerialBridgeNode._send_color_targets(self.bridge, target)
        self.bridge.ser.write.assert_not_called()

    def test_color_output_is_not_gated_by_stationary_state_with_one_roi(self):
        node = SimpleNamespace(br=CvBridge(), color_cfg=build_default_color_config(),
                               color_roi=[(0, 0, 640, 480)], lock_dominant=True,
                               min_blob_area=300., min_blob_dimension=12., show=False,
                               camera_frame='camera_optical_frame',
                               require_single_roi=True, stationary=Mock(), pub_motion=Mock(),
                               pub_centers=Mock(), pub_color=Mock(), get_logger=Mock(),
                               pub_arrived=Mock(), publish_arrived=True,
                               arrived_topic='/arm/arrived', _arrived_armed=True)
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        image[218:259, 268:309] = [0, 0, 255]
        msg = node.br.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = Stamp(sec=42)
        for state, count in (('WAITING', 0), ('TRACKING', 3), ('MISSING', 14)):
            node.stationary.update.return_value = {
                'state': state, 'count': count, 'required_frames': 25,
                'fps': 25., 'ready': False, 'missing_frames': 0}
            ColorDetectNode.image_cb(node, msg)
            # 未停稳也正常发布色块和坐标，只有 OK 不发。
            self.assertEqual(node.pub_color.publish.call_args.args[0].data, '1:288,238')
            published = node.pub_centers.publish.call_args.args[0]
            self.assertEqual(len(published.points), 1)
            self.assertEqual(list(published.channels[-1].values), [1.0])
            node.pub_arrived.publish.assert_not_called()
        node.stationary.update.return_value = {
            'state': 'READY', 'count': 25, 'required_frames': 25,
            'fps': 25., 'ready': True, 'missing_frames': 0}
        ColorDetectNode.image_cb(node, msg)
        self.assertEqual(node.pub_color.publish.call_args.args[0].data, '1:288,238')
        node.pub_arrived.publish.assert_called_once()
        # 多个 ROI 时仍拒绝输出目标，避免任务抓取选错物块。
        node.color_roi.append((0, 0, 100, 100))
        node.pub_color.publish.reset_mock()
        ColorDetectNode.image_cb(node, msg)
        node.pub_color.publish.assert_not_called()
        published = node.pub_centers.publish.call_args.args[0]
        self.assertFalse(published.points)

    def test_stationary_ok_fires_once_and_rearms_after_object_leaves(self):
        node = SimpleNamespace(br=CvBridge(), color_cfg=build_default_color_config(),
                               color_roi=[(0, 0, 640, 480)], lock_dominant=True,
                               min_blob_area=300., min_blob_dimension=12., show=False,
                               camera_frame='camera_optical_frame',
                               require_single_roi=True, stationary=Mock(), pub_motion=Mock(),
                               pub_centers=Mock(), pub_color=Mock(), get_logger=Mock(),
                               pub_arrived=Mock(), publish_arrived=True,
                               arrived_topic='/arm/arrived', _arrived_armed=True)
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        image[218:259, 268:309] = [0, 0, 255]
        msg = node.br.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = Stamp(sec=42)
        ready = {'state': 'READY', 'count': 15, 'required_frames': 15,
                 'fps': 25., 'ready': True, 'missing_frames': 0, 'reset': False}
        node.stationary.update.return_value = ready
        ColorDetectNode.image_cb(node, msg)
        node.pub_arrived.publish.assert_called_once()
        # 物块继续停稳：不重复发 OK。
        ColorDetectNode.image_cb(node, msg)
        node.pub_arrived.publish.assert_called_once()
        # 物块被抓走 -> WAITING，重新武装。
        node.stationary.update.return_value = {
            'state': 'WAITING', 'count': 0, 'required_frames': 15,
            'fps': 25., 'ready': False, 'missing_frames': 0, 'reset': True}
        ColorDetectNode.image_cb(node, msg)
        # 下一个物块停稳 -> 再发一次。
        node.stationary.update.return_value = ready
        ColorDetectNode.image_cb(node, msg)
        self.assertEqual(node.pub_arrived.publish.call_count, 2)
        # ROI 里始终有目标、从未 WAITING 时，位置跳变（reset）也要重新武装。
        node.stationary.update.return_value = {
            'state': 'TRACKING', 'count': 1, 'required_frames': 15,
            'fps': 25., 'ready': False, 'missing_frames': 0, 'reset': True}
        ColorDetectNode.image_cb(node, msg)
        node.stationary.update.return_value = ready
        ColorDetectNode.image_cb(node, msg)
        self.assertEqual(node.pub_arrived.publish.call_count, 3)

    def test_optical_motion_initial_stop_motion_stop_motion_and_texture_loss(self):
        rng = np.random.default_rng(5)
        gray = rng.integers(0, 256, (240, 320), dtype=np.uint8)
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        motion = OpticalFlowMotion(moving_speed=15., near_stop_speed=3.,
                                   required_moving_frames=2, required_low_speed_frames=3)
        for i in range(5):
            self.assertFalse(motion.update(image, 10. + i * .05)['ready'])
        for i in range(1, 4):
            moved = cv2.warpAffine(image, np.float32([[1, 0, i * 4], [0, 1, 0]]), (320, 240))
            result = motion.update(moved, 10.2 + i * .05)
            self.assertFalse(result['ready'])
        for i in range(1, 4):
            result = motion.update(moved, 10.35 + i * .05)
        self.assertTrue(result['ready'], result)
        moved = cv2.warpAffine(image, np.float32([[1, 0, 16], [0, 1, 0]]), (320, 240))
        self.assertFalse(motion.update(moved, 10.55)['ready'])
        blank = np.zeros_like(image)
        for i in range(3):
            result = motion.update(blank, 10.6 + i * .05)
        self.assertFalse(result['ready'])
