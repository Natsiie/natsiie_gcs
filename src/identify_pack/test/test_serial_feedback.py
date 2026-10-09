import math
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from std_msgs.msg import Bool, String

from identify_pack.serial.protocol import (
    GIMBAL_YAW_TYPE, GRAB_CMD_TYPE, OUTGOING_TOPIC_TYPES, make_frame,
    PLACE_COMMAND_TYPE, PLACE_RESULT_TYPE, parse_frame,
)
from identify_pack.serial.serial_bridge_node import SerialBridgeNode


class SerialFeedbackTests(unittest.TestCase):
    def make_bridge(self):
        port = Mock()
        port.is_open = True
        logger = Mock()
        clock = Mock()
        clock.now.return_value.to_msg.return_value = Time(sec=42, nanosec=123)
        return SimpleNamespace(
            ser=port, rx_buffer=bytearray(), yaw_feedback_type=GIMBAL_YAW_TYPE,
            yaw_joint_name='gimbal_yaw_joint', pub_joint_state=Mock(),
            joint_states_topic='/gimbal/joint_states',
            get_clock=lambda: clock, get_logger=lambda: logger,
            target_frame='gimbal_base', pub_place_result=Mock(),
        )

    def feed(self, bridge, data):
        bridge.ser.in_waiting = len(data)
        bridge.ser.read.return_value = data
        SerialBridgeNode._poll_feedback(bridge)

    def test_fragmented_angle_publishes_radians_and_timestamp(self):
        bridge = self.make_bridge()
        frame = make_frame(GIMBAL_YAW_TYPE, '90.0')
        self.feed(bridge, frame[:4])
        bridge.pub_joint_state.publish.assert_not_called()
        self.feed(bridge, frame[4:])
        message = bridge.pub_joint_state.publish.call_args.args[0]
        self.assertEqual(message.name, ['gimbal_yaw_joint'])
        self.assertAlmostEqual(message.position[0], math.pi / 2)
        self.assertEqual(message.header.stamp.sec, 42)
        self.assertEqual(message.header.stamp.nanosec, 123)
        self.assertEqual(bridge.rx_buffer, b'')

    def test_bad_angles_and_wrong_type_do_not_publish(self):
        bridge = self.make_bridge()
        self.feed(bridge, b''.join([
            make_frame(0x01, '90.0'),
            make_frame(GIMBAL_YAW_TYPE, 'nan'),
            make_frame(GIMBAL_YAW_TYPE, 'inf'),
            make_frame(GIMBAL_YAW_TYPE, 'hello'),
        ]))
        bridge.pub_joint_state.publish.assert_not_called()
        self.feed(bridge, make_frame(GIMBAL_YAW_TYPE, '-30.5'))
        message = bridge.pub_joint_state.publish.call_args.args[0]
        self.assertAlmostEqual(message.position[0], math.radians(-30.5))

    def test_yaw_feedback_is_not_an_outgoing_topic(self):
        self.assertNotIn('GimbalYaw', OUTGOING_TOPIC_TYPES)

    def test_raw_text_is_logged_but_not_published_as_angle(self):
        bridge = self.make_bridge()
        self.feed(bridge, b'90.0\r\n')
        bridge.pub_joint_state.publish.assert_not_called()
        bridge.get_logger().debug.assert_called_once()
        bridge.get_logger().warning.assert_called_once()
        self.assertEqual(bridge.rx_buffer, b'')


    def test_arrived_sends_grab_command_and_is_not_outgoing(self):
        self.assertNotIn('GrabCmd', OUTGOING_TOPIC_TYPES)
        bridge = self.make_bridge()
        bridge._send = SerialBridgeNode._send.__get__(bridge)
        SerialBridgeNode._on_arrived(bridge, Bool(data=True))
        bridge.ser.write.assert_called_once_with(make_frame(GRAB_CMD_TYPE, '1'))
        bridge.ser.write.reset_mock()
        SerialBridgeNode._on_arrived(bridge, Bool(data=False))
        bridge.ser.write.assert_not_called()

    def test_placement_feedback_and_invalid_results(self):
        bridge = self.make_bridge()
        task_id = 'a' * 32
        self.feed(bridge, make_frame(PLACE_RESULT_TYPE, f'{task_id},7,1'))
        self.assertEqual(json.loads(bridge.pub_place_result.publish.call_args.args[0].data),
                         dict(task_id=task_id, command_id=7, success=True))
        bridge.pub_place_result.publish.reset_mock()
        self.feed(bridge, make_frame(PLACE_RESULT_TYPE, f'{task_id},7,2'))
        bridge.pub_place_result.publish.assert_not_called()
        self.assertNotIn('PlaceResult', OUTGOING_TOPIC_TYPES)
        self.assertNotIn('PlaceCommand', OUTGOING_TOPIC_TYPES)

    def test_placement_transport_failure_does_not_claim_completion(self):
        bridge = self.make_bridge()
        bridge._send = SerialBridgeNode._send.__get__(bridge)
        command = dict(task_id='a' * 32, command_id=1, stage='stacked_ring',
                       batch=2, sequence=4, color=5, ring=2, layer=2,
                       frame_id='gimbal_base', x_mm=100., y_mm=50., z_mm=-12.5)
        bridge.ser.write.side_effect = lambda frame: len(frame)
        SerialBridgeNode._on_place_command(bridge, String(data=json.dumps(command)))
        frame = bridge.ser.write.call_args.args[0]
        self.assertEqual(parse_frame(frame)[:2],
                         (PLACE_COMMAND_TYPE, 'a' * 32 + ',1,2,2,4,5,2,2,100.00,50.00,-12.50'))
        bridge.pub_place_result.publish.assert_not_called()
        bridge.ser.write.side_effect = IOError('disconnected')
        SerialBridgeNode._on_place_command(bridge, String(data=json.dumps(command)))
        self.assertFalse(json.loads(bridge.pub_place_result.publish.call_args.args[0].data)['success'])


if __name__ == '__main__':
    unittest.main()
