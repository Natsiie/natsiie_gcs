import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from builtin_interfaces.msg import Time

from identify_pack.serial.protocol import GIMBAL_YAW_TYPE, OUTGOING_TOPIC_TYPES, make_frame
from identify_pack.serial.serial_bridge import SerialBridge


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
            get_clock=lambda: clock, get_logger=lambda: logger,
        )

    def feed(self, bridge, data):
        bridge.ser.in_waiting = len(data)
        bridge.ser.read.return_value = data
        SerialBridge._poll_feedback(bridge)

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


if __name__ == '__main__':
    unittest.main()
