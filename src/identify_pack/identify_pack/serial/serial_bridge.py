#!/usr/bin/env python3
"""统一串口桥：发送检测结果，接收 STM32F4 实际云台角度并发布 JointState。

取代原 qr_serial_bridge（两者不能同时运行，串口设备只能被一个进程打开）。
串口参数优先级：命令行 > serial_config.json > 默认值。
"""
import os
import sys
import json
import argparse
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from identify_pack.serial.protocol import (
    GIMBAL_YAW_TYPE, OUTGOING_TOPIC_TYPES, extract_frames, make_frame,
)

try:
    import serial
except Exception:
    serial = None


class SerialBridge(Node):
    def __init__(self, serial_port: str, baud: int, timeout: float = 0.1):
        super().__init__('serial_bridge')
        self.declare_parameter('joint_states_topic', '/gimbal/joint_states')
        self.declare_parameter('yaw_joint_name', 'gimbal_yaw_joint')
        self.declare_parameter('yaw_feedback_type', GIMBAL_YAW_TYPE)
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.yaw_joint_name = self.get_parameter('yaw_joint_name').value
        self.yaw_feedback_type = int(self.get_parameter('yaw_feedback_type').value)
        if not self.joint_states_topic or not self.yaw_joint_name:
            raise ValueError('joint_states_topic and yaw_joint_name cannot be empty')
        if self.yaw_feedback_type != GIMBAL_YAW_TYPE:
            raise ValueError(f'yaw_feedback_type must be 0x{GIMBAL_YAW_TYPE:02X}')
        self.ser = None
        self.rx_buffer = bytearray()
        self.pub_joint_state = self.create_publisher(
            JointState, self.joint_states_topic, 10)
        if serial is None:
            self.get_logger().error('pyserial 未安装，无法发送串口')
        else:
            try:
                self.ser = serial.Serial(port=serial_port, baudrate=int(baud),
                                         timeout=float(timeout))
                self.get_logger().info(f'已打开串口 {serial_port} @ {baud}')
                self.rx_timer = self.create_timer(0.01, self._poll_feedback)
            except Exception as e:
                self.get_logger().error(f'打开串口失败: {e}')

        for topic, type_byte in OUTGOING_TOPIC_TYPES.items():
            # 用默认参数闭包捕获当前 topic/type，避免 lambda 晚绑定
            self.create_subscription(
                String, topic,
                lambda msg, t=type_byte, name=topic: self._send(t, name, msg.data),
                10)

    def _poll_feedback(self):
        """读取并解析下位机反馈；角度载荷为 ASCII 度数，例如 ``90.0``。"""
        if not self.ser or not self.ser.is_open:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buffer.extend(self.ser.read(waiting))
            frames, remainder = extract_frames(self.rx_buffer)
            self.rx_buffer = bytearray(remainder)
        except Exception as error:
            self.get_logger().error(f'串口读取失败: {error}', throttle_duration_sec=2.0)
            return
        for type_byte, payload in frames:
            if type_byte != self.yaw_feedback_type:
                self.get_logger().debug(
                    f'忽略反馈帧 type=0x{type_byte:02X} payload={payload!r}')
                continue
            try:
                yaw_deg = float(payload.strip())
                if not math.isfinite(yaw_deg):
                    raise ValueError('角度不是有限数')
                yaw_rad = math.radians(yaw_deg)
                message = JointState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.name = [self.yaw_joint_name]
                message.position = [yaw_rad]
                self.pub_joint_state.publish(message)
                self.get_logger().debug(
                    f'RX GimbalYaw: {yaw_deg:.3f} deg ({yaw_rad:.6f} rad)')
            except (TypeError, ValueError, OverflowError) as error:
                self.get_logger().warning(
                    f'忽略非法云台角度 payload={payload!r}: {error}',
                    throttle_duration_sec=2.0)

    def _send(self, type_byte: int, topic: str, text: str):
        try:
            frame = make_frame(type_byte, text)
        except Exception as e:
            self.get_logger().error(f'打包失败 topic={topic}: {e}')
            return
        if self.ser:
            try:
                self.ser.write(frame)
                self.get_logger().info(
                    f'TX {topic}(0x{type_byte:02X}) len={len(frame)}: {text!r}')
            except Exception as e:
                self.get_logger().error(f'串口写入失败: {e}')
        else:
            self.get_logger().info(f'(无串口) {topic}(0x{type_byte:02X}): {text!r}')


def main(args=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--serial', dest='serial_port', default=None)
    parser.add_argument('--baud', dest='baud', type=int, default=None)
    known, remaining = parser.parse_known_args()

    # 优先读安装目录（share/identify_pack），失败回退源码同目录
    try:
        from ament_index_python.packages import get_package_share_directory
        config_dir = get_package_share_directory('identify_pack')
    except Exception:
        config_dir = os.path.dirname(__file__)
    config_path = os.path.join(config_dir, 'serial_config.json')
    file_cfg = {}
    if os.path.exists(config_path):
        try:
            with open(config_path) as f:
                file_cfg = json.load(f)
        except Exception as e:
            print(f'读取配置失败 {config_path}: {e}', file=sys.stderr)

    serial_port = known.serial_port or file_cfg.get('serial_port', '/dev/ttyUSB0')
    baud = known.baud or file_cfg.get('baud', 115200)
    timeout = file_cfg.get('timeout', 0.1)

    rclpy.init(args=remaining)
    node = SerialBridge(serial_port, baud, timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
