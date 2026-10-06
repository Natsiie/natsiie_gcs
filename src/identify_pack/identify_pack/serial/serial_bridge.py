#!/usr/bin/env python3
"""统一串口桥节点：订阅全部检测结果话题，按协议打包后发给下位机 STM32F4。

取代原 qr_serial_bridge（两者不能同时运行，串口设备只能被一个进程打开）。
串口参数优先级：命令行 > serial_config.json > 默认值。
"""
import os
import sys
import json
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from identify_pack.serial.protocol import make_frame, TOPIC_TYPES

try:
    import serial
except Exception:
    serial = None


class SerialBridge(Node):
    def __init__(self, serial_port: str, baud: int, timeout: float = 0.1):
        super().__init__('serial_bridge')
        self.ser = None
        if serial is None:
            self.get_logger().error('pyserial 未安装，无法发送串口')
        else:
            try:
                self.ser = serial.Serial(port=serial_port, baudrate=int(baud),
                                         timeout=float(timeout))
                self.get_logger().info(f'已打开串口 {serial_port} @ {baud}')
            except Exception as e:
                self.get_logger().error(f'打开串口失败: {e}')

        for topic, type_byte in TOPIC_TYPES.items():
            # 用默认参数闭包捕获当前 topic/type，避免 lambda 晚绑定
            self.create_subscription(
                String, topic,
                lambda msg, t=type_byte, name=topic: self._send(t, name, msg.data),
                10)

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
