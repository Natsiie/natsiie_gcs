#!/usr/bin/env python3
"""
ROS2 节点：订阅 `QR_Decode` 话题，将接收到的文本按原项目格式化并通过串口发送给单片机。
优先使用命令行参数，其次读取 `serial_config.json`，最后回退到默认值。
"""
import os
import sys
import json
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
try:
    import serial
except Exception:
    serial = None


class QRSerialBridge(Node):
    def __init__(self, serial_port: str, baud: int, timeout: float = 0.1):
        super().__init__('qr_serial_bridge')
        self.serial_port = serial_port
        self.baud = int(baud)
        self.timeout = float(timeout)

        self.sub = self.create_subscription(String, 'QR_Decode', self.cb, 10)
        self.ser = None
        if serial is not None:
            try:
                self.ser = serial.Serial(port=self.serial_port, baudrate=self.baud, timeout=self.timeout)
                self.get_logger().info(f'Opened serial {self.serial_port} @ {self.baud}')
            except Exception as e:
                self.get_logger().warning(f'Cannot open serial: {e}')
                self.ser = None
        else:
            self.get_logger().warning('pyserial not available; will not send to serial')

    def format_packet(self, data_str: str) -> bytes:
        arr = []
        for ch in data_str:
            try:
                arr.append(ord(ch))
            except Exception:
                arr.append(0)
        while len(arr) < 9:
            arr.append(0)
        return bytes([0xFF] + arr[:9] + [0xFE])

    def cb(self, msg: String):
        data = msg.data.strip()
        # 保持与原项目一致的格式：7 字符右填充 '0'，再补一个 '0'
        formatted = data.ljust(7, '0')[:7] + '0'
        packet = self.format_packet(formatted)
        if self.ser:
            try:
                self.ser.write(packet)
                self.get_logger().info(f'Sent to serial: {packet}')
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')
        else:
            self.get_logger().info(f'(no serial) Would send: {packet}')


def main(args=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--serial', dest='serial_port', default=None)
    parser.add_argument('--baud', dest='baud', type=int, default=None)
    known, remaining = parser.parse_known_args()

    # 优先从安装目录（share/identify_pack）读取，找不到时回退到源码同目录
    try:
        from ament_index_python.packages import get_package_share_directory
        config_dir = get_package_share_directory('identify_pack')
    except Exception:
        config_dir = os.path.dirname(__file__)
    config_path = os.path.join(config_dir, 'serial_config.json')
    file_cfg = {}
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                file_cfg = json.load(f)
        except Exception as e:
            print(f'Failed to read config {config_path}: {e}', file=sys.stderr)

    serial_port = known.serial_port or file_cfg.get('serial_port', '/dev/ttyUSB0')
    baud = known.baud or file_cfg.get('baud', 115200)
    timeout = file_cfg.get('timeout', 0.1)

    rclpy.init(args=remaining)
    node = QRSerialBridge(serial_port=serial_port, baud=baud, timeout=timeout)
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
