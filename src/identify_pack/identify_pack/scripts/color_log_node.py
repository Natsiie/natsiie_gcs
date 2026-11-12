#!/usr/bin/env python3
"""颜色识别调试日志节点（仅调试时启动）。

订阅 color_detect_node 发布的结果，把每条消息写成一行 JSON（JSONL），
方便事后对齐时间、回放异常。正常比赛不要启用。

话题：
  /Color                std_msgs/String   color_id:x,y/color_id:x,y
  /color/centers_pixel  sensor_msgs/PointCloud
  /ColorMotion          std_msgs/String   state:count
  /arm/arrived          std_msgs/Int32（停稳色编号）

参数：
  log_dir          日志目录（默认 log/color_debug）
  log_prefix       文件名前缀（默认 color_debug）
  min_interval_sec 每类消息最小记录间隔，0=逐帧记录
  color_topic / centers_topic / motion_topic / arrived_topic  可覆盖话题名
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int32, String


class ColorLogNode(Node):
    def __init__(self) -> None:
        super().__init__('color_log_node')

        self.declare_parameter('log_dir', 'log/color_debug')
        self.declare_parameter('log_prefix', 'color_debug')
        self.declare_parameter('min_interval_sec', 0.0)
        self.declare_parameter('color_topic', '/Color')
        self.declare_parameter('centers_topic', '/color/centers_pixel')
        self.declare_parameter('motion_topic', '/ColorMotion')
        self.declare_parameter('arrived_topic', '/arm/arrived')

        log_dir = os.path.expanduser(
            str(self.get_parameter('log_dir').value))
        os.makedirs(log_dir, exist_ok=True)
        prefix = str(self.get_parameter('log_prefix').value)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(log_dir, f'{prefix}_{stamp}.jsonl')
        self.stream = open(self.path, 'w', encoding='utf-8')
        self.min_interval = max(
            0.0, float(self.get_parameter('min_interval_sec').value))
        self._last_write = {}

        self._write({
            'event': 'start',
            'wall_time': time.time(),
            'topics': {
                'color': self.get_parameter('color_topic').value,
                'centers': self.get_parameter('centers_topic').value,
                'motion': self.get_parameter('motion_topic').value,
                'arrived': self.get_parameter('arrived_topic').value,
            },
            'min_interval_sec': self.min_interval,
        })
        self.get_logger().warning(f'颜色调试日志已启用 -> {self.path}')

        self.create_subscription(
            String, str(self.get_parameter('color_topic').value),
            self._on_color, 10)
        self.create_subscription(
            PointCloud, str(self.get_parameter('centers_topic').value),
            self._on_centers, 10)
        self.create_subscription(
            String, str(self.get_parameter('motion_topic').value),
            self._on_motion, 10)
        self.create_subscription(
            Int32, str(self.get_parameter('arrived_topic').value),
            self._on_arrived, 10)

    def _write(self, record: dict) -> None:
        self.stream.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.stream.flush()

    def _throttled(self, event: str) -> bool:
        if self.min_interval <= 0.0:
            return False
        now = time.monotonic()
        last = self._last_write.get(event, -float('inf'))
        if now - last < self.min_interval:
            return True
        self._last_write[event] = now
        return False

    def _base(self, event: str) -> dict:
        return {
            'event': event,
            'wall_time': time.time(),
            'ros_time_ns': self.get_clock().now().nanoseconds,
        }

    def _on_color(self, msg: String) -> None:
        if self._throttled('color'):
            return
        record = self._base('color')
        record['data'] = msg.data
        self._write(record)

    def _on_centers(self, msg: PointCloud) -> None:
        if self._throttled('centers'):
            return
        record = self._base('centers')
        record['stamp'] = {
            'sec': msg.header.stamp.sec,
            'nanosec': msg.header.stamp.nanosec,
        }
        record['frame_id'] = msg.header.frame_id
        record['points'] = [
            [round(float(p.x), 3), round(float(p.y), 3)] for p in msg.points]
        record['channels'] = {
            channel.name: [round(float(v), 3) for v in channel.values]
            for channel in msg.channels
        }
        self._write(record)

    def _on_motion(self, msg: String) -> None:
        if self._throttled('motion'):
            return
        record = self._base('motion')
        record['data'] = msg.data
        self._write(record)

    def _on_arrived(self, msg: Int32) -> None:
        if self._throttled('arrived'):
            return
        record = self._base('arrived')
        record['data'] = int(msg.data)
        self._write(record)

    def destroy_node(self) -> None:
        try:
            self._write({
                'event': 'stop',
                'wall_time': time.time(),
                'ros_time_ns': self.get_clock().now().nanoseconds,
            })
            self.stream.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorLogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
