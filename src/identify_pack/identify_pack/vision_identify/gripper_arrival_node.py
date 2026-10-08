#!/usr/bin/env python3
"""ROS2 节点：夹爪到位判定（粉色舵机外壳进入 ROI 连续满阈值秒 -> 到位）。

订阅副相机图像，在 ``roi_file``（与颜色识别共用同一 ROI）内检测粉色（第 7 号颜色），
连续识别满 ``min_confirm_sec`` 秒后发布到位信号（``std_msgs/Bool`` 到
``publish_topic``，默认 /arm/arrived）。只发信号，不控制下位机。
"""
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from identify_pack.scripts.arrival import GripperArrivalDetector
from identify_pack.scripts.color_config import (
    COLOR_DISPLAY, find_color_blobs, load_color_config, load_roi_file,
    make_color_mask,
)

PINK_COLOR_ID = 7


class GripperArrivalNode(Node):
    """夹爪到位检测节点：ROI 内粉色连续可见满阈值 -> 发布到位信号。"""

    def __init__(self):
        super().__init__('gripper_arrival')
        self.declare_parameter('image_topic', '/camera/secondary/image_raw')
        self.declare_parameter('color_config', '')
        self.declare_parameter('roi_file', 'ROI_Range.txt')
        self.declare_parameter('min_blob_area', 300)
        self.declare_parameter('min_blob_dimension', 12)
        self.declare_parameter('min_confirm_sec', 5.0)
        self.declare_parameter('publish_topic', '/arm/arrived')
        self.declare_parameter('show', False)

        image_topic = self.get_parameter('image_topic').value
        self.color_cfg = self._load_color_config()
        self.rois = self._resolve_roi()
        self.min_blob_area = float(self.get_parameter('min_blob_area').value)
        self.min_blob_dimension = float(self.get_parameter('min_blob_dimension').value)
        self.publish_topic = self.get_parameter('publish_topic').value
        self.show = self.get_parameter('show').value
        self.detector = GripperArrivalDetector(
            float(self.get_parameter('min_confirm_sec').value))

        if not self.rois:
            self.get_logger().warn('未读到有效 ROI，将在全画面检测粉色，可能误判到位')

        self.br = CvBridge()
        self.sub = self.create_subscription(
            Image, image_topic, self.image_cb, qos_profile_sensor_data)
        self.pub_arrived = self.create_publisher(
            Bool, self.publish_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info(
            f'夹爪到位检测订阅 {image_topic}，阈值 {self.detector.min_confirm_ns / 1e9:.1f}s，'
            f'ROI={self.rois}')

    def _load_color_config(self):
        """加载颜色配置；路径未指定时依次尝试安装目录、源码 scripts 目录。"""
        path = self.get_parameter('color_config').value
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(
                    get_package_share_directory('identify_pack'), 'color_config.json')
            except Exception:
                path = os.path.join(os.path.dirname(__file__), '..', 'scripts',
                                    'color_config.json')
        try:
            cfg = load_color_config(path)
            self.get_logger().info(f'已加载颜色配置 {path}')
            return cfg
        except Exception as error:
            self.get_logger().error(f'颜色配置加载失败({error})，无法检测粉色')
            raise

    def _resolve_roi(self):
        """读取 roi_file 里的 ROI（与颜色识别共用同一文件）；失败返回 None（全画面）。"""
        path = self.get_parameter('roi_file').value.strip()
        if not path:
            return None
        if not os.path.isabs(path) and not os.path.exists(path):
            fallback = os.path.join(os.path.dirname(__file__), '..', 'scripts',
                                    os.path.basename(path))
            if os.path.exists(fallback):
                path = fallback
            else:
                try:
                    from ament_index_python.packages import get_package_share_directory
                    path = os.path.join(get_package_share_directory('identify_pack'),
                                        os.path.basename(path))
                except Exception:
                    pass
        rois = load_roi_file(path)
        if rois:
            self.get_logger().info(f'从 {path} 读取 {len(rois)} 个到位检测 ROI')
            return rois
        self.get_logger().warn(f'ROI 文件 {path} 无有效内容，全画面检测粉色')
        return None

    def image_cb(self, msg):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'cv_bridge error: {error}')
            return

        pink_mask = make_color_mask(img, PINK_COLOR_ID, self.color_cfg)
        blobs = find_color_blobs(
            {PINK_COLOR_ID: pink_mask}, min_area=self.min_blob_area,
            min_dimension=self.min_blob_dimension, roi=self.rois)
        present = bool(blobs)

        if self.detector.update(present, time.monotonic_ns()):
            self.pub_arrived.publish(Bool(data=True))
            self.get_logger().info('夹爪到位：粉色标记连续识别满阈值，发布到位信号')

        if self.show:
            self.show_frame(img, pink_mask, present)

    def show_frame(self, img, pink_mask, present):
        overlay = img.copy()
        overlay[pink_mask > 0] = COLOR_DISPLAY['pink']
        out = cv2.addWeighted(overlay, 0.5, img, 0.5, 0)
        for (x, y, w, h) in (self.rois or []):
            cv2.rectangle(out, (x, y), (x + w, y + h), (255, 255, 0), 2)
        state = 'ARRIVED' if self.detector.arrived else ('present' if present else 'absent')
        elapsed = 0.0
        if present and self.detector.present_since_ns is not None:
            elapsed = (time.monotonic_ns() - self.detector.present_since_ns) / 1e9
        text = f'{state} {elapsed:.1f}s / {self.detector.min_confirm_ns / 1e9:.1f}s'
        cv2.putText(out, text, (10, out.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow('gripper_arrival', out)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = GripperArrivalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
