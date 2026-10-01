#!/usr/bin/env python3
"""ROS2 节点：六色轮廓 blob 色块定位 + 转盘转动/停稳判定（正式运行节点）。

订阅相机图像话题，在 ROI 内做色块检测，发布：
  - /Color       色块位置，每段 ``color_id:x,y``（1~6 六色，多段用 / 连接）
  - /ColorMotion 转盘运动状态 ``state:speed``（如 ``READY:12.3``）

不含数字圆环识别（那是 circle_detect_node 的职责）。
"""
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from identify_pack.vision_identify.color_config import (
    COLOR_DISPLAY, COLOR_NAMES, apply_dominant_color, build_default_color_config,
    find_color_blobs, load_color_config, load_roi_file, make_all_color_masks,
)
from identify_pack.vision_identify.motion import TurntableMotionTracker


class ColorDetectNode(Node):
    """色块识别节点：ROI 内主色锁定 + 六色 blob 定位 + 转盘运动判定。"""

    def __init__(self):
        super().__init__('color_detect_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('color_config', '')     # 颜色配置 JSON 路径（空=自动定位）
        self.declare_parameter('roi_file', 'ROI_Range.txt')  # ROI 坐标文件（每行 x,y,w,h）
        self.declare_parameter('min_blob_area', 300)   # 色块最小面积(px^2)
        self.declare_parameter('min_blob_dimension', 12)  # 色块最小边长(px)
        self.declare_parameter('lock_dominant_color', True)  # ROI 内主色锁定
        self.declare_parameter('motion_detect', True)  # 转盘转动/停稳判定
        self.declare_parameter('moving_speed_px_s', 30.0)
        self.declare_parameter('required_moving_frames', 2)
        self.declare_parameter('near_stop_speed_px_s', 25.0)
        self.declare_parameter('required_low_speed_frames', 3)
        self.declare_parameter('show', True)           # 是否弹窗显示

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.color_config = self.get_parameter('color_config').get_parameter_value().string_value
        self.roi_file = self.get_parameter('roi_file').get_parameter_value().string_value
        self.min_blob_area = self.get_parameter('min_blob_area').get_parameter_value().double_value
        self.min_blob_dimension = self.get_parameter('min_blob_dimension').get_parameter_value().double_value
        self.lock_dominant = self.get_parameter('lock_dominant_color').get_parameter_value().bool_value
        self.motion_detect = self.get_parameter('motion_detect').get_parameter_value().bool_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value

        self.color_cfg = self._load_color_config()
        self.color_roi = self._resolve_roi()
        self.motion = TurntableMotionTracker(
            moving_speed=self.get_parameter('moving_speed_px_s').get_parameter_value().double_value,
            required_moving_frames=self.get_parameter('required_moving_frames').get_parameter_value().integer_value,
            near_stop_speed=self.get_parameter('near_stop_speed_px_s').get_parameter_value().double_value,
            required_low_speed_frames=self.get_parameter('required_low_speed_frames').get_parameter_value().integer_value,
        )

        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub_color = self.create_publisher(String, 'Color', 10)
        self.pub_motion = self.create_publisher(String, 'ColorMotion', 10)
        self.get_logger().info(f'色块识别订阅 {image_topic}')

    def _load_color_config(self):
        """加载颜色配置；路径未指定时依次尝试安装目录、源码 scripts 目录。"""
        path = self.color_config
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
            self.get_logger().warn(f'颜色配置加载失败({error})，使用默认阈值')
            return build_default_color_config()

    def _resolve_roi(self):
        """读取 roi_file 里的多个 ROI；失败返回 None（全画面）。"""
        path = self.roi_file.strip()
        if not path:
            return None
        if not os.path.isabs(path):
            if not os.path.exists(path):
                fallback = os.path.join(os.path.dirname(__file__), '..', 'scripts',
                                        os.path.basename(path))
                if os.path.exists(fallback):
                    path = fallback
        rois = load_roi_file(path)
        if rois:
            self.get_logger().info(f'从 {path} 读取 {len(rois)} 个色块 ROI')
            return rois
        self.get_logger().warn(f'ROI 文件 {path} 无有效内容，全画面识别')
        return None

    def image_cb(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'cv_bridge error: {error}')
            return

        color_masks = make_all_color_masks(img, self.color_cfg)
        if self.color_roi and self.lock_dominant:
            # ROI 内主色锁定：面积最大的颜色锁定物块，盖住被误切的小面积颜色。
            color_masks = apply_dominant_color(
                color_masks, self.color_roi, min_area=self.min_blob_area)

        color_parts = []
        blob_pts = []
        for (color_id, cx, cy, x, y, w, h, area) in find_color_blobs(
                color_masks, min_area=self.min_blob_area,
                min_dimension=self.min_blob_dimension, roi=self.color_roi):
            name = COLOR_NAMES[color_id]
            self.get_logger().info(
                f'色块 {name}(质心 {cx:.0f},{cy:.0f} 框 {x},{y},{w},{h} 面积 {area:.0f})',
                throttle_duration_sec=1.0)
            if self.show:
                c = COLOR_DISPLAY[name]
                cv2.rectangle(img, (x, y), (x + w, y + h), c, 2)
                cv2.putText(img, f'{color_id}:{name}', (x, max(14, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)
            color_parts.append(f'{color_id}:{int(cx)},{int(cy)}')
            blob_pts.append((color_id, cx, cy))

        if color_parts:
            out_color = String()
            out_color.data = '/'.join(color_parts)
            self.pub_color.publish(out_color)

        if self.motion_detect:
            motion = self.motion.update(blob_pts, time.monotonic())
            out_motion = String()
            out_motion.data = f'{motion["state"]}:{motion["speed"]:.1f}'
            self.pub_motion.publish(out_motion)
            self.get_logger().info(
                f'Motion={motion["state"]} speed={motion["speed"]:.1f}px/s '
                f'move={motion["moving_frames"]}/{motion["required_moving_frames"]} '
                f'low={motion["low_speed_frames"]}/{motion["required_low_speed_frames"]}',
                throttle_duration_sec=1.0)
            if self.show:
                cv2.putText(img, out_motion.data, (10, img.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        if self.show:
            cv2.imshow('color_detect', img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
