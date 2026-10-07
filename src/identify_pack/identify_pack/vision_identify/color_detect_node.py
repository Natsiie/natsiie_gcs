#!/usr/bin/env python3
"""ROS2 节点：六色轮廓 blob 色块定位 + 转盘转动/停稳判定（正式运行节点）。

订阅相机图像话题，在 ROI 内做色块检测，发布：
  - /Color       色块位置，每段 ``color_id:x,y``（1~6 六色，多段用 / 连接）
  - /ColorMotion 转盘运动状态 ``state:speed``（如 ``READY:12.3``）
  - /color/centers_pixel 带原图时间戳的像素 PointCloud（x=u, y=v），附颜色/面积/分辨率

不含数字圆环识别（那是 circle_detect_node 的职责）。
"""
import os
import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from identify_pack.scripts.color_config import (
    COLOR_DISPLAY, COLOR_NAMES, apply_dominant_color, build_default_color_config,
    find_color_blobs, load_color_config, load_roi_file, make_all_color_masks,
)
from identify_pack.scripts.turntable_motion import OpticalFlowMotion


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
        self.declare_parameter('motion_roi_file', '')  # 运动区域独立于抓取 ROI，空为整幅图。
        self.declare_parameter('require_single_roi', False)
        self.declare_parameter('moving_speed_px_s', 30.0)
        self.declare_parameter('required_moving_frames', 2)
        self.declare_parameter('near_stop_speed_px_s', 25.0)
        self.declare_parameter('required_low_speed_frames', 3)
        self.declare_parameter('show', True)           # 是否弹窗显示
        self.declare_parameter('centers_topic', '/color/centers_pixel')
        self.declare_parameter('camera_frame', 'camera_optical_frame')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.color_config = self.get_parameter('color_config').get_parameter_value().string_value
        self.roi_file = self.get_parameter('roi_file').get_parameter_value().string_value
        self.min_blob_area = float(self.get_parameter('min_blob_area').value)
        self.min_blob_dimension = float(self.get_parameter('min_blob_dimension').value)
        self.lock_dominant = self.get_parameter('lock_dominant_color').get_parameter_value().bool_value
        self.motion_detect = self.get_parameter('motion_detect').get_parameter_value().bool_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value
        self.require_single_roi = self.get_parameter('require_single_roi').value

        self.color_cfg = self._load_color_config()
        self.color_roi = self._resolve_roi()
        motion_path = self.get_parameter('motion_roi_file').value
        motion_rois = load_roi_file(motion_path) if motion_path else None
        if motion_path and not motion_rois:
            raise ValueError(f'运动 ROI 文件无有效区域: {motion_path}')
        self.motion = OpticalFlowMotion(
            rois=motion_rois,
            moving_speed=self.get_parameter('moving_speed_px_s').get_parameter_value().double_value,
            required_moving_frames=self.get_parameter('required_moving_frames').get_parameter_value().integer_value,
            near_stop_speed=self.get_parameter('near_stop_speed_px_s').get_parameter_value().double_value,
            required_low_speed_frames=self.get_parameter('required_low_speed_frames').get_parameter_value().integer_value,
        )

        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)
        self.pub_color = self.create_publisher(String, 'Color', 10)
        self.centers_topic = self.get_parameter('centers_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.motion_text = ''
        self.pub_centers = self.create_publisher(PointCloud, self.centers_topic, 10)
        self.pub_motion = self.create_publisher(String, 'ColorMotion', 10)
        self.get_logger().info(f'色块识别订阅 {image_topic}')
        if self.require_single_roi and (not self.color_roi or len(self.color_roi) != 1):
            self.get_logger().error('任务抓取模式需要且仅允许一个 ROI；请重新圈定，当前禁止输出目标')

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
                else:
                    try:
                        from ament_index_python.packages import get_package_share_directory
                        path = os.path.join(get_package_share_directory('identify_pack'),
                                            os.path.basename(path))
                    except Exception:
                        pass
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

        # 运动检测先执行，不依赖抓取 ROI 中是否出现色块。
        ready = not self.motion_detect
        if self.motion_detect:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            motion = self.motion.update(img, stamp)
            ready = motion['ready']
            out_motion = String(data=f'{motion["state"]}:{motion["speed"]:.1f}')
            self.pub_motion.publish(out_motion)
            self.get_logger().info(
                f'Motion={motion["state"]} speed={motion["speed"]:.1f}px/s',
                throttle_duration_sec=1.0)
            self.motion_text = out_motion.data
        roi_valid = not self.require_single_roi or (self.color_roi and len(self.color_roi) == 1)
        if not ready or not roi_valid:
            empty = PointCloud()
            empty.header = copy.deepcopy(msg.header)
            if not empty.header.frame_id:
                empty.header.frame_id = self.camera_frame
            empty.channels = [ChannelFloat32(name='motion_ready', values=[0.0])]
            self.pub_centers.publish(empty)
            if self.show:
                self.show_frame(img)
            return

        color_masks = make_all_color_masks(img, self.color_cfg)
        if self.color_roi and self.lock_dominant:
            # ROI 内主色锁定：面积最大的颜色锁定物块，盖住被误切的小面积颜色。
            color_masks = apply_dominant_color(
                color_masks, self.color_roi, min_area=self.min_blob_area)

        color_parts = []
        centers = PointCloud()
        centers.header = copy.deepcopy(msg.header)
        if not centers.header.frame_id:
            centers.header.frame_id = self.camera_frame
        channels = {name: [] for name in ('color_id', 'area_px2', 'image_width', 'image_height')}
        blobs = find_color_blobs(
                color_masks, min_area=self.min_blob_area,
                min_dimension=self.min_blob_dimension, roi=self.color_roi)
        if self.require_single_roi and blobs:
            blobs = [max(blobs, key=lambda blob: blob[7])]
        for (color_id, cx, cy, x, y, w, h, area) in blobs:
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
            centers.points.append(Point32(x=float(cx), y=float(cy), z=0.0))
            for key, value in zip(channels, (color_id, area, img.shape[1], img.shape[0])):
                channels[key].append(float(value))

        # Pixel coordinates, not a metric cloud; preserve the source image timestamp.
        centers.channels = [ChannelFloat32(name=name, values=values)
                            for name, values in channels.items()]
        centers.channels.append(ChannelFloat32(name='motion_ready', values=[1.0]))
        self.pub_centers.publish(centers)

        if color_parts:
            out_color = String()
            out_color.data = '/'.join(color_parts)
            self.pub_color.publish(out_color)

        if self.show:
            self.show_frame(img)

    def show_frame(self, img):
        if self.motion_detect:
            cv2.putText(img, self.motion_text, (10, img.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        for index, (x, y, w, h) in enumerate(self.color_roi or [], start=1):
            left, top = max(0, x), max(0, y)
            right = min(img.shape[1] - 1, x + w - 1)
            bottom = min(img.shape[0] - 1, y + h - 1)
            if right < left or bottom < top:
                continue
            cv2.rectangle(img, (left, top), (right, bottom), (255, 255, 0), 2)
            cv2.putText(img, f'ROI {index}', (left, max(16, top - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)
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
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
