#!/usr/bin/env python3
"""ROS2 节点：六色轮廓 blob 色块定位 + 物块停稳帧计数（正式运行节点）。

订阅相机图像话题，在 ROI 内做色块检测。停稳判定直接统计「物块在 ROI 内
连续保持位置不变」的帧数：达到阈值后发一次抓取 OK；色块/坐标输出不再
门控，物块转走、被抓走、消失或位置跳变时计数清零，不影响下一个物块。

发布：
  - /Color       色块位置，每段 ``color_id:x,y``（1~6 六色，多段用 / 连接）
  - /ColorMotion 停稳状态 ``state:count``（如 ``READY:15``）
  - /arm/arrived 停稳后发一次抓取 OK（Int32，值为停稳物块的颜色编号 1~6）
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
from std_msgs.msg import Bool, Int32, String
from cv_bridge import CvBridge
import cv2

from identify_pack.scripts.color_config import (
    COLOR_DISPLAY, COLOR_NAMES, PICKUP_COLOR_IDS, apply_dominant_color,
    build_default_color_config, find_color_blobs, load_color_config,
    load_roi_file, make_all_color_masks,
)
from identify_pack.scripts.roi_stationary import RoiStationaryCounter


def _extract_blobs(img, color_cfg, color_roi, lock_dominant, min_blob_area,
                   min_blob_dimension, require_single_roi):
    """颜色掩膜 -> 色块列表；抓取色 1~6，ROI 内主色锁定。"""
    color_masks = make_all_color_masks(img, color_cfg)
    # 隔离标记色（粉色舵机外壳）：只处理抓取色 1~6，不参与主色锁定与
    # 抓取目标检测，否则粉色进入 ROI 会被判为主色、盖掉物块。抓取 OK
    # 现在由本节点的停稳计数触发，粉色不参与判定。
    color_masks = {cid: mask for cid, mask in color_masks.items()
                   if cid in PICKUP_COLOR_IDS}
    if color_roi and lock_dominant:
        # ROI 内主色锁定：面积最大的颜色锁定物块，盖住被误切的小面积颜色。
        color_masks = apply_dominant_color(
            color_masks, color_roi, min_area=min_blob_area)
    blobs = find_color_blobs(
        color_masks, min_area=min_blob_area,
        min_dimension=min_blob_dimension, roi=color_roi)
    if require_single_roi and blobs:
        blobs = [max(blobs, key=lambda blob: blob[7])]
    return blobs


def _empty_centers(msg, camera_frame, motion_ready):
    """构造无目标的点云，同时携带当前 ready 状态。"""
    empty = PointCloud()
    empty.header = copy.deepcopy(msg.header)
    if not empty.header.frame_id:
        empty.header.frame_id = camera_frame
    empty.channels = [ChannelFloat32(name='motion_ready', values=[motion_ready])]
    return empty


class ColorDetectNode(Node):
    """色块识别节点：ROI 内主色锁定 + 六色 blob 定位 + 物块停稳帧计数。"""

    def __init__(self):
        super().__init__('color_detect_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('color_config', '')     # 颜色配置 JSON 路径（空=自动定位）
        self.declare_parameter('roi_file', 'ROI_Range.txt')  # ROI 坐标文件（每行 x,y,w,h）
        self.declare_parameter('min_blob_area', 300)   # 色块最小面积(px^2)
        self.declare_parameter('min_blob_dimension', 12)  # 色块最小边长(px)
        self.declare_parameter('lock_dominant_color', True)  # ROI 内主色锁定
        self.declare_parameter('require_single_roi', False)
        # 停稳判定：hold_frames>0 时直接按帧数（便于实测调参），否则按
        # hold_sec × 实测 fps 换算。默认 1.0s，25fps 下约 25 帧。
        self.declare_parameter('stationary_hold_sec', 1.5)
        self.declare_parameter('stationary_hold_frames', 0)
        self.declare_parameter('stationary_max_pixel_move', 10.0)  # 允许抖动(px)
        # 连续丢帧容忍；0 = ROI 内目标一消失就立即复位（下一件可重新触发抓取 OK）。
        self.declare_parameter('stationary_miss_frames', 2)
        self.declare_parameter('stationary_fps', 0.0)         # 0=按时间戳自动估计
        self.declare_parameter('stationary_max_frame_interval_sec', 0.5)
        # 停稳后给下位机的抓取 OK：Int32 携带停稳色编号，串口桥校验与期望色一致后发 GrabCmd(0x09)。
        # 每个物块只触发一次，物块离开 ROI 后重新武装（与旧的粉色到位逻辑一致）。
        self.declare_parameter('publish_arrived', True)
        self.declare_parameter('arrived_topic', '/arm/arrived')
        self.declare_parameter('show', True)           # 是否弹窗显示
        self.declare_parameter('centers_topic', '/color/centers_pixel')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        # 常驻休眠：启动时不订阅图像；收到 enable_topic=true 才创建订阅。
        self.declare_parameter('enabled', False)
        self.declare_parameter('enable_topic', '/color/enable')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.color_config = self.get_parameter('color_config').get_parameter_value().string_value
        self.roi_file = self.get_parameter('roi_file').get_parameter_value().string_value
        self.min_blob_area = float(self.get_parameter('min_blob_area').value)
        self.min_blob_dimension = float(self.get_parameter('min_blob_dimension').value)
        self.lock_dominant = self.get_parameter('lock_dominant_color').get_parameter_value().bool_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value
        self.require_single_roi = self.get_parameter('require_single_roi').value
        self.image_topic = image_topic
        self.enabled = bool(self.get_parameter('enabled').value)
        enable_topic = self.get_parameter('enable_topic').get_parameter_value().string_value

        self.color_cfg = self._load_color_config()
        self.color_roi = self._resolve_roi()
        self.stationary_hold_sec = self.get_parameter(
            'stationary_hold_sec').get_parameter_value().double_value
        self.stationary = RoiStationaryCounter(
            hold_sec=self.get_parameter('stationary_hold_sec').get_parameter_value().double_value,
            hold_frames=self.get_parameter('stationary_hold_frames').get_parameter_value().integer_value,
            max_pixel_move=self.get_parameter('stationary_max_pixel_move').get_parameter_value().double_value,
            miss_frames=self.get_parameter('stationary_miss_frames').get_parameter_value().integer_value,
            fixed_fps=self.get_parameter('stationary_fps').get_parameter_value().double_value,
            max_frame_interval=self.get_parameter(
                'stationary_max_frame_interval_sec').get_parameter_value().double_value,
        )
        self._last_required_frames = None

        self.br = CvBridge()
        self.sub = None
        self.pub_color = self.create_publisher(String, 'Color', 10)
        self.centers_topic = self.get_parameter('centers_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.motion_text = ''
        self.publish_arrived = bool(self.get_parameter('publish_arrived').value)
        self.arrived_topic = self.get_parameter('arrived_topic').value
        self._arrived_armed = True
        self.pub_centers = self.create_publisher(PointCloud, self.centers_topic, 10)
        self.pub_motion = self.create_publisher(String, 'ColorMotion', 10)
        self.pub_arrived = self.create_publisher(Int32, self.arrived_topic, 10)
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.on_enable, 10)
        if self.enabled:
            self._enable_image_subscription()
        else:
            self.get_logger().info(
                f'颜色识别已启动（暂停，等待 {enable_topic} 使能）；'
                f'停稳 OK -> {self.arrived_topic}')

    def _enable_image_subscription(self):
        if self.sub is None:
            self.sub = self.create_subscription(
                Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
            self.get_logger().info(f'颜色识别已启用，订阅 {self.image_topic}')

    def on_enable(self, msg):
        enabled = bool(msg.data)
        if enabled and self.sub is None:
            self._enable_image_subscription()
        elif not enabled and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.stationary.reset()
            self._arrived_armed = True
            self.get_logger().info('颜色识别已暂停')
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

        roi_valid = not self.require_single_roi or (
            self.color_roi and len(self.color_roi) == 1)

        # 停稳计数只用于发抓取 OK，不再门控色块/坐标输出。
        # 物块转走/被抓走时检测为空或位置跳变，计数会自然清零。
        blobs = []
        if roi_valid:
            blobs = _extract_blobs(
                img, self.color_cfg, self.color_roi, self.lock_dominant,
                self.min_blob_area, self.min_blob_dimension,
                self.require_single_roi)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            primary = max(blobs, key=lambda blob: blob[7]) if blobs else None
            stationary = self.stationary.update(
                None if primary is None else (primary[0], primary[1], primary[2]),
                stamp)
            required = stationary['required_frames']
            if required != getattr(self, '_last_required_frames', None):
                self._last_required_frames = required
                self.get_logger().info(
                    f'停稳阈值 {required} 帧（hold='
                    f'{getattr(self, "stationary_hold_sec", 0.0):g}s，'
                    f'fps≈{stationary["fps"] or 25.0:.1f}）')
            self.motion_text = f'{stationary["state"]}:{stationary["count"]}'
            self.pub_motion.publish(String(data=self.motion_text))
            self.get_logger().info(
                f'Stationary={stationary["state"]} '
                f'count={stationary["count"]}/{required} '
                f'fps={stationary["fps"]:.1f} misses={stationary["missing_frames"]}',
                throttle_duration_sec=1.0)
            # 停稳 -> 给下位机抓取 OK（每个物块只发一次）。
            if getattr(self, 'publish_arrived', False):
                if stationary['ready'] and self._arrived_armed:
                    color_id = int(primary[0])
                    self.pub_arrived.publish(Int32(data=color_id))
                    self._arrived_armed = False
                    self.get_logger().warning(
                        f'物块停稳 {stationary["count"]}/{required} 帧，'
                        f'颜色 {color_id}({COLOR_NAMES.get(color_id, "?")})，'
                        f'发送抓取 OK -> {self.arrived_topic}')
                elif stationary.get('reset') or stationary['state'] == 'WAITING':
                    # 首次出现、换目标、转走、被抓走或掉帧后重新武装，
                    # 下一次停稳可再次触发 OK。
                    self._arrived_armed = True
        else:
            self.stationary.reset()

        if not roi_valid or not blobs:
            # 没有目标时仍把 motion_ready 置 1，串口桥不再用它拦目标。
            self.pub_centers.publish(_empty_centers(msg, self.camera_frame, 1.0))
            if self.show:
                self.show_frame(img)
            return

        color_parts = []
        centers = PointCloud()
        centers.header = copy.deepcopy(msg.header)
        if not centers.header.frame_id:
            centers.header.frame_id = self.camera_frame
        channels = {name: [] for name in ('color_id', 'area_px2', 'image_width', 'image_height')}
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
        if self.motion_text:
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
