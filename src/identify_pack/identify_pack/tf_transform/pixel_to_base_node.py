#!/usr/bin/env python3
"""Project stamped color centroids to base; optionally inspect raw images by clicking."""

import copy
import json
import math
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Point32
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from identify_pack.config_paths import data_path
from identify_pack.scripts.camera_geometry import pixel_to_plane
from identify_pack.scripts.color_config import COLOR_IDS, PICKUP_COLOR_IDS
from identify_pack.tf_transform.transforms import normalize_quaternion


def transform_matrix(transform):
    """Convert a target <- camera TransformStamped into a metric matrix."""
    q = transform.transform.rotation
    x, y, z, w = normalize_quaternion([q.x, q.y, q.z, q.w])
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    t = transform.transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Non-finite TF translation')
    return matrix


@dataclass
class ImageSample:
    image: np.ndarray
    stamp: object
    received_at: float
    base_from_camera: object = None


class PixelToBaseNode(Node):
    WINDOW = 'pixel_to_base'

    def __init__(self):
        super().__init__('pixel_to_base')
        defaults = {
            'calibration_file': 'camera_extrinsics.json',
            'image_topic': '/camera/image_raw',
            'base_frame': 'gimbal_base',
            'camera_frame': 'camera_optical_frame',
            'output_topic': '/pixel_to_base/point',
            'centers_topic': '/color/centers_pixel',
            'targets_topic': '/color/targets_base',
            'auto_project': True,
            'min_auto_interval_sec': 0.2,
            'target_max_age_sec': 1.0,
            'plane_mode': 'calibration',
            'object_plane_configured': False,
            'object_plane_z_mm': 0.0,
            'ring_plane_configured': False,
            'ring_plane_z_mm': 0.0,
            'ring_centers_topic': '/ring_station/centers_pixel',
            'ring_targets_topic': '/ring_station/targets_base',
            # 第一层放置（普通工位）用三圆环检测同时定位；累计成功放置
            # ring_source_switch_count 次后，切换到叠放区检测（中间圆 +
            # 上下边界线）做微调。0 = 不切换，始终用第一个话题。
            'ring_centers_topic_after': '/stacked_ring/centers_pixel',
            'ring_source_switch_count': 3,
            # 0x0C ring 投影开关：false 时不订阅三环中心、不发布
            # /ring_station/targets_base；颜色目标 0x08 不受影响。
            'ring_project_enabled': False,
            # 下位机放置结束回传（0x0B -> 串口桥 -> /arm/place_result）。
            'place_result_topic': '/arm/place_result',
            # 叠放区检测节点按需使能：切过去时发 true，新任务重置时发 false。
            'stacked_enable_topic': '/stacked_ring/enable',
            # 三环几何约束：以中间环为锚点，按已知中心距重建左右两环，
            # 消除三环各自投影误差。整体平移/旋转校正由下位机负责。
            'ring_geometry_enforce': True,
            'ring_center_spacing_mm': 150.0,
            # 备选校正：视觉系与塔吊系存在 yaw 偏置时，发 0x0C 前绕转轴原点
            # 旋转该角度（°）。正负号按实测落点方向试一次即可。0 = 不校正。
            'ring_frame_yaw_offset_deg': 0.0,
            # 单环额外转角（°）：只旋转对应环的目标点，不影响另外两个；0 = 不修正。
            'ring1_extra_yaw_deg': 0.0,
            'ring2_extra_yaw_deg': 0.0,
            'ring3_extra_yaw_deg': 0.0,
            # 相似变换标定文件：视觉系 -> 塔吊系（scale/rotation/translation）。
            # 设置后直接作用于原始圆心，并跳过几何约束/旋转/径向/单环修正。
            'ring_similarity_file': '',
            # 径向补偿：物块整体落得偏内(负)或偏外(正)时，发 0x0C 前把三点
            # 沿各自径向平移该距离（mm）。只影响放置，不影响抓取。
            'ring_radial_offset_mm': 0.0,
            'tf_wait_sec': 0.5,
            'show': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.base_frame = self.param('base_frame')
        self.camera_frame = self.param('camera_frame')
        if (not self.base_frame or not self.camera_frame
                or self.base_frame == self.camera_frame):
            raise ValueError('Distinct base_frame and camera_frame are required')
        calibration = json.loads(
            data_path(self.param('calibration_file')).read_text(encoding='utf-8'))
        if calibration['child_frame'] != self.camera_frame:
            raise ValueError('Calibration camera frame does not match camera_frame')
        self.matrix = np.asarray(calibration['camera_matrix'], dtype=np.float64)
        self.distortion = np.asarray(calibration['dist_coeffs'], dtype=np.float64).reshape(-1)
        self.image_size = tuple(calibration['image_size'])
        if (self.matrix.shape != (3, 3) or not np.isfinite(self.matrix).all()
                or self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0
                or not np.allclose(self.matrix[2], [0, 0, 1])
                or self.distortion.size not in (4, 5, 8, 12, 14)
                or not np.isfinite(self.distortion).all()
                or len(self.image_size) != 2 or min(self.image_size) <= 0):
            raise ValueError('Invalid calibration intrinsics or resolution')
        self.planes = {'calibration': float(calibration['calibration_plane_z_mm'])}
        for mode in ('object', 'ring'):
            if self.param(f'{mode}_plane_configured'):
                self.planes[mode] = self.param(f'{mode}_plane_z_mm')
        if not all(math.isfinite(z) for z in self.planes.values()):
            raise ValueError('Plane heights must be finite')
        self.mode = self.param('plane_mode')
        if self.mode not in self.planes:
            raise ValueError('Selected plane is not configured')
        self.ring_geometry_enforce = bool(self.param('ring_geometry_enforce'))
        self.ring_center_spacing_mm = float(self.param('ring_center_spacing_mm'))
        if (self.ring_geometry_enforce
                and (not math.isfinite(self.ring_center_spacing_mm)
                     or self.ring_center_spacing_mm <= 0)):
            raise ValueError('ring_center_spacing_mm must be finite and positive')
        self.ring_frame_yaw_offset_deg = float(self.param('ring_frame_yaw_offset_deg'))
        self.ring_radial_offset_mm = float(self.param('ring_radial_offset_mm'))
        self.ring_extra_yaw_deg = tuple(
            float(self.param(f'ring{i}_extra_yaw_deg')) for i in (1, 2, 3))
        if not math.isfinite(self.ring_frame_yaw_offset_deg):
            raise ValueError('ring_frame_yaw_offset_deg must be finite')
        if not math.isfinite(self.ring_radial_offset_mm):
            raise ValueError('ring_radial_offset_mm must be finite')
        if not all(math.isfinite(v) for v in self.ring_extra_yaw_deg):
            raise ValueError('ringN_extra_yaw_deg must be finite')
        self.ring_similarity = self._load_ring_similarity(
            self.param('ring_similarity_file'))
        self.tf_wait = self.param('tf_wait_sec')
        if not math.isfinite(self.tf_wait) or self.tf_wait <= 0:
            raise ValueError('tf_wait_sec must be finite and positive')
        self.auto_interval = float(self.param('min_auto_interval_sec'))
        self.target_max_age = float(self.param('target_max_age_sec'))
        if (not math.isfinite(self.auto_interval) or self.auto_interval < 0
                or not math.isfinite(self.target_max_age) or self.target_max_age <= 0):
            raise ValueError('Invalid automatic projection interval or target age')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointStamped, self.param('output_topic'), 10)
        self.target_publisher = self.create_publisher(
            PointCloud, self.param('targets_topic'), 10)
        self.subscription = self.create_subscription(
            Image, self.param('image_topic'), self.on_image, qos_profile_sensor_data)
        self.center_subscription = self.create_subscription(
            PointCloud, self.param('centers_topic'), self.on_color_centers,
            qos_profile_sensor_data)
        self.ring_project_enabled = bool(self.param('ring_project_enabled'))
        self.ring_target_publisher = None
        self.stacked_enable_publisher = None
        if self.ring_project_enabled:
            self.ring_target_publisher = self.create_publisher(
                PointCloud, self.param('ring_targets_topic'), 10)
            self.stacked_enable_publisher = self.create_publisher(
                Bool, self.param('stacked_enable_topic'), 10)
        self.ring_centers_topic = self.param('ring_centers_topic')
        self.ring_centers_topic_after = self.param('ring_centers_topic_after')
        self.ring_source_switch_count = int(self.param('ring_source_switch_count'))
        if self.ring_source_switch_count < 0:
            raise ValueError('ring_source_switch_count must be >= 0')
        self.active_ring_source = self.ring_centers_topic
        self.first_layer_placement_count = 0
        self.current_task_id = None
        self.counted_place_commands = set()
        self.declare_parameter('first_layer_placement_count', 0)
        self.ring_center_subscription = None
        self.ring_center_after_subscription = None
        self.place_result_subscription = None
        if self.ring_project_enabled:
            self.ring_center_subscription = self.create_subscription(
                PointCloud, self.ring_centers_topic,
                lambda message: self.on_ring_centers(
                    message, self.ring_centers_topic),
                qos_profile_sensor_data)
            if (self.ring_centers_topic_after
                    and self.ring_centers_topic_after != self.ring_centers_topic):
                self.ring_center_after_subscription = self.create_subscription(
                    PointCloud, self.ring_centers_topic_after,
                    lambda message: self.on_ring_centers(
                        message, self.ring_centers_topic_after),
                    qos_profile_sensor_data)
            self.place_result_subscription = self.create_subscription(
                String, self.param('place_result_topic'),
                self.on_place_result, 10)
            if self.ring_source_switch_count > 0:
                self.get_logger().info(
                    f'圆环识别源：{self.ring_centers_topic}'
                    f'（前 {self.ring_source_switch_count} 次放置）'
                    f' -> {self.ring_centers_topic_after}（之后）')
            else:
                self.get_logger().info(
                    f'圆环识别源固定为 {self.ring_centers_topic}（未启用切换）')
        else:
            self.get_logger().info(
                'ring 投影已关闭（0x0C 不再发送）；颜色目标 0x08 正常；'
                '叠料使能由 supervisor 负责')
        self.last_auto_publish = -math.inf
        self.pending_centers = deque(maxlen=10)
        self.last_center_stamp = 0
        self.pending_rings = deque(maxlen=10)
        self.last_ring_stamp = 0
        self.last_ring_publish = -math.inf
        self.latest = None
        self.displayed = None
        self.frozen = None
        self.last_result = None
        self.pair_anchor = None   # 两点偏差模式的第一点 (sample, (X,Y)m, (px,py))
        self.pair_line = None     # 最近一次两点偏差的连线 (sample, px1, px2)
        self.stop_requested = False
        self.show = self.param('show')
        if self.show:
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.WINDOW, self.on_mouse)
        self.timer = self.create_timer(1.0 / 30.0, self.refresh)
        self.get_logger().info(
            f'Subscribing {self.param("image_topic")}; plane={self.mode}, '
            f'Z={self.planes[self.mode]:.2f} mm in {self.base_frame}. '
            f'auto centers={self.param("centers_topic")} -> {self.param("targets_topic")}.')

    def on_color_centers(self, msg):
        """Project color centroids from the source image time into gimbal_base."""
        if not self.param('auto_project'):
            return
        if 'object' not in self.planes:
            return
        try:
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if stamp_ns <= 0:
                raise ValueError('Missing image timestamp')
            if stamp_ns <= self.last_center_stamp:
                return
            if msg.header.frame_id != self.camera_frame:
                raise ValueError(f'Unexpected camera frame {msg.header.frame_id!r}')
            if not msg.points:
                self.last_center_stamp = stamp_ns
                self.pending_centers.clear()
                return
            self.validate_centers(msg)
            self.last_center_stamp = stamp_ns
            self.pending_centers.append((msg, time.monotonic()))
            self.project_pending_centers()
        except ValueError as error:
            self.get_logger().warning(
                f'拒绝颜色中心: {error}',
                throttle_duration_sec=2.0)

    def validate_centers(self, msg):
        channels = {name: self._channel_values(msg, name)
                    for name in ('color_id', 'area_px2', 'image_width', 'image_height')}
        if any(len(values) != len(msg.points) for values in channels.values()):
            raise ValueError('Color metadata must match each point')
        for i, point in enumerate(msg.points):
            color = channels['color_id'][i]
            if (not all(math.isfinite(v) for v in
                        (point.x, point.y, point.z, color, channels['area_px2'][i]))
                    or color not in COLOR_IDS
                    or channels['area_px2'][i] <= 0):
                raise ValueError('Invalid color or centroid')
            if (channels['image_width'][i], channels['image_height'][i]) != self.image_size:
                raise ValueError('Color image resolution differs from calibration')
            if not (0 <= point.x < self.image_size[0] and 0 <= point.y < self.image_size[1]):
                raise ValueError('Color centroid outside image bounds')

    def project_pending_centers(self):
        while self.pending_centers:
            msg, received_at = self.pending_centers[0]
            stamp = Time.from_msg(msg.header.stamp, clock_type=self.get_clock().clock_type)
            age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            now = time.monotonic()
            if age < -0.05 or age > self.target_max_age or now - received_at > self.tf_wait:
                self.pending_centers.popleft()
                self.get_logger().warning('颜色目标过期或 TF 等待超时，跳过',
                                          throttle_duration_sec=2.0)
                continue
            if now - self.last_auto_publish < self.auto_interval:
                self.pending_centers.popleft()
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, stamp)
            except TransformException:
                # 图像时刻 TF 不可用。仅当最新 TF 不晚于图像时刻（TF 偏旧/停更）
                # 才退回最新 TF 兜底；若最新 TF 在图像时刻之后（未来），不兜底。
                try:
                    latest = self.tf_buffer.lookup_transform(
                        self.base_frame, self.camera_frame, Time())
                except TransformException:
                    # Later feedback can bracket this image time; retry until tf_wait expires.
                    return
                latest_ns = (latest.header.stamp.sec * 10**9
                             + latest.header.stamp.nanosec)
                if latest_ns > stamp.nanoseconds:
                    return
                transform = latest
                self.get_logger().warning(
                    '图像时刻 TF 不可用，使用最新 TF 兜底（云台转动时坐标可能略偏）',
                    throttle_duration_sec=2.0)
            try:
                out = self.project_centers(msg, transform_matrix(transform))
            except (ValueError, cv2.error) as error:
                self.pending_centers.popleft()
                self.get_logger().warning(f'颜色投影失败: {error}', throttle_duration_sec=2.0)
                continue
            self.pending_centers.popleft()
            self.target_publisher.publish(out)
            self.last_auto_publish = now
            self.get_logger().info(
                f'自动投影 {len(out.points)} 种颜色 -> {self.base_frame} (m)',
                throttle_duration_sec=1.0)

    def project_centers(self, msg, base_from_camera):
        colors = self._channel_values(msg, 'color_id')
        areas = self._channel_values(msg, 'area_px2')
        selected = {}
        for i, color in enumerate(colors):
            if color not in PICKUP_COLOR_IDS:
                continue  # 跳过标记色（如粉色舵机外壳），不投影为抓取目标。
            if color not in selected or areas[i] > areas[selected[color]]:
                selected[color] = i
        out = PointCloud()
        out.header.stamp = copy.deepcopy(msg.header.stamp)
        out.header.frame_id = self.base_frame
        for color in sorted(selected):
            point = msg.points[selected[color]]
            xyz = pixel_to_plane((point.x, point.y), self.matrix, self.distortion,
                                 base_from_camera, self.planes['object'] / 1000.0)
            out.points.append(Point32(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
            # 调试：打印每种物块的基座毫米坐标。
            self.get_logger().info(
                f'物块 颜色{int(color)} -> ({xyz[0] * 1000.0:.1f}, '
                f'{xyz[1] * 1000.0:.1f}, {xyz[2] * 1000.0:.1f}) mm',
                throttle_duration_sec=1.0)
        out.channels = [ChannelFloat32(name='color_id', values=sorted(selected))]
        ready = self._channel_values(msg, 'motion_ready')
        if len(ready) == 1:
            out.channels.append(ChannelFloat32(name='motion_ready', values=list(ready)))
        return out

    def on_ring_centers(self, msg, source=None):
        """把三圆环圆心从图像时刻投影到 ring 平面，输出 gimbal_base 毫米坐标。

        ``source`` 用于区分两个识别源；只有当前激活的源才参与投影。
        """
        if source is not None and source != self.active_ring_source:
            return
        if not self.param('auto_project'):
            return
        if 'ring' not in self.planes:
            return
        try:
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if stamp_ns <= 0:
                raise ValueError('Missing image timestamp')
            if stamp_ns <= self.last_ring_stamp:
                return
            if msg.header.frame_id != self.camera_frame:
                raise ValueError(f'Unexpected camera frame {msg.header.frame_id!r}')
            if not msg.points:
                self.last_ring_stamp = stamp_ns
                self.pending_rings.clear()
                return
            self.validate_ring_centers(msg)
            self.last_ring_stamp = stamp_ns
            self.pending_rings.append((msg, time.monotonic()))
            self.project_pending_rings()
        except ValueError as error:
            self.get_logger().warning(f'拒绝圆环圆心: {error}', throttle_duration_sec=2.0)

    def on_place_result(self, message):
        """下位机 0x0B 放置结束：成功后累计第一层放置次数，达到阈值切换识别源。"""
        try:
            data = json.loads(message.data)
            task_id = str(data['task_id'])
            command_id = int(data['command_id'])
            success = bool(data['success'])
        except (ValueError, KeyError, TypeError):
            return
        if not success:
            return
        if task_id != self.current_task_id:
            self.current_task_id = task_id
            self.counted_place_commands.clear()
            self._set_first_layer_placement_count(0)
            self._set_active_ring_source(self.ring_centers_topic, '新任务重置')
        key = (task_id, command_id)
        if key in self.counted_place_commands:
            return
        self.counted_place_commands.add(key)
        if self.ring_source_switch_count <= 0:
            return
        if self.first_layer_placement_count >= self.ring_source_switch_count:
            # 第一层已计数完成；后续回传属于叠料区，只去重、不再累加。
            return
        count = self.first_layer_placement_count + 1
        self._set_first_layer_placement_count(count)
        self.get_logger().warning(
            f'第一层放置计数 {count}/{self.ring_source_switch_count}'
            f'（task={task_id}, command={command_id}）')
        if (count >= self.ring_source_switch_count
                and self.ring_centers_topic_after
                and self.active_ring_source != self.ring_centers_topic_after):
            self._set_active_ring_source(
                self.ring_centers_topic_after,
                f'第一层放置 {count} 次完成')

    def _set_first_layer_placement_count(self, count):
        self.first_layer_placement_count = int(count)
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter(
                'first_layer_placement_count', Parameter.Type.INTEGER,
                self.first_layer_placement_count)])
        except (TypeError, ValueError, RuntimeError) as error:
            # 参数仅用于调试观测，失败不影响识别源切换。
            self.get_logger().debug(f'更新放置计数参数失败: {error}')

    def _set_active_ring_source(self, source, reason):
        if not source or source == self.active_ring_source:
            return
        self.active_ring_source = source
        self.pending_rings.clear()
        self.last_ring_stamp = 0
        # 叠放区检测节点按需启用：切到叠放源时为 true，切回三圆环时为 false。
        try:
            self.stacked_enable_publisher.publish(
                Bool(data=source == self.ring_centers_topic_after))
        except (AttributeError, RuntimeError) as error:
            self.get_logger().debug(f'发布叠放位使能失败: {error}')
        self.get_logger().warning(f'圆环识别源切换为 {source}（{reason}）')

    def validate_ring_centers(self, msg):
        ring_ids = self._channel_values(msg, 'ring_id')
        if len(msg.points) != 3 or len(ring_ids) != 3:
            raise ValueError('圆环圆心必须正好 3 个')
        widths = self._channel_values(msg, 'image_width')
        heights = self._channel_values(msg, 'image_height')
        if widths and heights:
            if any((w, h) != self.image_size for w, h in zip(widths, heights)):
                raise ValueError('圆环图像分辨率与标定不符')
        for point in msg.points:
            if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
                raise ValueError('Invalid ring centroid')
            if not (0 <= point.x < self.image_size[0] and 0 <= point.y < self.image_size[1]):
                raise ValueError('Ring centroid outside image bounds')

    def project_pending_rings(self):
        while self.pending_rings:
            msg, received_at = self.pending_rings[0]
            stamp = Time.from_msg(msg.header.stamp, clock_type=self.get_clock().clock_type)
            age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            now = time.monotonic()
            if age < -0.05 or age > self.target_max_age or now - received_at > self.tf_wait:
                self.pending_rings.popleft()
                self.get_logger().warning('圆环圆心过期或 TF 等待超时，跳过',
                                          throttle_duration_sec=2.0)
                continue
            if now - self.last_ring_publish < self.auto_interval:
                self.pending_rings.popleft()
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, stamp)
            except TransformException:
                try:
                    latest = self.tf_buffer.lookup_transform(
                        self.base_frame, self.camera_frame, Time())
                except TransformException:
                    return
                latest_ns = (latest.header.stamp.sec * 10**9
                             + latest.header.stamp.nanosec)
                if latest_ns > stamp.nanoseconds:
                    return
                transform = latest
                self.get_logger().warning(
                    '图像时刻 TF 不可用，使用最新 TF 兜底（云台转动时坐标可能略偏）',
                    throttle_duration_sec=2.0)
            try:
                out = self.project_ring_centers(msg, transform_matrix(transform))
            except (ValueError, cv2.error) as error:
                self.pending_rings.popleft()
                self.get_logger().warning(f'圆环投影失败: {error}', throttle_duration_sec=2.0)
                continue
            self.pending_rings.popleft()
            self.ring_target_publisher.publish(out)
            self.last_ring_publish = now
            self.get_logger().info(
                f'投影 3 个圆环圆心 -> {self.base_frame} (m)',
                throttle_duration_sec=1.0)

    def project_ring_centers(self, msg, base_from_camera):
        ring_ids = self._channel_values(msg, 'ring_id')
        out = PointCloud()
        out.header.stamp = copy.deepcopy(msg.header.stamp)
        out.header.frame_id = self.base_frame
        projected = []
        for point in msg.points:
            xyz = pixel_to_plane((point.x, point.y), self.matrix, self.distortion,
                                 base_from_camera, self.planes['ring'] / 1000.0)
            projected.append(np.asarray(xyz, dtype=np.float64))
        self._log_ring_points('原始', projected)
        if self.ring_similarity is not None:
            projected = self._apply_ring_similarity(projected)
        else:
            if self.ring_geometry_enforce and len(projected) == 3:
                projected = self._enforce_ring_geometry(projected)
                self._log_ring_points('几何', projected)
            projected = self._rotate_ring_targets(projected)
            projected = self._apply_ring_extra_yaw(projected)
            projected = self._offset_ring_radius(projected)
        self._log_ring_points('发送', projected)
        for xyz in projected:
            out.points.append(Point32(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
        out.channels = [ChannelFloat32(name='ring_id', values=[float(v) for v in ring_ids])]
        return out

    def _log_ring_points(self, label, points):
        """一行打印三环 XY（mm），方便直接对照物块落点。"""
        text = ' | '.join(f'({p[0] * 1000.0:.1f},{p[1] * 1000.0:.1f})' for p in points)
        self.get_logger().info(f'三环{label}: {text}', throttle_duration_sec=1.0)

    def _enforce_ring_geometry(self, projected):
        """以中间环为锚点，按已知中心距重建两侧环。

        三环从左到右排列，取 P1->P3 方向为单位向量 u，重建：
            P1 = P2 - S*u,  P3 = P2 + S*u
        这样三环严格共线、严格等距，只剩整体平移/旋转误差；
        这两项校正由下位机负责，上位机不做偏移补偿。
        """
        p1, p2, p3 = (np.asarray(p[:2], dtype=np.float64) for p in projected)
        direction = p3 - p1
        length = float(np.linalg.norm(direction))
        if not np.isfinite(length) or length < 1e-9:
            raise ValueError('三环圆心重合，无法确定连线方向')
        unit = direction / length
        spacing = self.ring_center_spacing_mm / 1000.0
        left = p2 - spacing * unit
        right = p2 + spacing * unit
        z = float(projected[1][2])
        self.get_logger().info(
            f'三环重建（S={self.ring_center_spacing_mm:g}mm，锚点为中间环）',
            throttle_duration_sec=5.0)
        return [np.append(left, z),
                np.append(p2, z),
                np.append(right, z)]

    def _rotate_ring_targets(self, projected):
        """绕 gimbal_base 原点旋转三环圆心，补偿视觉系与塔吊系的 yaw 偏置。

        旋转不改变三点间距和共线关系，只消除整体方向偏差。
        """
        angle = self.ring_frame_yaw_offset_deg
        if not angle:
            return projected
        theta = math.radians(angle)
        c, s = math.cos(theta), math.sin(theta)
        rotated = []
        for p in projected:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            rotated.append(np.asarray((c * x - s * y, s * x + c * y, z)))
        self.get_logger().info(
            f'三环整体旋转 {angle:+.2f}° 后再发 0x0C', throttle_duration_sec=5.0)
        return rotated

    def _load_ring_similarity(self, path):
        """读取视觉系 -> 塔吊系的相似变换：D = s * R(theta) * P + t。"""
        if not path:
            return None
        document = json.loads(data_path(path).read_text(encoding='utf-8'))
        if int(document.get('format_version', 0)) != 1:
            raise ValueError('ring_similarity 版本不受支持')
        scale = float(document['scale'])
        rotation = float(document['rotation_deg'])
        translation = [float(v) for v in document['translation_mm']]
        if (not math.isfinite(scale) or scale <= 0
                or not math.isfinite(rotation)
                or len(translation) != 2
                or not all(math.isfinite(v) for v in translation)):
            raise ValueError('ring_similarity 数值无效')
        self.get_logger().info(
            f'已加载三环相似变换: scale={scale:.5f}, '
            f'rot={rotation:+.3f}°, t=({translation[0]:+.2f},{translation[1]:+.2f})mm')
        return scale, rotation, translation

    def _apply_ring_similarity(self, projected):
        """按相似变换把视觉系圆心映射到塔吊系，直接作为 0x0C 发送值。"""
        scale, rotation, translation = self.ring_similarity
        theta = math.radians(rotation)
        c, s = math.cos(theta), math.sin(theta)
        tx, ty = translation
        mapped = []
        for p in projected:
            x, y, z = float(p[0]) * 1000.0, float(p[1]) * 1000.0, float(p[2])
            mapped.append(np.asarray((
                (scale * (c * x - s * y) + tx) / 1000.0,
                (scale * (s * x + c * y) + ty) / 1000.0,
                z)))
        self.get_logger().info('三环应用相似变换后再发 0x0C', throttle_duration_sec=5.0)
        return mapped

    def _apply_ring_extra_yaw(self, projected):
        """按环单独旋转（绕原点），修正各环独立的角误差。"""
        if not any(self.ring_extra_yaw_deg):
            return projected
        adjusted = []
        for index, p in enumerate(projected):
            angle = self.ring_extra_yaw_deg[index] if index < 3 else 0.0
            if not angle:
                adjusted.append(p)
                continue
            theta = math.radians(angle)
            c, s = math.cos(theta), math.sin(theta)
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            adjusted.append(np.asarray((c * x - s * y, s * x + c * y, z)))
            self.get_logger().info(
                f'环{index + 1} 单独旋转 {angle:+.2f}°', throttle_duration_sec=5.0)
        return adjusted

    def _offset_ring_radius(self, projected):
        """沿各自径向平移三环圆心，补偿整体「落得偏内/偏外」。

        只作用于发往下位机的 0x0C 放置坐标，不影响颜色抓取和底盘纠偏。
        """
        offset = self.ring_radial_offset_mm
        if not offset:
            return projected
        shifted = []
        for p in projected:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            radius = math.hypot(x, y)
            if radius < 1e-9:
                shifted.append(p)
                continue
            scale = (radius + offset / 1000.0) / radius
            shifted.append(np.asarray((x * scale, y * scale, z)))
        self.get_logger().info(
            f'三环径向补偿 {offset:+.1f}mm 后再发 0x0C', throttle_duration_sec=5.0)
        return shifted

    @staticmethod
    def _channel_values(msg, name):
        for channel in msg.channels:
            if channel.name == name:
                return channel.values
        return []

    def param(self, name):
        return self.get_parameter(name).value

    def on_image(self, msg):
        try:
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                raise ValueError('Image requires a nonzero acquisition timestamp')
            if msg.header.frame_id and msg.header.frame_id != self.camera_frame:
                raise ValueError(f'Unexpected image frame: {msg.header.frame_id}')
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if image.shape[1::-1] != self.image_size:
                raise ValueError('Image resolution differs from calibration')
            if not msg.header.frame_id:
                self.get_logger().warning(
                    f'Image frame_id is empty; using configured {self.camera_frame}',
                    throttle_duration_sec=5.0)
            sample = ImageSample(image.copy(), copy.deepcopy(msg.header.stamp), time.monotonic())
            self.resolve_tf(sample)
            self.latest = sample
        except (ValueError, cv2.error, RuntimeError) as error:
            self.get_logger().warning(f'Rejecting image: {error}', throttle_duration_sec=2.0)

    def resolve_tf(self, sample):
        if sample.base_from_camera is not None:
            return
        if time.monotonic() - sample.received_at > self.tf_wait:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                Time.from_msg(sample.stamp, clock_type=self.get_clock().clock_type))
            sample.base_from_camera = transform_matrix(transform)
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for image-time TF (no latest-TF fallback): {error}',
                throttle_duration_sec=2.0)

    def project_click(self, sample, pixel):
        if sample.base_from_camera is None:
            raise ValueError('This image has no matching TF; wait for feedback and a new frame')
        u, v = pixel
        if not (0 <= u < self.image_size[0] and 0 <= v < self.image_size[1]):
            raise ValueError('Pixel outside image bounds')
        xyz = pixel_to_plane(pixel, self.matrix, self.distortion,
                             sample.base_from_camera, self.planes[self.mode] / 1000.0)
        result = PointStamped()
        result.header.frame_id = self.base_frame
        result.header.stamp = copy.deepcopy(sample.stamp)
        result.point.x, result.point.y, result.point.z = (float(value) for value in xyz)
        return result

    def on_mouse(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN or self.displayed is None:
            return
        try:
            # Use the displayed frame's cached TF even if newer feedback has arrived.
            result = self.project_click(self.displayed, (x, y))
            self.publisher.publish(result)
            self.last_result = (self.displayed, (x, y), result)
            p = result.point
            self.get_logger().info(
                f'pixel=({x},{y}), plane={self.mode}, frame={result.header.frame_id}, '
                f'XYZ=({p.x*1000:.2f}, {p.y*1000:.2f}, {p.z*1000:.2f}) mm, '
                f'image_time={result.header.stamp.sec}.{result.header.stamp.nanosec:09d}')
            # 两点偏差模式：同一帧内连续点两次（先圆环中心、再物块中心），
            # 第二次点击打印 (dx, dy) 和距离，正方向 = base 系 +X 前 / +Y 左。
            if self.pair_anchor is not None and self.pair_anchor[0] is self.displayed:
                anchor = self.pair_anchor[1]
                anchor_px = self.pair_anchor[2]
                dx = (p.x - anchor[0]) * 1000.0
                dy = (p.y - anchor[1]) * 1000.0
                self.get_logger().info(
                    f'两点偏差: dx={dx:+.2f} mm, dy={dy:+.2f} mm, '
                    f'距离={math.hypot(dx, dy):.2f} mm (+X 前, +Y 左)')
                self.pair_line = (self.displayed, anchor_px, (x, y))
                self.pair_anchor = None  # 下一次点击重新开始一组
            else:
                self.pair_anchor = (self.displayed, (p.x, p.y), (x, y))
                self.get_logger().info('已记录第一点；再点第二点即输出两点偏差')
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'Click rejected: {error}')

    def refresh(self):
        self.project_pending_centers()
        if self.latest is not None:
            self.resolve_tf(self.latest)
        if not self.show:
            return
        sample = self.frozen if self.frozen is not None else self.latest
        if sample is not None:
            self.resolve_tf(sample)
            image = sample.image.copy()
            status = 'TF OK' if sample.base_from_camera is not None else 'TF unavailable'
            cv2.putText(image, f'{self.mode} Z={self.planes[self.mode]:.2f}mm {status}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            if self.last_result is not None and self.last_result[0] is sample:
                _, pixel, result = self.last_result
                p = result.point
                cv2.drawMarker(image, pixel, (0, 255, 0), cv2.MARKER_CROSS, 16, 1)
                cv2.putText(image, f'XYZ mm: {p.x*1000:.2f}, {p.y*1000:.2f}, {p.z*1000:.2f}',
                            (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
            if self.pair_anchor is not None and self.pair_anchor[0] is sample:
                # 第一点已记录：画出标记，提示再点第二点
                anchor_px = self.pair_anchor[2]
                cv2.drawMarker(image, anchor_px, (0, 180, 255), cv2.MARKER_TRIANGLE_UP, 14, 2)
                cv2.putText(image, '1st point; click 2nd for delta', (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1)
            if self.pair_line is not None and self.pair_line[0] is sample:
                _, first_px, second_px = self.pair_line
                cv2.line(image, first_px, second_px, (0, 180, 255), 1)
                cv2.drawMarker(image, first_px, (0, 180, 255), cv2.MARKER_TRIANGLE_UP, 14, 2)
            cv2.imshow(self.WINDOW, image)
            self.displayed = sample
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.stop_requested = True
        elif key == ord(' ') and self.displayed is not None:
            self.frozen = self.displayed if self.frozen is None else None
            self.last_result = None
            self.pair_anchor = None
            self.pair_line = None
        elif key in (ord('1'), ord('2'), ord('3')):
            mode = {ord('1'): 'calibration', ord('2'): 'object', ord('3'): 'ring'}[key]
            if mode in self.planes:
                self.mode = mode
                self.last_result = None
                # 保留 pair_anchor：允许「圆环在地面平面、物块在物块顶面」跨平面配对
                self.pair_line = None
            else:
                self.get_logger().warning(f'{mode} plane height has not been configured')

    def destroy_node(self):
        if getattr(self, 'show', False):
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PixelToBaseNode()
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
