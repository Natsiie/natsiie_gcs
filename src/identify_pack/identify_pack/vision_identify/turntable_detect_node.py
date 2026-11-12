#!/usr/bin/env python3
"""ROS2 节点：转盘外圆圆心检测（移植自 cyqm turntable_detector）。

订阅相机图像，检测转盘外圆并发布圆心。

发布：
  - /Turntable  圆心与半径，格式 ``cx,cy,radius,stable``（stable 为 0/1）

检测分两阶段：缩图上 Hough 大圆粗定位 → 全分辨率径向采样 + 鲁棒圆拟合；
锁定后只做局部跟踪，失败立即全图重定位。可选三物料辅助粗定位。
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2

from identify_pack.scripts.turntable_detector import (
    TurntableDetector, draw_turntable_detection, load_turntable_detector_config,
)
from identify_pack.scripts.turntable_material_seed import (
    ThreeMaterialTurntableSeeder, load_three_material_seed_config,
)
from identify_pack.scripts.color_config import load_color_config
from identify_pack.scripts.turntable_align import (
    compute_turntable_correction, load_turntable_align_config,
)
from identify_pack.scripts.ring_station_detector import load_ring_station_config
from identify_pack.scripts.trim_gate import should_publish_trim


class TurntableDetectNode(Node):
    """转盘外圆检测节点。"""

    def __init__(self):
        super().__init__('turntable_detect_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('turntable_config', '')   # turntable_detector.json 路径（空=自动定位）
        self.declare_parameter('material_seed_config', '')  # turntable_material_seed.json 路径
        self.declare_parameter('color_config', '')       # color_config.json 路径（三物料用）
        self.declare_parameter('enable_material_seed', True)
        self.declare_parameter('show', True)
        self.declare_parameter('align_config', '')   # turntable_align.json 路径（空=自动定位）
        self.declare_parameter('ring_config', '')    # ring_station_detector.json 路径（底盘纠偏矩阵）
        # 常驻休眠：启动时不订阅图像；收到 enable_topic=true 才创建订阅。
        self.declare_parameter('enabled', False)
        self.declare_parameter('enable_topic', '/turntable/enable')
        # 纠偏帧门控：运动拖影期间 stable/confidence 不达标时不发 0x0E。
        self.declare_parameter('publish_require_stable', True)
        self.declare_parameter('publish_min_confidence', 0.5)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.turntable_config = self.get_parameter('turntable_config').get_parameter_value().string_value
        self.material_seed_config = self.get_parameter('material_seed_config').get_parameter_value().string_value
        self.color_config = self.get_parameter('color_config').get_parameter_value().string_value
        self.enable_material_seed = self.get_parameter('enable_material_seed').get_parameter_value().bool_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value
        self.align_config_path = self.get_parameter('align_config').get_parameter_value().string_value
        self.ring_config_path = self.get_parameter('ring_config').get_parameter_value().string_value
        self.image_topic = image_topic
        self.enabled = bool(self.get_parameter('enabled').value)
        enable_topic = self.get_parameter('enable_topic').get_parameter_value().string_value
        self.publish_require_stable = bool(
            self.get_parameter('publish_require_stable').value)
        self.publish_min_confidence = float(
            self.get_parameter('publish_min_confidence').value)
        if not 0.0 <= self.publish_min_confidence <= 1.0:
            raise ValueError('publish_min_confidence must be in [0, 1]')

        self.detector = self._build_detector()
        self.align_config, self.chassis_matrix = self._build_align()

        self.br = CvBridge()
        self.sub = None
        self.pub = self.create_publisher(String, 'Turntable', 10)
        self.pub_align = self.create_publisher(String, 'TurntableAlign', 10)
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.on_enable, 10)
        if self.enabled:
            self._enable_image_subscription()
        else:
            self.get_logger().info(
                f'转盘检测已启动（暂停，等待 {enable_topic} 使能）')

    def _enable_image_subscription(self):
        if self.sub is None:
            self.sub = self.create_subscription(
                Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
            self.get_logger().info(f'转盘检测已启用，订阅 {self.image_topic}')

    def on_enable(self, msg):
        enabled = bool(msg.data)
        if enabled and self.sub is None:
            self._enable_image_subscription()
        elif not enabled and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            reset = getattr(self.detector, 'reset', None)
            if callable(reset):
                reset()
            self.get_logger().info('转盘检测已暂停')

    def _resolve_config(self, param_value, filename):
        if param_value:
            return os.path.expanduser(param_value)
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory('identify_pack'), filename)
        except Exception:
            return os.path.join(os.path.dirname(__file__), '..', 'scripts', filename)

    def _build_detector(self):
        try:
            config = load_turntable_detector_config(
                self._resolve_config(self.turntable_config, 'turntable_detector.json'))
        except Exception as error:
            raise RuntimeError(f'转盘检测配置加载失败：{error}') from error
        self.get_logger().info('已加载转盘检测配置')

        seeder = None
        if self.enable_material_seed:
            try:
                seed_config = load_three_material_seed_config(
                    self._resolve_config(self.material_seed_config, 'turntable_material_seed.json'))
                color_config = load_color_config(
                    self._resolve_config(self.color_config, 'color_config.json'))
                seeder = ThreeMaterialTurntableSeeder(seed_config, color_config)
                self.get_logger().info('三物料辅助粗定位已启用')
            except Exception as error:
                self.get_logger().warn(f'三物料辅助粗定位未启用：{error}')
        return TurntableDetector(config, seeder)

    def _build_align(self):
        try:
            align = load_turntable_align_config(
                self._resolve_config(self.align_config_path, 'turntable_align.json'))
            ring = load_ring_station_config(
                self._resolve_config(self.ring_config_path, 'ring_station_detector.json'))
        except Exception as error:
            self.get_logger().warn(f'转盘对齐未启用：{error}')
            return None, None
        if ring.chassis_image_to_motion is None:
            self.get_logger().warn('底盘纠偏矩阵 chassis_image_to_motion 未标定，转盘对齐未启用')
            return None, None
        self.get_logger().info('已加载转盘对齐配置（参考圆心 + 底盘纠偏矩阵）')
        return align, ring.chassis_image_to_motion

    def image_cb(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'cv_bridge error: {error}')
            return

        result = self.detector.detect(img)
        if result.detected:
            out = String()
            out.data = (f'{result.center_x_px:.1f},{result.center_y_px:.1f},'
                        f'{result.radius_px:.1f},{1 if result.stable else 0}')
            self.pub.publish(out)
            if self.align_config is not None:
                if not should_publish_trim(
                        result, self.publish_require_stable,
                        self.publish_min_confidence):
                    self.get_logger().debug(
                        f'跳过转盘纠偏帧: stable={result.stable} '
                        f'conf={result.confidence:.2f}',
                        throttle_duration_sec=1.0)
                else:
                    try:
                        corr_x, corr_y, within = compute_turntable_correction(
                            (result.center_x_px, result.center_y_px), result.radius_px,
                            self.align_config, self.chassis_matrix)
                        align = String()
                        align.data = f'{corr_x:.2f},{corr_y:.2f},{1 if within else 0}'
                        self.pub_align.publish(align)
                        self.get_logger().info(
                            f'align corr=({corr_x:.2f},{corr_y:.2f})mm within={within}',
                            throttle_duration_sec=1.0)
                    except ValueError as error:
                        self.get_logger().warn(f'转盘对齐计算失败：{error}',
                                               throttle_duration_sec=2.0)
            self.get_logger().info(
                f'center=({result.center_x_px:.1f},{result.center_y_px:.1f}) '
                f'r={result.radius_px:.1f} arc={result.visible_arc_deg:.0f} '
                f'rms={result.fit_rms_px:.2f} stable={result.stable} '
                f'track={result.tracking_mode}',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(f'转盘未检出：{result.detail}',
                                   throttle_duration_sec=1.0)

        if self.show:
            draw_turntable_detection(img, result)
            cv2.imshow('turntable', img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = TurntableDetectNode()
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
