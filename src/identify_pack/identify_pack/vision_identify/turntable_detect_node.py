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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from identify_pack.scripts.turntable_detector import (
    TurntableDetector, draw_turntable_detection, load_turntable_detector_config,
)
from identify_pack.scripts.turntable_material_seed import (
    ThreeMaterialTurntableSeeder, load_three_material_seed_config,
)
from identify_pack.scripts.color_config import load_color_config


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

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.turntable_config = self.get_parameter('turntable_config').get_parameter_value().string_value
        self.material_seed_config = self.get_parameter('material_seed_config').get_parameter_value().string_value
        self.color_config = self.get_parameter('color_config').get_parameter_value().string_value
        self.enable_material_seed = self.get_parameter('enable_material_seed').get_parameter_value().bool_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value

        self.detector = self._build_detector()

        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub = self.create_publisher(String, 'Turntable', 10)
        self.get_logger().info(f'转盘检测订阅 {image_topic}')

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
