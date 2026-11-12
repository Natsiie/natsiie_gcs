#!/usr/bin/env python3
"""ROS2 节点：叠放位中环 + 上下基准线检测（移植自 cyqm stacked_ring_detector）。

订阅相机图像，检测中环与上下两条基准线，重建三环圆心后输出 mm 误差、角度误差与纠偏量。

发布：
  - /stacked_ring  对位结果，格式 ``dx_mm,dy_mm,dyaw_deg,corr_x_mm,corr_y_mm,corr_yaw_deg,within``

注意：stacked_ring_detector.json 里的 search_calibration 与 ring_station_detector.json 的
reference 标定值均为 1280x720 占位，需先用 stacked_ring_calibrator.py 重标定，
否则标定模式会因 image_size 不匹配报 Recalibrate stacking。
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2

from identify_pack.scripts.stacked_ring_detector import (
    StackedRingDetector, camera_geometry_signature, draw_stacked_ring_detection,
    load_stacked_ring_config,
)
from identify_pack.scripts.ring_station_detector import (
    load_ring_station_config,
)
from identify_pack.vision_identify.ring_centers import ring_centers_message
from identify_pack.scripts.trim_gate import should_publish_trim


class StackedRingDetectNode(Node):
    """叠放位检测节点。"""

    def __init__(self):
        super().__init__('stacked_ring_detect_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('stacked_ring_config', '')   # stacked_ring_detector.json 路径
        self.declare_parameter('ring_station_config', '')   # ring_station_detector.json 路径
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('show', True)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('centers_topic', '/stacked_ring/centers_pixel')
        # 默认休眠：不订阅图像、不逐帧检测；等 /stacked_ring/enable 为 true 再启用。
        self.declare_parameter('enabled', False)
        self.declare_parameter('enable_topic', '/stacked_ring/enable')
        # 纠偏帧门控：运动拖影期间 stable/confidence 不达标时不发 0x05。
        self.declare_parameter('publish_require_stable', True)
        self.declare_parameter('publish_min_confidence', 0.5)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.image_topic = image_topic
        self.enabled = bool(self.get_parameter('enabled').value)
        enable_topic = self.get_parameter('enable_topic').get_parameter_value().string_value
        self.publish_require_stable = bool(
            self.get_parameter('publish_require_stable').value)
        self.publish_min_confidence = float(
            self.get_parameter('publish_min_confidence').value)
        if not 0.0 <= self.publish_min_confidence <= 1.0:
            raise ValueError('publish_min_confidence must be in [0, 1]')
        self.stacked_ring_config = self.get_parameter('stacked_ring_config').get_parameter_value().string_value
        self.ring_station_config = self.get_parameter('ring_station_config').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value

        self.detector = self._build_detector()

        self.br = CvBridge()
        self.sub = None
        self.pub = self.create_publisher(String, 'stacked_ring', 10)
        self.pub_centers = self.create_publisher(
            PointCloud, self.get_parameter('centers_topic').value, 10)
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.on_enable, 10)
        if self.enabled:
            self._enable_image_subscription()
        else:
            self.get_logger().info(
                f'叠放位检测已启动（暂停，等待 {enable_topic} 使能）')

    def _enable_image_subscription(self):
        if self.sub is None:
            self.sub = self.create_subscription(
                Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
            self.get_logger().info(f'叠放位检测已启用，订阅 {self.image_topic}')

    def on_enable(self, msg):
        """按需启用/暂停图像订阅；检测逻辑本身保持不变。"""
        enabled = bool(msg.data)
        if enabled and self.sub is None:
            self._enable_image_subscription()
        elif not enabled and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.detector.reset()
            self.get_logger().info('叠放位检测已暂停')

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
            config = load_stacked_ring_config(
                self._resolve_config(self.stacked_ring_config, 'stacked_ring_detector.json'))
            ring_config = load_ring_station_config(
                self._resolve_config(self.ring_station_config, 'ring_station_detector.json'))
        except Exception as error:
            raise RuntimeError(f'叠放位配置加载失败：{error}') from error
        self.get_logger().info('已加载叠放位检测配置')
        detector = StackedRingDetector(config, ring_config)
        detector.set_camera_geometry(
            camera_geometry_signature(self.width, self.height, False, None))
        return detector

    def image_cb(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'cv_bridge error: {error}')
            return

        result = self.detector.detect(img)
        self.pub_centers.publish(ring_centers_message(
            msg.header, result, img.shape[1::-1], self.get_parameter('camera_frame').value))
        if result.detected:
            if should_publish_trim(
                    result, self.publish_require_stable,
                    self.publish_min_confidence):
                out = String()
                out.data = (
                    f'{result.error_x_mm:.2f},{result.error_y_mm:.2f},'
                    f'{result.angle_error_deg:.2f},'
                    f'{result.correction_body_x_mm or 0.0:.2f},'
                    f'{result.correction_body_y_mm or 0.0:.2f},'
                    f'{result.correction_yaw_deg or 0.0:.2f},'
                    f'{1 if result.within_tolerance else 0}'
                )
                self.pub.publish(out)
            else:
                self.get_logger().debug(
                    f'跳过纠偏帧: stable={result.stable} '
                    f'conf={result.confidence:.2f}',
                    throttle_duration_sec=1.0)
            extra = ''
            if result.line_lock_active:
                dy = (f'{result.line_mid_y_error_px:+.1f}px'
                      if result.line_mid_y_error_px is not None else 'n/a')
                dy_mm = (f'{result.line_error_y_mm:+.1f}mm'
                         if result.line_error_y_mm is not None else 'n/a')
                separation = (f'{result.line_separation_px:.1f}px'
                              if result.line_separation_px is not None else 'n/a')
                lower = 'derived' if result.lower_line_derived else 'measured'
                extra = (f' line_lock dY={dy} ({dy_mm}) '
                         f'sep={separation} lower={lower}')
            self.get_logger().info(
                f'middle={result.middle_center_px} '
                f'err=({result.error_x_mm:.2f},{result.error_y_mm:.2f})mm '
                f'yaw={result.angle_error_deg:.2f}deg stable={result.stable} '
                f'circle={result.circle_support:.2f} line={result.line_quality:.2f}'
                f'{extra}',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(f'叠放位未检出：{result.detail}',
                                   throttle_duration_sec=1.0)

        if self.show:
            draw_stacked_ring_detection(img, result)
            cv2.imshow('stacked_ring', img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = StackedRingDetectNode()
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
