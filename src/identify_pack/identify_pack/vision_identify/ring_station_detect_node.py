#!/usr/bin/env python3
"""ROS2 节点：三圆环工位检测与视觉对位（移植自 cyqm ring_station_detector）。

订阅相机图像，用 Canny + 椭圆拟合 + 三环共线筛选检测三圆环工位，
多帧稳定后输出 mm 误差、角度误差与底盘纠偏量。

发布：
  - /ring_station  对位结果，格式 ``dx_mm,dy_mm,dyaw_deg,corr_x_mm,corr_y_mm,corr_yaw_deg,within``
    （within 为 0/1；配置缺 chassis_image_to_motion 时 corr_* 按 0 占位）

圆环编号固定为画面左到右 1、2、3；模板核验可见数字。
毫米误差与放置精度依赖当前相机分辨率、视角及现场标定。
"""
import os
import json
from dataclasses import replace
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2

from identify_pack.scripts.ring_station_detector import (
    RingStationDetector, draw_ring_station_detection, load_ring_station_config,
)
from identify_pack.vision_identify.ring_centers import ring_centers_message
from identify_pack.scripts.ring_station_digits import (
    load_templates, recognize_detection, DigitMappingTracker,
)


class RingStationDetectNode(Node):
    """三圆环工位检测节点。"""

    def __init__(self):
        super().__init__('ring_station_detect_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('ring_station_config', '')  # ring_station_detector.json 路径（空=自动定位）
        # 仅复用叠料区配置里的上下边线检测参数，不改叠料区检测逻辑。
        self.declare_parameter('stacked_ring_config', '')
        # 数字模板识别每帧做模板匹配；0x0C 流程用不到，关闭可显著提速。
        self.declare_parameter('enable_digit_recognition', True)
        self.declare_parameter('show', True)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('centers_topic', '/ring_station/centers_pixel')
        self.declare_parameter('digit_templates', '')  # 模板只用于核验固定的左/中/右环号
        self.declare_parameter('digit_inner_ratio', 0.55)
        self.declare_parameter('digit_min_confidence', 0.55)
        self.declare_parameter('digit_min_margin', 0.08)
        self.declare_parameter('digits_topic', '/ring_station/digits')
        # 常驻休眠：启动时不订阅图像；收到 enable_topic=true 才创建订阅。
        self.declare_parameter('enabled', False)
        self.declare_parameter('enable_topic', '/ring_station/enable')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.ring_station_config = self.get_parameter('ring_station_config').get_parameter_value().string_value
        self.stacked_ring_config = self.get_parameter('stacked_ring_config').get_parameter_value().string_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value
        self.image_topic = image_topic
        self.enabled = bool(self.get_parameter('enabled').value)
        self.enable_digit = bool(
            self.get_parameter('enable_digit_recognition').value)
        enable_topic = self.get_parameter('enable_topic').get_parameter_value().string_value

        self.detector = self._build_detector()
        inner, minimum, margin = (self.get_parameter(name).value for name in
                                  ('digit_inner_ratio', 'digit_min_confidence', 'digit_min_margin'))
        if not (0.1 <= inner <= 0.85 and 0 <= minimum <= 1 and 0 <= margin <= 2):
            raise ValueError('Invalid printed digit extraction or matching thresholds')
        self.digit_options = (inner, minimum, margin)
        if self.enable_digit:
            self.templates, _ = load_templates(Path(self._resolve_config(
                self.get_parameter('digit_templates').value,
                'ring_station_digit_templates')), allow_builtin=False)
            self.digit_tracker = DigitMappingTracker(
                self.detector.config.stable_frame_count,
                self.detector.config.maximum_center_jitter_px,
                expected_digits=(1, 2, 3))
        else:
            self.templates = None
            self.digit_tracker = None
            self.get_logger().info('数字模板识别已关闭（0x0C 流程不需要）')
        self.last_stamp_ns = None

        self.br = CvBridge()
        self.sub = None
        self.pub = self.create_publisher(String, 'ring_station', 10)
        self.pub_centers = self.create_publisher(
            PointCloud, self.get_parameter('centers_topic').value, 10)
        self.pub_digits = self.create_publisher(String, self.get_parameter('digits_topic').value, 10)
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.on_enable, 10)
        if self.enabled:
            self._enable_image_subscription()
        else:
            self.get_logger().info(
                f'三圆环检测已启动（暂停，等待 {enable_topic} 使能）')

    def _enable_image_subscription(self):
        if self.sub is None:
            self.sub = self.create_subscription(
                Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
            self.get_logger().info(f'三圆环检测已启用，订阅 {self.image_topic}')

    def on_enable(self, msg):
        enabled = bool(msg.data)
        if enabled and self.sub is None:
            self._enable_image_subscription()
        elif not enabled and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.detector.reset()
            self.digit_tracker.reset()
            self.last_stamp_ns = None
            self.get_logger().info('三圆环检测已暂停')

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
            config = load_ring_station_config(
                self._resolve_config(self.ring_station_config, 'ring_station_detector.json'))
        except Exception as error:
            raise RuntimeError(f'三圆环工位配置加载失败：{error}') from error
        self.get_logger().info('已加载三圆环工位配置')
        line_config = None
        if config.line_constraint_enabled:
            try:
                from identify_pack.scripts.stacked_ring_detector import (
                    load_stacked_ring_config,
                )
                line_config = load_stacked_ring_config(self._resolve_config(
                    self.stacked_ring_config, 'stacked_ring_detector.json'))
                self.get_logger().info('三圆环检测已启用上下边线约束')
            except (OSError, ValueError, KeyError, RuntimeError) as error:
                self.get_logger().warning(
                    f'边线约束配置加载失败，回退三环拟合：{error}')
        # Both stations are always observed in left-to-right 1/2/3 order.
        return RingStationDetector(replace(config, ring1_side='left'),
                                   use_hough=True, line_config=line_config)

    def image_cb(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'cv_bridge error: {error}')
            return

        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.detector.reset()
            if self.digit_tracker is not None:
                self.digit_tracker.reset()
            return
        if self.last_stamp_ns is not None and stamp_ns - self.last_stamp_ns > 500000000:
            self.detector.reset()
            if self.digit_tracker is not None:
                self.digit_tracker.reset()
        self.last_stamp_ns = stamp_ns
        result = self.detector.detect(img)
        report = None
        if self.enable_digit:
            report, _ = recognize_detection(
                img, result, self.templates, *self.digit_options)
            self.digit_tracker.update(report, img.shape[1::-1])
            report.update({'stamp_ns': stamp_ns, 'frame_id': msg.header.frame_id or
                           self.get_parameter('camera_frame').value,
                           'image_size': list(img.shape[1::-1])})
            self.pub_digits.publish(
                String(data=json.dumps(report, allow_nan=False)))
        self.pub_centers.publish(ring_centers_message(
            msg.header, result, img.shape[1::-1], self.get_parameter('camera_frame').value, report))
        if result.detected:
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
            self.get_logger().info(
                f'centers={result.centers_px} '
                f'err=({result.error_x_mm:.2f},{result.error_y_mm:.2f})mm '
                f'yaw={result.angle_error_deg:.2f}deg '
                f'conf={result.confidence:.2f} stable={result.stable} '
                f'within={result.within_tolerance} scope={result.detection_scope} '
                f'line={int(result.line_constraint_applied)} '
                f'line_q={(result.line_quality if result.line_quality is not None else -1):.2f} '
                f'proc={result.processing_ms:.0f}ms',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(f'三圆环工位未检出：{result.detail}',
                                   throttle_duration_sec=1.0)

        if self.show:
            draw_ring_station_detection(img, result)
            if report is not None:
                for ring in report['rings']:
                    x, y = (int(round(value)) for value in ring['center_px'])
                    cv2.putText(
                        img,
                        f"R{ring['ring_id'] or '?'} digit={ring['digit'] or '?'}",
                        (max(0, x - 40), min(img.shape[0] - 8, y + 30)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow('ring_station', img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RingStationDetectNode()
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
