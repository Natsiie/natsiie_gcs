#!/usr/bin/env python3
"""ROS2 节点：三圆环工位检测与视觉对位（移植自 cyqm ring_station_detector）。

订阅相机图像，用 Canny + 椭圆拟合 + 三环共线筛选检测三圆环工位，
多帧稳定后输出 mm 误差、角度误差与底盘纠偏量。

发布：
  - /ring_station  对位结果，格式 ``dx_mm,dy_mm,dyaw_deg,corr_x_mm,corr_y_mm,corr_yaw_deg,within``
    （within 为 0/1；配置缺 chassis_image_to_motion 时 corr_* 按 0 占位）

注意：配置 ring_station_detector.json 的标定值是 1280x720 的模板占位，
需针对 natsiie 的 640x480 相机重新标定后才能得到真实 mm/yaw。
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from identify_pack.scripts.ring_station_detector import (
    RingStationDetector, draw_ring_station_detection, load_ring_station_config,
)
from identify_pack.vision_identify.ring_centers import ring_centers_message


class RingStationDetectNode(Node):
    """三圆环工位检测节点。"""

    def __init__(self):
        super().__init__('ring_station_detect_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('ring_station_config', '')  # ring_station_detector.json 路径（空=自动定位）
        self.declare_parameter('show', True)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('centers_topic', '/ring_station/centers_pixel')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.ring_station_config = self.get_parameter('ring_station_config').get_parameter_value().string_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value

        self.detector = self._build_detector()

        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, 'ring_station', 10)
        self.pub_centers = self.create_publisher(
            PointCloud, self.get_parameter('centers_topic').value, 10)
        self.get_logger().info(f'三圆环工位检测订阅 {image_topic}')

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
        return RingStationDetector(config)

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
                f'within={result.within_tolerance} scope={result.detection_scope}',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(f'三圆环工位未检出：{result.detail}',
                                   throttle_duration_sec=1.0)

        if self.show:
            draw_ring_station_detection(img, result)
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
