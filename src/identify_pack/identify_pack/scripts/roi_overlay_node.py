#!/usr/bin/env python3
"""
ROS2 节点：订阅图像话题，把 ROI 区域画到画面上并弹窗显示，用于调试 ROI 划分。

图像由 camera_node 发布；本节点不打开摄像头，只订阅并叠加 ROI。
ROI 坐标基于 640x480（与 zongchengxu20250724chusai.py 顶部定义一致）。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 与 zongchengxu20250724chusai.py 顶部一致的 ROI 定义（格式: x, y, w, h）
ROIS = [
    ("dingxin centering", (160, 140, 320, 210), (0, 255, 255)),  # yellow
    ("A3 below",          (220, 120, 200, 140), (0, 255, 0)),    # green
    ("A1 bottom-left",    (150, 300, 160, 160), (255, 0, 0)),    # blue
    ("A2 bottom-right",   (330, 300, 160, 160), (0, 0, 255)),    # red
    ("zhuazi",            (210, 300, 220, 160), (255, 0, 255)),  # purple
]


class RoiOverlayNode(Node):
    def __init__(self):
        super().__init__('roi_overlay_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.cb, 10)
        self._logged_size = False
        self.get_logger().info(f'ROI overlay 订阅 {image_topic}')

    def cb(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        if not self._logged_size:
            h, w = img.shape[:2]
            self.get_logger().info(f'图像尺寸: {w}x{h} (ROI 按 640x480 设计)')
            if w != 640 or h != 480:
                self.get_logger().warn('图像分辨率不是 640x480，ROI 位置可能偏移！')
            self._logged_size = True

        for name, (x, y, w, h), color in ROIS:
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(img, name, (x, max(y - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 1, cv2.LINE_AA)

        cv2.imshow('ROI overlay', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RoiOverlayNode()
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
