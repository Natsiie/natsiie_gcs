#!/usr/bin/env python3
"""
ROS2 节点：用 cv2.VideoCapture 打开主机摄像头，并发布到 image_topic（默认 /camera/image_raw）。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        device = self.get_parameter('device').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            # 尝试把 device 当作整数索引再开一次
            try:
                self.cap = cv2.VideoCapture(int(device))
            except Exception:
                pass
        if not self.cap.isOpened():
            self.cap = None
            self.get_logger().error(f'无法打开摄像头: {device}，请检查 camera_device 参数')
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.get_logger().info(f'Publishing {device} -> {image_topic} @ {fps}fps')

        self.br = CvBridge()
        self.pub = self.create_publisher(Image, image_topic, 10)
        self.timer = self.create_timer(1.0 / fps, self.timer_cb)

    def timer_cb(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('读取帧失败', throttle_duration_sec=5.0)
            return
        try:
            msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def destroy_node(self):
        if getattr(self, 'cap', None) is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
