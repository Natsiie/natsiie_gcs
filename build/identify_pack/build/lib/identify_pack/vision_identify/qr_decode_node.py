#!/usr/bin/env python3
"""
ROS2 节点：订阅图像话题，识别 QR 码并发布到 `/QR_Decode` （std_msgs/String）。
可配置参数：图像话题、最小连续一致帧数、是否显示窗口。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import time


class QRDecodeNode(Node):
    def __init__(self):
        super().__init__('qr_decode_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('min_consistent', 3)
        self.declare_parameter('show', False)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.min_consistent = self.get_parameter('min_consistent').get_parameter_value().integer_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value

        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub = self.create_publisher(String, 'QR_Decode', 10)
        self.br = CvBridge()

        self.last_data = None
        self.consistent_count = 0
        self.get_logger().info(f'Listening to image topic: {image_topic}')

    def image_cb(self, msg: Image):
        try:
            cv_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        decoded = decode(gray)
        current = None
        for obj in decoded:
            try:
                txt = obj.data.decode('utf-8').strip()
            except Exception:
                txt = None
            if txt:
                current = txt
                break

        if current:
            if current == self.last_data:
                self.consistent_count += 1
            else:
                self.last_data = current
                self.consistent_count = 1

            self.get_logger().info(f'Detected QR: {current} (count={self.consistent_count})')

            if self.consistent_count >= self.min_consistent:
                msg = String()
                msg.data = current
                self.pub.publish(msg)
                self.get_logger().info(f'Published QR_Decode: {current}')
                # reset to avoid duplicate continual publishes
                self.consistent_count = 0
                self.last_data = None

        if self.show:
            # draw small overlay if detected
            if current:
                cv2.putText(cv_img, current, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
            cv2.imshow('qr_decode', cv_img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = QRDecodeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
