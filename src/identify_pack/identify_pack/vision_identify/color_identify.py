#!/usr/bin/env python3
"""
ROS2 节点：订阅图像话题，做初步的霍夫圆检测，输出圆心 + 半径。

检测思路提取自 zongchengxu20250724chusai.py 的 run_wukuaiyuanxin_* 函数：
    ① 灰度化  ->  ② 高斯模糊  ->  ③ HoughCircles(HOUGH_GRADIENT)  ->  ④ 圆心 + 半径
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorIdentifyNode(Node):
    def __init__(self):
        super().__init__('color_identify_node')

        # 可调参数（默认值与原始文件的 circle_params 一致）
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('param1', 25)      # HoughCircles 内部 Canny 高阈值
        self.declare_parameter('param2', 25)      # 圆心累加器阈值
        self.declare_parameter('min_radius', 35)  # 最小半径
        self.declare_parameter('max_radius', 45)  # 最大半径
        self.declare_parameter('min_dist', 20)    # 圆心最小间距
        self.declare_parameter('show', True)      # 是否弹窗显示

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.param1 = self.get_parameter('param1').get_parameter_value().integer_value
        self.param2 = self.get_parameter('param2').get_parameter_value().integer_value
        self.min_radius = self.get_parameter('min_radius').get_parameter_value().integer_value
        self.max_radius = self.get_parameter('max_radius').get_parameter_value().integer_value
        self.min_dist = self.get_parameter('min_dist').get_parameter_value().integer_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value

        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub = self.create_publisher(String, 'circle_detect', 10)
        self.get_logger().info(f'霍夫圆检测订阅 {image_topic}')

    def image_cb(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # ① 灰度化
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # ② 高斯模糊（去噪，提升霍夫圆稳定性）
        gray = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        # ③ 霍夫圆检测
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1, self.min_dist,
            param1=self.param1, param2=self.param2,
            minRadius=self.min_radius, maxRadius=self.max_radius
        )

        if circles is None:
            self.get_logger().info('未检测到圆', throttle_duration_sec=1.0)
            if self.show:
                cv2.imshow('circle_detect', img)
                cv2.waitKey(1)
            return

        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (int(i[0]), int(i[1]))
            radius = int(i[2])
            self.get_logger().info(f'检测到圆: 圆心 {center}, 半径 {radius}')

            if self.show:
                # 画圆轮廓 + 圆心
                cv2.circle(img, center, radius, (0, 255, 0), 2)
                cv2.circle(img, center, 2, (0, 0, 255), 3)

            # 发布结果（与原始格式一致：圆心X4位 + 圆心Y4位 + 半径1位）
            out = String()
            out.data = f'{center[0]:04}{center[1]:04}{radius:01}'
            self.pub.publish(out)

        if self.show:
            cv2.imshow('circle_detect', img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorIdentifyNode()
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
