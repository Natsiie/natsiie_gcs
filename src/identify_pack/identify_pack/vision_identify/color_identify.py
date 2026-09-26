#!/usr/bin/env python3
"""
ROS2 节点：订阅图像话题，做霍夫圆检测，针对同心圆输出稳定的共同圆心 + 平均半径。

同心圆思路：霍夫会检出多个同心环（半径不同、圆心几乎相同），
按圆心距离分组后组内平均，得到比单个圆更稳定的共同圆心。
    ① 灰度化 -> ② 高斯模糊 -> ③ HoughCircles -> ④ 分组 -> ⑤ 组内平均圆心/半径
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

        # 可调参数
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('param1', 80)         # HoughCircles 内部 Canny 高阈值
        self.declare_parameter('param2', 50)         # 圆心累加器阈值（调低多检出几个环）
        self.declare_parameter('min_radius', 20)     # 最小半径（放宽，覆盖内环）
        self.declare_parameter('max_radius', 60)     # 最大半径（放宽，覆盖外环）
        self.declare_parameter('min_dist', 20)       # 圆心最小间距
        self.declare_parameter('center_dist', 10)    # 同心圆分组阈值（圆心距离 < 该值算同一组）
        self.declare_parameter('close_kernel', 7)    # 形态学闭运算核大小（焊虚线间隙，0=关闭）
        self.declare_parameter('show', True)         # 是否弹窗显示
        self.declare_parameter('show_edges', True)   # 是否显示 Canny 边缘（调试）

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.param1 = self.get_parameter('param1').get_parameter_value().integer_value
        self.param2 = self.get_parameter('param2').get_parameter_value().integer_value
        self.min_radius = self.get_parameter('min_radius').get_parameter_value().integer_value
        self.max_radius = self.get_parameter('max_radius').get_parameter_value().integer_value
        self.min_dist = self.get_parameter('min_dist').get_parameter_value().integer_value
        self.center_dist = self.get_parameter('center_dist').get_parameter_value().integer_value
        self.close_kernel = self.get_parameter('close_kernel').get_parameter_value().integer_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value
        self.show_edges = self.get_parameter('show_edges').get_parameter_value().bool_value

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
        # ②.5 形态学闭运算：把虚线的间隙"焊"上（核要大于虚线间隙、小于实虚线间距）
        if self.close_kernel > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.close_kernel, self.close_kernel))
            gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, k)
        # ②.6 调试：显式 Canny 边缘（闭运算之后，看虚线是否被连上）
        if self.show_edges:
            edges = cv2.Canny(gray, max(int(self.param1 / 2), 1), self.param1)
            cv2.imshow('edges (Canny, 闭运算后)', edges)
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

        circles = np.uint16(np.around(circles))[0]  # (N, 3) 每行 [x, y, r]

        # ④ 按圆心距离分组（同心圆归一组），组内平均圆心
        groups = []
        for x, y, r in circles:
            x, y, r = int(x), int(y), int(r)
            placed = False
            for g in groups:
                if np.hypot(x - g['cx'], y - g['cy']) < self.center_dist:
                    g['xs'].append(x)
                    g['ys'].append(y)
                    g['rs'].append(r)
                    g['cx'] = float(np.mean(g['xs']))
                    g['cy'] = float(np.mean(g['ys']))
                    placed = True
                    break
            if not placed:
                groups.append({'xs': [x], 'ys': [y], 'rs': [r],
                               'cx': float(x), 'cy': float(y)})

        # ⑤ 每组输出稳定的共同圆心 + 平均半径
        for g in groups:
            cx = float(np.mean(g['xs']))
            cy = float(np.mean(g['ys']))
            r = float(np.mean(g['rs']))
            center = (int(round(cx)), int(round(cy)))
            radius = int(round(r))
            n = len(g['xs'])
            self.get_logger().info(f'检测到圆组({n} 个同心环): 共同圆心 {center}, 平均半径 {radius}')

            if self.show:
                # 蓝细线：原始检出的每个环；绿圈+红点：平均后的共同圆心+半径
                for k in range(n):
                    cv2.circle(img, (g['xs'][k], g['ys'][k]), g['rs'][k], (255, 0, 0), 1)
                cv2.circle(img, center, radius, (0, 255, 0), 2)
                cv2.circle(img, center, 2, (0, 0, 255), 3)

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
