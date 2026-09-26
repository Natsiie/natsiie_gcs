#!/usr/bin/env python3
"""
ROS2 节点：订阅图像话题，做霍夫圆检测，针对同心圆锁定最内层圆（稳定圆心 + 半径）。

流程：灰度 -> 高斯模糊 -> 形态学焊虚线 -> HoughCircles -> 分组(同心圆)
      -> 取最内层圆(最小半径) -> 时间平滑(EMA)锁定圆心/半径
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
        self.declare_parameter('param2', 50)         # 圆心累加器阈值
        self.declare_parameter('min_radius', 20)     # 最小半径
        self.declare_parameter('max_radius', 60)     # 最大半径
        self.declare_parameter('min_dist', 20)       # 圆心最小间距
        self.declare_parameter('center_dist', 10)    # 同心圆分组阈值
        self.declare_parameter('close_kernel', 15)   # 形态学核大小（焊虚线间隙，0=关闭）
        self.declare_parameter('morph_mode', 'open') # 'open'=深色线(默认)，'close'=浅色线
        self.declare_parameter('smooth_alpha', 0.3)  # 时间平滑系数：0~1，越小越"锁"，1=不平滑
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
        self.morph_mode = self.get_parameter('morph_mode').get_parameter_value().string_value
        self.smooth_alpha = self.get_parameter('smooth_alpha').get_parameter_value().double_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value
        self.show_edges = self.get_parameter('show_edges').get_parameter_value().bool_value

        # 平滑状态（None = 尚未初始化）
        self.s_cx = None
        self.s_cy = None
        self.s_r = None

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
        # ② 高斯模糊（去噪）
        gray = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        # ②.5 形态学：把虚线的间隙"焊"上（深色线用开运算 OPEN）
        if self.close_kernel > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.close_kernel, self.close_kernel))
            op = cv2.MORPH_OPEN if self.morph_mode == 'open' else cv2.MORPH_CLOSE
            gray = cv2.morphologyEx(gray, op, k)
        # ②.6 调试：显式 Canny 边缘
        if self.show_edges:
            edges = cv2.Canny(gray, max(int(self.param1 / 2), 1), self.param1)
            cv2.imshow('edges (Canny)', edges)
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

        circles = circles[0]  # (N, 3) 每行 [x, y, r]，保留浮点亚像素精度

        # ④ 按圆心距离分组（同心圆归一组）
        groups = []
        for x, y, r in circles:
            placed = False
            for g in groups:
                if np.hypot(x - g['cx'], y - g['cy']) < self.center_dist:
                    g['xs'].append(float(x))
                    g['ys'].append(float(y))
                    g['rs'].append(float(r))
                    g['cx'] = float(np.mean(g['xs']))
                    g['cy'] = float(np.mean(g['ys']))
                    placed = True
                    break
            if not placed:
                groups.append({'xs': [float(x)], 'ys': [float(y)], 'rs': [float(r)],
                               'cx': float(x), 'cy': float(y)})

        # ⑤ 选环最多的那组（真正的同心圆组），输出平均圆心 + 最内层半径(最小半径)
        if groups:
            g = max(groups, key=lambda gg: len(gg['xs']))
            cx = float(np.mean(g['xs']))
            cy = float(np.mean(g['ys']))
            r = float(min(g['rs']))   # 最内层圆 = 最小半径
            n = len(g['xs'])

            # ⑥ 时间平滑（EMA）锁定圆心/半径
            a = self.smooth_alpha
            if self.s_cx is None:
                self.s_cx, self.s_cy, self.s_r = cx, cy, r
            else:
                self.s_cx = (1.0 - a) * self.s_cx + a * cx
                self.s_cy = (1.0 - a) * self.s_cy + a * cy
                self.s_r = (1.0 - a) * self.s_r + a * r

            center = (int(round(self.s_cx)), int(round(self.s_cy)))
            radius = int(round(self.s_r))
            self.get_logger().info(
                f'内层圆锁定: 圆心 {center}, 半径 {radius} '
                f'(原始 {cx:.1f},{cy:.1f},r={r:.1f}, {n}环)')

            if self.show:
                # 蓝细线：原始检出的每个环；绿圈+红点：锁定后的内层圆
                for k in range(n):
                    cv2.circle(img, (int(round(g['xs'][k])), int(round(g['ys'][k]))),
                               int(round(g['rs'][k])), (255, 0, 0), 1)
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
