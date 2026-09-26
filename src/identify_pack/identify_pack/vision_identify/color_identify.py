#!/usr/bin/env python3
"""
ROS2 节点：检测同心圆并锁定最内层圆，识别圆内单个印刷数字，发布到 /Order。

流程：灰度 -> 高斯模糊 -> 形态学焊虚线 -> HoughCircles -> 分组
      -> 最内层圆锁定(EMA) -> 裁剪圆内 ROI -> 数字二值化 -> 模板匹配 -> 发布 /Order

模板采集：运行后把数字对准相机，在窗口上按 0~9 把当前数字存为对应模板。
"""
import os
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

        # 圆检测参数
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('param1', 80)         # HoughCircles 内部 Canny 高阈值
        self.declare_parameter('param2', 50)         # 圆心累加器阈值
        self.declare_parameter('min_radius', 20)     # 最小半径
        self.declare_parameter('max_radius', 60)     # 最大半径
        self.declare_parameter('min_dist', 20)       # 圆心最小间距
        self.declare_parameter('center_dist', 10)    # 同心圆分组阈值
        self.declare_parameter('close_kernel', 15)   # 形态学核大小（焊虚线间隙，0=关闭）
        self.declare_parameter('morph_mode', 'open') # 'open'=深色线(默认)，'close'=浅色线
        self.declare_parameter('smooth_alpha', 0.3)  # 时间平滑系数：0~1，越小越"锁"
        # 数字识别参数
        self.declare_parameter('crop_fraction', 0.75)  # 裁剪圆内中心区域的比例(0~1)
        self.declare_parameter('digit_size', 40)       # 数字归一化尺寸
        self.declare_parameter('min_score', 0.5)       # 模板匹配最低置信度，低于则不发布
        self.declare_parameter('templates_dir', os.path.expanduser('~/.ros/digit_templates'))
        self.declare_parameter('recognize', True)      # 是否识别数字
        self.declare_parameter('show', True)           # 是否弹窗显示
        self.declare_parameter('show_edges', True)     # 是否显示 Canny 边缘（调试）

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
        self.crop_fraction = self.get_parameter('crop_fraction').get_parameter_value().double_value
        self.digit_size = self.get_parameter('digit_size').get_parameter_value().integer_value
        self.min_score = self.get_parameter('min_score').get_parameter_value().double_value
        self.templates_dir = self.get_parameter('templates_dir').get_parameter_value().string_value
        self.recognize = self.get_parameter('recognize').get_parameter_value().bool_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value
        self.show_edges = self.get_parameter('show_edges').get_parameter_value().bool_value

        # 平滑状态（None = 尚未初始化）
        self.s_cx = None
        self.s_cy = None
        self.s_r = None

        # 数字模板 {数字字符: 二值图}
        self.templates = {}
        self._load_templates()

        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub = self.create_publisher(String, 'circle_detect', 10)
        self.pub_order = self.create_publisher(String, 'Order', 10)
        self.get_logger().info(f'圆检测订阅 {image_topic}')

    def _load_templates(self):
        os.makedirs(self.templates_dir, exist_ok=True)
        if not os.listdir(self.templates_dir):
            self.get_logger().warn(f'模板目录为空 {self.templates_dir}，请先按 0~9 采集模板')
            return
        for f in os.listdir(self.templates_dir):
            if f.startswith('digit_') and f.endswith('.png'):
                d = f[len('digit_'):-len('.png')]
                img = cv2.imread(os.path.join(self.templates_dir, f), cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    self.templates[d] = cv2.resize(img, (self.digit_size, self.digit_size))
        self.get_logger().info(f'已加载 {len(self.templates)} 个数字模板')

    def preprocess_digit(self, roi_bgr):
        """把圆内 ROI 转成归一化二值数字图。失败返回 None。"""
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        # OTSU 二值化（反转：深色数字变白、背景变黑）
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        # 找最大连通域（数字本体）
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        pad = int(0.15 * max(w, h)) + 2
        x1 = max(0, x - pad)
        y1 = max(0, y - pad)
        x2 = min(binary.shape[1], x + w + pad)
        y2 = min(binary.shape[0], y + h + pad)
        digit = binary[y1:y2, x1:x2]
        return cv2.resize(digit, (self.digit_size, self.digit_size), interpolation=cv2.INTER_AREA)

    def recognize_digit(self, digit_img):
        """模板匹配，返回 (数字, 置信度)。无模板或无匹配返回 (None, 0)。"""
        if not self.templates:
            return None, 0.0
        best_d, best_s = None, -1.0
        for d, tpl in self.templates.items():
            res = cv2.matchTemplate(digit_img, tpl, cv2.TM_CCOEFF_NORMED)
            score = float(res[0][0])
            if score > best_s:
                best_d, best_s = d, score
        return best_d, best_s

    def image_cb(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # ① 灰度化
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # ② 高斯模糊
        gray = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        # ②.5 形态学焊虚线
        if self.close_kernel > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.close_kernel, self.close_kernel))
            op = cv2.MORPH_OPEN if self.morph_mode == 'open' else cv2.MORPH_CLOSE
            gray = cv2.morphologyEx(gray, op, k)
        edges = None
        if self.show_edges:
            edges = cv2.Canny(gray, max(int(self.param1 / 2), 1), self.param1)
        # ③ 霍夫圆检测
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1, self.min_dist,
            param1=self.param1, param2=self.param2,
            minRadius=self.min_radius, maxRadius=self.max_radius
        )

        center = None
        radius = None
        digit_img = None

        if circles is not None:
            circles = circles[0]  # (N, 3) 浮点
            # ④ 分组（同心圆）
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
            # ⑤ 选最大组，取平均圆心 + 最内层半径(最小半径)
            g = max(groups, key=lambda gg: len(gg['xs']))
            cx = float(np.mean(g['xs']))
            cy = float(np.mean(g['ys']))
            r = float(min(g['rs']))
            n = len(g['xs'])
            # ⑥ EMA 时间平滑
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
                f'(原始 {cx:.1f},{cy:.1f},r={r:.1f}, {n}环)', throttle_duration_sec=1.0)

            if self.show:
                for k in range(n):
                    cv2.circle(img, (int(round(g['xs'][k])), int(round(g['ys'][k]))),
                               int(round(g['rs'][k])), (255, 0, 0), 1)
                cv2.circle(img, center, radius, (0, 255, 0), 2)
                cv2.circle(img, center, 2, (0, 0, 255), 3)

            # ⑦ 数字识别：裁剪圆内中心区域
            if self.recognize:
                half = int(radius * self.crop_fraction)
                x1 = max(0, center[0] - half)
                y1 = max(0, center[1] - half)
                x2 = min(img.shape[1], center[0] + half)
                y2 = min(img.shape[0], center[1] + half)
                if x2 > x1 and y2 > y1:
                    digit_img = self.preprocess_digit(img[y1:y2, x1:x2])
        else:
            self.get_logger().info('未检测到圆', throttle_duration_sec=1.0)

        # ⑧ 统一显示
        if self.show_edges and edges is not None:
            cv2.imshow('edges (Canny)', edges)
        if self.show:
            cv2.imshow('circle_detect', img)
        if digit_img is not None:
            cv2.imshow('digit', digit_img)

        key = cv2.waitKey(1) & 0xFF

        # ⑨ 模板采集：按 0~9 把当前数字存为对应模板
        if digit_img is not None and ord('0') <= key <= ord('9'):
            d = chr(key)
            path = os.path.join(self.templates_dir, f'digit_{d}.png')
            cv2.imwrite(path, digit_img)
            self.templates[d] = digit_img.copy()
            self.get_logger().info(f'已保存数字模板 {d} -> {path}')

        # ⑩ 识别 + 发布 /Order
        if digit_img is not None:
            digit, score = self.recognize_digit(digit_img)
            if digit is not None and score >= self.min_score:
                self.get_logger().info(f'识别数字: {digit} (置信度 {score:.2f})', throttle_duration_sec=0.5)
                out = String()
                out.data = digit
                self.pub_order.publish(out)


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
