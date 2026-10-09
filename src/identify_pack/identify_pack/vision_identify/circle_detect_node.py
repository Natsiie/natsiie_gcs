#!/usr/bin/env python3
"""
ROS2 节点：检测多个同心圆环目标，锁定每个目标的最内层圆，识别圆内数字。

流程：灰度 -> 高斯模糊 -> HoughCircles -> 分组(同心圆)
      -> 多目标跟踪(按位置匹配 + 各自 EMA 锁定) -> 裁剪圆内 ROI -> 数字模板匹配

发布：
  - /circle_detect  每个目标内层圆 圆心+半径，格式 ``f'{x:04}{y:04}{r:01}'``
  - /Order          数字识别结果，格式 ``idx:digit/...``（如 ``0:3/1:2/2:1``）

多目标：画面里同时有 N 个同心圆环时，各自独立锁定，不会在目标之间滑动。

颜色识别 / 转盘运动判定不在此节点，见 color_detect_node.py。
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class CircleDetectNode(Node):
    def __init__(self):
        super().__init__('circle_detect_node')

        # 圆检测参数
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('param1', 80)         # HoughCircles 内部 Canny 高阈值
        self.declare_parameter('param2', 36)         # 圆心累加器阈值
        self.declare_parameter('min_radius', 15)     # 最小半径
        self.declare_parameter('max_radius', 40)     # 最大半径
        self.declare_parameter('min_dist', 20)       # 圆心最小间距
        self.declare_parameter('center_dist', 10)    # 同心圆分组阈值
        self.declare_parameter('close_kernel', 0)    # 形态学核大小（焊虚线间隙，0=关闭）
        self.declare_parameter('morph_mode', 'open') # 'open'=深色线(默认)，'close'=浅色线
        self.declare_parameter('smooth_alpha', 0.15)  # 时间平滑系数：0~1，越小越"锁"
        self.declare_parameter('match_dist', 20)      # 多目标匹配阈值（< 相邻目标间距）
        self.declare_parameter('max_miss', 10)        # 目标失配多少帧后移除
        self.declare_parameter('min_hits', 3)         # 连续检测多少帧才确认目标（防闪烁）
        self.declare_parameter('min_support', 0.6)    # 圆周边缘覆盖率阈值（排除圆弧假圆）
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
        self.match_dist = self.get_parameter('match_dist').get_parameter_value().integer_value
        self.max_miss = self.get_parameter('max_miss').get_parameter_value().integer_value
        self.min_hits = self.get_parameter('min_hits').get_parameter_value().integer_value
        self.min_support = self.get_parameter('min_support').get_parameter_value().double_value
        self.crop_fraction = self.get_parameter('crop_fraction').get_parameter_value().double_value
        self.digit_size = self.get_parameter('digit_size').get_parameter_value().integer_value
        self.min_score = self.get_parameter('min_score').get_parameter_value().double_value
        self.templates_dir = self.get_parameter('templates_dir').get_parameter_value().string_value
        self.recognize = self.get_parameter('recognize').get_parameter_value().bool_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value
        self.show_edges = self.get_parameter('show_edges').get_parameter_value().bool_value

        # 多目标跟踪状态：每个目标 {cx, cy, r, miss}
        self.targets = []

        # 数字模板 {数字字符: 二值图}
        self.templates = {}
        self._load_templates()

        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)
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
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
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

    def update_targets(self, detected):
        """把本帧检测到的各组匹配到跟踪目标，各自 EMA 平滑。detected: [(cx,cy,r), ...]"""
        a = self.smooth_alpha
        matched = set()
        for (cx, cy, r) in detected:
            best_i, best_d = None, float('inf')
            for i, t in enumerate(self.targets):
                if i in matched:
                    continue
                d = np.hypot(cx - t['cx'], cy - t['cy'])
                if d < best_d:
                    best_d, best_i = d, i
            if best_i is not None and best_d < self.match_dist:
                t = self.targets[best_i]
                t['cx'] = (1.0 - a) * t['cx'] + a * cx
                t['cy'] = (1.0 - a) * t['cy'] + a * cy
                t['r'] = (1.0 - a) * t['r'] + a * r
                t['hits'] += 1
                t['miss'] = 0
                matched.add(best_i)
            else:
                self.targets.append({'cx': cx, 'cy': cy, 'r': r, 'hits': 1, 'miss': 0})
                matched.add(len(self.targets) - 1)
        # 失配的目标：miss+1、hits 清零；超过 max_miss 则移除
        for i, t in enumerate(self.targets):
            if i not in matched:
                t['miss'] += 1
                t['hits'] = 0
        self.targets = [t for t in self.targets if t['miss'] <= self.max_miss]

    def _edge_support(self, edge_map, cx, cy, r):
        """圆周边缘覆盖率：完整圆≈1.0，圆弧 <1.0。采样 36 点看 3x3 邻域是否有边缘。"""
        n = 36
        h, w = edge_map.shape
        cnt = 0
        for k in range(n):
            th = 2.0 * np.pi * k / n
            x = int(round(cx + r * np.cos(th)))
            y = int(round(cy + r * np.sin(th)))
            if 0 <= x < w and 0 <= y < h:
                x0, x1 = max(0, x - 1), min(w, x + 2)
                y0, y1 = max(0, y - 1), min(h, y + 2)
                if np.any(edge_map[y0:y1, x0:x1] > 0):
                    cnt += 1
        return cnt / n

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
        # ②.5 形态学（可选）
        if self.close_kernel > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.close_kernel, self.close_kernel))
            op = cv2.MORPH_OPEN if self.morph_mode == 'open' else cv2.MORPH_CLOSE
            gray = cv2.morphologyEx(gray, op, k)
        # ②.6 Canny 边缘（用于圆周覆盖率校验 + 调试显示）
        edges = cv2.Canny(gray, max(int(self.param1 / 2), 1), self.param1)
        # ③ 霍夫圆检测
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1, self.min_dist,
            param1=self.param1, param2=self.param2,
            minRadius=self.min_radius, maxRadius=self.max_radius
        )

        if circles is not None:
            circles = circles[0]
            # ③.5 圆周覆盖率过滤：排除圆弧假圆（圆周上边缘覆盖不足的）
            kept = [c for c in circles
                    if self._edge_support(edges, float(c[0]), float(c[1]), float(c[2])) >= self.min_support]
            circles = np.array(kept, dtype=np.float32) if kept else None

        if circles is None:
            self.get_logger().info('未检测到圆', throttle_duration_sec=1.0)
            if self.show:
                cv2.imshow('circle_detect', img)
                cv2.waitKey(1)
            return

        # ④ 分组（同心圆归一组）
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

        # ⑤ 同心多环(>=2 个圆) = 数字圆环
        digit_groups = [g for g in groups if len(g['rs']) >= 2]

        # 数字圆环：平均圆心 + 最内层半径（最小半径）
        detected = []
        for g in digit_groups:
            cx = float(np.mean(g['xs']))
            cy = float(np.mean(g['ys']))
            r = float(min(g['rs']))
            detected.append((cx, cy, r))

        # ⑥ 多目标匹配 + 各自 EMA 锁定
        self.update_targets(detected)

        # 按 x 坐标排序，保证索引稳定（0=最左，1=中间，2=最右）
        self.targets.sort(key=lambda t: t['cx'])

        # 只输出连续检测达到 min_hits 帧的已确认目标（防闪烁）
        confirmed = [t for t in self.targets if t['hits'] >= self.min_hits]

        # 用干净副本裁剪数字，避免画上去的圆/点污染识别
        img_clean = img.copy()

        # ⑦ 输出每个目标：圆心 + 半径 + 数字
        first_digit_img = None
        order_parts = []   # 收集各目标的 "idx:digit"，最后合并成一条发布
        for idx, t in enumerate(confirmed):
            center = (int(round(t['cx'])), int(round(t['cy'])))
            radius = int(round(t['r']))
            self.get_logger().info(f'[{idx}] 内层圆: 圆心 {center}, 半径 {radius}',
                                   throttle_duration_sec=1.0)

            if self.show:
                cv2.circle(img, center, radius, (0, 255, 0), 2)
                cv2.circle(img, center, 2, (0, 0, 255), 3)

            # 发布每个圆（circle_detect）
            out_c = String()
            out_c.data = f'{center[0]:04}{center[1]:04}{radius:01}'
            self.pub.publish(out_c)

            # 数字识别
            if self.recognize:
                half = int(radius * self.crop_fraction)
                x1 = max(0, center[0] - half)
                y1 = max(0, center[1] - half)
                x2 = min(img.shape[1], center[0] + half)
                y2 = min(img.shape[0], center[1] + half)
                if x2 > x1 and y2 > y1:
                    d_img = self.preprocess_digit(img_clean[y1:y2, x1:x2])
                    if d_img is not None:
                        if idx == 0:
                            first_digit_img = d_img
                        digit, score = self.recognize_digit(d_img)
                        if digit is not None and score >= self.min_score:
                            self.get_logger().info(f'[{idx}] 数字 {digit} (置信度 {score:.2f})',
                                                   throttle_duration_sec=0.5)
                            order_parts.append(f'{idx}:{digit}')

        # 发布合并的 /Order（如 "0:3/1:2/2:1"）
        if order_parts:
            out_order = String()
            out_order.data = '/'.join(order_parts)
            self.pub_order.publish(out_order)

        # ⑧ 统一显示
        if self.show_edges and edges is not None:
            cv2.imshow('edges (Canny)', edges)
        if self.show:
            cv2.imshow('circle_detect', img)
        if first_digit_img is not None:
            cv2.imshow('digit', first_digit_img)

        key = cv2.waitKey(1) & 0xFF

        # ⑨ 模板采集：按 0~9 把第一个目标的数字存为模板
        if first_digit_img is not None and ord('0') <= key <= ord('9'):
            d = chr(key)
            path = os.path.join(self.templates_dir, f'digit_{d}.png')
            cv2.imwrite(path, first_digit_img)
            self.templates[d] = first_digit_img.copy()
            self.get_logger().info(f'已保存数字模板 {d} -> {path}')


def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectNode()
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
