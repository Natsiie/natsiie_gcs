#!/usr/bin/env python3
"""
颜色识别：参考 zongchengxu20250724chusai.py 的「HSV 阈值 + 圆内面积统计」方案。
针对六种颜色：红、黄、蓝、绿、黑、浅蓝。

核心流程：
  1. BGR -> HSV
  2. 每种颜色用 inRange 生成掩膜（红色双阈值处理 H 环绕）
  3. 形态学去噪（腐蚀 + 膨胀）
  4. 圆形掩膜限定，只统计圆内颜色面积
  5. 面积最大者 = 该圆的颜色

单独运行（调试）：
  python3 color_detect.py --device /dev/camera_upper
  → 打开相机，检测圆，显示各颜色掩膜 + 识别出的颜色
"""
import argparse
import cv2
import numpy as np

# 六种颜色的 HSV 范围（H:0~179, S/V:0~255）
# ⚠️ 需要针对实际颜色现场微调，尤其黄色、黑色、浅蓝
COLOR_RANGES = {
    'red':        [((0, 100, 100), (10, 255, 255)), ((160, 100, 100), (180, 255, 255))],
    'yellow':     [((22, 100, 100), (38, 255, 255))],
    'green':      [((38, 40, 40), (85, 255, 255))],
    'blue':       [((85, 60, 60), (135, 255, 150))],
    'light_blue': [((85, 40, 150), (135, 255, 255))],
    'black':      [((0, 0, 0), (180, 255, 50))],
}

# 每种颜色用于显示的 BGR 颜色（调试可视化）
COLOR_DISPLAY = {
    'red':        (0, 0, 255),
    'yellow':     (0, 255, 255),
    'green':      (0, 255, 0),
    'blue':       (255, 0, 0),
    'light_blue': (255, 255, 0),
    'black':      (128, 128, 128),
}


def build_circle_mask(shape, center, radius):
    """生成圆形掩膜：圆内=255，圆外=0。"""
    mask = np.zeros(shape, np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    return mask


def color_masks(hsv_img):
    """对整幅 HSV 图生成六种颜色的掩膜（含形态学去噪）。返回 {颜色名: 掩膜}。"""
    kernel = np.ones((3, 3), np.uint8)
    masks = {}
    for name, ranges in COLOR_RANGES.items():
        m = np.zeros(hsv_img.shape[:2], np.uint8)
        for lo, hi in ranges:
            m = cv2.bitwise_or(m, cv2.inRange(hsv_img, np.array(lo), np.array(hi)))
        # 形态学去噪（参考原文件：不同颜色可用不同迭代次数，这里先统一 1 次腐蚀 2 次膨胀）
        m = cv2.erode(m, kernel, iterations=1)
        m = cv2.dilate(m, kernel, iterations=2)
        masks[name] = m
    return masks


def detect_color(hsv_img, circle_mask):
    """在圆内统计各颜色面积，返回 (颜色名, 面积, 各颜色面积dict)。"""
    masks = color_masks(hsv_img)
    areas = {}
    for name, m in masks.items():
        areas[name] = int(cv2.countNonZero(cv2.bitwise_and(m, circle_mask)))
    best = max(areas, key=areas.get)
    return best, areas[best], areas


# 点击采样的当前帧（BGR / HSV），用于标定颜色范围
_sample_bgr = None
_sample_hsv = None


def on_mouse(event, x, y, flags, param):
    """鼠标左键点击 → 打印该点的 BGR 和 HSV 值。"""
    if event == cv2.EVENT_LBUTTONDOWN and _sample_hsv is not None:
        h, s, v = _sample_hsv[y, x]
        b, g, r = _sample_bgr[y, x]
        print(f'点击 ({x},{y})  BGR=({int(b)},{int(g)},{int(r)})  H={int(h)} S={int(s)} V={int(v)}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--device', '-d', default='/dev/camera_upper', help='相机设备路径')
    args = p.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        try:
            cap = cv2.VideoCapture(int(args.device))
        except Exception:
            pass
    if not cap.isOpened():
        print(f'无法打开相机: {args.device}')
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    global _sample_bgr, _sample_hsv
    cv2.namedWindow('color_detect')
    cv2.setMouseCallback('color_detect', on_mouse)

    print('颜色识别调试：鼠标点击画面采样 HSV，q=退出')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        _sample_bgr = frame
        _sample_hsv = hsv
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2, 2)

        # 各颜色掩膜（用于显示叠加）
        masks = color_masks(hsv)
        overlay = frame.copy()
        for name, m in masks.items():
            color = COLOR_DISPLAY[name]
            colored = np.zeros_like(frame)
            colored[:] = color
            idx = m > 0
            overlay[idx] = (overlay[idx] * 0.4 + colored[idx] * 0.6).astype(np.uint8)

        # 霍夫圆检测（与 circle_detect 一致）
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=80, param2=36, minRadius=15, maxRadius=40)
        if circles is not None:
            for (cx, cy, r) in np.uint16(np.around(circles[0])):
                center = (int(cx), int(cy))
                radius = int(r)
                cmask = build_circle_mask(gray.shape, center, radius)
                color, area, areas = detect_color(hsv, cmask)
                cv2.circle(frame, center, radius, (255, 255, 255), 2)
                cv2.putText(frame, color, (center[0] - 25, center[1] - radius - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_DISPLAY[color], 2)
                print(f'圆({center}, r={radius}) -> {color} (面积 {area}, {areas})')

        cv2.imshow('color_detect', frame)
        cv2.imshow('color masks', overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
