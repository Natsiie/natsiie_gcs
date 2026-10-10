#!/usr/bin/env python3
"""圆环 + 圆环内颜色（数字）识别（提取自 zongchengxu20250724chusai.py）。

方法：
  1. 灰度 + 高斯模糊后 HoughCircles 检测圆环；
  2. 对每个圆环画圆形掩膜，在 HSV 空间统计红/绿/蓝面积，取最大者作为环内颜色；
  3. 颜色编号：1=红 2=绿 3=蓝。

用法：
  python3 ring_number_detect.py --device /dev/camera_upper --flip-code 0 --show
操作：
  实时显示检测结果；s 保存当前帧到 ring_number_snapshot.jpg；q 退出。
"""
import argparse

import cv2
import numpy as np

# 红/绿/蓝 HSV 阈值（来自源文件）
COLOR_THRESHOLDS = {
    1: [([0, 50, 50], [10, 255, 255]), ([160, 50, 50], [180, 255, 255])],  # 红
    2: [([30, 40, 40], [89, 255, 255])],                                   # 绿
    3: [([90, 50, 90], [140, 255, 255])],                                  # 蓝
}
COLOR_NAMES = {1: "红", 2: "绿", 3: "蓝"}
COLOR_BGR = {1: (0, 0, 255), 2: (0, 255, 0), 3: (255, 0, 0)}


def detect_circles_and_colors(frame, min_radius=35, max_radius=45,
                              param1=25, param2=25, min_dist=20):
    """检测圆环并识别环内颜色。返回 [(x, y, radius, color_id_or_None), ...]。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1, minDist=min_dist,
        param1=param1, param2=param2,
        minRadius=min_radius, maxRadius=max_radius)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    results = []
    if circles is None:
        return results

    circles = np.round(circles[0, :]).astype("int")
    for (x, y, r) in circles:
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.circle(mask, (x, y), int(1.4 * r), 255, -1)

        areas = {}
        for color_id, ranges in COLOR_THRESHOLDS.items():
            color_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                color_mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
            areas[color_id] = cv2.countNonZero(cv2.bitwise_and(color_mask, mask))

        best = max(areas, key=areas.get)
        if areas[best] == 0:
            best = None
        results.append((int(x), int(y), int(r), best))
    return results


def draw_results(frame, results):
    for (x, y, r, color_id) in results:
        cv2.circle(frame, (x, y), r, (0, 255, 255), 2)
        label = f"{color_id}({COLOR_NAMES[color_id]})" if color_id else "?"
        color = COLOR_BGR.get(color_id, (0, 0, 0))
        cv2.putText(frame, label, (x - 20, y - r - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    return frame


def main():
    parser = argparse.ArgumentParser(description='圆环 + 环内颜色（数字）识别')
    parser.add_argument('--device', '-d', default='/dev/camera_upper')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--flip-code', type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument('--min-radius', type=int, default=35)
    parser.add_argument('--max-radius', type=int, default=45)
    parser.add_argument('--param1', type=int, default=25)
    parser.add_argument('--param2', type=int, default=25)
    parser.add_argument('--min-dist', type=int, default=20)
    parser.add_argument('--show', action='store_true', help='弹出窗口显示')
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'无法打开相机: {args.device}')
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print('=== 圆环 + 环内数字识别 ===')
    print('s 保存快照；q 退出')
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print('读取帧失败')
                break
            if args.flip_code:
                frame = cv2.flip(frame, args.flip_code)

            results = detect_circles_and_colors(
                frame, args.min_radius, args.max_radius,
                args.param1, args.param2, args.min_dist)

            for (x, y, r, color_id) in results:
                name = COLOR_NAMES[color_id] if color_id else "无"
                print(f'圆环 圆心=({x},{y}) 半径={r} 颜色={color_id}({name})')

            if args.show:
                display = draw_results(frame.copy(), results)
                cv2.imshow('ring_number_detect', display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                elif key in (ord('s'), ord('S')):
                    cv2.imwrite('ring_number_snapshot.jpg', frame)
                    print('已保存 ring_number_snapshot.jpg')
            else:
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
