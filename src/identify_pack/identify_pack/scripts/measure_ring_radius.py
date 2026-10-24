#!/usr/bin/env python3
"""量取画面中圆环的像素半径（用于设置 stacked_ring_detector.json 的半径范围）。

只保留「圆周边缘支持率」足够的圆，避免把纹理/反光误判成圆。
按键：Space 冻结/恢复，[ ] 调 Hough 阈值，- + 调支持率，S 保存，Q 退出。
"""
import argparse
import sys
import os

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stacked_ring_detector import _circle_edge_support  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description='量取圆环像素半径')
    p.add_argument('--device', default='/dev/camera_upper')
    p.add_argument('--min-radius', type=int, default=10)
    p.add_argument('--max-radius', type=int, default=140)
    p.add_argument('--param2', type=int, default=45, help='Hough 累加阈值，越大越严')
    p.add_argument('--min-dist', type=int, default=30, help='圆心最小间距(px)')
    p.add_argument('--min-support', type=float, default=0.55, help='圆周边缘支持率下限')
    p.add_argument('--output', default='ring_radius_annotated.png')
    return p.parse_args()


def main():
    args = parse_args()
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'无法打开相机 {args.device}')
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    frozen = None
    print('Space 冻结/恢复，[ ] 调阈值，- + 调支持率，S 保存，Q 退出')
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        source = frame if frozen is None else frozen
        gray = cv2.cvtColor(source, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 30, 90)
        # 圆环线宽仅 1~2px：先闭运算连接虚线/断边，再用 2x2 开运算去掉孤立噪点。
        # 不用大核腐蚀，否则会把圆环本身抹掉。
        edges = cv2.morphologyEx(
            edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        edges = cv2.morphologyEx(
            edges, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=args.min_dist,
            param1=60, param2=args.param2,
            minRadius=args.min_radius, maxRadius=args.max_radius)

        display = source.copy()
        validated = []
        if circles is not None:
            for raw in circles[0]:
                x, y, r = float(raw[0]), float(raw[1]), float(raw[2])
                support = _circle_edge_support(edges, (x, y), r)
                if support < args.min_support:
                    continue
                if not (5 <= x < display.shape[1] - 5 and 5 <= y < display.shape[0] - 5):
                    continue
                validated.append((r, x, y, support))
        validated.sort(key=lambda item: item[0])
        for index, (r, x, y, support) in enumerate(validated, 1):
            center = (int(round(x)), int(round(y)))
            radius = int(round(r))
            cv2.circle(display, center, radius, (0, 255, 0), 1)
            cv2.drawMarker(display, center, (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
            cv2.putText(display, f'{index}: r={radius} s={support:.2f}',
                        (center[0] - 45, center[1] - radius - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            print(f'  [{index}] 中心=({x:.0f},{y:.0f}) 半径={radius}px '
                  f'直径={2 * radius}px 支持率={support:.2f}')
        cv2.putText(display, f'param2={args.param2} support>={args.min_support:.2f} '
                             f'found={len(validated)}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.imshow('measure_ring_radius', display)

        key = cv2.waitKey(20) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord(' '):
            frozen = None if frozen is not None else source.copy()
            print('已冻结' if frozen is not None else '已恢复')
        elif key == ord('['):
            args.param2 = max(10, args.param2 - 5)
        elif key == ord(']'):
            args.param2 = min(150, args.param2 + 5)
        elif key == ord('-'):
            args.min_support = max(0.1, round(args.min_support - 0.05, 2))
        elif key == ord('='):
            args.min_support = min(0.99, round(args.min_support + 0.05, 2))
        elif key in (ord('s'), ord('S')):
            cv2.imwrite(args.output, display)
            print(f'已保存 {args.output}')
    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
