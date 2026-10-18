#!/usr/bin/env python3
"""3x3 对称圆点板自动外参标定（findCirclesGrid 自动检圆心）。

自动检测 9 个圆心（亚像素），配已知 XY 解外参，省去手动点击。
XY = (origin_x + c*spacing, origin_y + r*spacing)，(c,r) 为列/行索引，左下角为 (0,0)。

板必须 3x3 对称、方向固定摆放（对称板有 180° 歧义，摆反会导致 XY 映射翻转）。

用法：
  python3 calibrate_camera_extrinsics_auto.py \
    --device /dev/camera_upper \
    --matrix camera_matrix.npy --distortion dist_coeffs.npy \
    --plane-z-mm -166 \
    --spacing-mm 77 --origin-x-mm <X0> --origin-y-mm <Y0> \
    --output camera_extrinsics_auto.json

按键：S 解算保存，Q/Esc 退出。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

try:
    from .camera_geometry import estimate_camera_transform, load_intrinsics
    from .calibrate_camera_extrinsics import build_result
except ImportError:
    from camera_geometry import estimate_camera_transform, load_intrinsics
    from calibrate_camera_extrinsics import build_result


def build_blob_detector():
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 10
    params.maxThreshold = 220
    params.filterByArea = True
    params.minArea = 40
    params.maxArea = 3000
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.filterByConvexity = True
    params.minConvexity = 0.8
    params.filterByInertia = True
    params.minInertiaRatio = 0.7
    return cv2.SimpleBlobDetector_create(params)


def detect_grid(frame, blob_detector):
    """检测 3x3 圆心，返回按 (行,列) 排序的 3x3 网格（图上从上到下、从左到右）。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, centers = cv2.findCirclesGrid(
        gray, (3, 3), flags=cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector=blob_detector)
    if not ret:
        return None
    centers = centers.reshape(-1, 2)
    # 按 y（行）再 x（列）排序成 3 行
    ordered = sorted(centers, key=lambda p: (float(p[1]), float(p[0])))
    return [ordered[i * 3:(i + 1) * 3] for i in range(3)]


def draw_grid(frame, grid):
    if not grid:
        cv2.putText(frame, '未检出 9 圆', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return frame
    for r, row in enumerate(grid):
        for c, (x, y) in enumerate(row):
            center = (int(round(x)), int(round(y)))
            cv2.circle(frame, center, 12, (0, 255, 0), 2)
            # 图上从上到下是 r=2..0，从左到右是 c=0..2
            board_r = 2 - r
            cv2.putText(frame, f'({c},{board_r})', (center[0] - 20, center[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    return frame


def parse_args():
    parser = argparse.ArgumentParser(description='3x3 圆点板自动外参标定')
    parser.add_argument('--device', default='/dev/camera_upper')
    parser.add_argument('--matrix', type=Path, default=Path('camera_matrix.npy'))
    parser.add_argument('--distortion', type=Path, default=Path('dist_coeffs.npy'))
    parser.add_argument('--plane-z-mm', required=True, type=float)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--spacing-mm', required=True, type=float,
                        help='实际圆间距（mm），打印后量出')
    parser.add_argument('--origin-x-mm', required=True, type=float,
                        help='左下角圆相对转轴的 X（mm）')
    parser.add_argument('--origin-y-mm', required=True, type=float,
                        help='左下角圆相对转轴的 Y（mm）')
    parser.add_argument('--output', type=Path, default=Path('camera_extrinsics_auto.json'))
    return parser.parse_args()


def main():
    args = parse_args()
    matrix, distortion = load_intrinsics(args.matrix, args.distortion)
    detector = build_blob_detector()

    device = int(args.device) if args.device.isdecimal() else args.device
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'无法打开相机：{args.device}')
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print('对准 3x3 圆点板，画面显示 9 个圆与 (列,行) 标签；确认映射对后按 S 解算，Q 退出。')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('读取帧失败')
                continue
            grid = detect_grid(frame, detector)
            display = draw_grid(frame.copy(), grid)
            cv2.imshow('auto_extrinsics', display)

            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in (ord('s'), ord('S')):
                if not grid:
                    print('未检出 9 圆，不能解算')
                    continue
                samples = []
                for r, row in enumerate(grid):
                    board_r = 2 - r  # 图上第 r 行 = 板上第 (2-r) 行
                    for c, (x, y) in enumerate(row):
                        samples.append({
                            'pixel': [float(x), float(y)],
                            'xy_mm': [args.origin_x_mm + c * args.spacing_mm,
                                      args.origin_y_mm + board_r * args.spacing_mm],
                        })
                result = build_result(samples, args.plane_z_mm, matrix, distortion,
                                      (args.width, args.height))
                if args.output.exists():
                    print(f'输出已存在，换 --output：{args.output}')
                    continue
                with args.output.open('x', encoding='utf-8') as f:
                    json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
                    f.write('\n')
                print(f'已保存：{args.output.resolve()}')
                print(f'外参重投影 RMS：{result["reprojection_rms_px"]:.3f} px')
                print(f'相机位置（gimbal_link，m）：{result["translation_m"]}')
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
