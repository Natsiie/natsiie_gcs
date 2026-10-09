#!/usr/bin/env python3
"""底盘纠偏矩阵标定工具（chassis_image_to_motion）。

引导用户做三次已知位移，反推「工位系误差 → 底盘系纠偏」的 3x3 矩阵：

  1. 摆正工位，按 ENTER 记录基准误差 e0
  2. 底盘沿 X 轴走 +dx mm，按 ENTER 记录 e1
  3. 底盘沿 Y 轴走 +dy mm，按 ENTER 记录 e2
  4. 底盘转 +dr 度，按 ENTER 记录 e3

由三次位移与误差变化反推 A（Δe = A·Δm），chassis_image_to_motion = -A^{-1}，
回写 ring_station_detector.json 的 correction_calibration.chassis_image_to_motion。

按键：ENTER/空格 记录当前误差  R 重置  S 保存  Q/Esc 退出
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from ring_station_detector import (  # noqa: E402
    RingStationDetectionError, RingStationDetector, draw_ring_station_detection,
    load_ring_station_config, save_ring_station_config,
)

DEFAULT_CONFIG = os.path.join(_SCRIPT_DIR, 'ring_station_detector.json')


def open_camera(device, width, height, fps):
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not camera.isOpened():
        try:
            camera = cv2.VideoCapture(int(device))
        except Exception:
            pass
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f'无法打开摄像头：{device}')
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, float(fps))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    for _ in range(10):
        ok, frame = camera.read()
        if ok and frame is not None:
            return camera
    camera.release()
    raise RuntimeError('摄像头已打开，但无法读取画面')


def compute_matrix(dx, dy, dr, e0, e1, e2, e3):
    """反推 chassis_image_to_motion = -A^{-1}。

    A 的第 i 列 = (e_i - e0) / 对应位移量；三次位移产生的误差若线性相关则不可逆。
    """
    A = np.column_stack([
        (np.asarray(e1, dtype=float) - np.asarray(e0, dtype=float)) / dx,
        (np.asarray(e2, dtype=float) - np.asarray(e0, dtype=float)) / dy,
        (np.asarray(e3, dtype=float) - np.asarray(e0, dtype=float)) / dr,
    ])
    if abs(np.linalg.det(A)) < 1e-9:
        raise RingStationDetectionError(
            '三次位移产生的误差线性相关，矩阵不可逆；请确认做了三次独立方向的位移')
    if not np.isfinite(A).all():
        raise RingStationDetectionError('误差读取到非有限值，请重新记录')
    return -np.linalg.inv(A)


def parse_args():
    parser = argparse.ArgumentParser(description='标定底盘纠偏矩阵 chassis_image_to_motion')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=25.0)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--flip-code', type=int, default=-1,
                        help='画面翻转：-1=上下颠倒(180°)，1=左右镜像，0=不翻转（与 launch 一致）')
    parser.add_argument('--dx', type=float, default=100.0, help='X 轴位移量(mm)')
    parser.add_argument('--dy', type=float, default=100.0, help='Y 轴位移量(mm)')
    parser.add_argument('--dr', type=float, default=10.0, help='旋转量(deg)')
    return parser.parse_args()


def run(args):
    config_path = Path(args.config).expanduser().resolve()
    try:
        document = json.loads(config_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f'无法读取配置 {config_path}：{error}') from error
    config = load_ring_station_config(config_path)
    detector = RingStationDetector(config)
    camera = open_camera(args.device, args.width, args.height, args.fps)

    steps = [
        ('基准', '摆正工位，按 ENTER 记录基准误差'),
        ('X轴', f'底盘沿 X 轴走 +{args.dx:g} mm，按 ENTER 记录'),
        ('Y轴', f'底盘沿 Y 轴走 +{args.dy:g} mm，按 ENTER 记录'),
        ('旋转', f'底盘转 +{args.dr:g} 度，按 ENTER 记录'),
    ]
    errors = [None, None, None, None]
    step = 0
    matrix = None
    message = steps[0][1]

    window = 'Chassis Correction Calibration'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError('标定过程中摄像头读取失败')
            if args.flip_code:
                frame = cv2.flip(frame, args.flip_code)
            result = detector.detect(frame)
            current = None
            if result.detected:
                current = (result.error_x_mm, result.error_y_mm, result.angle_error_deg)

            preview = frame.copy()
            draw_ring_station_detection(preview, result)

            if step < len(steps):
                step_line = f'step {step + 1}/4: {steps[step][0]}'
            else:
                step_line = 'step 4/4: 完成'
            lines = ['Chassis correction calibration', step_line]
            if current is not None:
                state = 'STABLE' if result.stable else 'detected'
                lines.append(f'error=({current[0]:.2f}, {current[1]:.2f})mm '
                             f'angle={current[2]:.2f}deg  {state}')
            else:
                lines.append('三环未检出')
            for i, (name, _) in enumerate(steps):
                if errors[i] is not None:
                    e = errors[i]
                    lines.append(f'  [{name}] error=({e[0]:.2f},{e[1]:.2f})mm '
                                 f'angle={e[2]:.2f}deg')
            if matrix is not None:
                lines.append('矩阵已计算，按 S 保存到 JSON')
            lines.append(message)

            box_height = 26 * len(lines) + 12
            cv2.rectangle(preview, (0, 0), (preview.shape[1] - 1, box_height),
                          (0, 0, 0), -1)
            for i, line in enumerate(lines):
                cv2.putText(preview, line, (10, 25 + i * 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 240, 0), 1, cv2.LINE_AA)
            cv2.imshow(window, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('\r'), ord('\n'), ord(' ')):
                if current is None:
                    message = '当前未检出三环，无法记录'
                    print(message)
                else:
                    errors[step] = current
                    print(f'记录 {steps[step][0]}：error=({current[0]:.2f},{current[1]:.2f})mm '
                          f'angle={current[2]:.2f}deg')
                    step += 1
                    if step >= 4:
                        matrix = compute_matrix(args.dx, args.dy, args.dr, *errors)
                        print('chassis_image_to_motion =')
                        print(np.array2string(matrix, precision=8))
                        message = '矩阵已计算，按 S 保存；按 R 重来'
                    else:
                        message = steps[step][1]
            elif key in (ord('r'), ord('R')):
                errors = [None, None, None, None]
                step = 0
                matrix = None
                message = steps[0][1]
                print('已重置')
            elif key in (ord('s'), ord('S')):
                if matrix is None:
                    message = '还没有计算矩阵，先完成四次记录'
                    print(message)
                    continue
                document['correction_calibration']['chassis_image_to_motion'] = [
                    [round(float(v), 8) for v in row] for row in matrix]
                try:
                    save_ring_station_config(config_path, document)
                except RingStationDetectionError as error:
                    message = f'保存失败：{error}'
                    print(message)
                    continue
                message = '已保存 chassis_image_to_motion'
                print(message)
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main():
    args = parse_args()
    try:
        run(args)
    except (RuntimeError, RingStationDetectionError, cv2.error) as error:
        print(f'错误：{error}')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
