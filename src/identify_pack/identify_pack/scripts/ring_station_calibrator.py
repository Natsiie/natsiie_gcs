#!/usr/bin/env python3
"""三圆环工位参考零点标定工具（摆正工位后回写参考圆心 + px/mm 标定矩阵）。

把工位摆到「零点/对正」位置：
  - 自动检测稳定后按 S 保存；自动检测失败时，鼠标左键依次点选三环（R1/R2/R3）再按 S；
  - 保存时用 geometry.center_spacing_mm 自动推算像素→毫米比例，写入 image_to_station_affine。

回写到 ring_station_detector.json 的：
  - reference.ring_centers_px           三环参考圆心（像素）
  - reference.station_yaw_deg           参考角度（deg）
  - reference.image_to_station_affine   像素→毫米矩阵（2x2，对角近似，忽略相机倾斜/旋转）

按键：左键点选  S 保存  R 重置点选  Q/Esc 退出
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys

import cv2
import numpy as np

try:
    from .camera_capture import TimedCamera
    from .camera_diagnostics import require_display
except ImportError:
    from camera_capture import TimedCamera
    from camera_diagnostics import require_display

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from ring_station_detector import (  # noqa: E402
    RingStationDetector, RingStationDetectionError,
    draw_ring_station_detection, load_ring_station_config,
    save_ring_station_config,
)

DEFAULT_CONFIG = os.path.join(_SCRIPT_DIR, 'ring_station_detector.json')


class RingStationCalibrationError(RuntimeError):
    """标定工具无法打开摄像头或画面无效。"""


def open_camera(device, width, height, fps, fourcc='MJPG', timeout=5.0):
    camera = TimedCamera(device, width, height, fps, fourcc, timeout)
    ok, frame = camera.read()
    if ok and frame is not None:
        if frame.shape[:2] == (height, width):
            return camera
        camera.release()
        raise RingStationCalibrationError(
            f'摄像头输出 {frame.shape[1]}x{frame.shape[0]}，需要 {width}x{height}')
    camera.release()
    raise RingStationCalibrationError('摄像头已打开，但无法读取画面')


def order_centers(points, ring1_side):
    """按 X 排序三环（ring1_side='right' 时反向），与检测器约定一致。"""
    return sorted(points, key=lambda point: float(point[0]),
                  reverse=(ring1_side == 'right'))


def compute_calibration(centers, center_spacing_mm):
    """由三环圆心（已排序 R1,R2,R3）推算间距、px/mm、yaw 和对角仿射矩阵。"""
    p1 = np.asarray(centers[0], dtype=float)
    p2 = np.asarray(centers[1], dtype=float)
    p3 = np.asarray(centers[2], dtype=float)
    spacing_12 = float(np.linalg.norm(p2 - p1))
    spacing_23 = float(np.linalg.norm(p3 - p2))
    avg_spacing = (spacing_12 + spacing_23) / 2.0
    if avg_spacing <= 0:
        raise RingStationDetectionError('三环间距为 0，无法推算 px/mm')
    mm_per_px = center_spacing_mm / avg_spacing
    yaw = math.degrees(math.atan2(p3[1] - p1[1], p3[0] - p1[0]))
    affine = [[mm_per_px, 0.0], [0.0, mm_per_px]]
    return spacing_12, spacing_23, mm_per_px, yaw, affine


def draw_manual_points(frame, points, ordered):
    """绘制已点选的圆心与 R1→R2→R3 连线。"""
    colors = ((0, 180, 255), (0, 255, 0), (255, 160, 0))
    for index, (x, y) in enumerate(points):
        point = (int(round(x)), int(round(y)))
        cv2.drawMarker(frame, point, (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(frame, f'{index + 1}', (point[0] + 8, point[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    if len(ordered) >= 3:
        line = np.asarray([(int(round(x)), int(round(y))) for x, y in ordered], np.int32)
        cv2.polylines(frame, [line], False, (255, 0, 255), 2)
    return frame


def draw_status(frame, lines):
    box_height = 26 * len(lines) + 12
    cv2.rectangle(frame, (0, 0), (frame.shape[1] - 1, box_height), (0, 0, 0), -1)
    for index, line in enumerate(lines):
        cv2.putText(frame, line, (10, 25 + index * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 240, 0), 1, cv2.LINE_AA)
    return frame


def parse_args():
    parser = argparse.ArgumentParser(description='标定三圆环工位参考零点与 px/mm 矩阵')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=25.0)
    parser.add_argument('--fourcc', default='MJPG')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--flip-code', type=int, choices=(-1, 0, 1), default=-1,
                        help='画面翻转：-1=上下颠倒(180°)，1=左右镜像，0=不翻转（与 launch 一致）')
    return parser.parse_args()


def run(args):
    config_path = Path(args.config).expanduser().resolve()
    try:
        document = json.loads(config_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise RingStationCalibrationError(f'无法读取配置 {config_path}：{error}') from error
    config = load_ring_station_config(config_path)
    detector = RingStationDetector(config)
    require_display()
    camera = open_camera(args.device, args.width, args.height, args.fps,
                         args.fourcc, args.timeout)

    window = 'Ring Station Zero Calibration'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    manual_points = []
    message = '摆正工位后稳定即按 S；自动失败可左键点选三环 R1/R2/R3'

    def on_mouse(event, x, y, _flags, _param):
        nonlocal message
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if len(manual_points) < 3:
            manual_points.append((float(x), float(y)))
            message = f'已点选 {len(manual_points)}/3 个圆心'
        else:
            message = '已点满 3 点，按 S 保存或 R 重置'

    cv2.setMouseCallback(window, on_mouse)

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RingStationCalibrationError('标定过程中摄像头读取失败')
            if args.flip_code:
                frame = cv2.flip(frame, args.flip_code)

            result = detector.detect(frame)

            active = None  # (centers, source)
            if len(manual_points) == 3:
                active = (order_centers(manual_points, config.ring1_side), 'manual')
            elif result.detected and len(result.centers_px) == 3:
                active = (list(result.centers_px), 'auto')

            preview = frame.copy()
            draw_ring_station_detection(preview, result)

            lines = ['Ring station zero calibration']
            if result.detected and len(result.centers_px) == 3:
                lines.append(f"auto: {'STABLE' if result.stable else 'DETECTED'}  "
                             f"centers={result.centers_px}")
            else:
                lines.append('auto: SEARCHING')
            if active is None:
                lines.append('三环未就绪：自动未检出且未点满 3 点')
            else:
                centers, source = active
                spacing_12, spacing_23, mm_per_px, yaw, affine = compute_calibration(
                    centers, config.center_spacing_mm)
                lines.append(f'source={source}  gap={spacing_12:.1f}/{spacing_23:.1f}px  '
                             f'yaw={yaw:+.2f}deg')
                lines.append(f'px/mm={1.0 / mm_per_px:.2f}  mm/px={mm_per_px:.4f}  '
                             f'(spacing={config.center_spacing_mm:g}mm)')
                if len(manual_points) < 3:
                    lines.append('按 S 保存参考零点 + 自动推算的 px/mm 矩阵')
            lines.append(message)

            draw_manual_points(preview, manual_points, active[0] if active else [])
            draw_status(preview, lines)
            cv2.imshow(window, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('r'), ord('R')):
                manual_points.clear()
                message = '已重置点选'
            elif key in (ord('s'), ord('S')):
                if active is None:
                    message = '尚无可用三环，无法保存'
                    continue
                centers, source = active
                spacing_12, spacing_23, mm_per_px, yaw, affine = compute_calibration(
                    centers, config.center_spacing_mm)
                document['reference']['ring_centers_px'] = [
                    [round(float(x), 2), round(float(y), 2)] for x, y in centers]
                document['reference']['station_yaw_deg'] = round(yaw, 4)
                document['reference']['image_to_station_affine'] = [
                    [round(float(v), 6) for v in row] for row in affine]
                try:
                    save_ring_station_config(config_path, document)
                except RingStationDetectionError as error:
                    message = f'保存失败：{error}'
                    print(message)
                    continue
                message = (f'已保存参考零点（{source}）：yaw={yaw:+.2f}deg  '
                           f'mm/px={mm_per_px:.4f}')
                print(message)
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main():
    args = parse_args()
    try:
        run(args)
    except (RuntimeError, ValueError, cv2.error) as error:
        print(f'错误：{error}')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
