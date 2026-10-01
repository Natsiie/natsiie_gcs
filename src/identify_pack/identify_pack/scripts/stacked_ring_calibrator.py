#!/usr/bin/env python3
"""叠放位外圈和上下基准线搜索标定工具（移植自 cyqm 叠放位直线标定.py）。

把「中环 + 上下两条基准线」摆进画面，标定其搜索范围；不改工位零点、不打开串口。

用法:
    python3 stacked_ring_calibrator.py --device /dev/camera_upper

左键点选中环，S 确认外圈/直线并保存，R 重新寻找，Q/Esc 退出。
注意：本工具跳过畸变矫正（与 ring_station 一致），标定结果只对本相机/分辨率有效。
"""
from __future__ import annotations

import argparse
from collections import deque
import os
from pathlib import Path
import sys

import cv2
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from ring_station_detector import (  # noqa: E402
    RingStationDetectionError, load_ring_station_config,
)
from stacked_ring_detector import (  # noqa: E402
    StackedRingDetectionError, calibration_samples_stable,
    camera_geometry_signature, detect_scaled_reference_lines,
    draw_search_calibration, find_calibration_outer_ring,
    has_both_reference_lines, load_stacked_ring_config,
    make_search_calibration, save_search_calibration, stacked_edge_image,
)

DEFAULT_CONFIG = os.path.join(_SCRIPT_DIR, 'stacked_ring_detector.json')
DEFAULT_RING_CONFIG = os.path.join(_SCRIPT_DIR, 'ring_station_detector.json')


class StackedRingCalibrationError(RuntimeError):
    """标定工具无法打开摄像头。"""


def open_camera(device, width, height, fps):
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not camera.isOpened():
        try:
            camera = cv2.VideoCapture(int(device))
        except Exception:
            pass
    if not camera.isOpened():
        camera.release()
        raise StackedRingCalibrationError(f'无法打开摄像头：{device}')
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, float(fps))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    for _ in range(10):
        ok, frame = camera.read()
        if ok and frame is not None:
            if frame.shape[:2] != (height, width):
                camera.release()
                raise StackedRingCalibrationError(
                    f'摄像头输出 {frame.shape[1]}x{frame.shape[0]}，需要 {width}x{height}')
            return camera
    camera.release()
    raise StackedRingCalibrationError('摄像头已打开，但无法读取画面')


def draw_preview(frame, observation, stable, message, diameter_mm, offset_mm):
    preview = frame.copy()
    if observation is not None:
        draw_search_calibration(preview, observation)
    lines = [
        'Stacking calibration: ' + ('STABLE' if stable else 'SEARCHING'),
        f'Outer={diameter_mm:g}mm  Lines=+/-{offset_mm:g}mm',
        'Click: select ring | S: save | R: reset | Q/Esc: quit',
        message,
    ]
    if observation is not None:
        lines.append(f"Yaw={observation.get('yaw', 0):+.2f}deg - verify GREEN is outermost")
    cv2.rectangle(preview, (0, 0), (preview.shape[1] - 1, 25 * len(lines) + 12), (0, 0, 0), -1)
    for index, line in enumerate(lines):
        cv2.putText(preview, line, (10, 25 + index * 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 240, 0) if stable else (0, 220, 255), 1, cv2.LINE_AA)
    return preview


def parse_args():
    parser = argparse.ArgumentParser(description='标定叠放位中环与上下基准线搜索范围')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--ring-config', default=DEFAULT_RING_CONFIG)
    parser.add_argument('--diameter-mm', type=float, default=95.0)
    parser.add_argument('--line-offset-mm', type=float, default=75.0)
    return parser.parse_args()


def run(args):
    config = load_stacked_ring_config(args.config)
    ring_config = load_ring_station_config(args.ring_config)
    signature = camera_geometry_signature(args.width, args.height, False, None)
    camera = open_camera(args.device, args.width, args.height, args.fps)

    window = 'Stacking line calibration'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    samples = deque(maxlen=ring_config.stable_frame_count)
    selection = None
    current_observation = None
    current_frame_size = None
    status_message = 'Click middle ring if automatic selection fails'

    def clear_observation():
        nonlocal current_observation
        samples.clear()
        current_observation = None

    def reset_selection():
        nonlocal selection, status_message
        selection = None
        clear_observation()
        status_message = 'Click middle ring if automatic selection fails'

    def handle_mouse(event, x, y, _flags, _param):
        nonlocal selection, status_message
        if event != cv2.EVENT_LBUTTONDOWN or current_frame_size is None:
            return
        width, height = current_frame_size
        if not (0 <= x < width and 0 <= y < height):
            return
        selection = (float(x), float(y))
        clear_observation()
        status_message = 'Selected ring - checking outer boundary and lines'

    cv2.setMouseCallback(window, handle_mouse)

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                status_message = 'Camera read failed'
                display = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                cv2.imshow(window, display)
                if cv2.waitKey(20) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
                continue

            current_frame_size = (args.width, args.height)
            stable = False
            try:
                edges = stacked_edge_image(frame, config)
                ellipse, _ = find_calibration_outer_ring(
                    frame, ring_config, selection, edge_image=edges)
                observation = detect_scaled_reference_lines(
                    frame, ellipse, config, args.diameter_mm, args.line_offset_mm,
                    edge_image=edges, require_both=True)
                current_observation = observation
                samples.append(observation)
                stable = calibration_samples_stable(samples, ring_config)
                if not status_message.startswith('SAVED'):
                    status_message = 'Check GREEN outermost boundary and both lines before S'
            except (StackedRingDetectionError, RingStationDetectionError,
                    cv2.error, np.linalg.LinAlgError) as error:
                clear_observation()
                status_message = 'Check ring selection, outer boundary and both lines; see terminal'
                print(f'标定等待：{error}')

            preview = draw_preview(frame, current_observation, stable, status_message,
                                   args.diameter_mm, args.line_offset_mm)
            cv2.imshow(window, preview)

            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('r'), ord('R')):
                reset_selection()
            elif key in (ord('s'), ord('S')):
                if (current_observation is None
                        or not has_both_reference_lines(current_observation)
                        or not stable):
                    status_message = 'Cannot save: wait for STABLE with BOTH lines'
                    continue
                try:
                    data = make_search_calibration(
                        current_observation, signature, args.diameter_mm, args.line_offset_mm)
                    save_search_calibration(args.config, data)
                    status_message = 'SAVED - restart node/calibrator to load calibration'
                    print(f'已保存叠放位搜索标定：{Path(args.config).resolve()}')
                except (OSError, ValueError, StackedRingDetectionError) as error:
                    status_message = 'Save failed - original configuration preserved'
                    print(f'保存失败：{error}')
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main():
    args = parse_args()
    try:
        run(args)
    except (StackedRingCalibrationError, StackedRingDetectionError,
            RingStationDetectionError, OSError, ValueError, cv2.error) as error:
        print(f'错误：{error}')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
