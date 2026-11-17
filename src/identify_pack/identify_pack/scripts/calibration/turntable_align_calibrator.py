#!/usr/bin/env python3
"""转盘圆心对齐参考标定：把车摆到理想对齐位姿，保存当前转盘圆心为参考。

自动检测转盘圆心；自动失败时左键手动点圆心。按 S 把当前圆心写回 turntable_align.json
的 reference_center_px，同时写入转盘直径与容差。

按键：左键点选圆心  S 保存  Q/Esc 退出
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import cv2

_SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

try:
    from identify_pack.scripts.camera_capture import TimedCamera
    from identify_pack.scripts.turntable_detector import (
        TurntableDetector, draw_turntable_detection, load_turntable_detector_config,
    )
    from identify_pack.scripts.turntable_align import load_turntable_align_config
except ImportError:
    from identify_pack.scripts.camera_capture import TimedCamera
    from identify_pack.scripts.turntable_detector import (
        TurntableDetector, draw_turntable_detection, load_turntable_detector_config,
    )
    from identify_pack.scripts.turntable_align import load_turntable_align_config

DEFAULT_CONFIG = os.path.join(_SCRIPT_DIR, 'turntable_align.json')
DEFAULT_DETECTOR_CONFIG = os.path.join(_SCRIPT_DIR, 'turntable_detector.json')


def parse_args():
    parser = argparse.ArgumentParser(description='标定转盘圆心对齐参考')
    parser.add_argument('--device', default='/dev/camera_upper')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--fourcc', default='MJPG')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--detector-config', default=DEFAULT_DETECTOR_CONFIG)
    parser.add_argument('--diameter-mm', type=float, default=300.0)
    parser.add_argument('--tolerance-mm', type=float, default=10.0)
    return parser.parse_args()


def draw_status(frame, lines, detected):
    height = 25 * len(lines) + 12
    cv2.rectangle(frame, (0, 0), (frame.shape[1] - 1, height), (0, 0, 0), -1)
    color = (0, 240, 0) if detected else (0, 220, 255)
    for index, line in enumerate(lines):
        cv2.putText(frame, line, (10, 25 + index * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return frame


def run(args):
    config_path = Path(args.config).expanduser().resolve()
    try:
        document = json.loads(config_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f'无法读取配置 {config_path}：{error}') from error
    load_turntable_align_config(config_path)
    detector = TurntableDetector(load_turntable_detector_config(args.detector_config))

    camera = TimedCamera(args.device, args.width, args.height, args.fps,
                         args.fourcc, args.timeout)
    ok, _ = camera.read()
    if not ok:
        camera.release()
        raise RuntimeError('摄像头已打开，但无法读取画面')

    window = 'Turntable Align Calibration'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    manual = None
    message = '摆正车后稳定即按 S；自动失败可左键点圆心'

    def on_mouse(event, x, y, _flags, _param):
        nonlocal manual, message
        if event == cv2.EVENT_LBUTTONDOWN and 0 <= x < args.width and 0 <= y < args.height:
            manual = (float(x), float(y))
            message = f'已手动选圆心 ({x},{y})，按 S 保存'

    cv2.setMouseCallback(window, on_mouse)

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError('标定过程中摄像头读取失败')

            result = detector.detect(frame)
            if manual is not None:
                current = (manual[0], manual[1])
                source = 'manual'
            elif result.detected:
                current = (result.center_x_px, result.center_y_px)
                source = 'auto'
            else:
                current = None
                source = ''

            preview = frame.copy()
            draw_turntable_detection(preview, result)
            lines = ['Turntable align calibration']
            if current is not None:
                lines.append(f'{source} center=({current[0]:.1f},{current[1]:.1f})')
                lines.append('按 S 保存为参考圆心')
            else:
                lines.append('转盘未检出：左键点圆心，或先跑 turntable_calibrator 调参数')
            lines.append(message)
            draw_status(preview, lines, current is not None)
            cv2.imshow(window, preview)

            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('s'), ord('S')):
                if current is None:
                    message = '尚未检出圆心，无法保存'
                    continue
                document['reference_center_px'] = [round(current[0], 2), round(current[1], 2)]
                document['turntable_diameter_mm'] = float(args.diameter_mm)
                document['tolerance_mm'] = float(args.tolerance_mm)
                with open(config_path, 'w', encoding='utf-8') as stream:
                    json.dump(document, stream, ensure_ascii=False, indent=2)
                    stream.write('\n')
                load_turntable_align_config(config_path)
                message = f'已保存参考圆心 ({current[0]:.2f},{current[1]:.2f})'
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
