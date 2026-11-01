#!/usr/bin/env python3
"""独立测试叠放位“边线独立锁定”效果（不启动 ROS / race_start）。

用法（Pi 桌面或 RDP/X11 下）：
    python3 stacked_ring_line_lock_test.py --device /dev/camera_upper

按键：
    L  切换 独立锁定 / 中环相对 两种模式
    R  重置检测器（清空稳定窗口和线锁）
    S  保存当前 overlay 截图到 /tmp
    Q / Esc  退出
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys
import time

import cv2

try:
    from identify_pack.scripts.ring_station_detector import load_ring_station_config
    from identify_pack.scripts.stacked_ring_detector import (
        StackedRingDetector, camera_geometry_signature,
        draw_stacked_ring_detection, load_stacked_ring_config,
    )
except ImportError:  # 直接在源码目录运行
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from identify_pack.scripts.ring_station_detector import load_ring_station_config
    from identify_pack.scripts.stacked_ring_detector import (
        StackedRingDetector, camera_geometry_signature,
        draw_stacked_ring_detection, load_stacked_ring_config,
    )


DEFAULT_CONFIG = Path(__file__).with_name('stacked_ring_detector.json')
DEFAULT_RING_CONFIG = Path(__file__).with_name('ring_station_detector.json')


def parse_args():
    parser = argparse.ArgumentParser(description='叠放位边线独立锁定测试（无 ROS）')
    parser.add_argument('--device', default='/dev/camera_upper')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=25.0)
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--ring-config', default=str(DEFAULT_RING_CONFIG))
    parser.add_argument('--no-line-lock', action='store_true',
                        help='启动时关闭 independent_line_lock')
    return parser.parse_args()


def open_camera(device, width, height, fps):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f'无法打开摄像头 {device}')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def build_detector(config, ring_config, width, height):
    detector = StackedRingDetector(config, ring_config)
    detector.set_camera_geometry(
        camera_geometry_signature(width, height, False, None))
    return detector


def run(args):
    config = load_stacked_ring_config(args.config)
    ring_config = load_ring_station_config(args.ring_config)
    if args.no_line_lock:
        config = replace(config, independent_line_lock=False)
    detector = build_detector(config, ring_config, args.width, args.height)
    cap = open_camera(args.device, args.width, args.height, args.fps)

    window = 'Stacked ring line lock test'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)
    save_count = 0
    last_report = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print('读帧失败，退出')
                break
            result = detector.detect(frame)
            draw_stacked_ring_detection(frame, result)
            mode = 'LINE-LOCK' if config.independent_line_lock else 'RING-RELATIVE'
            cv2.putText(
                frame, f'{mode}  L=toggle  R=reset  S=save  Q=quit',
                (20, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 220, 80),
                2, cv2.LINE_AA)
            if result.detected and time.monotonic() - last_report >= 1.0:
                last_report = time.monotonic()
                extra = ''
                if result.line_lock_active:
                    dy = (f'{result.line_mid_y_error_px:+.1f}px'
                          if result.line_mid_y_error_px is not None else 'n/a')
                    sep = (f'{result.line_separation_px:.1f}px'
                           if result.line_separation_px is not None else 'n/a')
                    extra = f' line_lock dY={dy} sep={sep}'
                print(f"{mode} state={result.stable and 'stable' or 'wait'} "
                      f"err=({result.error_x_mm:+.1f},{result.error_y_mm:+.1f})mm "
                      f"dYaw={result.angle_error_deg:+.2f}deg{extra}")
            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                break
            if key in (ord('l'), ord('L')):
                config = replace(
                    config, independent_line_lock=not config.independent_line_lock)
                detector = build_detector(config, ring_config, args.width, args.height)
                print('切换为', 'LINE-LOCK' if config.independent_line_lock
                      else 'RING-RELATIVE')
            elif key in (ord('r'), ord('R')):
                detector.reset()
                print('检测器已重置')
            elif key in (ord('s'), ord('S')):
                save_count += 1
                path = f'/tmp/stacked_line_lock_{int(time.time())}_{save_count}.png'
                cv2.imwrite(path, frame)
                print('已保存', path)
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    run(parse_args())


if __name__ == '__main__':
    main()
