#!/usr/bin/env python3
"""Enumerate V4L2 cameras and test capture, detection and optional GUI without ROS."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2

try:
    from identify_pack.scripts.camera_capture import TimedCamera
    from identify_pack.scripts.ring_station_detector import (
        RingStationDetector, draw_ring_station_detection, load_ring_station_config,
    )
    from identify_pack.scripts.turntable_detector import (
        TurntableDetector, draw_turntable_detection, load_turntable_detector_config,
    )
except ImportError:
    from identify_pack.scripts.camera_capture import TimedCamera
    from identify_pack.scripts.ring_station_detector import (
        RingStationDetector, draw_ring_station_detection, load_ring_station_config,
    )
    from identify_pack.scripts.turntable_detector import (
        TurntableDetector, draw_turntable_detection, load_turntable_detector_config,
    )


def enumerate_cameras(sys_root=Path('/sys/class/video4linux'), dev_root=Path('/dev')):
    devices = []
    for entry in sorted(sys_root.glob('video*'), key=lambda p: int(p.name[5:])):
        try:
            device = dev_root / entry.name
            devices.append({
                'device': str(device),
                'name': (entry / 'name').read_text().strip(),
                'index': int((entry / 'index').read_text().strip()),
                'usb_interface': str((entry / 'device').resolve()),
                'accessible': device.exists() and os.access(device, os.R_OK | os.W_OK),
            })
        except OSError:
            continue
    return devices


def require_display():
    # Qt can abort the interpreter; probe it in a child before calling HighGUI.
    code = ('import cv2, numpy as np; '
            'cv2.imshow("Camera display check", np.zeros((60,160,3),np.uint8)); '
            'cv2.waitKey(100); cv2.destroyAllWindows()')
    try:
        result = subprocess.run([sys.executable, '-c', code], capture_output=True,
                                text=True, timeout=8)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError('Display check timed out; use --no-show') from error
    if result.returncode:
        raise RuntimeError('Cannot open desktop display; use --no-show. '
                           + result.stderr.strip()[:500])


def build_detector(mode, config_path=None):
    root = Path(__file__).resolve().parent.parent
    if mode == 'ring':
        path = config_path or root / 'ring_station_detector.json'
        return RingStationDetector(load_ring_station_config(path)), draw_ring_station_detection
    path = config_path or root / 'turntable_detector.json'
    return TurntableDetector(load_turntable_detector_config(path)), draw_turntable_detection


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    return value


def run(args):
    devices = enumerate_cameras()
    if args.list:
        return {'devices': devices}, 0
    print(json.dumps(devices, ensure_ascii=False, indent=2), flush=True)
    detector, draw = build_detector(args.detector, args.config)
    if not args.no_show:
        require_display()
    camera = None
    stats = {'devices': devices, 'detector': args.detector, 'frames': 0,
             'detected_frames': 0, 'stable_frames': 0, 'display': not args.no_show}
    try:
        if args.image:
            image = cv2.imread(str(args.image))
            if image is None:
                raise RuntimeError(f'Cannot read image: {args.image}')
        else:
            device = args.device
            if device == 'auto':
                candidates = [d for d in devices if d['index'] == 0
                              and 'IR' not in d['name'] and d['accessible']]
                if not candidates:
                    raise RuntimeError('No accessible RGB camera. Check device access and sandbox.')
                device = candidates[0]['device']
            stats['device'] = device
            camera = TimedCamera(device, args.width, args.height, args.fps,
                                 args.fourcc, args.timeout)
        started = time.monotonic()
        next_log = started
        for _ in range(args.frames):
            if camera is not None:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError(f'Camera stopped or no new frame within {args.timeout:g}s')
            else:
                frame = image.copy()
            if args.flip_code:
                frame = cv2.flip(frame, args.flip_code)
            result = detector.detect(frame)
            stats['frames'] += 1
            stats['detected_frames'] += int(result.detected)
            stats['stable_frames'] += int(result.stable)
            stats['resolution'] = list(frame.shape[1::-1])
            stats['last_result'] = asdict(result)
            preview = draw(frame.copy(), result)
            if args.snapshot and not cv2.imwrite(str(args.snapshot), preview):
                raise RuntimeError(f'Cannot write snapshot: {args.snapshot}')
            now = time.monotonic()
            if now >= next_log:
                print(f"frames={stats['frames']} detected={result.detected} "
                      f"stable={result.stable} processing={result.processing_ms:.1f}ms "
                      f"detail={result.detail}", flush=True)
                next_log = now + 1
            if not args.no_show:
                cv2.imshow('Camera ring diagnostics', preview)
                if cv2.waitKey(1) & 0xFF in (27, ord('q'), ord('Q')):
                    break
                if cv2.getWindowProperty('Camera ring diagnostics', cv2.WND_PROP_VISIBLE) < 1:
                    break
        stats['elapsed_sec'] = time.monotonic() - started
        stats['processed_fps'] = stats['frames'] / max(stats['elapsed_sec'], 1e-6)
        # No target is a distinct outcome from camera/display failure.
        return stats, 0 if stats['detected_frames'] else 2
    finally:
        if camera is not None:
            camera.release()
        if not args.no_show:
            cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--device', default='auto')
    parser.add_argument('--detector', choices=('ring', 'turntable'), default='ring')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--image', type=Path)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=30)
    parser.add_argument('--fourcc', default='MJPG')
    parser.add_argument('--timeout', type=float, default=5)
    parser.add_argument('--frames', type=int, default=150)
    parser.add_argument('--flip-code', type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--snapshot', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    if args.frames <= 0:
        parser.error('--frames must be positive')
    try:
        report, status = run(args)
    except (RuntimeError, ValueError, cv2.error, OSError) as error:
        report, status = {'error': str(error)}, 1
    output = json.dumps(_json_safe(report), ensure_ascii=False, indent=2, allow_nan=False)
    print(output)
    if args.report:
        args.report.write_text(output + '\n', encoding='utf-8')
    return status


if __name__ == '__main__':
    raise SystemExit(main())
