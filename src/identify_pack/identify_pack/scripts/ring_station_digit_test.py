#!/usr/bin/env python3
"""Standalone cyqm ring geometry + printed 1/2/3 recognition experiment.

See RING_STATION_DIGIT_TEST.md for camera, image and template capture commands.
Only OpenCV/NumPy are required; this script never sends motion commands.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import cv2
import numpy as np

try:
    from .camera_capture import TimedCamera
    from .camera_diagnostics import require_display
    from .ring_station_detector import (
        _cluster_observations, _extract_observations,
        _select_triple, load_ring_station_config,
    )
    from .ring_station_digits import (
        PATCH_SIZE, DIGIT_SIZE, normalize_digit, extract_digit, load_templates,
        match_digit, recognize_rings, DigitMappingTracker, placement_plan,
    )
    from .pick_task import parse_task
except ImportError:
    from camera_capture import TimedCamera
    from camera_diagnostics import require_display
    from ring_station_detector import (
        _cluster_observations, _extract_observations,
        _select_triple, load_ring_station_config,
    )
    from ring_station_digits import (
        PATCH_SIZE, DIGIT_SIZE, normalize_digit, extract_digit, load_templates,
        match_digit, recognize_rings, DigitMappingTracker, placement_plan,
    )
    from pick_task import parse_task

ROOT = Path(__file__).resolve().parent
WINDOW = 'Ring station digits'


def default_resource(name):
    local = ROOT / name
    if local.exists():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('identify_pack')) / name
    except (ImportError, LookupError):
        return local


def process_frame(frame, config, templates, inner_ratio=0.55, minimum=0.55, margin=0.08):
    observations = _extract_observations(frame, config)
    candidates = _cluster_observations(observations, frame, config, use_hough=True)
    selection = _select_triple(candidates, frame.shape[1], config)
    rings = sorted(selection[1] if selection else candidates, key=lambda ring: ring.center[0])
    results, crops = recognize_rings(frame, rings, templates, inner_ratio, minimum, margin)
    complete = selection is not None and {ring['digit'] for ring in results} == {1, 2, 3}
    return {'three_ring_geometry': selection is not None,
            'complete_digit_mapping': complete, 'rings': results}, crops


def draw_results(frame, report, selected):
    display = frame.copy()
    for index, ring in enumerate(report['rings']):
        color = (0, 210, 0) if ring['digit'] is not None else (0, 180, 255)
        center = tuple(ring['center_px'])
        cv2.ellipse(display, (center, tuple(ring['ellipse_axes_px']),
                             ring['ellipse_angle_deg']), color, 2)
        point = tuple(int(round(value)) for value in center)
        cv2.drawMarker(display, point, color, cv2.MARKER_CROSS, 12, 1)
        text = f"R{ring.get('ring_id') or '?'}: {ring['digit'] or '?'} ({ring['score']:.2f})"
        x = max(0, min(point[0] - 45, frame.shape[1] - 180))
        y = max(20, min(point[1] - 20, frame.shape[0] - 10))
        cv2.putText(display, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if index == selected:
            cv2.circle(display, point, 8, (255, 0, 255), 2)
    return display


def draw_crops(crops):
    tiles = []
    for index, (patch, digit) in enumerate(crops):
        tile = np.zeros((PATCH_SIZE + 30, PATCH_SIZE + 90, 3), np.uint8)
        tile[30:, :PATCH_SIZE] = patch
        if digit is not None:
            tile[65:145, PATCH_SIZE + 5:PATCH_SIZE + 85] = cv2.cvtColor(
                cv2.resize(digit, (80, 80)), cv2.COLOR_GRAY2BGR)
        cv2.putText(tile, f'#{index + 1}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        tiles.append(tile)
    return np.concatenate(tiles, axis=1) if tiles else np.zeros((190, 250, 3), np.uint8)


def save_snapshot(directory, frame, display, report, crops):
    directory.mkdir(parents=True, exist_ok=True)
    stem = time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1000000000:09d}'
    for suffix, image in [('raw', frame), ('result', display), ('crops', draw_crops(crops))]:
        if not cv2.imwrite(str(directory / f'{stem}_{suffix}.png'), image):
            raise OSError('Could not save snapshot')
    (directory / f'{stem}.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(f'Saved snapshot: {directory / stem}', flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--device', default='/dev/camera_upper')
    source.add_argument('--image', type=Path)
    parser.add_argument('--config', type=Path, default=default_resource('ring_station_detector.json'))
    parser.add_argument('--templates', type=Path, default=default_resource('ring_station_digit_templates'))
    order = parser.add_mutually_exclusive_group()
    order.add_argument('--place-order', default='123', help='Task-code placement permutation')
    order.add_argument('--task-code', help='Example: 156+123+516+231')
    parser.add_argument('--numbering', choices=('fixed', 'templates'), default='fixed',
                        help='fixed: guaranteed left-to-right 1/2/3; templates: read IDs only')
    parser.add_argument('--batch', type=int, choices=(1, 2), default=1)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'ring_station_digit_snapshots')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=25)
    parser.add_argument('--fourcc', default='', help='Empty uses camera default; e.g. MJPG')
    parser.add_argument('--timeout', type=float, default=5)
    parser.add_argument('--flip-code', type=int, choices=(-1, 0, 1), default=0,
                        help='Same convention as launch: 0=unchanged, -1=180deg, 1=mirror')
    parser.add_argument('--rotate', type=int, choices=(0, 90, 180, 270), default=0)
    parser.add_argument('--inner-ratio', type=float, default=0.55)
    parser.add_argument('--min-confidence', type=float, default=0.55)
    parser.add_argument('--min-margin', type=float, default=0.08)
    parser.add_argument('--max-major-axis', type=float)
    parser.add_argument('--min-minor-axis', type=float)
    parser.add_argument('--canny-low', type=int)
    parser.add_argument('--canny-high', type=int)
    parser.add_argument('--capture', action='store_true', help='Click a ring, press 1/2/3 to label it')
    display = parser.add_mutually_exclusive_group()
    display.add_argument('--show', dest='show', action='store_true')
    display.add_argument('--no-show', dest='show', action='store_false')
    parser.set_defaults(show=True)
    args = parser.parse_args(argv)
    if not 0.1 <= args.inner_ratio <= 0.85:
        parser.error('--inner-ratio must be between 0.1 and 0.85')
    if not 0 <= args.min_confidence <= 1 or not 0 <= args.min_margin <= 2:
        parser.error('Invalid confidence or margin')
    if args.capture and not args.show:
        parser.error('--capture requires a display')
    if args.task_code:
        try:
            picks = parse_task(args.task_code)
        except ValueError as error:
            parser.error(str(error))
        args.order = tuple(pick.ring for pick in picks if pick.batch == args.batch)
    else:
        if len(args.place_order) != 3 or set(args.place_order) != set('123'):
            parser.error('--place-order must be a permutation of 123')
        args.order = tuple(int(value) for value in args.place_order)
    if (min(args.width, args.height, args.fps, args.timeout) <= 0
            or (args.fourcc and len(args.fourcc) != 4)):
        parser.error('Invalid camera dimensions, fps, timeout or fourcc')
    return args


def run(args):
    config = load_ring_station_config(args.config)
    overrides = {name: getattr(args, option) for name, option in (
        ('max_major_axis_px', 'max_major_axis'), ('min_minor_axis_px', 'min_minor_axis'),
        ('canny_low', 'canny_low'), ('canny_high', 'canny_high'))
        if getattr(args, option) is not None}
    config = replace(config, **overrides)
    if (not 0 < config.canny_low < config.canny_high <= 255
            or not 0 < config.min_minor_axis_px < config.max_major_axis_px):
        raise ValueError('Invalid Canny thresholds or ellipse size range')
    templates, sources = load_templates(args.templates)
    tracker = DigitMappingTracker(config.stable_frame_count, config.maximum_center_jitter_px,
                                  expected_digits=(1, 2, 3) if args.numbering == 'fixed' else None)
    print(f'Templates: {sources}; scores are similarity, not probabilities.', flush=True)
    print('Click ring; 1/2/3 capture selected digit; S snapshot; Q/Esc exit.', flush=True)
    camera = None
    report = {'rings': []}
    selected_center = None

    def select_ring(event, x, y, _flags, _param):
        nonlocal selected_center
        if event == cv2.EVENT_LBUTTONDOWN and report['rings']:
            selected_center = min((ring['center_px'] for ring in report['rings']),
                                  key=lambda center: np.linalg.norm(np.asarray(center) - (x, y)))

    try:
        if args.show:
            require_display()
            cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(WINDOW, select_ring)
        still = cv2.imread(str(args.image)) if args.image else None
        if args.image and still is None:
            raise ValueError(f'Could not read image: {args.image}')
        if args.image is None:
            camera = TimedCamera(args.device, args.width, args.height,
                                 args.fps, args.fourcc, args.timeout)
        last_print = 0.0
        while True:
            if camera is not None:
                ok, frame = camera.read()
                if not ok or frame is None:
                    raise RuntimeError('Camera read failed or timed out')
            else:
                frame = still.copy()
            if args.flip_code:
                frame = cv2.flip(frame, args.flip_code)
            if args.rotate:
                frame = np.ascontiguousarray(np.rot90(frame, -(args.rotate // 90)))
            report, crops = process_frame(frame, config, templates, args.inner_ratio,
                                         args.min_confidence, args.min_margin)
            tracker.update(report, frame.shape[1::-1])
            report['placement_order'] = list(args.order)
            report['placement_targets_px'] = placement_plan(report, args.order)
            selected = None
            if selected_center is not None and report['rings']:
                distances = [np.linalg.norm(np.asarray(ring['center_px']) - selected_center)
                             for ring in report['rings']]
                if min(distances) <= config.center_cluster_radius_px * 2:
                    selected = int(np.argmin(distances))
            display = draw_results(frame, report, selected)
            if time.monotonic() - last_print >= 1 or args.image:
                print(json.dumps(report, ensure_ascii=False), flush=True)
                last_print = time.monotonic()
            if not args.show:
                if args.image:
                    save_snapshot(args.output_dir, frame, display, report, crops)
                    break
                continue
            cv2.imshow(WINDOW, display)
            cv2.imshow('Digit crops', draw_crops(crops))
            key = cv2.waitKey(0 if args.image else 1) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                break
            if key in (ord('s'), ord('S')):
                save_snapshot(args.output_dir, frame, display, report, crops)
            if args.capture and key in (ord('1'), ord('2'), ord('3')):
                if selected is None or crops[selected][1] is None:
                    print('Select a ring with a visible digit first.', flush=True)
                    continue
                args.templates.mkdir(parents=True, exist_ok=True)
                label = key - ord('0')
                path = args.templates / f'digit_{label}_{time.time_ns()}.png'
                if not cv2.imwrite(str(path), crops[selected][1]):
                    raise OSError(f'Could not save template: {path}')
                templates, sources = load_templates(args.templates)
                tracker.reset()
                print(f'Saved {path}; templates: {sources}', flush=True)
    finally:
        if camera is not None:
            camera.release()
        if args.show:
            cv2.destroyAllWindows()
    return 0


def main(argv=None):
    args = parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, ValueError, cv2.error) as error:
        print(f'Error: {error}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
