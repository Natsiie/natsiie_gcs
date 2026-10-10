#!/usr/bin/env python3
"""转盘外圆参数的实时可视化设置工具（调试用，移植自 cyqm 转盘参数设置.py）。

左侧显示相机画面、Hough 粗圆、精修圆和可选径向采样点；右侧滑条修改参数。
按 A 在当前帧执行宽范围自动诊断并填入半径建议；按 S 才原子保存到配置。

按键：A 自动诊断  Tab 换页  Space 冻结  P 采样点  S 保存  R 重载  D 默认  Q 退出
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

try:
    from identify_pack.scripts.turntable_detector import (
        build_default_turntable_detector_document, DebugCircleCandidate,
        draw_turntable_detection, load_turntable_detector_document,
        save_turntable_detector_config, TurntableDebugResult,
        TurntableDetectionError, TurntableDetector,
        turntable_config_from_document,
    )
except ImportError:  # 直接在源码目录运行
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from turntable_detector import (  # type: ignore
        build_default_turntable_detector_document, DebugCircleCandidate,
        draw_turntable_detection, load_turntable_detector_document,
        save_turntable_detector_config, TurntableDebugResult,
        TurntableDetectionError, TurntableDetector,
        turntable_config_from_document,
    )

try:
    from .camera_capture import TimedCamera
    from .camera_diagnostics import require_display
except ImportError:
    from camera_capture import TimedCamera
    from camera_diagnostics import require_display

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), 'turntable_detector.json')
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
PANEL_WIDTH = 360
CANVAS_WIDTH = FRAME_WIDTH + PANEL_WIDTH


class TurntableSettingsError(RuntimeError):
    """参数页面无法打开摄像头。"""


@dataclass(frozen=True)
class ParameterSpec:
    """右侧滑条与 JSON 字段之间的映射。"""

    page: str
    group: str
    key: str
    label: str
    minimum: float
    maximum: float
    step: float
    integer: bool = False

    def format_value(self, value) -> str:
        if self.integer:
            return str(int(value))
        if self.step >= 1.0:
            return f'{float(value):.0f}'
        if self.step >= 0.1:
            return f'{float(value):.1f}'
        return f'{float(value):.2f}'


# 半径相关范围已按 640x480 缩小，其余阈值与 cyqm 一致。
PARAMETER_SPECS = (
    ParameterSpec('common', 'coarse_search', 'image_scale', 'image_scale', 0.20, 1.00, 0.01),
    ParameterSpec('common', 'coarse_search', 'minimum_radius_px', 'min_radius_px', 20, 350, 1, True),
    ParameterSpec('common', 'coarse_search', 'maximum_radius_px', 'max_radius_px', 40, 400, 1, True),
    ParameterSpec('common', 'coarse_search', 'canny_threshold', 'canny_threshold', 1, 255, 1, True),
    ParameterSpec('common', 'coarse_search', 'accumulator_threshold', 'accumulator', 5, 100, 1, True),
    ParameterSpec('common', 'refinement', 'radial_search_half_width_px', 'radial_width', 10, 120, 1, True),
    ParameterSpec('common', 'refinement', 'minimum_visible_arc_deg', 'min_arc_deg', 10, 360, 1, True),
    ParameterSpec('common', 'refinement', 'maximum_fit_rms_px', 'max_rms_px', 0.5, 10.0, 0.1),
    ParameterSpec('advanced', 'coarse_search', 'hough_dp', 'hough_dp', 1.0, 3.0, 0.1),
    ParameterSpec('advanced', 'coarse_search', 'minimum_center_distance_px', 'center_dist', 20, 300, 1, True),
    ParameterSpec('advanced', 'coarse_search', 'maximum_candidates', 'max_candidates', 1, 20, 1, True),
    ParameterSpec('advanced', 'refinement', 'angular_step_deg', 'angular_step', 0.5, 10.0, 0.5),
    ParameterSpec('advanced', 'refinement', 'minimum_edge_strength', 'min_edge', 0.5, 20.0, 0.1),
    ParameterSpec('advanced', 'refinement', 'saturated_pixel_threshold', 'saturated', 0, 255, 1, True),
    ParameterSpec('advanced', 'refinement', 'local_recheck_confidence_below', 'recheck_conf', 0.0, 1.0, 0.01),
    ParameterSpec('advanced', 'refinement', 'local_recheck_rms_above_px', 'recheck_rms', 0.1, 10.0, 0.1),
    ParameterSpec('advanced', 'refinement', 'local_recheck_interval_frames', 'recheck_interval', 1, 30, 1, True),
    ParameterSpec('advanced', 'refinement', 'reacquisition_jump_px', 'reacq_jump', 1, 100, 1, True),
    ParameterSpec('advanced', 'stability', 'window_frames', 'window_frames', 2, 30, 1, True),
    ParameterSpec('advanced', 'stability', 'maximum_center_jitter_px', 'center_jitter', 0.5, 30.0, 0.5),
    ParameterSpec('advanced', 'stability', 'maximum_radius_jitter_px', 'radius_jitter', 0.5, 30.0, 0.5),
)


@dataclass(frozen=True)
class AutoDiagnosisResult:
    success: bool
    document: dict
    candidate: DebugCircleCandidate | None
    message: str
    accumulator_threshold: float


def _accumulator_attempts(current: float) -> tuple:
    values = [float(current)]
    for value in (30.0, 26.0, 22.0, 18.0):
        if value < float(current) and value not in values:
            values.append(value)
    return tuple(values)


def auto_diagnose_frame(frame, document) -> AutoDiagnosisResult:
    """宽范围寻找最高质量圆，只建议半径范围和必要 Hough 阈值。"""
    original = copy.deepcopy(document)
    current_accumulator = float(original['coarse_search']['accumulator_threshold'])
    failure_details = []
    for accumulator in _accumulator_attempts(current_accumulator):
        trial = copy.deepcopy(original)
        trial['coarse_search']['minimum_radius_px'] = 40.0
        trial['coarse_search']['maximum_radius_px'] = 350.0
        trial['coarse_search']['accumulator_threshold'] = accumulator
        trial['coarse_search']['maximum_candidates'] = max(
            8, int(trial['coarse_search']['maximum_candidates']))
        trial['refinement']['radial_search_half_width_px'] = max(
            60.0, float(trial['refinement']['radial_search_half_width_px']))
        trial['refinement']['minimum_visible_arc_deg'] = 40.0
        trial['refinement']['maximum_fit_rms_px'] = 8.0
        trial['refinement']['local_recheck_rms_above_px'] = min(
            6.0, float(trial['refinement']['maximum_fit_rms_px']))
        try:
            debug = TurntableDetector(
                turntable_config_from_document(trial)).inspect_frame(frame)
        except (TurntableDetectionError, cv2.error) as error:
            failure_details.append(str(error))
            continue
        if debug.selected_index is None:
            failure_details.append(debug.detail)
            continue
        candidate = debug.candidates[debug.selected_index]
        radius = float(candidate.fit.radius_px)
        proposed = copy.deepcopy(original)
        proposed['coarse_search']['minimum_radius_px'] = float(max(20, math.floor(radius - 40.0)))
        proposed['coarse_search']['maximum_radius_px'] = float(min(400, math.ceil(radius + 40.0)))
        if accumulator < current_accumulator:
            proposed['coarse_search']['accumulator_threshold'] = accumulator
        turntable_config_from_document(proposed)
        return AutoDiagnosisResult(
            True, proposed, candidate,
            f'找到外圆：中心({candidate.fit.center_x_px:.1f},{candidate.fit.center_y_px:.1f})，'
            f'半径{candidate.fit.radius_px:.1f}px', accumulator)
    detail = failure_details[-1] if failure_details else '没有产生诊断结果'
    return AutoDiagnosisResult(False, original, None, f'自动诊断失败：{detail}', current_accumulator)


class ParameterPanel:
    """维护两页自绘滑条，并把鼠标动作映射到配置字典。"""

    ROW_START = 90
    ROW_HEIGHT = 32
    TRACK_LEFT = 14
    TRACK_RIGHT = PANEL_WIDTH - 14

    def __init__(self, document):
        self.document = copy.deepcopy(document)
        self.page = 'common'
        self.dragging_index = None
        self.selected_index = None
        self.dirty = False
        self.slider_changed = False

    @property
    def specs(self):
        return tuple(spec for spec in PARAMETER_SPECS if spec.page == self.page)

    def replace_document(self, document):
        turntable_config_from_document(document)
        self.document = copy.deepcopy(document)
        self.dragging_index = None
        self.selected_index = None
        self.dirty = True
        self.slider_changed = False

    def toggle_page(self):
        self.page = 'advanced' if self.page == 'common' else 'common'
        self.dragging_index = None
        self.selected_index = None

    def value(self, spec):
        return self.document[spec.group][spec.key]

    def set_value(self, spec, value):
        clipped = min(spec.maximum, max(spec.minimum, float(value)))
        steps = round((clipped - spec.minimum) / spec.step)
        quantized = spec.minimum + steps * spec.step
        quantized = int(round(quantized)) if spec.integer else float(quantized)
        self.document[spec.group][spec.key] = quantized
        coarse = self.document['coarse_search']
        if spec.key == 'minimum_radius_px' and coarse['minimum_radius_px'] >= coarse['maximum_radius_px']:
            coarse['maximum_radius_px'] = float(min(400, coarse['minimum_radius_px'] + 1))
        elif spec.key == 'maximum_radius_px' and coarse['maximum_radius_px'] <= coarse['minimum_radius_px']:
            coarse['minimum_radius_px'] = float(max(20, coarse['maximum_radius_px'] - 1))
        refinement = self.document['refinement']
        if spec.key == 'maximum_fit_rms_px' and refinement['local_recheck_rms_above_px'] > refinement['maximum_fit_rms_px']:
            refinement['local_recheck_rms_above_px'] = float(refinement['maximum_fit_rms_px'])
        elif spec.key == 'local_recheck_rms_above_px' and refinement['local_recheck_rms_above_px'] > refinement['maximum_fit_rms_px']:
            refinement['maximum_fit_rms_px'] = float(refinement['local_recheck_rms_above_px'])
        turntable_config_from_document(self.document)
        self.dirty = True
        self.slider_changed = True

    def set_fraction(self, spec, fraction):
        self.set_value(spec, spec.minimum + min(1.0, max(0.0, fraction)) * (spec.maximum - spec.minimum))

    def mouse_callback(self, event, x_value, y_value, _flags, _parameter):
        if event == cv2.EVENT_LBUTTONUP:
            self.dragging_index = None
            return
        if x_value < FRAME_WIDTH:
            return
        panel_x = x_value - FRAME_WIDTH
        row_index = int((y_value - self.ROW_START) // self.ROW_HEIGHT)
        specs = self.specs
        if event == cv2.EVENT_LBUTTONDOWN:
            if 0 <= row_index < len(specs):
                self.dragging_index = row_index
                self.selected_index = row_index
                fraction = (panel_x - self.TRACK_LEFT) / (self.TRACK_RIGHT - self.TRACK_LEFT)
                self.set_fraction(specs[row_index], fraction)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging_index is not None:
            fraction = (panel_x - self.TRACK_LEFT) / (self.TRACK_RIGHT - self.TRACK_LEFT)
            self.set_fraction(specs[self.dragging_index], fraction)


def _best_display_candidate(debug):
    if debug.selected_index is not None:
        return debug.candidates[debug.selected_index]
    usable = [c for c in debug.candidates
              if c.fit.radius_px > 0.0 and math.isfinite(c.fit.fit_rms_px)]
    if not usable:
        return None
    return max(usable, key=lambda item: (item.fit.visible_arc_deg, -item.fit.fit_rms_px))


def draw_debug_overlay(frame, debug, show_points):
    """黄色画粗圆、绿色画通过圆、红色画被拒绝圆。"""
    best = _best_display_candidate(debug)
    for index, candidate in enumerate(debug.candidates, start=1):
        seed_x, seed_y, seed_radius = candidate.seed
        cv2.circle(frame, (int(round(seed_x)), int(round(seed_y))),
                   int(round(seed_radius)), (0, 220, 255), 1, cv2.LINE_8)
        fit = candidate.fit
        if fit.radius_px > 0.0 and math.isfinite(fit.radius_px):
            color = (40, 255, 40) if fit.detected else (30, 30, 255)
            thickness = 3 if candidate is best else 1
            center = (int(round(fit.center_x_px)), int(round(fit.center_y_px)))
            cv2.circle(frame, center, int(round(fit.radius_px)), color, thickness, cv2.LINE_8)
            cv2.putText(frame, f'C{index}', center, cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 1, cv2.LINE_AA)
    if show_points and best is not None:
        for point in best.sample_points[::2]:
            cv2.circle(frame, (int(round(point[0])), int(round(point[1]))),
                       2, (255, 0, 255), -1)
    return frame


def render_panel(panel, result, debug, message, fps, frozen, show_points):
    """ASCII 侧栏：标题、状态、每条参数的滑条。"""
    output = np.full((FRAME_HEIGHT, PANEL_WIDTH, 3), 26, dtype=np.uint8)
    state = 'STABLE' if result.stable else ('DETECTED' if result.detected else 'SEARCH')
    cv2.putText(output, f'Turntable [{panel.page}]', (12, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
    cv2.putText(output, f'{state} cand={len(debug.candidates)} fps={fps:.0f}',
                (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (80, 240, 120) if result.detected else (255, 110, 90), 1, cv2.LINE_AA)
    cv2.putText(output, str(message)[:44], (12, 72), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (200, 200, 200), 1, cv2.LINE_AA)
    for index, spec in enumerate(panel.specs):
        y_value = panel.ROW_START + index * panel.ROW_HEIGHT
        value = float(panel.value(spec))
        fraction = (value - spec.minimum) / (spec.maximum - spec.minimum)
        selected = index == panel.selected_index
        color = (255, 220, 90) if selected else (220, 220, 220)
        cv2.putText(output, f'{spec.label} {spec.format_value(value)}',
                    (panel.TRACK_LEFT, y_value + 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, color, 1, cv2.LINE_AA)
        track_y = y_value + 16
        cv2.line(output, (panel.TRACK_LEFT, track_y),
                 (panel.TRACK_RIGHT, track_y), (65, 68, 76), 4)
        knob_x = int(round(panel.TRACK_LEFT + fraction * (panel.TRACK_RIGHT - panel.TRACK_LEFT)))
        cv2.circle(output, (knob_x, track_y), 5, (255, 255, 255), -1)
    hint = f'{"FROZEN" if frozen else "LIVE"} pts={"on" if show_points else "off"}'
    cv2.putText(output, hint, (12, FRAME_HEIGHT - 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (180, 210, 255), 1, cv2.LINE_AA)
    return output


def open_camera(device, fps, fourcc='MJPG', timeout=5.0):
    camera = TimedCamera(device, FRAME_WIDTH, FRAME_HEIGHT, fps, fourcc, timeout)
    ok, frame = camera.read()
    if ok and frame is not None:
        if frame.shape[:2] == (FRAME_HEIGHT, FRAME_WIDTH):
            return camera
        camera.release()
        raise TurntableSettingsError(
            f'摄像头输出{frame.shape[1]}x{frame.shape[0]}，需要{FRAME_WIDTH}x{FRAME_HEIGHT}')
    camera.release()
    raise TurntableSettingsError('摄像头已打开，但无法读取画面')


def parse_args():
    parser = argparse.ArgumentParser(description='可视化设置转盘外圆识别参数')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--fourcc', default='MJPG')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--flip-code', type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    return parser.parse_args()


def run(args):
    document = load_turntable_detector_document(args.config)
    panel = ParameterPanel(document)
    detector = TurntableDetector(turntable_config_from_document(document))
    require_display()
    camera = open_camera(args.device, args.fps, args.fourcc, args.timeout)
    window_name = 'Turntable Parameter Settings'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, CANVAS_WIDTH, FRAME_HEIGHT)
    cv2.setMouseCallback(window_name, panel.mouse_callback)

    frozen = False
    frozen_frame = None
    show_points = False
    latest_frame = None
    debug = TurntableDebugResult((), None, 0.0, '等待第一帧')
    message = 'A 自动诊断，或拖动右侧滑条'
    frame_times = []
    result = None
    try:
        while True:
            if not frozen or frozen_frame is None:
                ok, raw = camera.read()
                if not ok or raw is None:
                    raise TurntableSettingsError('设置过程中摄像头读取失败')
                latest_frame = cv2.flip(raw, args.flip_code) if args.flip_code else raw.copy()
                if frozen:
                    frozen_frame = latest_frame.copy()
            source = frozen_frame if frozen and frozen_frame is not None else latest_frame
            if source is None:
                continue

            configuration_changed = panel.dirty
            if configuration_changed:
                detector = TurntableDetector(turntable_config_from_document(panel.document))
                panel.dirty = False
                message = '参数已应用，检测历史已清空' if panel.slider_changed else message
                panel.slider_changed = False

            if not frozen or result is None or configuration_changed:
                result, debug = detector.detect_with_debug(source)
            preview = source.copy()
            draw_debug_overlay(preview, debug, show_points)
            draw_turntable_detection(preview, result)

            now = time.monotonic()
            frame_times.append(now)
            if len(frame_times) > 45:
                del frame_times[0]
            fps = 0.0
            if len(frame_times) >= 2:
                elapsed = frame_times[-1] - frame_times[0]
                if elapsed > 0.0:
                    fps = (len(frame_times) - 1) / elapsed
            side = render_panel(panel, result, debug, message, fps, frozen, show_points)
            canvas = np.hstack((preview, side))
            cv2.imshow(window_name, canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key == 9:  # Tab
                panel.toggle_page()
            elif key == 32:  # Space
                frozen = not frozen
                frozen_frame = source.copy() if frozen else None
                message = '画面已冻结' if frozen else '已恢复实时画面'
            elif key in (ord('p'), ord('P')):
                show_points = not show_points
                message = '已显示采样点' if show_points else '已隐藏采样点'
            elif key in (ord('a'), ord('A')):
                diagnosis = auto_diagnose_frame(source, panel.document)
                message = diagnosis.message
                if diagnosis.success:
                    panel.replace_document(diagnosis.document)
            elif key in (ord('r'), ord('R')):
                panel.replace_document(load_turntable_detector_document(args.config))
                message = '已重新读取磁盘中最后保存的参数'
            elif key in (ord('d'), ord('D')):
                panel.replace_document(build_default_turntable_detector_document())
                message = '已恢复推荐默认值；按 S 才会写入磁盘'
            elif key in (ord('s'), ord('S')):
                save_turntable_detector_config(args.config, panel.document)
                message = ('参数已保存' if result.stable else '警告：当前未稳定，仍已保存')
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
