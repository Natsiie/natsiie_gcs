#!/usr/bin/env python3
"""六色 HSV 标定工具（GUI）。

数字键 1~6 选择颜色，随后在物料内部按住鼠标左键拖出采样区域；同一颜色
可累积多个区域，工具用 ``estimate_hsv_bounds`` 自动统计稳健的 HSV 上下限。
下方六条 trackbar（H/S/V 低/高）可继续人工微调，按 ``S`` 保存 JSON。

运行方式::

    python3 color_calibrator.py --device /dev/camera_upper --config color_config.json

按键：1-6 选色；U 撤销；C 清空当前色；R 恢复预设；S 保存；Q/Esc 退出。
未匹配区域被压暗到 25% 亮度，直接看出漏选与串色。
"""

from __future__ import annotations

import argparse
import os
import sys
from collections import defaultdict

import cv2
import numpy as np

try:
    from identify_pack.vision_identify.color_config import (
        COLOR_DISPLAY, COLOR_NAMES, PRESET_RANGES, bounds_to_ranges,
        build_default_color_config, estimate_hsv_bounds, get_color_bounds,
        load_color_config, ranges_to_bounds, save_color_config,
        set_color_bounds,
    )
except ImportError:  # 直接在源码目录运行
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'vision_identify'))
    from color_config import (  # type: ignore
        COLOR_DISPLAY, COLOR_NAMES, PRESET_RANGES, bounds_to_ranges,
        build_default_color_config, estimate_hsv_bounds, get_color_bounds,
        load_color_config, ranges_to_bounds, save_color_config,
        set_color_bounds,
    )


COLOR_IDS = sorted(COLOR_NAMES)
BOUND_KEYS = ("H_LOW", "H_HIGH", "S_LOW", "S_HIGH", "V_LOW", "V_HIGH")
MAXIMA = (179, 179, 255, 255, 255, 255)
WINDOW = "color_calibrator"
MAX_SAMPLES_PER_COLOR = 20
DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), "color_config.json")


class CalibrationSession:
    """多颜色采样、撤销、重置和阈值状态（不依赖窗口）。"""

    def __init__(self, config: dict):
        self.config = config
        self.samples = {color_id: [] for color_id in COLOR_IDS}
        self.baseline = {color_id: None for color_id in COLOR_IDS}
        self.selected = 1
        self.dirty = False

    def bounds(self, color_id: int) -> tuple[int, ...]:
        return get_color_bounds(self.config, color_id)

    def set_bounds(self, color_id: int, bounds) -> None:
        if bounds != self.bounds(color_id):
            set_color_bounds(self.config, color_id, bounds)
            self.dirty = True

    def add_sample(self, color_id: int, hsv_pixels: np.ndarray) -> None:
        color_id = int(color_id)
        pixels = np.asarray(hsv_pixels, dtype=np.uint8).reshape(-1, 3)
        if len(pixels) < 100:
            raise ValueError("采样区域太小，有效像素不足100")
        if len(self.samples[color_id]) >= MAX_SAMPLES_PER_COLOR:
            raise ValueError(f"每种颜色最多 {MAX_SAMPLES_PER_COLOR} 个样本")
        if not self.samples[color_id]:
            self.baseline[color_id] = self.bounds(color_id)
        self.samples[color_id].append(pixels)
        self.set_bounds(color_id, estimate_hsv_bounds(self.samples[color_id], color_id))

    def undo(self, color_id: int) -> bool:
        color_id = int(color_id)
        if not self.samples[color_id]:
            return False
        self.samples[color_id].pop()
        if self.samples[color_id]:
            self.set_bounds(color_id, estimate_hsv_bounds(self.samples[color_id], color_id))
        elif self.baseline[color_id] is not None:
            self.set_bounds(color_id, self.baseline[color_id])
            self.baseline[color_id] = None
        return True

    def clear(self, color_id: int) -> None:
        color_id = int(color_id)
        if self.samples[color_id]:
            self.samples[color_id].clear()
            if self.baseline[color_id] is not None:
                self.set_bounds(color_id, self.baseline[color_id])
        self.baseline[color_id] = None

    def reset(self, color_id: int) -> None:
        color_id = int(color_id)
        self.samples[color_id].clear()
        self.baseline[color_id] = None
        self.set_bounds(color_id, ranges_to_bounds(PRESET_RANGES[color_id]))

    def mark_saved(self) -> None:
        self.dirty = False


def _mask_from_bounds(hsv: np.ndarray, bounds) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for hsv_range in bounds_to_ranges(bounds):
        lower = np.asarray(hsv_range["lower"], dtype=np.uint8)
        upper = np.asarray(hsv_range["upper"], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    return mask


def _preview(frame: np.ndarray, bounds) -> np.ndarray:
    """未匹配区域压暗到 25% 亮度，匹配像素保持原色。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = _mask_from_bounds(hsv, bounds)
    output = (frame.astype(np.float32) * 0.25).astype(np.uint8)
    output[mask > 0] = frame[mask > 0]
    return output


def _sync_trackbars(bounds) -> None:
    for key, value in zip(BOUND_KEYS, bounds):
        cv2.setTrackbarPos(key, WINDOW, int(value))


def main() -> int:
    p = argparse.ArgumentParser(description="六色 HSV 拖框采样与滑条标定工具")
    p.add_argument("--device", default="/dev/camera_upper", help="相机设备路径")
    p.add_argument("--config", default=DEFAULT_CONFIG, help="颜色配置 JSON 路径")
    p.add_argument("--fps", type=float, default=30.0)
    args = p.parse_args()

    try:
        config = load_color_config(args.config) if os.path.exists(args.config) \
            else build_default_color_config()
    except Exception as error:
        print(f"加载颜色配置失败：{error}")
        return 1

    session = CalibrationSession(config)
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        try:
            cap = cv2.VideoCapture(int(args.device))
        except Exception:
            pass
    if not cap.isOpened():
        print(f"无法打开相机: {args.device}")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    for key, maximum in zip(BOUND_KEYS, MAXIMA):
        cv2.createTrackbar(key, WINDOW, 0, maximum, lambda x: None)

    state = {
        "frozen": None,
        "drag_start": None,
        "drag_current": None,
        "rects": defaultdict(list),
        "status": "1-6 选色，拖框采样，S 保存",
    }

    def _sync() -> None:
        _sync_trackbars(session.bounds(session.selected))

    def on_mouse(event, x, y, _flags, _param) -> None:
        if state["frozen"] is None:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            state["drag_start"] = (x, y)
            state["drag_current"] = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and state["drag_start"] is not None:
            state["drag_current"] = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and state["drag_start"] is not None:
            x1, y1 = state["drag_start"]
            x2, y2 = (x, y)
            rect = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
            try:
                rx1, ry1, rx2, ry2 = rect
                if rx2 - rx1 < 10 or ry2 - ry1 < 10:
                    raise ValueError("ROI 宽高至少 10px")
                roi = state["frozen"][ry1:ry2, rx1:rx2]
                hsv_pixels = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).reshape(-1, 3)
                session.add_sample(session.selected, hsv_pixels)
                state["rects"][session.selected].append(rect)
                _sync()
                n = len(session.samples[session.selected])
                state["status"] = f"采样已接受：{n}/{MAX_SAMPLES_PER_COLOR}"
            except ValueError as error:
                state["status"] = f"采样被拒绝：{error}"
            finally:
                state["drag_start"] = None
                state["drag_current"] = None
                state["frozen"] = None

    cv2.setMouseCallback(WINDOW, on_mouse)
    _sync()
    print("1-6 选色；拖框采样；U 撤销；C 清空；R 预设；S 保存；Q/Esc 退出")

    while True:
        # 读取当前颜色阈值，供拖框采样时冻结画面使用。
        bounds = session.bounds(session.selected)
        ok, raw = cap.read()
        if ok and raw is not None:
            # 拖框期间冻结画面，供 LBUTTONUP 时从按下那帧采样。
            if state["drag_start"] is None:
                state["frozen"] = raw.copy()

        # 应用 trackbar 微调。
        values = tuple(cv2.getTrackbarPos(key, WINDOW) for key in BOUND_KEYS)
        valid = values[2] <= values[3] and values[4] <= values[5]
        if valid:
            if values != bounds:
                session.set_bounds(session.selected, values)
                bounds = values
        else:
            state["status"] = "INVALID: S_LOW<=S_HIGH 且 V_LOW<=V_HIGH"

        display = _preview(raw if raw is not None else np.zeros((480, 640, 3), np.uint8), bounds)
        name = COLOR_NAMES[session.selected]
        color = COLOR_DISPLAY[name]
        for rect in state["rects"][session.selected]:
            x1, y1, x2, y2 = rect
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        if state["drag_start"] is not None and state["drag_current"] is not None:
            x1, y1 = state["drag_start"]
            x2, y2 = state["drag_current"]
            cv2.rectangle(display, (x1, y1), (x2, y2), (255, 255, 255), 2)

        h_low, h_high, s_low, s_high, v_low, v_high = bounds
        panel = (
            f"{session.selected}:{name}  samples={len(session.samples[session.selected])}  "
            f"{'DIRTY' if session.dirty else 'SAVED'}  "
            f"H={h_low}..{h_high}{' WRAP' if h_low > h_high else ''}  "
            f"S={s_low}..{s_high}  V={v_low}..{v_high}"
        )
        cv2.putText(display, panel, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(display, state["status"], (10, 44), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow(WINDOW, display)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            if session.dirty:
                print("未保存的 HSV 修改已放弃")
            break
        if ord("1") <= key <= ord("6"):
            session.selected = key - ord("0")
            _sync()
            state["status"] = f"已选择 {session.selected} {COLOR_NAMES[session.selected]}"
        elif key in (ord("u"), ord("U")):
            if session.undo(session.selected):
                state["rects"][session.selected].pop()
                _sync()
                state["status"] = "已撤销最后一次采样"
            else:
                state["status"] = "没有可撤销的采样"
        elif key in (ord("c"), ord("C")):
            session.clear(session.selected)
            state["rects"][session.selected].clear()
            _sync()
            state["status"] = "已清空当前颜色采样"
        elif key in (ord("r"), ord("R")):
            session.reset(session.selected)
            state["rects"][session.selected].clear()
            _sync()
            state["status"] = "已恢复当前颜色预设"
        elif key in (ord("s"), ord("S")):
            try:
                if not valid:
                    raise ValueError("S_LOW<=S_HIGH 且 V_LOW<=V_HIGH")
                save_color_config(args.config, session.config)
                session.mark_saved()
                state["status"] = f"已保存：{args.config}"
                print(f"颜色配置已保存：{os.path.abspath(args.config)}")
            except Exception as error:
                state["status"] = f"保存失败：{error}"
                print(f"保存失败：{error}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
