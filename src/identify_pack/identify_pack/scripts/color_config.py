#!/usr/bin/env python3
"""七色 HSV 阈值配置与掩膜工具（纯库，无 ROS / 无 GUI）。

六种抓取色（1~6）+ 第 7 号粉色标记（舵机外壳，仅用于夹爪到位检测，不参与抓取）。

把颜色识别从硬编码阈值迁移为 JSON 配置驱动：
  - ``load_color_config`` 读取并校验配置；``save_color_config`` 原子写回。
  - ``make_all_color_masks`` 一次 BGR→HSV 生成七色掩膜。
  - ``estimate_hsv_bounds`` 根据采样 ROI 稳健估计 HSV 上下限。
  - ``classify_circle`` 在圆盘内按颜色支持率判色（含「空」）。

颜色编号沿用创源启明方案：1红 2黄 3蓝 4绿 5黑 6浅蓝 7粉。
红色 Hue 环绕（179→0）在 ``bounds_to_ranges`` 中自动拆成两段；
黑色 Hue 无意义，估计时使用全 Hue 并约束 S/V。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable

import cv2
import numpy as np


class ColorConfigError(ValueError):
    """颜色配置或输入不满足要求。"""


# 颜色编号 -> 名称 / BGR 显示色。
COLOR_NAMES = {
    1: "red",
    2: "yellow",
    3: "blue",
    4: "green",
    5: "black",
    6: "light_blue",
    7: "pink",
}
NAME_TO_ID = {name: color_id for color_id, name in COLOR_NAMES.items()}

# 所有已知颜色编号 / 抓取目标色 / 标记色（粉色舵机外壳，不参与抓取）。
COLOR_IDS = tuple(sorted(COLOR_NAMES))
PICKUP_COLOR_IDS = (1, 2, 3, 4, 5, 6)
MARKER_COLOR_IDS = tuple(c for c in COLOR_IDS if c not in PICKUP_COLOR_IDS)

COLOR_DISPLAY = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "light_blue": (255, 255, 0),
    "black": (128, 128, 128),
    "pink": (203, 192, 255),
}

# 滑条 / 估计输出的六元组顺序。
BOUND_KEYS = ("h_low", "h_high", "s_low", "s_high", "v_low", "v_high")

# 与旧 COLOR_RANGES 一致的预设阈值。
PRESET_RANGES = {
    1: [{"lower": [0, 100, 100], "upper": [10, 255, 255]},
        {"lower": [160, 100, 100], "upper": [179, 255, 255]}],
    2: [{"lower": [22, 100, 100], "upper": [38, 255, 255]}],
    3: [{"lower": [85, 60, 60], "upper": [135, 255, 150]}],
    4: [{"lower": [38, 40, 40], "upper": [85, 255, 255]}],
    5: [{"lower": [0, 0, 0], "upper": [179, 255, 50]}],
    6: [{"lower": [85, 40, 150], "upper": [135, 255, 255]}],
    7: [{"lower": [150, 40, 120], "upper": [179, 200, 255]}],
}


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ColorConfigError(message)


# ---------- 阈值区间与六元组互转 ----------
def bounds_to_ranges(bounds: Iterable[int]) -> list[dict]:
    """把 (h_low,h_high,s_low,s_high,v_low,v_high) 转成 inRange 区间列表。

    Hue 是长度 180 的圆环；H_LOW > H_HIGH 表示跨越红色 179→0 边界，
    因此拆成 ``0..h_high`` 与 ``h_low..179`` 两段。
    """
    values = tuple(int(v) for v in bounds)
    _require(len(values) == 6, "HSV 六元组必须正好包含 6 个整数")
    h_low, h_high, s_low, s_high, v_low, v_high = values
    _require(0 <= h_low <= 179 and 0 <= h_high <= 179, "Hue 必须位于 0~179")
    _require(0 <= s_low <= s_high <= 255, "S 上下限无效")
    _require(0 <= v_low <= v_high <= 255, "V 上下限无效")
    if h_low <= h_high:
        return [{"lower": [h_low, s_low, v_low],
                 "upper": [h_high, s_high, v_high]}]
    return [
        {"lower": [0, s_low, v_low], "upper": [h_high, s_high, v_high]},
        {"lower": [h_low, s_low, v_low], "upper": [179, s_high, v_high]},
    ]


def ranges_to_bounds(ranges: list[dict]) -> tuple[int, ...]:
    """把配置区间恢复成六元组，严格拒绝无法表达的组合。"""
    _require(isinstance(ranges, list) and 1 <= len(ranges) <= 2,
             "每种颜色必须包含 1 或 2 个 HSV 区间")

    def triplet(hsv_range: dict, key: str) -> tuple[int, int, int]:
        try:
            values = tuple(int(v) for v in hsv_range[key])
        except (KeyError, TypeError, ValueError) as error:
            raise ColorConfigError("HSV 区间字段格式无效") from error
        _require(len(values) == 3, "HSV 上下限必须正好包含 3 个整数")
        return values

    if len(ranges) == 1:
        lower = triplet(ranges[0], "lower")
        upper = triplet(ranges[0], "upper")
        return (lower[0], upper[0], lower[1], upper[1], lower[2], upper[2])

    low_hue_part, high_hue_part = ranges
    first_lower = triplet(low_hue_part, "lower")
    first_upper = triplet(low_hue_part, "upper")
    second_lower = triplet(high_hue_part, "lower")
    second_upper = triplet(high_hue_part, "upper")
    _require(first_lower[0] == 0 and second_upper[0] == 179,
             "双 HSV 区间必须表示跨越 Hue 边界的范围")
    _require(second_lower[0] > first_upper[0],
             "双 HSV 区间必须在 Hue 边界两侧且不能互相重叠")
    _require(first_lower[1:] == second_lower[1:]
             and first_upper[1:] == second_upper[1:],
             "跨 Hue 边界的两个区间必须使用相同 S/V 范围")
    return (second_lower[0], first_upper[0],
            first_lower[1], first_upper[1],
            first_lower[2], first_upper[2])


# ---------- 配置构造 / 校验 / 读写 ----------
def build_default_color_config() -> dict:
    """用预设阈值构造一份全新配置。"""
    colors = []
    for color_id in sorted(COLOR_NAMES):
        colors.append({
            "color_id": color_id,
            "name": COLOR_NAMES[color_id],
            "ranges": [dict(r) for r in PRESET_RANGES[color_id]],
        })
    return {"format_version": 1, "colors": colors}


def _find_color(document: dict, color) -> dict:
    if isinstance(color, str):
        matches = [item for item in document["colors"] if item["name"] == color]
    else:
        matches = [item for item in document["colors"]
                   if int(item["color_id"]) == int(color)]
    if len(matches) != 1:
        raise ColorConfigError(f"未知颜色：{color}")
    return matches[0]


def validate_color_config(document: dict) -> dict:
    """严格校验精简配置格式，返回原字典供调用方继续使用。"""
    try:
        _require(isinstance(document, dict), "颜色配置根节点必须是对象")
        _require(int(document["format_version"]) == 1,
                 "不支持的颜色配置版本")
        colors = document["colors"]
        _require(isinstance(colors, list) and len(colors) == len(COLOR_NAMES),
                 f"颜色配置必须正好包含 {len(COLOR_NAMES)} 种颜色")
        ids = {int(item["color_id"]) for item in colors}
        _require(ids == set(COLOR_NAMES), "颜色编号不完整或重复")
        limits = (179, 255, 255)
        for item in colors:
            color_id = int(item["color_id"])
            _require(item["name"] == COLOR_NAMES[color_id],
                     f"颜色{color_id}名称必须为{COLOR_NAMES[color_id]}")
            ranges = item["ranges"]
            _require(isinstance(ranges, list) and 1 <= len(ranges) <= 2,
                     f"{COLOR_NAMES[color_id]}必须包含1或2个HSV区间")
            # ranges_to_bounds 同时验证双区间是否是合法的 Hue 环绕。
            bounds = ranges_to_bounds(ranges)
            bounds_to_ranges(bounds)
            for hsv_range in ranges:
                lower = tuple(int(v) for v in hsv_range["lower"])
                upper = tuple(int(v) for v in hsv_range["upper"])
                _require(len(lower) == 3 and len(upper) == 3,
                         f"{COLOR_NAMES[color_id]} HSV 区间长度无效")
                _require(all(0 <= lower[i] <= upper[i] <= limits[i]
                             for i in range(3)),
                         f"{COLOR_NAMES[color_id]} HSV 区间越界")
    except ColorConfigError:
        raise
    except (IndexError, KeyError, TypeError, ValueError) as error:
        raise ColorConfigError("颜色配置字段格式无效") from error
    return document


def load_color_config(path: Path | str) -> dict:
    """读取并校验六色 HSV JSON 配置。"""
    resolved = Path(path).expanduser().resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8"))
    except OSError as error:
        raise ColorConfigError(f"无法读取颜色配置：{resolved}: {error}") from error
    except json.JSONDecodeError as error:
        raise ColorConfigError(f"颜色配置不是有效 JSON：{resolved}") from error
    return validate_color_config(document)


def save_color_config(path: Path | str, document: dict) -> None:
    """在同目录原子保存配置，避免中断留下半个 JSON。"""
    validate_color_config(document)
    resolved = Path(path).expanduser().resolve()
    _require(resolved.parent.is_dir(), f"配置目录不存在：{resolved.parent}")
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=resolved.parent,
            prefix=f".{resolved.name}.", suffix=".tmp", delete=False,
        ) as temporary:
            temporary_name = temporary.name
            json.dump(document, temporary, ensure_ascii=False, indent=2)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.chmod(temporary_name, 0o644)
        os.replace(temporary_name, resolved)
    except OSError as error:
        if temporary_name is not None:
            try:
                Path(temporary_name).unlink(missing_ok=True)
            except OSError:
                pass
        raise ColorConfigError(f"无法保存颜色配置：{resolved}: {error}") from error


def get_color_bounds(document: dict, color_id: int) -> tuple[int, ...]:
    """返回指定颜色的六元组滑条值。"""
    return ranges_to_bounds(_find_color(document, color_id)["ranges"])


def set_color_bounds(document: dict, color_id: int, bounds: Iterable[int]) -> None:
    """用六元组滑条值更新指定颜色的区间。"""
    item = _find_color(document, color_id)
    item["ranges"] = bounds_to_ranges(bounds)


# ---------- 掩膜生成 ----------
def _mask_from_hsv(hsv: np.ndarray, ranges: list[dict]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for hsv_range in ranges:
        lower = np.asarray(hsv_range["lower"], dtype=np.uint8)
        upper = np.asarray(hsv_range["upper"], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    return mask


def make_color_mask(frame: np.ndarray, color, config: dict) -> np.ndarray:
    """从 BGR 图像生成指定颜色的单通道掩膜。"""
    validate_color_config(config)
    _require(frame is not None and frame.ndim == 3 and frame.shape[2] == 3,
             "颜色识别要求 BGR 三通道图像")
    item = _find_color(config, color)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return _mask_from_hsv(hsv, item["ranges"])


def make_all_color_masks(frame: np.ndarray, config: dict) -> dict[int, np.ndarray]:
    """只做一次 BGR→HSV 转换并生成全部六色掩膜。"""
    validate_color_config(config)
    _require(frame is not None and frame.ndim == 3 and frame.shape[2] == 3,
             "颜色识别要求 BGR 三通道图像")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {
        int(item["color_id"]): _mask_from_hsv(hsv, item["ranges"])
        for item in config["colors"]
    }


def classify_circle(masks: dict[int, np.ndarray], center, radius,
                    min_support: float = 0.5) -> tuple[int, float]:
    """在圆盘内统计六色支持率，返回 (color_id, support)。

    取半径 0.85×r 的圆盘避免边界噪声。支持率最大且不低于
    ``min_support`` 的颜色作为该圆的颜色，否则返回 ``0``（空）。
    """
    if not masks:
        return 0, 0.0
    height, width = next(iter(masks.values())).shape[:2]
    cx = int(round(float(center[0])))
    cy = int(round(float(center[1])))
    r = max(1, int(round(float(radius) * 0.85)))
    x0, x1 = max(0, cx - r), min(width, cx + r + 1)
    y0, y1 = max(0, cy - r), min(height, cy + r + 1)
    if x1 <= x0 or y1 <= y0:
        return 0, 0.0
    yy, xx = np.ogrid[y0:y1, x0:x1]
    disk = (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r
    pixel_count = max(1, int(np.count_nonzero(disk)))
    best_id, best_support = 0, 0.0
    for color_id in sorted(masks):
        colored = int(np.count_nonzero(masks[color_id][y0:y1, x0:x1][disk]))
        support = colored / pixel_count
        if support > best_support:
            best_support = support
            best_id = color_id
    if best_support < float(min_support):
        best_id = 0
    return best_id, best_support


def find_color_blobs(masks: dict[int, np.ndarray],
                     min_area: float = 300.0,
                     min_dimension: float = 12.0,
                     open_kernel_px: int = 5,
                     close_kernel_px: int = 11,
                     max_aspect_ratio: float | None = None,
                     min_solidity: float | None = None,
                     min_extent: float | None = None,
                     roi: tuple | None = None):
    """在六色掩膜上找连通域色块（轮廓 blob 定位，不依赖圆形几何）。

    每张 mask 先开运算去噪、闭运算填缝，再 ``findContours`` 提取外轮廓，
    按面积/最小边长/长宽比/实心度/填充率过滤，返回色块列表。

    ``roi`` 为 ``(x, y, w, h)``、``[(x, y, w, h), ...]`` 或 ``None``。
    指定时仅在给定矩形内检测，ROI 外的掩膜清零，跨边界目标只保留 ROI 内部分。

    返回：``[(color_id, cx, cy, x, y, w, h, area), ...]``
    其中 ``(cx, cy)`` 是质心，``(x, y, w, h)`` 是外接矩形。
    """
    blobs = []
    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (int(open_kernel_px), int(open_kernel_px)))
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (int(close_kernel_px), int(close_kernel_px)))
    roi_mask = None
    if roi is not None:
        # 支持单个 (x,y,w,h) 或 [(x,y,w,h), ...] 列表。
        if isinstance(roi, (list, tuple)) and roi and isinstance(roi[0], (list, tuple)):
            roi_list = list(roi)
        else:
            roi_list = [roi]
        height, width = next(iter(masks.values())).shape[:2]
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        for item in roi_list:
            rx, ry, rw, rh = (int(v) for v in item)
            x2, y2 = min(width, rx + rw), min(height, ry + rh)
            if x2 > rx and y2 > ry:
                cv2.rectangle(roi_mask, (rx, ry), (x2, y2), 255, -1)
    for color_id in sorted(masks):
        source = masks[color_id]
        if roi_mask is not None:
            source = cv2.bitwise_and(source, roi_mask)
        mask = cv2.morphologyEx(source, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if min(w, h) < min_dimension:
                continue
            if max_aspect_ratio is not None and max(w, h) / max(1.0, min(w, h)) > max_aspect_ratio:
                continue
            if min_solidity is not None:
                hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                if hull_area <= 0.0 or area / hull_area < min_solidity:
                    continue
            if min_extent is not None:
                if area / max(1.0, float(w * h)) < min_extent:
                    continue
            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-6:
                continue
            cx = float(moments["m10"] / moments["m00"])
            cy = float(moments["m01"] / moments["m00"])
            blobs.append((int(color_id), cx, cy, int(x), int(y), int(w), int(h), area))
    return blobs


def apply_dominant_color(masks: dict[int, np.ndarray], rois,
                         min_area: float = 0.0) -> dict[int, np.ndarray]:
    """每个 ROI 内按颜色面积锁定主色，把 ROI 内所有彩色像素归到主色。

    解决「一个物块被阈值切成两种颜色（如蓝 ↔ 浅蓝）」的误识别：
    ROI 内面积最大的颜色锁定为该物块颜色，其余颜色像素并入主色（盖住）。
    无 ROI，或主色面积低于 ``min_area`` 时不做处理。

    返回新的 masks dict，不修改入参。
    """
    if not rois:
        return masks
    height, width = next(iter(masks.values())).shape[:2]
    result = {color_id: mask.copy() for color_id, mask in masks.items()}
    for (rx, ry, rw, rh) in rois:
        x2, y2 = min(width, rx + rw), min(height, ry + rh)
        if x2 <= rx or y2 <= ry:
            continue
        region = (slice(ry, y2), slice(rx, x2))
        areas = {color_id: int(cv2.countNonZero(mask[region]))
                 for color_id, mask in result.items()}
        dominant = max(areas, key=areas.get)
        if areas[dominant] < min_area:
            continue
        # ROI 内所有彩色像素的并集，作为主色的「盖住」区域。
        union = np.zeros((y2 - ry, x2 - rx), dtype=np.uint8)
        for mask in result.values():
            union = cv2.bitwise_or(union, mask[region])
        for color_id, mask in result.items():
            if color_id == dominant:
                mask[region] = union
            else:
                mask[region] = 0
    return result


def load_roi_file(path: Path | str) -> list[tuple]:
    """读取 ROI 坐标文件（每行 ``x,y,w,h``），非法/越界行跳过。

    文件缺失或无法读取时返回空列表。供 circle_detect 节点读取
    color_detect.py 拖框生成的 ``ROI_Range.txt``。
    """
    try:
        text = Path(path).expanduser().resolve().read_text(encoding="utf-8")
    except OSError:
        return []
    rois = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        if len(parts) != 4:
            continue
        try:
            x, y, w, h = (int(p.strip()) for p in parts)
        except ValueError:
            continue
        if w > 0 and h > 0:
            rois.append((x, y, w, h))
    return rois


# ---------- HSV 阈值稳健估计 ----------
def estimate_hsv_bounds(sample_arrays: Iterable[np.ndarray],
                        color_id: int) -> tuple[int, ...]:
    """根据同色多个 ROI 的 HSV 像素稳健估计六个滑条值。

    黑色（id=5）Hue 不稳定，使用全 Hue 并按 S/V 百分位约束亮度；
    其余颜色对 Hue 做圆形均值展开后再取百分位，红色不会被错误扩成全环。
    """
    arrays = [np.asarray(item, dtype=np.float64).reshape(-1, 3)
              for item in sample_arrays if np.asarray(item).size]
    _require(bool(arrays), "没有可用于统计的 HSV 样本")
    values = np.concatenate(arrays, axis=0)
    _require(len(values) >= 100, "有效 HSV 样本至少需要 100 个像素")

    saturation = values[:, 1]
    value = values[:, 2]
    if int(color_id) == 5:
        s_high = min(255, int(math.ceil(np.percentile(saturation, 98))) + 20)
        v_high = min(255, int(math.ceil(np.percentile(value, 98))) + 20)
        return (0, 179, 0, s_high, 0, v_high)

    hue = values[:, 0]
    angles = hue * (2.0 * math.pi / 180.0)
    mean_angle = math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
    center_hue = (mean_angle * 180.0 / (2.0 * math.pi)) % 180.0
    delta = ((hue - center_hue + 90.0) % 180.0) - 90.0
    h_low_unwrapped = math.floor(np.percentile(delta, 2)) + center_hue - 3
    h_high_unwrapped = math.ceil(np.percentile(delta, 98)) + center_hue + 3
    if h_high_unwrapped - h_low_unwrapped >= 179.0:
        h_low, h_high = 0, 179
    else:
        h_low = int(math.floor(h_low_unwrapped)) % 180
        h_high = int(math.ceil(h_high_unwrapped)) % 180

    s_low = max(0, int(math.floor(np.percentile(saturation, 2))) - 15)
    s_high = min(255, int(math.ceil(np.percentile(saturation, 98))) + 15)
    v_low = max(0, int(math.floor(np.percentile(value, 2))) - 20)
    v_high = min(255, int(math.ceil(np.percentile(value, 98))) + 20)
    return (h_low, h_high, s_low, s_high, v_low, v_high)
