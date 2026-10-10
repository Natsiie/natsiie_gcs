"""转盘圆心对齐：加载参考圆心，计算底盘纠偏量 (corr_x, corr_y)。

把「当前转盘圆心相对参考圆心的像素偏差」乘 mm/px 转成毫米，再乘底盘纠偏矩阵的
2x2 位置部分，得到下位机底盘纠偏帧 (0x0E) 需要的 corr_x, corr_y（无 yaw）。
mm/px 由转盘物理直径与检测半径像素反推，随相机到转盘距离自动修正。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class TurntableAlignConfig:
    reference_center_px: tuple[float, float]
    turntable_diameter_mm: float
    tolerance_mm: float


def load_turntable_align_config(path: str | Path) -> TurntableAlignConfig:
    """读取并校验转盘对齐配置。"""
    resolved = Path(path).expanduser().resolve()
    try:
        raw = json.loads(resolved.read_text(encoding="utf-8"))
        if int(raw["format_version"]) != 1:
            raise ValueError("turntable_align 配置版本不受支持")
        center = raw["reference_center_px"]
        config = TurntableAlignConfig(
            reference_center_px=(float(center[0]), float(center[1])),
            turntable_diameter_mm=float(raw["turntable_diameter_mm"]),
            tolerance_mm=float(raw["tolerance_mm"]),
        )
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        raise ValueError(f"无法读取转盘对齐配置 {resolved}：{error}") from error
    if (len(config.reference_center_px) != 2
            or not all(np.isfinite(v) for v in config.reference_center_px)
            or not np.isfinite(config.turntable_diameter_mm)
            or config.turntable_diameter_mm <= 0
            or not np.isfinite(config.tolerance_mm)
            or config.tolerance_mm <= 0):
        raise ValueError("转盘对齐配置数值无效")
    return config


def compute_turntable_correction(center_px, radius_px, config, chassis_image_to_motion):
    """由圆心/半径像素计算底盘纠偏量，返回 (corr_x_mm, corr_y_mm, within)。

    chassis_image_to_motion 是 ring_station_detector.json 里的 3x3 矩阵，
    这里只取 2x2 位置部分（转盘无 yaw）。
    """
    radius = float(radius_px)
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("转盘半径像素无效")
    mm_per_px = config.turntable_diameter_mm / (2.0 * radius)
    error_px = np.asarray(center_px, dtype=float) - np.asarray(
        config.reference_center_px, dtype=float)
    error_mm = mm_per_px * error_px
    matrix = np.asarray(chassis_image_to_motion, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("底盘纠偏矩阵必须是有限的 3x3 矩阵")
    corr = matrix[0:2, 0:2] @ error_mm
    within = bool(float(np.hypot(error_mm[0], error_mm[1])) <= config.tolerance_mm)
    return float(corr[0]), float(corr[1]), within
