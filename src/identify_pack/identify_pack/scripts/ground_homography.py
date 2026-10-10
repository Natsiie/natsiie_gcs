"""Ground-plane homography: raw pixel -> gimbal_base (x, y) mm, yaw-aware.

相机垂直朝下、与夹爪同轴共转；地面是水平平面，标定标记与要放置的圆环
处于同一平面。标定拟合一个「像素 -> gimbal_link 平面坐标」的固定单应
H_link（相机刚性固定在 gimbal_link 上，H_link 不随 yaw 变），运行时按当前
yaw 转回 gimbal_base：

    [x_link, y_link] = H_link @ [u, v, 1]
    [x_base, y_base] = R_z(yaw) @ [x_link, y_link]

原始像素直接参与拟合，镜头畸变、相机姿态、平面高度都被吸收进 H_link，
无需单独的内参/外参/平面高度。纯函数模块，无 ROS / 无 GUI，供标定脚本
与放置节点共用。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


def rot2(angle_rad: float) -> np.ndarray:
    """绕 Z 轴的 2x2 旋转矩阵（对应 R_z 的 xy 子块）。"""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s], [s, c]], dtype=float)


def link_from_base_xy(xy_mm, yaw_deg):
    """gimbal_base (x,y) -> gimbal_link (x,y)，标定拟合时用。"""
    return rot2(-math.radians(float(yaw_deg))) @ np.asarray(xy_mm, dtype=float)


def base_from_link_xy(xy_mm, yaw_deg):
    """gimbal_link (x,y) -> gimbal_base (x,y)，运行时用。"""
    return rot2(math.radians(float(yaw_deg))) @ np.asarray(xy_mm, dtype=float)


def apply_homography(pixel, homography):
    """原始像素 (u,v) -> gimbal_link 平面 (x,y) mm。"""
    vector = np.array([float(pixel[0]), float(pixel[1]), 1.0], dtype=float)
    projected = np.asarray(homography, dtype=float) @ vector
    if not np.isfinite(projected).all() or abs(projected[2]) < 1e-12:
        raise ValueError('homography 产生了非有限/退化点')
    return projected[:2] / projected[2]


def pixel_to_base_xy(pixel, homography, yaw_deg):
    """原始像素 (u,v) -> gimbal_base (x,y) mm。"""
    link = apply_homography(pixel, homography)
    return base_from_link_xy(link, yaw_deg)


def load(path):
    """读取单应标定 JSON，返回校验过的 dict。"""
    document = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
    homography = np.asarray(document['homography'], dtype=float)
    if homography.shape != (3, 3) or not np.isfinite(homography).all():
        raise ValueError('homography 必须是 3x3 有限矩阵')
    image_size = document.get('image_size')
    if (image_size is None or len(image_size) != 2
            or any(not isinstance(value, int) or value <= 0 for value in image_size)):
        raise ValueError('image_size 无效')
    return {
        'homography': homography,
        'image_size': tuple(int(value) for value in image_size),
        'parent_frame': document.get('parent_frame', 'gimbal_link'),
        'base_frame': document.get('base_frame', 'gimbal_base'),
        'yaw_sign': float(document.get('yaw_sign', 1.0)),
        'yaw_offset_rad': float(document.get('yaw_offset_rad', 0.0)),
    }
