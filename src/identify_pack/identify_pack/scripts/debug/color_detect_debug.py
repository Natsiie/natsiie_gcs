#!/usr/bin/env python3
"""颜色识别调试脚本（调试用，非 ROS 节点）。

核心流程：
  1. BGR -> HSV（一次），``make_all_color_masks`` 生成六色掩膜
  2. ``find_color_blobs`` 在掩膜上找连通域色块（不依赖圆形几何，任意形状都行）
  3. 画外接矩形框 + 颜色标签；只在 ROI 内检测，其余区域忽略

ROI：
  - 启动时读取 ``ROI_Range.txt`` 里的 ROI 做只读检测（不在脚本里编辑）
  - 鼠标左键点击画面采样该点 BGR/HSV，便于核对阈值

单独运行（调试）：
  python3 color_detect_debug.py --device /dev/camera_upper
  python3 color_detect_debug.py --device /dev/camera_upper --roi-file ROI_Range.txt
"""
import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    from identify_pack.scripts.color_config import (
        COLOR_DISPLAY, COLOR_NAMES, apply_dominant_color, find_color_blobs,
        load_color_config, load_roi_file, make_all_color_masks,
    )
except ImportError:  # 直接在源码目录运行
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from identify_pack.scripts.color_config import (  # type: ignore
        COLOR_DISPLAY, COLOR_NAMES, apply_dominant_color, find_color_blobs,
        load_color_config, load_roi_file, make_all_color_masks,
    )

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "color_config.json")

# 点击采样：打印该点 BGR/HSV，便于手动核对阈值。
_sample_bgr = None
_sample_hsv = None


def on_mouse(event, x, y, flags, _param):
    """左键点击打印该点 BGR/HSV，便于手动核对阈值。"""
    if event == cv2.EVENT_LBUTTONDOWN and _sample_hsv is not None:
        h, s, v = _sample_hsv[y, x]
        b, g, r = _sample_bgr[y, x]
        print(f'点击 ({x},{y})  BGR=({int(b)},{int(g)},{int(r)})  '
              f'H={int(h)} S={int(s)} V={int(v)}')


def color_overlay(frame, masks):
    """把六色掩膜半透明叠加到画面副本上，返回叠加图。"""
    overlay = frame.copy()
    kernel = np.ones((3, 3), np.uint8)
    for color_id, raw in masks.items():
        name = COLOR_NAMES[color_id]
        mask = cv2.erode(raw, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        color = COLOR_DISPLAY[name]
        colored = np.zeros_like(frame)
        colored[:] = color
        idx = mask > 0
        overlay[idx] = (overlay[idx] * 0.4 + colored[idx] * 0.6).astype(np.uint8)
    return overlay


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--device', '-d', default='/dev/camera_upper', help='相机设备路径')
    p.add_argument('--config', default=DEFAULT_CONFIG, help='颜色配置 JSON 路径')
    p.add_argument('--roi-file', default='ROI_Range.txt',
                   help='ROI 坐标文件（每行 x,y,w,h），启动时默认读取')
    p.add_argument('--min-area', type=float, default=300.0, help='色块最小面积(px^2)')
    p.add_argument('--min-dimension', type=float, default=12.0, help='色块最小边长(px)')
    p.add_argument('--max-aspect', type=float, default=None, help='色块最大长宽比，None=不限')
    p.add_argument('--min-solidity', type=float, default=None, help='最小实心度，None=不限')
    p.add_argument('--no-lock', action='store_true',
                   help='关闭 ROI 内主色锁定（默认开启，解决蓝↔浅蓝等误切）')
    args = p.parse_args()

    try:
        config = load_color_config(args.config)
    except Exception as error:
        print(f'加载颜色配置失败：{error}')
        return 1

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        try:
            cap = cv2.VideoCapture(int(args.device))
        except Exception:
            pass
    if not cap.isOpened():
        print(f'无法打开相机: {args.device}')
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    rois = load_roi_file(os.path.abspath(args.roi_file))
    print(f'从 {args.roi_file} 加载 {len(rois)} 个 ROI（只读）')

    global _sample_bgr, _sample_hsv
    cv2.namedWindow('color_detect')
    cv2.setMouseCallback('color_detect', on_mouse, None)

    print('颜色识别调试：鼠标点击采样HSV  q=退出')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        _sample_bgr = frame
        _sample_hsv = hsv

        masks = make_all_color_masks(frame, config)
        active_rois = rois or None
        if active_rois and not args.no_lock:
            masks = apply_dominant_color(masks, active_rois, min_area=args.min_area)
        blobs = find_color_blobs(
            masks, min_area=args.min_area, min_dimension=args.min_dimension,
            max_aspect_ratio=args.max_aspect, min_solidity=args.min_solidity,
            roi=active_rois,
        )

        for (color_id, cx, cy, x, y, w, h, area) in blobs:
            name = COLOR_NAMES[color_id]
            color = COLOR_DISPLAY[name]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            label = f'{color_id}:{name}'
            cv2.putText(frame, label, (x, max(14, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
            cv2.circle(frame, (int(cx), int(cy)), 3, color, -1)
            print(f'{name} 质心({int(cx)},{int(cy)}) 框({x},{y},{w},{h}) 面积{area:.0f}')

        cv2.putText(frame, f'ROI {len(rois)}（只读）', (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow('color_detect', frame)
        cv2.imshow('color masks', color_overlay(frame, masks))

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
