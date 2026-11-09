#!/usr/bin/env python3
"""三圆环工位参考零点标定工具（摆正工位后回写参考圆心 + px/mm 标定矩阵）。

=====================================================================
一、标定前必须先确认的三个物理量（写在 ring_station_detector.json 的 geometry）
=====================================================================
  center_spacing_mm = 150.0   相邻环中心距（mm）。现场实测 15cm，2026-10 确认；
  outer_diameter_mm =  95.0   圆环外径（mm）。依据《任务命题文档1》表3/图5：
                              1~6 环外径为 Φ+3 / Φ1+5 / Φ2+7 / Φ3+10 /
                              Φ4+10 / Φ5+10，Φ=物料最大直径；
                              本物料底座 Ø50 → Φ=50 → 6 环外径 50+45 = 95mm。
                              检测取最外圈，故配置 95。
  ring1_side        = left    三环按图像 X 排序时，1 号环在左还是右。
  这三个数错了，px→mm 比例和环号会整体错，先量准再跑。

=====================================================================
二、操作前提
=====================================================================
  1. 把车/云台摆到「放置物块时的参考位姿」——以后 err=0 就代表这个位姿；
  2. 相机用 /dev/camera_upper（上相机，照圆环），分辨率 640x480，
     fourcc 与 launch 一致用 MJPG（压缩传输，避免 USB2.0 带宽被原始画质占满卡死）；
  3. 画面不翻转，与 launch 的 secondary_camera_flip_code:=0 一致；
  4. 标定期间车不动、灯不动（自动检测需要连续稳定）。

=====================================================================
三、运行命令（在 scripts 目录下）
=====================================================================
  python3 ring_station_calibrator.py \
      --device /dev/camera_upper \
      --width 640 --height 480 --fps 25 --fourcc MJPG \
      --config ./ring_station_detector.json \
      --flip-code 0

  注意：脚本默认 --device 是 /dev/video0（会被系统摄像头占住），必须显式传
        --device /dev/camera_upper；默认 --fourcc MJPG。

=====================================================================
四、界面与按键
=====================================================================
  自动模式：三环检出且稳定后，画面显示 auto: STABLE 与三个圆心；
           直接按 S 保存。
  手动模式：自动检测失败时，鼠标左键按 R1→R2→R3 顺序点三个圆心
           （ring1_side=left 即从左到右点），点满 3 个后按 S。
  S  保存   R  重置点选   Q/Esc 退出

=====================================================================
五、保存内容（回写 ring_station_detector.json）
=====================================================================
  reference.ring_centers_px           三环参考圆心（像素）
                                      auto 模式：中环 y 使用上下边线约束（距离），
                                      左右环与原 yaw 仍按三环椭圆逻辑
  reference.station_yaw_deg           参考角度（deg）
  reference.image_to_station_affine   像素→毫米 2x2 矩阵
                                      = center_spacing_mm / 平均像素间距，对角近似
  保存后要重启 ring_station_detect_node 才生效。

=====================================================================
六、标定后验证
=====================================================================
  1. 车保持该位姿，看 /ring_station：
       err_x/err_y ≈ 0、angle_err ≈ 0、within=1；
  2. 车沿臂方向平移一段，err 应主要沿一个方向变化（不串轴）；
  3. 纠偏方向（corr_x/corr_y）应与误差反号，把车拉回零点；
  4. 不通过先查 geometry 三个物理量，再查本标定，最后查底盘矩阵
     chassis_calibrator.py（chassis_image_to_motion）。
=====================================================================
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys

import cv2
import numpy as np

try:
    from .camera_capture import TimedCamera
    from .camera_diagnostics import require_display
except ImportError:
    from camera_capture import TimedCamera
    from camera_diagnostics import require_display

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from ring_station_detector import (  # noqa: E402
    RingStationDetector, RingStationDetectionError,
    draw_ring_station_detection, load_ring_station_config,
    save_ring_station_config,
)
from stacked_ring_detector import load_stacked_ring_config  # noqa: E402

DEFAULT_CONFIG = os.path.join(_SCRIPT_DIR, 'ring_station_detector.json')


class RingStationCalibrationError(RuntimeError):
    """标定工具无法打开摄像头或画面无效。"""


def open_camera(device, width, height, fps, fourcc='MJPG', timeout=5.0):
    camera = TimedCamera(device, width, height, fps, fourcc, timeout)
    ok, frame = camera.read()
    if ok and frame is not None:
        if frame.shape[:2] == (height, width):
            return camera
        camera.release()
        raise RingStationCalibrationError(
            f'摄像头输出 {frame.shape[1]}x{frame.shape[0]}，需要 {width}x{height}')
    camera.release()
    raise RingStationCalibrationError('摄像头已打开，但无法读取画面')


def order_centers(points, ring1_side):
    """按 X 排序三环（ring1_side='right' 时反向），与检测器约定一致。"""
    return sorted(points, key=lambda point: float(point[0]),
                  reverse=(ring1_side == 'right'))


def compute_calibration(centers, center_spacing_mm):
    """由三环圆心（已排序 R1,R2,R3）推算间距、px/mm、yaw 和对角仿射矩阵。"""
    p1 = np.asarray(centers[0], dtype=float)
    p2 = np.asarray(centers[1], dtype=float)
    p3 = np.asarray(centers[2], dtype=float)
    spacing_12 = float(np.linalg.norm(p2 - p1))
    spacing_23 = float(np.linalg.norm(p3 - p2))
    avg_spacing = (spacing_12 + spacing_23) / 2.0
    if avg_spacing <= 0:
        raise RingStationDetectionError('三环间距为 0，无法推算 px/mm')
    mm_per_px = center_spacing_mm / avg_spacing
    yaw = math.degrees(math.atan2(p3[1] - p1[1], p3[0] - p1[0]))
    affine = [[mm_per_px, 0.0], [0.0, mm_per_px]]
    return spacing_12, spacing_23, mm_per_px, yaw, affine


def draw_manual_points(frame, points, ordered):
    """绘制已点选的圆心与 R1→R2→R3 连线。"""
    colors = ((0, 180, 255), (0, 255, 0), (255, 160, 0))
    for index, (x, y) in enumerate(points):
        point = (int(round(x)), int(round(y)))
        cv2.drawMarker(frame, point, (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(frame, f'{index + 1}', (point[0] + 8, point[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    if len(ordered) >= 3:
        line = np.asarray([(int(round(x)), int(round(y))) for x, y in ordered], np.int32)
        cv2.polylines(frame, [line], False, (255, 0, 255), 2)
    return frame


def draw_status(frame, lines):
    box_height = 26 * len(lines) + 12
    cv2.rectangle(frame, (0, 0), (frame.shape[1] - 1, box_height), (0, 0, 0), -1)
    for index, line in enumerate(lines):
        cv2.putText(frame, line, (10, 25 + index * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 240, 0), 1, cv2.LINE_AA)
    return frame


def parse_args():
    parser = argparse.ArgumentParser(description='标定三圆环工位参考零点与 px/mm 矩阵')
    parser.add_argument('--device', default='/dev/camera_upper')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=25.0)
    parser.add_argument('--fourcc', default='MJPG')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--stacked-config', default='',
                        help='叠料区配置（默认同目录 stacked_ring_detector.json）；'
                             '只复用其中的上下边线检测参数')
    parser.add_argument('--flip-code', type=int, choices=(-1, 0, 1), default=0,
                        help='画面翻转：-1=上下颠倒(180°)，1=左右镜像，0=不翻转（与 launch 一致）')
    return parser.parse_args()


def run(args):
    config_path = Path(args.config).expanduser().resolve()
    try:
        document = json.loads(config_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise RingStationCalibrationError(f'无法读取配置 {config_path}：{error}') from error
    config = load_ring_station_config(config_path)
    line_config = None
    if config.line_constraint_enabled:
        stacked_path = (Path(args.stacked_config).expanduser()
                        if args.stacked_config
                        else config_path.parent / 'stacked_ring_detector.json')
        try:
            line_config = load_stacked_ring_config(stacked_path)
            print(f'边线约束已启用：{stacked_path}')
        except (OSError, ValueError, KeyError, RuntimeError) as error:
            print(f'警告：叠料区配置加载失败，使用纯椭圆标定：{error}')
    detector = RingStationDetector(config, line_config=line_config)
    require_display()
    camera = open_camera(args.device, args.width, args.height, args.fps,
                         args.fourcc, args.timeout)

    window = 'Ring Station Zero Calibration'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    manual_points = []
    message = '摆正工位后稳定即按 S；自动失败可左键点选三环 R1/R2/R3'

    def on_mouse(event, x, y, _flags, _param):
        nonlocal message
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if len(manual_points) < 3:
            manual_points.append((float(x), float(y)))
            message = f'已点选 {len(manual_points)}/3 个圆心'
        else:
            message = '已点满 3 点，按 S 保存或 R 重置'

    cv2.setMouseCallback(window, on_mouse)

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RingStationCalibrationError('标定过程中摄像头读取失败')
            if args.flip_code:
                frame = cv2.flip(frame, args.flip_code)

            result = detector.detect(frame)

            active = None  # (centers, source)
            if len(manual_points) == 3:
                active = (order_centers(manual_points, config.ring1_side), 'manual')
            elif result.detected and len(result.centers_px) == 3:
                active = (list(result.centers_px), 'auto')

            preview = frame.copy()
            draw_ring_station_detection(preview, result)

            lines = ['Ring station zero calibration']
            if result.detected and len(result.centers_px) == 3:
                line_state = 'Y' if result.line_constraint_applied else 'N'
                line_q = (result.line_quality
                          if result.line_quality is not None else -1.0)
                lines.append(f"auto: {'STABLE' if result.stable else 'DETECTED'}  "
                             f"line={line_state} q={line_q:.2f}  "
                             f"centers={result.centers_px}")
            else:
                lines.append('auto: SEARCHING')
            if active is None:
                lines.append('三环未就绪：自动未检出且未点满 3 点')
            else:
                centers, source = active
                spacing_12, spacing_23, mm_per_px, yaw, affine = compute_calibration(
                    centers, config.center_spacing_mm)
                lines.append(f'source={source}  gap={spacing_12:.1f}/{spacing_23:.1f}px  '
                             f'yaw={yaw:+.2f}deg')
                lines.append(f'px/mm={1.0 / mm_per_px:.2f}  mm/px={mm_per_px:.4f}  '
                             f'(spacing={config.center_spacing_mm:g}mm)')
                if len(manual_points) < 3:
                    lines.append('按 S 保存参考零点 + 自动推算的 px/mm 矩阵')
            lines.append(message)

            draw_manual_points(preview, manual_points, active[0] if active else [])
            draw_status(preview, lines)
            cv2.imshow(window, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('r'), ord('R')):
                manual_points.clear()
                message = '已重置点选'
            elif key in (ord('s'), ord('S')):
                if active is None:
                    message = '尚无可用三环，无法保存'
                    continue
                centers, source = active
                spacing_12, spacing_23, mm_per_px, yaw, affine = compute_calibration(
                    centers, config.center_spacing_mm)
                document['reference']['ring_centers_px'] = [
                    [round(float(x), 2), round(float(y), 2)] for x, y in centers]
                document['reference']['station_yaw_deg'] = round(yaw, 4)
                document['reference']['image_to_station_affine'] = [
                    [round(float(v), 6) for v in row] for row in affine]
                try:
                    save_ring_station_config(config_path, document)
                except RingStationDetectionError as error:
                    message = f'保存失败：{error}'
                    print(message)
                    continue
                line_note = (
                    ' / 边线约束 y'
                    if source == 'auto' and result.line_constraint_applied else '')
                message = (f'已保存参考零点（{source}{line_note}）：'
                           f'yaw={yaw:+.2f}deg  mm/px={mm_per_px:.4f}')
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
