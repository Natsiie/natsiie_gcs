#!/usr/bin/env python3
"""地面单应标定（相机垂直朝下 · 固定地面标记 · 单次冻结多点）。

地面标记/标定板平放在地面上，与要放置的圆环处于同一水平面。云台固定一个
角度、冻结一次画面，连续点击多个固定地面标记，每个标记输入其 gimbal_base
坐标 (x, y)，所有点共用同一个 yaw。拟合「像素 -> gimbal_link 平面坐标」的
单应矩阵 H_link（单位 mm）：

    [x_link, y_link, 1]^T ~ H_link @ [u, v, 1]^T

相机刚性固定在 gimbal_link 上，H_link 不随 yaw 变；运行时按当前 yaw 转回
base：

    [x_base, y_base] = R_z(yaw) @ [x_link, y_link]

标记必须二维散开（x、y 都有变化），不能全在一条直线上——否则单应退化、
残差爆炸。原始像素直接参与拟合，镜头畸变也被吸收进 H_link，无需内参/外参。

流程：
  1. 在地面放若干个散开的固定标记（不要都排在径向线上）。
  2. 逐个把夹爪挪到每个标记正上方，读下位机 (x, y) 记下（测量步骤）。
  3. 固定云台一个角度，让所有标记都进入画面，停稳。
  4. Space 冻结 -> 输入当前 yaw；随后逐个左键点标记、输入各自的 x y。
  5. u 撤销上一点；采够 >=6 点后按 s 解算，输出 homography JSON。

用法：
  python3 calibrate_ground_homography.py --device /dev/camera_upper \
      --output /home/lulu/gcs/ground_homography.json --force
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

try:
    from ground_homography import apply_homography, link_from_base_xy
except ImportError:
    from .ground_homography import apply_homography, link_from_base_xy


def validate_samples(samples):
    if len(samples) < 6:
        raise ValueError('至少需要 6 个点')
    link = np.asarray([link_from_base_xy(s['xy_base'], s['yaw_deg'])
                       for s in samples], dtype=float)
    if not np.isfinite(link).all():
        raise ValueError('坐标存在非有限值')
    centered = link - link.mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    ratio = singular[1] / singular[0] if singular[0] > 0 else 0.0
    if ratio <= 1e-3:
        raise ValueError(
            f'点在 link 平面接近共线（二维展开比例 {ratio:.1%}）；'
            '请把标记往左右/前后散开，避免排成一条直线')


def fit_homography(samples):
    """Return (H, errors_mm)；H 把原始像素映射到 gimbal_link 平面 (x,y) mm。"""
    validate_samples(samples)
    src = np.asarray([s['pixel'] for s in samples], dtype=np.float32)
    dst = np.asarray([link_from_base_xy(s['xy_base'], s['yaw_deg'])
                      for s in samples], dtype=np.float32)
    homography, _ = cv2.findHomography(src, dst, method=0)
    if homography is None:
        raise ValueError('单应求解失败：检查点是否共线或重复')
    errors = []
    for sample in samples:
        predicted = apply_homography(sample['pixel'], homography)
        true = link_from_base_xy(sample['xy_base'], sample['yaw_deg'])
        errors.append(float(np.linalg.norm(predicted - true)))
    return homography, np.asarray(errors)


def build_result(samples, image_size):
    homography, errors = fit_homography(samples)
    annotated = []
    for sample, error in zip(samples, errors):
        link = link_from_base_xy(sample['xy_base'], sample['yaw_deg'])
        annotated.append({
            'pixel': sample['pixel'],
            'xy_base_mm': sample['xy_base'],
            'yaw_deg': sample['yaw_deg'],
            'xy_link_mm': link.tolist(),
            'error_mm': float(error),
        })
    return {
        'schema_version': 1,
        'method': 'ground_homography',
        'image_size': list(image_size),
        'parent_frame': 'gimbal_link',
        'base_frame': 'gimbal_base',
        'yaw_sign': 1.0,
        'yaw_offset_rad': 0.0,
        'homography': homography.tolist(),
        'residual_rms_mm': float(np.sqrt(np.mean(errors ** 2))),
        'max_error_mm': float(errors.max()),
        'point_errors_mm': errors.tolist(),
        'samples': annotated,
    }


def collect_samples(args):
    device = int(args.device) if args.device.isdecimal() else args.device
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise ValueError(f'无法打开相机: {args.device}')
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    frozen = None
    frozen_yaw = None
    pending = []
    samples = []

    def on_mouse(event, x, y, _flags, _param):
        if (event == cv2.EVENT_LBUTTONDOWN and frozen is not None and not pending
                and 0 <= x < args.width and 0 <= y < args.height):
            pending.append((x, y))

    cv2.namedWindow('calibrate_homography')
    cv2.setMouseCallback('calibrate_homography', on_mouse)
    print('=== 地面单应标定（固定地面标记 · 单次冻结多点）===')
    print('Space 冻结/恢复；冻结后先输 yaw，再逐个左键点标记、输 x y')
    print('U 撤销上一点；S 解算；Q 退出')
    print('所有点共用同一个 yaw；标记须二维散开、不要排成一条直线')
    try:
        while True:
            ok, live_frame = capture.read()
            if not ok or live_frame is None:
                raise ValueError('无法读取相机画面')
            frame = live_frame if frozen is None else frozen
            if frame.shape[1::-1] != (args.width, args.height):
                raise ValueError('图像分辨率与 --width/--height 不一致')
            while pending:
                u, v = pending.pop(0)
                try:
                    text = input(f'像素({u},{v}) 对应 x y（mm），r 取消: ').strip()
                    if text.lower() == 'r':
                        continue
                    parts = [float(t) for t in text.split()]
                    if len(parts) != 2:
                        raise ValueError('需要 2 个数值：x y')
                    x, y = parts
                    if not all(math.isfinite(t) for t in (x, y)):
                        raise ValueError('坐标须为有限数值')
                    if any(s['pixel'] == [u, v] or s['xy_base'] == [x, y]
                           for s in samples):
                        raise ValueError('重复点，请换一个标记')
                    samples.append({'pixel': [u, v], 'xy_base': [x, y],
                                    'yaw_deg': frozen_yaw})
                    print(f'  已采集 {len(samples)} 点: xy=({x:.1f},{y:.1f})mm '
                          f'(yaw={frozen_yaw:.2f}°)')
                except ValueError as error:
                    print(f'  输入无效：{error}；请重新点击')
            display = frame.copy()
            for index, sample in enumerate(samples, 1):
                pt = tuple(sample['pixel'])
                cv2.circle(display, pt, 4, (0, 0, 255), -1)
                cv2.putText(display, str(index), pt, cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            if frozen is not None:
                cv2.putText(display, f'FROZEN  yaw={frozen_yaw:.2f} deg',
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 255), 2)
            cv2.imshow('calibrate_homography', display)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                return None
            if key == ord(' '):
                if frozen is None:
                    frozen = live_frame.copy()
                    pending.clear()
                    try:
                        frozen_yaw = float(input('当前云台 yaw（deg，下位机 0x07）: ').strip())
                        if not math.isfinite(frozen_yaw):
                            raise ValueError('yaw 须为有限数值')
                    except ValueError as error:
                        print(f'  yaw 无效：{error}；已取消冻结，请重按 Space')
                        frozen = None
                        frozen_yaw = None
                        pending.clear()
                        continue
                    print(f'  已冻结，yaw={frozen_yaw:.2f}°；现在可逐个点标记')
                else:
                    frozen = None
                    frozen_yaw = None
                    pending.clear()
                    print('  已解冻，可移动云台/标记')
            elif key in (ord('u'), ord('U')) and samples:
                samples.pop()
                pending.clear()
                print(f'已撤销上一点，剩余 {len(samples)} 点')
            elif key in (ord('s'), ord('S')):
                try:
                    fit_homography(samples)
                except (ValueError, cv2.error) as error:
                    print(f'暂不能解算：{error}；可继续采点或按 U 撤销')
                    continue
                return samples
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='地面单应标定（像素 -> gimbal_base mm）')
    parser.add_argument('--device', default='/dev/camera_upper')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--output', type=Path, default=Path('ground_homography.json'))
    parser.add_argument('--force', action='store_true', help='覆盖已存在的输出文件')
    args = parser.parse_args()
    try:
        if args.width <= 0 or args.height <= 0:
            raise ValueError('Invalid image size')
        if args.output.exists() and not args.force:
            raise ValueError(f'输出文件已存在，加 --force 覆盖或换 --output：{args.output}')
        samples = collect_samples(args)
        if samples is None:
            return 0
        result = build_result(samples, (args.width, args.height))
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                          allow_nan=False) + '\n', encoding='utf-8')
        print(f'\n✅ 已保存：{args.output.resolve()}')
        print(f'标定方法：{result["method"]}（像素 -> gimbal_link mm，yaw 转 base）')
        print(f'残差 RMS：{result["residual_rms_mm"]:.2f} mm；'
              f'最大 {result["max_error_mm"]:.2f} mm')
        print('逐点误差（mm）：')
        for sample in result['samples']:
            print(f'  xy_base=({sample["xy_base_mm"][0]:.1f},{sample["xy_base_mm"][1]:.1f}) '
                  f'yaw={sample["yaw_deg"]:.2f}° -> {sample["error_mm"]:.2f}mm')
        if result['residual_rms_mm'] > 5.0:
            print('⚠️ 残差偏大（>5mm），可能点接近共线或 (x,y) 与像素不对应，请检查后重标。')
        print('提示：请用未参与标定的点再验证一次；并确认 yaw 符号/零点与 tf_transform.yaml 一致。')
        return 0
    except (ValueError, OSError, KeyError, TypeError, cv2.error) as error:
        print(f'标定失败：{error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
