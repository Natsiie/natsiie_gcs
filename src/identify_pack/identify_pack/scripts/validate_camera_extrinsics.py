#!/usr/bin/env python3
"""Validate a fixed camera pose against independent measured points at gimbal zero."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

if __package__:
    from .camera_geometry import pixel_to_plane
else:
    from camera_geometry import pixel_to_plane


def evaluate_point(sample, calibration, plane_z_mm):
    matrix = np.asarray(calibration['camera_matrix'], dtype=np.float64)
    distortion = np.asarray(calibration['dist_coeffs'], dtype=np.float64)
    transform = np.asarray(calibration['parent_from_camera_matrix'], dtype=np.float64)
    pixel = np.asarray(sample['pixel'], dtype=np.float64)
    expected = np.asarray(sample['xy_mm'], dtype=np.float64)
    if expected.shape != (2,) or not np.isfinite(expected).all():
        raise ValueError('Expected two finite measured XY coordinates')
    width, height = calibration['image_size']
    if pixel.shape != (2,) or not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
        raise ValueError('Pixel outside calibration image bounds')
    predicted = pixel_to_plane(pixel, matrix, distortion, transform, plane_z_mm / 1000) * 1000
    difference = predicted[:2] - expected
    return {
        'pixel': pixel.tolist(),
        'measured_xy_mm': expected.tolist(),
        'predicted_xyz_mm': predicted.tolist(),
        'dx_mm': float(difference[0]),
        'dy_mm': float(difference[1]),
        'xy_error_mm': float(np.linalg.norm(difference)),
    }


def print_point(result):
    x, y, z = result['predicted_xyz_mm']
    print(f'预测 X={x:.2f} Y={y:.2f} Z={z:.2f} mm；'
          f'dX={result["dx_mm"]:+.2f} dY={result["dy_mm"]:+.2f} mm；'
          f'XY误差={result["xy_error_mm"]:.2f} mm')


def collect_checks(args, calibration, plane_z_mm):
    capture = None
    frozen = None
    pending = []
    checks = []
    window = 'camera_extrinsics_check'

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and frozen is not None:
            pending.append([x, y])

    try:
        width, height = calibration['image_size']
        if args.image:
            frozen = cv2.imread(str(args.image))
            if frozen is None:
                raise ValueError(f'Cannot read image: {args.image}')
        else:
            device = int(args.device) if args.device.isdecimal() else args.device
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not capture.isOpened():
                raise ValueError(f'Cannot open camera: {args.device}')
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, on_mouse)
        print('云台必须保持实际零位；原始图像、焦距与标定时一致。')
        print('Space 冻结；左键点击新检查点并输入实测 X Y；U 撤销；S 完成；Q 取消。')
        print('结果在 gimbal_link 中，不使用动态 TF，不重新拟合外参。')
        while True:
            if frozen is None:
                ok, frame = capture.read()
                if not ok:
                    raise ValueError('Cannot read camera frame')
            else:
                frame = frozen
            if list(frame.shape[1::-1]) != calibration['image_size']:
                raise ValueError('Image resolution does not match calibration')
            while pending:
                pixel = pending.pop(0)
                try:
                    text = input(f'像素 {pixel} 的实测 X Y（mm），R 取消: ').strip()
                    if text.lower() == 'r':
                        continue
                    sample = {'pixel': pixel, 'xy_mm': [float(value) for value in text.split()]}
                    result = evaluate_point(sample, calibration, plane_z_mm)
                    print_point(result)
                    checks.append(result)
                except ValueError as error:
                    print(f'检查点无效：{error}；请重新点击')
            display = frame.copy()
            for index, check in enumerate(checks, 1):
                point = tuple(int(value) for value in check['pixel'])
                cv2.circle(display, point, 4, (0, 255, 0), -1)
                cv2.putText(display, str(index), point, cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            cv2.imshow(window, display)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), 27):
                return None
            if key == ord(' ') and frozen is None:
                frozen = frame.copy()
            elif key == ord('u') and checks:
                checks.pop()
            elif key == ord('s'):
                if not checks:
                    print('请先采集检查点')
                    continue
                return checks
    finally:
        if capture is not None:
            capture.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='独立检查点验证相机外参：云台零位，不重新拟合')
    parser.add_argument('--extrinsics', type=Path, default=Path('camera_extrinsics.json'))
    parser.add_argument('--device', default='/dev/camera_upper')
    parser.add_argument('--plane-z-mm', type=float,
                        help='检查平面在 gimbal_link 中的高度；默认使用标定平面高度')
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--image', type=Path, help='从原始静止图像点击检查点')
    source.add_argument('--points', type=Path, help='离线 JSON 列表，每项含 pixel 和 xy_mm')
    parser.add_argument('--output', type=Path, help='可选检查报告 JSON；不会覆盖已有文件')
    parser.add_argument('--tolerance-mm', type=float, help='可选 XY 误差上限，由夹爪容差确定')
    args = parser.parse_args()
    try:
        if args.output and args.output.exists():
            raise ValueError('Output exists; choose another --output')
        if (args.tolerance_mm is not None
                and (not np.isfinite(args.tolerance_mm) or args.tolerance_mm <= 0)):
            raise ValueError('Tolerance must be finite and positive')
        calibration = json.loads(args.extrinsics.read_text(encoding='utf-8'))
        if calibration['parent_frame'] != 'gimbal_link':
            raise ValueError('Expected calibration in gimbal_link')
        plane_z_mm = (calibration['calibration_plane_z_mm'] if args.plane_z_mm is None
                      else args.plane_z_mm)
        if not np.isfinite(plane_z_mm):
            raise ValueError('Plane height must be finite')
        if args.points:
            samples = json.loads(args.points.read_text(encoding='utf-8'))
            checks = [evaluate_point(sample, calibration, plane_z_mm) for sample in samples]
            if not checks:
                raise ValueError('No check points supplied')
            for check in checks:
                print_point(check)
        else:
            checks = collect_checks(args, calibration, plane_z_mm)
        if checks is None:
            return 0
        errors = np.asarray([check['xy_error_mm'] for check in checks])
        report = {
            'extrinsics_file': str(args.extrinsics.resolve()),
            'frame_id': calibration['parent_frame'],
            'gimbal_angle_deg': 0.0,
            'plane_z_mm': plane_z_mm,
            'checks': checks,
            'mean_xy_error_mm': float(errors.mean()),
            'rms_xy_error_mm': float(np.sqrt(np.mean(errors ** 2))),
            'max_xy_error_mm': float(errors.max()),
        }
        passed = args.tolerance_mm is None or errors.max() <= args.tolerance_mm
        if args.tolerance_mm is not None:
            report.update(tolerance_mm=args.tolerance_mm, passed=bool(passed))
        print(f'{len(checks)} 个检查点：平均误差 {errors.mean():.2f} mm；'
              f'RMS {report["rms_xy_error_mm"]:.2f} mm；最大误差 {errors.max():.2f} mm')
        print('请确认这些点没有参与外参拟合，且覆盖中心、边缘与实际工作范围。')
        if args.tolerance_mm is not None:
            print('满足指定误差上限' if passed else '超出指定误差上限')
        if args.output:
            with args.output.open('x', encoding='utf-8') as stream:
                json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write('\n')
            print(f'已保存检查报告：{args.output.resolve()}')
        return 0 if passed else 2
    except (ValueError, TypeError, KeyError, OSError, cv2.error) as error:
        print(f'验证失败：{error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
