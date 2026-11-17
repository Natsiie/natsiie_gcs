#!/usr/bin/env python3
"""相机外参标定：gimbal_link <- camera_optical_frame（云台零位，已知 XY 采点）。

=====================================================================
一、运行命令（在 scripts 目录，云台零位、相机固定、标记铺平）
=====================================================================
  python3 calibrate_camera_extrinsics.py \
      --device /dev/camera_upper \
      --matrix camera_matrix.npy \
      --distortion dist_coeffs.npy \
      --plane-z-mm -116 \
      --output /home/lulu/gcs/camera_extrinsics_new12.json

  参数说明：
    --device     上相机，按串号绑定为 /dev/camera_upper
    --matrix     内参 npy（calibrate_camera.py 的输出，fx≈302.5）
    --distortion 畸变 npy（同上）
    --plane-z-mm 标记所在水平面相对 gimbal_link 原点的高度（mm）。
                 地面/圆环平面在转轴下方 → -116（车垫高 50mm 时为 -166）。
                 物块顶面 = 地面 + 物块高。
    --output     输出 JSON；已存在会拒绝，换新文件名。

=====================================================================
二、采点覆盖范围（决定标定能不能用）
=====================================================================
  外参只在「标定点围住的区域」内可靠，区域外是外推、误差按距离放大。
  因此标定点必须覆盖实际工作区，不能只挤在画面中间：
    像素范围至少  x 180~470, y 190~310
    地面范围至少  X  80~310mm, Y -260~+150mm
  推荐 8~12 个点（3x3 或 4x3 圆点板铺开），必须包含：
    中心、四角、以及三个圆环各自的像素位置附近（尤其最外侧的环3）。

  注意：点数少且集中时，多个位姿都能拟合这些点，但在区域外差别巨大，
  典型现象是「近处的点准、最外侧的点偏几十 mm」。

=====================================================================
三、操作
=====================================================================
  Space 冻结画面；左键点标记中心 → 终端输入该点实测 X Y（mm）；
  U 撤销；S 解算保存；Q 退出。
  每个点的 XY 用卷尺量到 ±1mm，相对「转轴正下方」那一点 (0,0)。
  采样点必须同一平面、不重复、不共线。

=====================================================================
四、离线合并多组点（板子需要摆两处时）
=====================================================================
  --points 可直接读 JSON（{"image_size":[640,480],"samples":[...]} 或样本列表），
  把两次摆放采到的 samples 合并后一次解算，例如：
    python3 calibrate_camera_extrinsics.py --matrix ... --distortion ... \
        --plane-z-mm -116 --points /tmp/merged_samples.json \
        --output /home/lulu/gcs/camera_extrinsics_new12.json

=====================================================================
五、验收
=====================================================================
  1. 外参重投影 RMS < 2px；
  2. 相机离平面 ≈ 实测值（相机离地高度 − 平面高度），camera z 与物理一致；
  3. 用 validate_camera_extrinsics.py 拿未参与标定的点验证，max_xy_error ≤ 5mm；
  4. 三个环的投影间距应接近相等的 150mm（不应出现 1.4 倍这种比值）。
=====================================================================
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

if __package__:
    from identify_pack.scripts.camera_geometry import estimate_camera_transform, load_intrinsics, pixel_to_plane
else:
    from identify_pack.scripts.camera_geometry import estimate_camera_transform, load_intrinsics, pixel_to_plane


def build_result(samples, plane_z_mm, matrix, distortion, image_size):
    pixels = [sample['pixel'] for sample in samples]
    xy_mm = np.asarray([sample['xy_mm'] for sample in samples], dtype=np.float64)
    transform, errors = estimate_camera_transform(pixels, xy_mm, plane_z_mm,
                                                  matrix, distortion)
    predictions_mm = np.asarray([
        pixel_to_plane(pixel, matrix, distortion, transform, plane_z_mm / 1000.0)[:2]
        * 1000.0 for pixel in pixels
    ])
    xy_errors = np.linalg.norm(predictions_mm - xy_mm, axis=1)
    axis_angle = cv2.Rodrigues(transform[:3, :3])[0].reshape(3)
    angle = np.linalg.norm(axis_angle)
    quaternion = np.append(axis_angle * (np.sin(angle / 2.0) / angle),
                           np.cos(angle / 2.0)) if angle > 1e-12 else np.array([0, 0, 0, 1])
    return {
        'schema_version': 1,
        'parent_frame': 'gimbal_link',
        'child_frame': 'camera_optical_frame',
        'transform_convention': 'p_parent_m = R @ p_child_m + translation_m',
        'calibration_gimbal_angle_deg': 0.0,
        'image_size': list(image_size),
        'camera_matrix': matrix.tolist(),
        'dist_coeffs': distortion.tolist(),
        'calibration_plane_z_mm': plane_z_mm,
        'rotation_matrix': transform[:3, :3].tolist(),
        'translation_m': transform[:3, 3].tolist(),
        'quaternion_xyzw': quaternion.tolist(),
        'parent_from_camera_matrix': transform.tolist(),
        'reprojection_rms_px': float(np.sqrt(np.mean(errors ** 2))),
        'point_errors_px': errors.tolist(),
        'fit_point_errors_mm': xy_errors.tolist(),
        'samples': samples,
    }


def collect_samples(args):
    capture = None
    frozen = None
    samples = []
    pending = []
    window = 'camera_extrinsics'

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and frozen is not None:
            pending.append((x, y))

    try:
        if args.image:
            frozen = cv2.imread(str(args.image))
            if frozen is None:
                raise ValueError(f'Cannot read image: {args.image}')
        else:
            device = int(args.device) if args.device.isdecimal() else args.device
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not capture.isOpened():
                raise ValueError(f'Cannot open camera: {args.device}')
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, on_mouse)
        print('云台保持零位，标记固定不动。Space 冻结画面；左键采点；U 撤销；S 解算保存；Q 退出。')
        print('输入每个点相对旋转轴的 X Y（mm）；所有点必须处于指定的同一水平面。')
        while True:
            if frozen is None:
                ok, frame = capture.read()
                if not ok:
                    raise ValueError('Cannot read camera frame')
            else:
                frame = frozen
            if frame.shape[1::-1] != (args.width, args.height):
                raise ValueError('Image resolution does not match --width/--height')
            while pending:
                pixel = pending.pop(0)
                try:
                    text = input(f'像素 {pixel} 对应 X Y（mm），R 取消: ').strip()
                    if text.lower() == 'r':
                        continue
                    xy = [float(value) for value in text.split()]
                    if len(xy) != 2 or not np.isfinite(xy).all():
                        raise ValueError('Expected two finite coordinates')
                    samples.append({'pixel': list(pixel), 'xy_mm': xy})
                    print(f'已采集 {len(samples)} 个点')
                except ValueError as error:
                    print(f'输入无效：{error}；请重新点击')
            display = frame.copy()
            for index, sample in enumerate(samples, 1):
                point = tuple(sample['pixel'])
                cv2.circle(display, point, 4, (0, 0, 255), -1)
                cv2.putText(display, str(index), point, cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            cv2.imshow(window, display)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), 27):
                return None
            if key == ord(' ') and frozen is None:
                frozen = frame.copy()
            elif key == ord('u') and samples:
                samples.pop()
            elif key == ord('s'):
                if frozen is None or len(samples) < 6:
                    print('请先冻结画面并采集至少 6 个点')
                    continue
                return samples
    finally:
        if capture is not None:
            capture.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='相机到云台旋转系外参标定：云台零位、水平已知坐标点')
    parser.add_argument('--device', default='/dev/camera_upper')
    parser.add_argument('--matrix', type=Path, default=Path('camera_matrix.npy'))
    parser.add_argument('--distortion', type=Path, default=Path('dist_coeffs.npy'))
    parser.add_argument('--plane-z-mm', required=True, type=float,
                        help='标记平面相对 gimbal_link 原点的 Z 高度（mm）')
    parser.add_argument('--width', type=int, default=640, help='必须与内参标定分辨率一致')
    parser.add_argument('--height', type=int, default=480)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--image', type=Path, help='从静止图像交互采点')
    source.add_argument('--points', type=Path, help='离线读取 JSON 对应点，无需相机/窗口')
    parser.add_argument('--output', type=Path, default=Path('camera_extrinsics.json'))
    args = parser.parse_args()
    try:
        if args.width <= 0 or args.height <= 0 or not np.isfinite(args.plane_z_mm):
            raise ValueError('Invalid image size or plane height')
        if args.output.exists():
            raise ValueError(f'Output already exists; choose another --output: {args.output}')
        matrix, distortion = load_intrinsics(args.matrix, args.distortion)
        if args.points:
            document = json.loads(args.points.read_text(encoding='utf-8'))
            if isinstance(document, dict):
                if document.get('image_size', [args.width, args.height]) != [args.width, args.height]:
                    raise ValueError('Stored image resolution does not match arguments')
                samples = document['samples']
            else:
                samples = document
        else:
            samples = collect_samples(args)
        if samples is None:
            return 0
        for sample in samples:
            u, v = sample['pixel']
            if not (0 <= u < args.width and 0 <= v < args.height):
                raise ValueError('Pixel outside image bounds')
        result = build_result(samples, args.plane_z_mm, matrix, distortion,
                              (args.width, args.height))
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        print(f'已保存：{args.output.resolve()}')
        print(f'外参重投影 RMS：{result["reprojection_rms_px"]:.3f} px')
        print(f'相机位置（gimbal_link，m）：{result["translation_m"]}')
        print('采样点拟合误差（mm）：', result['fit_point_errors_mm'])
        x, y, z = result['translation_m']
        qx, qy, qz, qw = result['quaternion_xyzw']
        print('静态 TF 发布命令（标定验证后使用）：')
        print('ros2 run tf2_ros static_transform_publisher '
              f'--x {x:.9g} --y {y:.9g} --z {z:.9g} '
              f'--qx {qx:.9g} --qy {qy:.9g} --qz {qz:.9g} --qw {qw:.9g} '
              '--frame-id gimbal_link --child-frame-id camera_optical_frame')
        print('拟合误差不代表独立定位精度；请用未参与标定的点验证后再接抓取。')
        return 0
    except (ValueError, OSError, KeyError, TypeError, cv2.error) as error:
        print(f'标定失败：{error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
