#!/usr/bin/env python3
"""夹爪 3D 点相机外参标定（塔吊臂 · 相机随云台）。

相机固定在云台上、随云台偏航旋转；夹爪只能外伸(x) + 升降(z)，无法侧移(y=0)。
用夹爪在 (x, 0, z) 上采 >=6 个 3D 点，solvePnP 解「相机相对 gimbal_link」的位姿。

坐标系约定（与现场一致）：
  x = 标记到云台旋转轴的径向距离（mm），使用 kin_pose_r，不是 kin_pose_x
  y = 面朝 +x 的左手边为正（恒为 0，夹爪不能侧移）
  z = 升降高度，相对云台旋转轴原点（mm，向上为正）

流程：
  1. 移动夹爪到不同 (x, z) 位置——伸展 x 和升降 z 都要变，尽量铺满画面
  2. Space 冻结/恢复画面；左键点击同一个固定标记；终端输入 x z（mm）
     每采一个位置后恢复画面，移动夹爪，停稳后再次冻结；u 撤销上一点
  3. 采够 >=6 点后按 s 解算，输出 camera_extrinsics JSON（内参+外参）

标记必须位于 y=0 平面；z 是标记相对旋转系原点的真实高度，向上为正，
不能直接使用未经零点/比例校准的电机读数。标记相对运动学参考点有偏移时，
x、z 还需加入实测标记偏移。建议 x、z 各取 3 个位置组成网格；
运动点和图像点都不能共线或接近共线。使用与内参一致的原始画面、分辨率和焦距。

用法：
  python3 calibrate_camera_extrinsics_arm.py --device /dev/camera_upper \
      --matrix camera_matrix.npy --distortion dist_coeffs.npy \
      --output /home/lulu/gcs/camera_extrinsics_v2.json --force
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

try:
    from camera_geometry import load_intrinsics
except ImportError:
    from .camera_geometry import load_intrinsics


# 把 X-Z 平面（y=0）旋转到 Z=0 平面，供 SOLVEPNP_IPPE 使用。
# 映射 [x, y, z] -> [x, z, -y]
R_PLANE = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0]])


def load_pink_ranges(color_config):
    """从 color_config.json 读取粉色(color_id=7)的 HSV 区间。"""
    document = json.loads(Path(color_config).read_text(encoding='utf-8'))
    for color in document.get('colors', []):
        if color.get('color_id') == 7:
            ranges = color.get('ranges')
            if not ranges:
                raise ValueError('粉色(color_id=7)配置缺少 ranges')
            return ranges
    raise ValueError('color_config.json 中未找到粉色(color_id=7)配置')


def detect_pink_center(frame, pink_ranges):
    """在 BGR 帧中检测粉色方块的中心像素，找不到返回 None。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for rng in pink_ranges:
        lower = np.array(rng['lower'], dtype=np.uint8)
        upper = np.array(rng['upper'], dtype=np.uint8)
        mask |= cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 60:
        return None
    moments = cv2.moments(largest)
    if moments['m00'] <= 0:
        return None
    return int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])


def validate_correspondences(objects_mm, pixels):
    objects = np.asarray(objects_mm, dtype=np.float64)
    pixels = np.asarray(pixels, dtype=np.float64)
    if (objects.ndim != 2 or objects.shape[1] != 3
            or pixels.shape != (len(objects), 2) or len(objects) < 6
            or not np.isfinite(objects).all() or not np.isfinite(pixels).all()):
        raise ValueError('至少需要 6 个有限的 3D 点/像素对')
    if not np.allclose(objects[:, 1], 0.0, rtol=0.0, atol=1e-6):
        raise ValueError('标记必须位于 gimbal_link 的 y=0 平面')
    for points, name in ((objects[:, [0, 2]], 'x-z 坐标'), (pixels, '像素坐标')):
        if len(np.unique(points, axis=0)) != len(points):
            raise ValueError(f'{name}存在重复点')
        singular = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
        minimum_ratio = 0.05 if name == '像素坐标' else 1e-3
        ratio = singular[1] / singular[0] if singular[0] > 0 else 0.0
        if ratio <= minimum_ratio:
            if name == '像素坐标':
                raise ValueError(
                    f'像素坐标共线或接近共线：二维展开比例 {ratio:.1%}，'
                    f'要求大于 {minimum_ratio:.0%}。请核对点击标记与 x z 坐标；'
                    '若伸展/升降仍只形成一条窄线，应调整相机侧向安装位置后重采，'
                    '或改用水平已知坐标点标定；单纯转云台不能改善夹爪采点覆盖')
            raise ValueError(f'{name}共线或接近共线，请在 x、z 两方向采点并扩大二维覆盖')


def solve_camera_transform(objects_mm, pixels, matrix, distortion):
    """Return gimbal_link <- camera_optical_frame from 3D gripper points (mm)."""
    validate_correspondences(objects_mm, pixels)
    objects = np.asarray(objects_mm, dtype=np.float64) / 1000.0  # 转米
    pixels = np.asarray(pixels, dtype=np.float64)
    normalized = cv2.undistortPoints(pixels.reshape(-1, 1, 2), matrix, distortion).reshape(-1, 2)
    validate_correspondences(objects_mm, normalized)
    origin = objects.mean(axis=0)
    planar = (objects - origin) @ R_PLANE.T  # 居中后落到 Z=0 平面
    result = cv2.solvePnPGeneric(planar, pixels, matrix, distortion,
                                 flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for rvec, tvec in zip(result[1], result[2]):
        rvec, tvec = cv2.solvePnPRefineLM(planar, pixels, matrix, distortion,
                                          rvec.copy(), tvec.copy())
        cam_from_plane, _ = cv2.Rodrigues(rvec)      # P_cam = R' @ P_plane + t'
        cam_from_world = cam_from_plane @ R_PLANE    # P_cam = R @ P_world + t
        world_tvec = tvec.reshape(3) - cam_from_world @ origin
        depths = (objects @ cam_from_world.T + world_tvec)[:, 2]
        if not np.isfinite(depths).all() or np.any(depths <= 0):
            continue  # 相机不在点的前方
        rotation = cam_from_world.T                  # gimbal_link <- camera
        translation = -rotation @ world_tvec
        rvec_world, _ = cv2.Rodrigues(cam_from_world)
        projected, _ = cv2.projectPoints(objects, rvec_world, world_tvec, matrix, distortion)
        errors = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1)
        if not np.isfinite(errors).all():
            continue
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        candidates.append((float(np.sqrt(np.mean(errors ** 2))), transform, errors))
    if not candidates:
        raise ValueError(
            '求解候选均未通过正深度/有限数值检查；请核对每张冻结图像的标记坐标、'
            '径向距离(mm)、高度零点与正方向，以及原始图像与内参是否一致')
    rms, transform, errors = min(candidates, key=lambda item: item[0])
    return transform, rms, errors


def build_result(samples, matrix, distortion, image_size, plane_z_mm=-116):
    if not np.isfinite(plane_z_mm):
        raise ValueError('验证平面高度必须为有限数值')
    transform, rms, errors = solve_camera_transform(
        [s['xyz_mm'] for s in samples],
        [s['pixel'] for s in samples], matrix, distortion)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    axis_angle = cv2.Rodrigues(rotation)[0].reshape(3)
    angle = np.linalg.norm(axis_angle)
    quaternion = (np.append(axis_angle * (np.sin(angle / 2.0) / angle),
                            np.cos(angle / 2.0)) if angle > 1e-12
                  else np.array([0.0, 0.0, 0.0, 1.0]))
    return {
        'schema_version': 1,
        'parent_frame': 'gimbal_link',
        'child_frame': 'camera_optical_frame',
        'transform_convention': 'p_parent_m = R @ p_child_m + translation_m',
        'calibration_method': 'gripper_xz',
        'image_size': list(image_size),
        'camera_matrix': matrix.tolist(),
        'dist_coeffs': distortion.tolist(),
        # 下游 calibration 模式用此水平面高度投影；它不是本次采点的 X-Z 平面。
        'calibration_plane_z_mm': float(plane_z_mm),
        'rotation_matrix': rotation.tolist(),
        'translation_m': translation.tolist(),
        'quaternion_xyzw': quaternion.tolist(),
        'parent_from_camera_matrix': transform.tolist(),
        'reprojection_rms_px': rms,
        'point_errors_px': errors.tolist(),
        'samples': samples,
    }


def collect_samples(args, matrix, distortion, pink_ranges):
    device = int(args.device) if args.device.isdecimal() else args.device
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise ValueError(f'Cannot open camera: {args.device}')
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    frozen = None
    pending = []
    samples = []
    detected = None

    def on_mouse(event, x, y, _flags, _param):
        if (event == cv2.EVENT_LBUTTONDOWN and frozen is not None and not pending
                and 0 <= x < args.width and 0 <= y < args.height):
            pending.append((x, y))

    cv2.namedWindow('calibrate_arm')
    cv2.setMouseCallback('calibrate_arm', on_mouse)
    print('=== 夹爪 3D 点相机外参标定 ===')
    print('Space 冻结/恢复；左键采点；输入 x z（mm）；U 撤销上一点；S 解算；Q 退出')
    print('x 填 kin_pose_r（径向距离），z 填标记的真实高度（相对旋转系原点、向上为正）')
    print('每次使用同一标记；每采完一个位置恢复画面，移动夹爪，停稳后重新冻结')
    try:
        while True:
            # 冻结仅影响显示，持续采集避免恢复时使用积压的旧帧。
            ok, live_frame = capture.read()
            if not ok or live_frame is None:
                raise ValueError('Cannot read camera frame')
            frame = live_frame if frozen is None else frozen
            if frame.shape[1::-1] != (args.width, args.height):
                raise ValueError('图像分辨率与 --width/--height 不一致')
            while pending:
                u, v = pending.pop(0)
                try:
                    text = input(f'像素({u},{v}) 对应 x z（mm，径向距离 真实高度），r 取消本次: ').strip()
                    if text.lower() == 'r':
                        continue
                    parts = [float(t) for t in text.split()]
                    if len(parts) != 2 or not np.isfinite(parts).all():
                        raise ValueError('需要两个有限坐标')
                    x, z = parts
                    if any(s['pixel'] == [u, v] or s['xyz_mm'] == [x, 0.0, z]
                           for s in samples):
                        raise ValueError('重复点，请恢复画面并移动夹爪后重新采点')
                    samples.append({'pixel': [u, v], 'xyz_mm': [x, 0.0, z]})
                    print(f'  已采集 {len(samples)} 个点: (x={x:.1f}, z={z:.1f})mm')
                except ValueError as error:
                    print(f'  输入无效：{error}；请重新点击')
            display = frame.copy()
            for index, sample in enumerate(samples, 1):
                pt = tuple(sample['pixel'])
                cv2.circle(display, pt, 4, (0, 0, 255), -1)
                cv2.putText(display, str(index), pt, cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            if frozen is not None and detected is not None:
                cv2.circle(display, detected, 6, (0, 255, 255), 2)
                cv2.line(display, (detected[0] - 12, detected[1]),
                         (detected[0] + 12, detected[1]), (0, 255, 255), 1)
                cv2.line(display, (detected[0], detected[1] - 12),
                         (detected[0], detected[1] + 12), (0, 255, 255), 1)
            cv2.imshow('calibrate_arm', display)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                return None
            if key == ord(' '):
                frozen = live_frame.copy() if frozen is None else None
                pending.clear()
                detected = None
                if frozen is not None:
                    detected = detect_pink_center(frozen, pink_ranges)
                    if detected is None:
                        print('  未检测到粉色方块，可左键手动采点')
                    else:
                        print(f'  检测到粉色方块中心 {detected}，按 Enter 确认采点')
            elif key in (13, ord('a'), ord('A')):
                if frozen is not None and detected is not None and not pending:
                    pending.append(detected)
                    detected = None
            elif key in (ord('u'), ord('U')) and samples:
                samples.pop()
                pending.clear()
                print(f'已撤销上一点，剩余 {len(samples)} 个点')
            elif key in (ord('s'), ord('S')):
                try:
                    validate_correspondences([s['xyz_mm'] for s in samples],
                                             [s['pixel'] for s in samples])
                    solve_camera_transform([s['xyz_mm'] for s in samples],
                                           [s['pixel'] for s in samples], matrix, distortion)
                except (ValueError, cv2.error) as error:
                    print(f'暂不能解算：{error}；可继续采点或按 U 撤销')
                    continue
                return samples
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='夹爪 3D 点相机外参标定（塔吊臂）')
    parser.add_argument('--device', default='/dev/camera_upper')
    parser.add_argument('--matrix', type=Path, default=Path('camera_matrix.npy'))
    parser.add_argument('--distortion', type=Path, default=Path('dist_coeffs.npy'))
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--plane-z-mm', type=float, default=-116,
                        help='下游 calibration 模式使用的水平验证平面高度(mm)，需实测；不参与外参求解')
    parser.add_argument('--color-config', default='',
                        help='粉色(color_id=7) HSV 配置 color_config.json 路径；留空用脚本同目录')
    parser.add_argument('--output', type=Path, default=Path('camera_extrinsics_arm.json'))
    parser.add_argument('--force', action='store_true', help='覆盖已存在的输出文件')
    args = parser.parse_args()
    try:
        if args.width <= 0 or args.height <= 0 or not np.isfinite(args.plane_z_mm):
            raise ValueError('Invalid image size')
        if args.output.exists() and not args.force:
            raise ValueError(f'输出文件已存在，加 --force 覆盖或换个 --output：{args.output}')
        matrix, distortion = load_intrinsics(args.matrix, args.distortion)
        color_config = args.color_config or str(Path(__file__).with_name('color_config.json'))
        pink_ranges = load_pink_ranges(color_config)
        samples = collect_samples(args, matrix, distortion, pink_ranges)
        if samples is None:
            return 0
        result = build_result(samples, matrix, distortion, (args.width, args.height), args.plane_z_mm)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                          allow_nan=False) + '\n', encoding='utf-8')
        print(f'\n✅ 已保存：{args.output.resolve()}')
        print(f'外参重投影 RMS：{result["reprojection_rms_px"]:.3f} px')
        print(f'相机位置（gimbal_link，m）：{result["translation_m"]}')
        print(f'各点重投影误差（px）：{[f"{e:.2f}" for e in result["point_errors_px"]]}')
        print('提示：RMS 越小越好；请用未参与标定的点再验证一次。')
        return 0
    except (ValueError, OSError, KeyError, TypeError, cv2.error) as error:
        print(f'标定失败：{error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
