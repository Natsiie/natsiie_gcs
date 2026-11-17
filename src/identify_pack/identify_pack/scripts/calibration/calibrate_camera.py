#!/usr/bin/env python3
"""
相机内参标定（棋盘格）：解出 camera_matrix + dist_coeffs，用于去畸变。

流程：
  1. 打印一张棋盘格（默认 9x6 个内角点，即 10x7 个格子）
  2. 运行本脚本，把棋盘格对着相机，从不同角度/位置缓慢移动、轻微旋转
  3. 脚本自动检测角点并采样（画面显示角点连线 + 已采样计数）
  4. 采够约 15~20 张后按 s 解算，保存 camera_matrix.npy + dist_coeffs.npy

用法：
  python3 calibrate_camera.py --device /dev/camera_upper --cols 9 --rows 6 --square 20.0

操作：
  自动采样（检测到棋盘格且间隔足够就存一张）；c=手动采一张；s=解算保存；q=退出
"""
import argparse
import time
import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--device', '-d', default='/dev/camera_upper', help='相机设备路径')
    p.add_argument('--cols', type=int, default=9, help='棋盘格内角点列数')
    p.add_argument('--rows', type=int, default=6, help='棋盘格内角点行数')
    p.add_argument('--square', type=float, default=24.0, help='每个格子的实际边长(mm)')
    args = p.parse_args()

    pattern = (args.cols, args.rows)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        try:
            cap = cv2.VideoCapture(int(args.device))
        except Exception:
            pass
    if not cap.isOpened():
        print(f'无法打开相机: {args.device}')
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 棋盘格 3D 物体点（z=0 平面）
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    objpoints = []   # 3D 点
    imgpoints = []   # 2D 像素点

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    last_capture = 0.0

    print('=== 相机内参标定 ===')
    print('把棋盘格对着相机，缓慢移动/旋转，覆盖画面的各个区域')
    print('采够约 15 张后按 s 解算保存，q 退出')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        corners2 = None
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(frame, pattern, corners2, found)
            # 自动采样：检测到且距上次采样 > 0.6s
            now = time.time()
            if now - last_capture > 0.6:
                imgpoints.append(corners2)
                objpoints.append(objp)
                last_capture = now
                print(f'  已采样 {len(imgpoints)} 张')

        cv2.putText(frame, f'captured: {len(imgpoints)}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('calibrate', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c'):
            if found and corners2 is not None:
                imgpoints.append(corners2)
                objpoints.append(objp)
                print(f'  手动采样，共 {len(imgpoints)} 张')
        elif key == ord('s'):
            if len(imgpoints) < 10:
                print(f'样本不足：当前 {len(imgpoints)} 张，至少 10 张')
                continue
            rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None)
            np.save('camera_matrix.npy', mtx)
            np.save('dist_coeffs.npy', dist)
            # 顺手输出 .txt
            with open('camera_calibration.txt', 'w') as f:
                f.write('相机内参标定结果\n')
                f.write('=' * 40 + '\n')
                f.write(f'采样张数: {len(imgpoints)}\n')
                f.write(f'图像尺寸: {gray.shape[1]} x {gray.shape[0]}\n')
                f.write(f'重投影误差 RMS: {rms:.4f}\n\n')
                f.write('内参矩阵 camera_matrix (3x3):\n')
                for row in mtx:
                    f.write('  ' + ' '.join(f'{v:.6f}' for v in row) + '\n')
                f.write('\n畸变系数 dist_coeffs (k1, k2, p1, p2, k3):\n')
                f.write('  ' + ' '.join(f'{v:.6f}' for v in dist.ravel()) + '\n')
            print(f'\n✅ 已保存 camera_matrix.npy + dist_coeffs.npy + camera_calibration.txt（共 {len(imgpoints)} 张）')
            print(f'重投影误差 RMS = {rms:.4f}（越小越好，<0.5 较理想）')
            print('内参矩阵 mtx =')
            print(mtx)
            print('畸变系数 dist =')
            print(dist.ravel())
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
