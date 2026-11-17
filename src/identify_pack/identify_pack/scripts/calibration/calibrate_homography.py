#!/usr/bin/env python3
"""
单应矩阵标定（眼在手外 · 手眼标定简化版）：像素坐标 -> 塔吊(机械臂)基座坐标(mm)。

流程：
  1. 塔吊末端/夹爪上贴一个醒目标记
  2. 依次移动到 4~6 个已知位置（尽量铺满工作区，不要共线）
  3. 每个位置：
     - 鼠标左键点击画面里的标记 -> 记录像素坐标
     - 在终端输入该位置的塔吊坐标 (x_mm y_mm)
  4. 采够 >= 4 个点后按 s 解算单应矩阵 H，保存到 homography_matrix.npy

用法：
  python3 calibrate_homography.py --device /dev/camera_upper

操作：
  左键点击 = 采一个像素点；s = 解算并保存；q = 退出
"""
import argparse
import cv2
import numpy as np

pixel_points = []   # 已采的像素点 (u, v)
robot_points = []   # 对应的塔吊坐标 (x_mm, y_mm)
pending = []        # 点击后等待输入塔吊坐标的像素点


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pending.append((x, y))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--device', '-d', default='/dev/camera_upper', help='相机设备路径')
    args = p.parse_args()

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

    cv2.namedWindow('calibrate')
    cv2.setMouseCallback('calibrate', on_mouse)

    print('=== 单应矩阵标定（像素 -> 塔吊 mm）===')
    print('1. 移动塔吊末端标记到已知位置')
    print('2. 左键点击画面里的标记')
    print('3. 在终端输入该位置的塔吊坐标，如: 120.5 300.0（单位 mm）')
    print('4. 重复 4~6 次，点尽量铺满工作区、不要共线')
    print('   按 s 解算保存，按 q 退出')

    while True:
        # 处理等待输入塔吊坐标的像素点（点击后逐个提示输入）
        while pending:
            u, v = pending.pop(0)
            try:
                inp = input(f'像素({u},{v}) 对应的塔吊坐标 (x_mm y_mm)，输入 r 撤销: ')
                if inp.strip().lower() == 'r':
                    print('  已撤销该点')
                    continue
                x, y = [float(t) for t in inp.split()]
                pixel_points.append((u, v))
                robot_points.append((x, y))
                print(f'  ✅ 已记录: 像素({u},{v}) -> 塔吊({x:.1f},{y:.1f})mm  (共 {len(pixel_points)} 点)')
            except Exception as e:
                print(f'  ❌ 输入无效({e})，请重新点击该点')

        ret, frame = cap.read()
        if not ret:
            break
        # 画已采集的点
        for (u, v) in pixel_points:
            cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)
        cv2.putText(frame, f'points: {len(pixel_points)}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('calibrate', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            if len(pixel_points) < 4:
                print(f'点数不足：当前 {len(pixel_points)} 个，至少 4 个')
                continue
            H, _ = cv2.findHomography(np.array(pixel_points, dtype=np.float32),
                                      np.array(robot_points, dtype=np.float32))
            if H is None:
                print('解算失败：检查点是否共线或重复')
                continue
            np.save('homography_matrix.npy', H)
            # 顺手输出 .txt（含每个点的映射误差）
            with open('homography_result.txt', 'w') as f:
                f.write('单应矩阵标定结果\n')
                f.write('=' * 40 + '\n')
                f.write(f'采样点数: {len(pixel_points)}\n\n')
                f.write('单应矩阵 H (3x3):\n')
                for row in H:
                    f.write('  ' + ' '.join(f'{v:.6f}' for v in row) + '\n')
                f.write('\n映射误差 (mm):\n')
                for (u, v), (x, y) in zip(pixel_points, robot_points):
                    pm = cv2.perspectiveTransform(np.array([[[u, v]]], dtype=np.float32), H)
                    err = np.hypot(pm[0][0][0] - x, pm[0][0][1] - y)
                    f.write(f'  像素({u},{v}) -> 预测({pm[0][0][0]:.1f},{pm[0][0][1]:.1f}) '
                            f'实际({x:.1f},{y:.1f}) 误差 {err:.1f}mm\n')
            print(f'\n✅ 已保存 homography_matrix.npy + homography_result.txt（共 {len(pixel_points)} 点）')
            print('单应矩阵 H =')
            print(H)
            # 打印每个点的映射误差（评估标定质量）
            print('--- 映射误差 ---')
            for (u, v), (x, y) in zip(pixel_points, robot_points):
                pm = cv2.perspectiveTransform(np.array([[[u, v]]], dtype=np.float32), H)
                err = np.hypot(pm[0][0][0] - x, pm[0][0][1] - y)
                print(f'  像素({u},{v}) -> 预测({pm[0][0][0]:.1f},{pm[0][0][1]:.1f}) '
                      f'实际({x:.1f},{y:.1f}) 误差 {err:.1f}mm')
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
