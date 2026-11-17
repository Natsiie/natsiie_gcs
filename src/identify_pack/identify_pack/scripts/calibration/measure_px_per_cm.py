#!/usr/bin/env python3
"""
测量「像素/厘米」比例（方法二：放尺子）。

步骤：
  1. 把直尺平放在目标位置（相机正下方 38.5cm 处）
  2. 运行本脚本，在画面里依次左键点击直尺上两个刻度点（如 0cm 和 10cm）
  3. 脚本自动算出 像素/厘米，并顺便给出圆环参数建议

用法：
  python3 measure_px_per_cm.py --device /dev/camera_upper --cm 10

操作：
  鼠标左键：点两个点（先后两个刻度）
  r：重新选点
  q：退出
"""
import argparse
import cv2

points = []


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
        points.append((x, y))


def dist(p1, p2):
    return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--device', '-d', default='/dev/camera_upper', help='相机设备路径')
    p.add_argument('--cm', type=float, default=10.0, help='你点的两个点之间的实际长度(厘米)')
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

    cv2.namedWindow('measure')
    cv2.setMouseCallback('measure', on_mouse)
    print(f'请依次点击直尺上两个刻度点（实际长度 {args.cm} cm）。r=重选，q=退出')

    computed = False
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        show = frame.copy()
        for pt in points:
            cv2.circle(show, pt, 4, (0, 0, 255), -1)

        if len(points) == 2:
            cv2.line(show, points[0], points[1], (0, 255, 0), 2)
            d_px = dist(points[0], points[1])
            cv2.putText(show, f'{d_px:.1f} px', (points[0][0], points[0][1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if not computed:
                computed = True
                px_per_cm = d_px / args.cm
                print(f'\n像素距离 {d_px:.1f} px / {args.cm} cm = {px_per_cm:.2f} 像素/厘米')
                print('--- 据此推算圆环参数 ---')
                print(f'  数字半长 0.8cm  = {0.8 * px_per_cm:.1f} px')
                print(f'  内环半径 2.5cm  = {2.5 * px_per_cm:.1f} px')
                print(f'  外环半径 4.75cm = {4.75 * px_per_cm:.1f} px')
                min_r = max(5, int(0.5 * 2.5 * px_per_cm))
                max_r = int(2.1 * 4.75 * px_per_cm)
                print(f'  -> 建议 min_radius={min_r}, max_radius={max_r}\n')

        cv2.imshow('measure', show)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            points.clear()
            computed = False
            print('已清空，重新选点')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
