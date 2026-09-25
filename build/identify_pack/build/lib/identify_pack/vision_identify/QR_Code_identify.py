#!/usr/bin/env python3
"""
轻量级 QR 码识别脚本（参考 zongchengxu20250724chusai.py 的策略）

功能：
- 从指定摄像头设备读取视频帧
- 使用 pyzbar 解码二维码（在灰度图上解码以提高稳定性）
- 要求连续 N 帧读到相同二维码数据才确认（默认 3 次）
- 可选通过串口发送格式化数据（与原代码兼容的包头/尾）

用法示例：
    python QR_Code_identify.py --device /dev/video_xia0 --serial /dev/ttyUSB0 --baud 9600

"""
import argparse
import sys
import time
import cv2
from pyzbar.pyzbar import decode

try:
    import serial
except Exception:
    serial = None


def send_serial(port, baud, data_str):
    """按原脚本格式化并发送数据：包头 0xFF，包尾 0xFE，中间最多 9 字节（不足补零）。"""
    if serial is None:
        print("pyserial 未安装，无法发送串口数据")
        return
    try:
        s = serial.Serial(port=port, baudrate=baud, timeout=0.1)
    except Exception as e:
        print(f"打开串口失败: {e}")
        return
    try:
        arr = []
        for ch in data_str:
            try:
                arr.append(ord(ch))
            except Exception:
                arr.append(0)
        while len(arr) < 9:
            arr.append(0)
        packet = bytes([0xFF] + arr[:9] + [0xFE])
        s.write(packet)
        print(f"已发送串口数据: {packet}")
    finally:
        s.close()


def run(device, min_consistent=3, timeout=None, serial_port=None, baud=9600, show=False):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"无法打开摄像头设备: {device}")
        return 1

    last_data = None
    consistent_count = 0
    last_valid = None
    start_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取帧，重试...")
                time.sleep(0.05)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            decoded = decode(gray)
            current = None
            for obj in decoded:
                try:
                    txt = obj.data.decode('utf-8').strip()
                except Exception:
                    txt = None
                if txt:
                    current = txt
                    break

            if current:
                # 连续 N 帧一致确认
                if current == last_data:
                    consistent_count += 1
                else:
                    last_data = current
                    consistent_count = 1

                print(f"检测到 QR: {current} (一致计数={consistent_count})")

                if consistent_count >= min_consistent:
                    print(f"确认 QR 数据: {current}")
                    last_valid = current
                    # 格式化：如果长度 <7 则右填充 '0'，再补一个 '0'（与原项目兼容）
                    formatted = current.ljust(7, '0')[:7] + '0'
                    if serial_port:
                        send_serial(serial_port, baud, formatted)
                    else:
                        print(f"格式化发送内容(未串口): {formatted}")
                    break

            # 可视化用于调试
            if show:
                cv2.imshow('QR', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if timeout and (time.time() - start_time) > timeout:
                print("达到超时，退出")
                break

    except KeyboardInterrupt:
        print("用户中断")
    finally:
        cap.release()
        if show:
            cv2.destroyAllWindows()

    return 0 if last_valid else 2


def parse_args():
    p = argparse.ArgumentParser(description='QR 码识别脚本（连续一致性验证）')
    p.add_argument('--device', '-d', default='/dev/video4', help='摄像头设备路径')
    p.add_argument('--min-consistent', type=int, default=3, help='需要连续相同帧数确认')
    p.add_argument('--timeout', type=float, default=None, help='最长运行秒数（可选）')
    p.add_argument('--serial', '-s', default=None, help='串口设备路径（可选）')
    p.add_argument('--baud', type=int, default=115200, help='串口波特率')
    p.add_argument('--show', action='store_true', help='显示调试窗口')
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()
    sys.exit(run(args.device, args.min_consistent, args.timeout, args.serial, args.baud, args.show))
