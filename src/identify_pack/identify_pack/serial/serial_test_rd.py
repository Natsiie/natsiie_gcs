#!/usr/bin/env python3
"""纯 Python 串口读取：数据一到立即显示（select 事件驱动，类似下位机中断）。

不轮询、不等固定间隔，串口缓冲区一旦有字节，select 立刻返回并打印，
每条带到达时间戳，用于验证硬件环回是否即时。

不依赖 ROS，不依赖 protocol。
"""

import argparse
import datetime
import select
import time

import serial


def main():
    parser = argparse.ArgumentParser(description='事件驱动串口读取')
    parser.add_argument('--serial', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=0.0,
                        help='读取时长（秒），0 表示一直读直到 Ctrl+C')
    args = parser.parse_args()

    try:
        ser = serial.Serial(port=args.serial, baudrate=args.baud, timeout=0)
    except Exception as error:
        print(f'打开串口失败 {args.serial}: {error}')
        return 1

    fd = ser.fileno()
    print(f'已打开 {ser.port} @ {ser.baudrate}，数据一到即显示，Ctrl+C 停止')
    total = 0
    last_ts = None
    start = time.monotonic()
    try:
        while True:
            # 阻塞等待 fd 可读（有数据到达）；0.1s 超时仅为让 Ctrl+C 及时生效
            ready, _, _ = select.select([fd], [], [], 0.1)
            if not ready:
                if args.duration > 0 and time.monotonic() - start >= args.duration:
                    break
                continue

            # 有数据：把当前缓冲区里的字节一次取走
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            total += len(chunk)
            now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
            gap = ''
            if last_ts is not None:
                gap = f'  +{time.monotonic() - last_ts:.3f}s'
            last_ts = time.monotonic()
            hex_str = chunk.hex(' ')
            text = chunk.decode('utf-8', errors='replace')
            print(f'[{now}] [{total:4d} B]{gap}\n    hex: {hex_str}\n    txt: {text!r}')

            if args.duration > 0 and time.monotonic() - start >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(f'\n已停止，共收到 {total} 字节')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
