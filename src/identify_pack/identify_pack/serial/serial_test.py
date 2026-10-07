#!/usr/bin/env python3
"""串口通信自检：把 01~08 打包成协议帧连续发送，不等待回传。

独立运行，不依赖 ROS。参数优先级：命令行 > serial_config.json > 默认值。
"""

import argparse
import json
import os
import sys

import serial

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 0.1


def make_frame(type_byte, payload):
    """在函数内打包协议帧：FF | TYPE | LEN | PAYLOAD | CRC8 | FE。"""
    payload = bytes(payload)
    if len(payload) > 255:
        raise ValueError(f'payload 过长: {len(payload)} > 255')
    body = bytes([type_byte, len(payload)]) + payload
    crc = 0
    for b in body:
        crc ^= b
    return bytes([0xFF]) + body + bytes([crc, 0xFE])


def load_file_config():
    """读同目录 serial_config.json，读不到就返回空字典。"""
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'serial_config.json')
    if not os.path.exists(config_path):
        return {}
    try:
        with open(config_path, encoding='utf-8') as stream:
            return json.load(stream)
    except Exception as error:
        print(f'读取配置失败 {config_path}: {error}', file=sys.stderr)
        return {}


def main():
    parser = argparse.ArgumentParser(description='连续发送固定字节 01~08 验证串口通信')
    parser.add_argument('--serial', default=None, help='串口设备，默认 /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=None, help='波特率，默认 115200')
    parser.add_argument('--timeout', type=float, default=None,
                        help='读超时（秒），默认 0.2')
    parser.add_argument('--type', type=lambda s: int(s, 0), default=0x01,
                        help='消息类型号（默认 0x01），可用 0x 前缀')
    parser.add_argument('--interval', type=float, default=0.5,
                        help='发送间隔（秒），默认 0.5')
    args = parser.parse_args()

    file_cfg = load_file_config()
    port = args.serial or file_cfg.get('serial_port', DEFAULT_PORT)
    baud = args.baud or file_cfg.get('baud', DEFAULT_BAUD)
    timeout = args.timeout if args.timeout is not None \
        else file_cfg.get('timeout', DEFAULT_TIMEOUT)

    try:
        ser = serial.Serial(port=port, baudrate=int(baud), timeout=float(timeout))
    except Exception as error:
        print(f'打开串口失败 {port}: {error}', file=sys.stderr)
        return 1

    print(f'已打开 {ser.port} @ {ser.baudrate}，超时 {timeout}s')
    payload = bytes(range(0x05, 0x0D))   # 05 06 07 08 09 0A 0B 0C
    frame = make_frame(args.type, payload)
    print(f'payload: {payload.hex(" ")}，type=0x{args.type:02X}')
    print(f'帧: {frame.hex(" ")}（{len(frame)} 字节）')
    print(f'开始连续发送，间隔 {args.interval}s，Ctrl+C 停止')
    import time
    count = 0
    try:
        while True:
            try:
                written = ser.write(frame)
                ser.flush()
            except Exception as error:
                print(f'发送失败: {error}', file=sys.stderr)
                ser.close()
                return 1
            count += 1
            print(f'[{count}] 已发送 {written} 字节: {frame.hex(" ")}')
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print(f'\n已停止，共发送 {count} 次')
    finally:
        ser.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
