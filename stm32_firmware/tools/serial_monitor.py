#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口协议监视器 —— 联调时把 STM32 <-> 上位机 的帧翻译成人能看的一行行。

帧格式 (与 PROTOCOL.md / Devices/Serial.h 一致):
    0xFF | TYPE | LEN | PAYLOAD(UTF-8) | CRC8 | 0xFE
    CRC8 = TYPE ^ LEN ^ payload 逐字节异或

用法 (用 pyOCD 那个 venv 的 python, 里面已经有 pyserial):
    C:\\Users\\xszx1\\.dsh\\tools\\pyocd\\Scripts\\python.exe tools\\serial_monitor.py --port COM3
    ... --baud 115200            串口参数 (默认 115200 8N1)
    ... --types 0x0F,0x10        只看这几类帧
    ... --raw                    额外打印原始 HEX
    ... --log run.log            同时写文件

按 Ctrl+C 结束, 会打印统计: 各类帧数量 / CRC 错误 / 丢弃字节。
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('缺少 pyserial。用 pyOCD 的 venv 运行: '
             r'C:\Users\xszx1\.dsh\tools\pyocd\Scripts\python.exe tools\serial_monitor.py --port COM3')

HEAD, TAIL = 0xFF, 0xFE

TYPE_NAMES = {
    0x01: 'Color', 0x02: 'ColorMotion', 0x03: 'Turntable', 0x04: 'ring_station',
    0x05: 'stacked_ring', 0x06: 'QR_Decode', 0x07: 'GimbalYaw', 0x08: 'ColorTarget',
    0x09: 'GrabCmd', 0x0A: 'PlaceCommand', 0x0B: 'PlaceResult', 0x0C: 'RingTarget/UnloadCoord',
    0x0D: 'PlaceControl', 0x0E: 'TurntableAlign', 0x0F: 'PickDone', 0x10: 'NextBatch',
}

# 每类的 payload 说明, 用于打印抬头
TYPE_HINT = {
    0x07: '云台角度(deg)', 0x0F: '抓取完成序号', 0x10: '批次就绪',
    0x0B: 'task_id,command_id,success', 0x0D: 'start,stage,batch | retry,command_id',
    0x0A: 'task_id,cmd_id,stage,batch,seq,color,ring,layer,x,y,z',
    0x0C: 'x1,y1,x2,y2,x3,y3', 0x0E: 'corr_x,corr_y,within',
}


def parse_extra(payload, type_byte):
    """把常见 payload 拆开看得更清楚; 解析失败就原样返回。"""
    try:
        if type_byte in (0x01,):
            return ' | '.join(payload.split('/'))
        if type_byte in (0x02, 0x03, 0x04, 0x05, 0x0C, 0x0E, 0x08):
            return ', '.join(payload.split(','))
    except Exception:
        pass
    return ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True, help='例如 COM3 (/dev/ttyUSB0)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--types', default='', help='只看这些类型, 逗号分隔, 例如 0x0F,0x10')
    ap.add_argument('--raw', action='store_true', help='打印原始 HEX')
    ap.add_argument('--log', default='', help='同时写入日志文件')
    args = ap.parse_args()

    only = set()
    for token in args.types.split(','):
        token = token.strip()
        if token:
            only.add(int(token, 16) if token.lower().startswith('0x') else int(token, 10))

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    logf = open(args.log, 'a', encoding='utf-8') if args.log else None

    buf = bytearray()
    counts = {}
    crc_errors = 0
    dropped = 0
    t0 = time.time()

    def emit(line):
        stamp = time.strftime('%H:%M:%S')
        text = f'[{stamp}] {line}'
        print(text, flush=True)
        if logf:
            logf.write(text + '\n')
            logf.flush()

    emit(f'# 打开 {args.port} @ {args.baud} 8N1; 只看 {sorted(hex(t) for t in only) if only else "全部类型"}')
    try:
        while True:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                start = buf.find(HEAD)
                if start < 0:
                    dropped += len(buf)
                    buf.clear()
                    break
                if start:
                    dropped += start
                    del buf[:start]
                if len(buf) < 3:
                    break
                length = buf[2]
                need = 5 + length
                if len(buf) < need:
                    break
                if buf[need - 1] != TAIL:
                    dropped += 1
                    del buf[0]
                    continue
                body = buf[1:3 + length]
                crc = 0
                for b in body:
                    crc ^= b
                if crc != buf[need - 2]:
                    crc_errors += 1
                    emit(f'!! CRC 不匹配 type=0x{buf[1]:02X} len={length} '
                         f'算得=0x{crc:02X} 收到=0x{buf[need - 2]:02X}')
                    del buf[0]
                    continue
                type_byte = buf[1]
                payload = buf[3:3 + length].decode('utf-8', errors='replace')
                frame = bytes(buf[:need])
                del buf[:need]

                counts[type_byte] = counts.get(type_byte, 0) + 1
                if only and type_byte not in only:
                    continue
                name = TYPE_NAMES.get(type_byte, '未知')
                hint = TYPE_HINT.get(type_byte, '')
                extra = parse_extra(payload, type_byte)
                line = f'0x{type_byte:02X} {name:<22} {payload!r}'
                if hint:
                    line += f'   <{hint}>'
                if extra and extra != payload:
                    line += f'\n            {extra}'
                if args.raw:
                    line += f'\n            HEX {frame.hex(" ")}'
                emit(line)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if logf:
            logf.close()
        emit(f'# 结束: 共 {sum(counts.values())} 帧, 用时 {time.time() - t0:.1f}s, '
             f'CRC 错误 {crc_errors}, 丢弃 {dropped} 字节')
        for t in sorted(counts):
            emit(f'#   0x{t:02X} {TYPE_NAMES.get(t, "未知"):<22} {counts[t]}')


if __name__ == '__main__':
    main()
