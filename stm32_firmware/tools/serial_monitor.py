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
    ... --self-test              不接串口, 用合成帧自检解析逻辑

按 Ctrl+C 结束, 会打印统计: 各类帧数量 / CRC 错误 / 丢弃字节。
"""

import argparse
import sys
import time

HEAD, TAIL = 0xFF, 0xFE

TYPE_NAMES = {
    0x01: 'Color', 0x02: 'ColorMotion', 0x03: 'Turntable', 0x04: 'ring_station',
    0x05: 'stacked_ring', 0x06: 'QR_Decode', 0x07: 'GimbalYaw', 0x08: 'ColorTarget',
    0x09: 'GrabCmd', 0x0A: 'PlaceCommand', 0x0B: 'PlaceResult', 0x0C: 'RingTarget/UnloadCoord',
    0x0D: 'PlaceControl', 0x0E: 'TurntableAlign', 0x0F: 'PickDone', 0x10: 'NextBatch',
}

TYPE_HINT = {
    0x07: '云台角度(deg)', 0x0F: '抓取完成序号', 0x10: '批次就绪',
    0x0B: 'task_id,command_id,success', 0x0D: 'start,stage,batch | retry,command_id',
    0x0A: 'task_id,cmd_id,stage,batch,seq,color,ring,layer,x,y,z',
    0x0C: 'x1,y1,x2,y2,x3,y3', 0x0E: 'corr_x,corr_y,within',
}


def make_frame(type_byte, text):
    """按协议打包一帧, 用于自检。"""
    payload = text.encode('utf-8')
    body = bytes([type_byte, len(payload)]) + payload
    crc = 0
    for b in body:
        crc ^= b
    return bytes([HEAD]) + body + bytes([crc, TAIL])


def extract_frames(buf):
    """从 bytearray 里取出全部完整且 CRC 正确的帧; 原地消费 buf。

    返回 [(type_byte, payload_str, raw_bytes), ...] 与 (丢弃字节数, CRC 错误数)。
    规则与上位机 protocol.py / 固件 RX 状态机一致: 坏帧丢 1 字节后重新找帧头,
    不会因为一帧坏了就卡住后面的合法帧。
    """
    frames = []
    dropped = 0
    crc_errors = 0
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
        crc = 0
        for b in buf[1:3 + length]:
            crc ^= b
        if crc != buf[need - 2]:
            crc_errors += 1
            del buf[0]
            continue
        raw = bytes(buf[:need])
        payload = buf[3:3 + length].decode('utf-8', errors='replace')
        frames.append((buf[1], payload, raw))
        del buf[:need]
    return frames, dropped, crc_errors


def describe(type_byte, payload, raw, show_raw=False):
    name = TYPE_NAMES.get(type_byte, '未知')
    hint = TYPE_HINT.get(type_byte, '')
    line = f'0x{type_byte:02X} {name:<22} {payload!r}'
    if hint:
        line += f'   <{hint}>'
    extra = ''
    try:
        if type_byte == 0x01:
            extra = ' | '.join(payload.split('/'))
        elif type_byte in (0x02, 0x03, 0x04, 0x05, 0x08, 0x0C, 0x0E):
            extra = ', '.join(payload.split(','))
    except Exception:
        extra = ''
    if extra and extra != payload:
        line += f'\n            {extra}'
    if show_raw:
        line += f'\n            HEX {raw.hex(" ")}'
    return line


def self_test():
    """不接硬件的自检: 正常帧 / 粘包 / 帧内夹垃圾 / CRC 坏帧 / 截断帧。"""
    checks = []
    buf = bytearray()
    buf += make_frame(0x0F, '3')
    buf += make_frame(0x10, '1')
    frames, dropped, crc_err = extract_frames(buf)
    checks.append(('两个正常帧被解出', [f[0] for f in frames] == [0x0F, 0x10] and not dropped and not crc_err))

    buf = bytearray(b'\x00\x11\x22')
    buf += make_frame(0x07, '90.0')
    frames, dropped, crc_err = extract_frames(buf)
    checks.append(('帧前垃圾被计为丢弃', [f[0] for f in frames] == [0x07] and dropped == 3 and not crc_err))

    bad = bytearray(make_frame(0x0F, '5'))
    bad[-2] ^= 0xFF                                   # 故意破坏 CRC
    buf = bytearray(bad) + make_frame(0x0F, '5')      # 坏帧后面跟一帧好的
    frames, dropped, crc_err = extract_frames(buf)
    checks.append(('坏 CRC 被报出且不吞掉后续帧',
                   [f[1] for f in frames] == ['5'] and crc_err == 1))

    whole = make_frame(0x0F, '6')
    buf = bytearray(whole[:4])                        # 只给半帧
    frames, dropped, crc_err = extract_frames(buf)
    checks.append(('半帧被留在缓冲区等待', not frames and not dropped))

    ok = True
    for name, passed in checks:
        print(('  ok   ' if passed else '  FAIL ') + name)
        ok = ok and passed
    print('自检结果: ' + ('全部通过' if ok else '有失败项'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', help='例如 COM3 (/dev/ttyUSB0)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--types', default='', help='只看这些类型, 逗号分隔, 例如 0x0F,0x10')
    ap.add_argument('--raw', action='store_true', help='打印原始 HEX')
    ap.add_argument('--log', default='', help='同时写入日志文件')
    ap.add_argument('--self-test', action='store_true', help='不接串口, 合成帧自检解析逻辑')
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if not args.port:
        ap.error('需要 --port (或用 --self-test)')

    try:
        import serial
    except ImportError:
        sys.exit('缺少 pyserial。装一下: '
                 r'C:\Users\xszx1\.dsh\tools\pyocd\Scripts\python.exe -m pip install pyserial')

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
        text = f'[{time.strftime("%H:%M:%S")}] {line}'
        print(text, flush=True)
        if logf:
            logf.write(text + '\n')
            logf.flush()

    emit(f'# 打开 {args.port} @ {args.baud} 8N1; 只看 '
         f'{sorted(hex(t) for t in only) if only else "全部类型"}')
    try:
        while True:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf.extend(chunk)
            frames, d, ce = extract_frames(buf)
            dropped += d
            crc_errors += ce
            if ce:
                emit(f'!! 本轮有 {ce} 帧 CRC 不匹配 (类型号/长度对但校验错, 通常是波特率或接线)')
            for type_byte, payload, raw in frames:
                counts[type_byte] = counts.get(type_byte, 0) + 1
                if only and type_byte not in only:
                    continue
                emit(describe(type_byte, payload, raw, args.raw))
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
