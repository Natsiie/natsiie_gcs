"""串口帧协议：上位机(ROS) <-> 下位机(STM32F4)。

帧格式：
    +------+------+------+------------------+--------+------+
    | 0xFF | TYPE | LEN  |   PAYLOAD(UTF-8) |  CRC8  | 0xFE |
    +------+------+------+------------------+--------+------+

- TYPE  : 消息类型（见 TOPIC_TYPES）；0x07 为反馈角度，0x08 为颜色目标
- LEN   : payload 字节数（最大 255）
- CRC8  : 对 TYPE+LEN+PAYLOAD 逐字节异或
- 帧头/帧尾固定 0xFF / 0xFE

类型号必须与 STM32F4 侧 serial_protocol.h 保持一致。
"""
HEAD = 0xFF
TAIL = 0xFE
MAX_PAYLOAD = 255

# 话题名 -> 类型号（新增话题在此扩展，并同步修改 STM32 侧枚举）
TOPIC_TYPES = {
    'Color':        0x01,
    'ColorMotion':  0x02,
    'Turntable':    0x03,
    'ring_station': 0x04,
    'stacked_ring': 0x05,
    'QR_Decode':    0x06,
    'GimbalYaw':    0x07,  # 下位机 -> 上位机，ASCII 角度，单位 deg
    'ColorTarget':  0x08,  # 上位机 -> 下位机，color_id,x_mm,y_mm,z_mm，底座绝对坐标
}

# 旧 String 发送话题；ColorTarget 由带时间戳的米制 PointCloud 单独转换发送。
OUTGOING_TOPIC_TYPES = {
    topic: type_byte for topic, type_byte in TOPIC_TYPES.items()
    if topic not in ('GimbalYaw', 'ColorTarget')
}
GIMBAL_YAW_TYPE = TOPIC_TYPES['GimbalYaw']
COLOR_TARGET_TYPE = TOPIC_TYPES['ColorTarget']

TYPE_NAMES = {v: k for k, v in TOPIC_TYPES.items()}


def make_frame(type_byte: int, text: str) -> bytes:
    """把一段文本打包成一帧字节。"""
    if not isinstance(type_byte, int) or not 0 <= type_byte <= 0xFF:
        raise ValueError(f"type_byte 必须是 0..255 的整数: {type_byte!r}")
    payload = text.encode('utf-8')
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload 过长: {len(payload)} > {MAX_PAYLOAD}')
    body = bytes([type_byte, len(payload)]) + payload
    crc = 0
    for b in body:
        crc ^= b
    return bytes([HEAD]) + body + bytes([crc, TAIL])


def extract_frames(buffer: bytes):
    """从串口字节流中提取完整合法帧，返回 ``(frames, remainder)``。

    不完整帧会保留到下一次调用；坏帧会丢弃一个字节后重新寻找帧头，
    这样校验失败不会卡住后续合法帧。
    """
    pending = bytearray(buffer)
    frames = []
    while pending:
        start = pending.find(bytes([HEAD]))
        if start < 0:
            pending.clear()
            break
        if start:
            del pending[:start]
        if len(pending) < 3:
            break
        length = pending[2]
        frame_size = 5 + length
        if len(pending) < frame_size:
            break
        parsed = parse_frame(bytes(pending[:frame_size]))
        if parsed is None:
            del pending[0]
            continue
        type_byte, payload, _ = parsed
        frames.append((type_byte, payload))
        del pending[:frame_size]
    return frames, bytes(pending)


def parse_frame(buf: bytes):
    """从字节流里解析一帧，成功返回 (type_byte, payload_text, 剩余字节)，
    找不到完整/合法帧返回 None。主要用于自测和调试。"""
    i = buf.find(HEAD)
    if i < 0:
        return None
    buf = buf[i:]
    if len(buf) < 3:
        return None
    length = buf[2]
    need = 5 + length  # HEAD+TYPE+LEN+PAYLOAD+CRC+TAIL
    if len(buf) < need:
        return None
    if buf[need - 1] != TAIL:
        return None
    body = buf[1:3 + length]
    crc = 0
    for b in body:
        crc ^= b
    if crc != buf[need - 2]:
        return None
    type_byte = buf[1]
    payload = buf[3:3 + length].decode('utf-8', errors='replace')
    return type_byte, payload, buf[need:]
