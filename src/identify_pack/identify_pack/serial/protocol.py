"""串口帧协议：上位机(ROS) <-> 下位机(STM32F4)。

帧格式：
    +------+------+------+------------------+--------+------+
    | 0xFF | TYPE | LEN  |   PAYLOAD(UTF-8) |  CRC8  | 0xFE |
    +------+------+------+------------------+--------+------+

- TYPE  : 消息类型（见 TOPIC_TYPES）
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
}

TYPE_NAMES = {v: k for k, v in TOPIC_TYPES.items()}


def make_frame(type_byte: int, text: str) -> bytes:
    """把一段文本打包成一帧字节。"""
    payload = text.encode('utf-8')
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload 过长: {len(payload)} > {MAX_PAYLOAD}')
    body = bytes([type_byte, len(payload)]) + payload
    crc = 0
    for b in body:
        crc ^= b
    return bytes([HEAD]) + body + bytes([crc, TAIL])


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
