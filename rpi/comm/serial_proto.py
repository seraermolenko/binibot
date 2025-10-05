# rpi/comm/serial_proto.py
from typing import Optional, Tuple

START = 0xAA  

def crc8(data: bytes, poly: int = 0x31, init: int = 0x00) -> int:
    c = init
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
    return c

def encode_frame(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_type]) + payload
    length = len(body) + 1  # include CRC8
    c = crc8(body)
    return bytes([START, length]) + body + bytes([c])

class FrameDecoder:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.expected: Optional[int] = None

    def reset(self):
        self.buf.clear()
        self.expected = None

    def push(self, chunk: bytes):
        for b in chunk:
            if not self.buf:
                if b == START:
                    self.buf.append(b)
            elif len(self.buf) == 1:
                self.buf.append(b)                # LEN
                self.expected = 2 + b             # total bytes: AA + LEN + body+crc
                if self.expected > 260:           # sanity cap
                    self.reset()
            else:
                self.buf.append(b)

    def pop_frame(self) -> Optional[Tuple[int, bytes]]:
        if not self.expected or len(self.buf) < self.expected:
            return None
        frame = bytes(self.buf)
        self.reset()

        length = frame[1]
        body = frame[2:2+length]                  # TYPE..CRC
        if len(body) != length:
            return None
        typ = body[0]
        payload = body[1:-1] if length >= 2 else b""
        c = body[-1]
        if crc8(body[:-1]) != c:
            return None
        return (typ, payload)
