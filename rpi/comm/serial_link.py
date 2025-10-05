# rpi/comm/serial_link.py
import serial
from typing import Optional, Tuple
from .serial_proto import encode_frame, FrameDecoder

class SerialLink:
    def __init__(self, port: str = "/dev/serial0", baud: int = 115200, timeout: float = 0.01):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.dec = FrameDecoder()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, msg_type: int, payload: bytes = b"") -> None:
        self.ser.write(encode_frame(msg_type, payload))

    def recv(self) -> Optional[Tuple[int, bytes]]:
        chunk = self.ser.read(256)  
        if chunk:
            self.dec.push(chunk)
            return self.dec.pop_frame()
        return None
