import sys, os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from comm.serial_link import SerialLink
from comm.msg_types import CMD, STATUS, ACK_MASK, STATE_ID
import struct, time

## local feedback loop

def test_loopback():
    import io
    from comm.serial_proto import encode_frame, FrameDecoder
    frame = encode_frame(0x10, b"hi")
    fd = FrameDecoder()
    fd.push(frame)
    parsed = fd.pop_frame()
    assert parsed == (0x10, b"hi")
    print("encoder/decoder OK")