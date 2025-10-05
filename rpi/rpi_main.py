
# binibot RPi orchestrator: webcam → YOLOv8+ByteTrack → FSM → UART → ESP32
# NOTE: policy: largest bbox area (closest)

# cls==0 assumed to be 'person' from YOLO model

import time
import yaml
import struct
from typing import Optional, Dict, Any, List

from adapter import bbox_to_target
from yolo_bytrack import YoloByteTrack
from fsm import binibotFSM
from comm.serial_link import SerialLink
from comm.msg_types import CMD, STATUS, ACK_MASK, STATE_ID


def select_target(tracks: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not tracks:
        return None
    people = [t for t in tracks if t.get("cls") == 0]
    if not people:
        return None
    # choose the largest bbox area (closest)
    people.sort(
        key=lambda t: (t["xyxy"][3] - t["xyxy"][1]) * (t["xyxy"][2] - t["xyxy"][0]),
        reverse=True,
    )
    return people[0]

def main():
    cfg = yaml.safe_load(open("config.yaml"))

    vcfg = cfg.get("vision", {})
    tracker = YoloByteTrack(
        model_path=vcfg.get("model_path", "yolov8n.pt"),
        imgsz=vcfg.get("imgsz", 416),
        conf=vcfg.get("conf", 0.6),
        camera_index=vcfg.get("camera_index", 0),
        tracker_cfg=vcfg.get("tracker_cfg", "bytetrack.yaml"),
    )

    fcfg = cfg.get("fsm", {})
    fsm = binibotFSM(
        conf_min=fcfg.get("conf_min", 0.35),
        lost_timeout=fcfg.get("lost_timeout", 1.5),
        avoid_dist=fcfg.get("avoid_dist", 0.40),
        avoid_clear=fcfg.get("avoid_clear", 0.8),
        follow_stop_dist_m=fcfg.get("follow_stop_dist_m", 1.0),
        wait_duration_s=fcfg.get("still_bored_after_s", 5.0),
        bearing_deadband=fcfg.get("bearing_deadband", 0.08),
        kp_ang=fcfg.get("kp_ang", 1.8),
        kp_lin=fcfg.get("kp_lin", 0.35),
        max_speed=fcfg.get("max_speed", 0.35),
        frame_w=vcfg.get("width", 640),
        frame_h=vcfg.get("height", 480),
    )

    ccfg = cfg.get("comms", {})
    link = SerialLink(
        port=ccfg.get("uart_port", "/dev/serial0"),
        baud=ccfg.get("baud", 115200),
    )
    send_hz = ccfg.get("send_hz", 20)
    send_dt = 1.0 / max(1, send_hz)
    next_send = 0.0

    try:
        for tracks in tracker.stream(src=vcfg.get("camera_index", 0)):
            raw = select_target(tracks)
            target = bbox_to_target(
                raw,
                vcfg.get("width", 640),
                vcfg.get("height", 480),
            ) if raw else None

            obstacle_range = None  # meters
            evt = link.recv()
            if evt:
                typ, payload = evt
                if typ == STATUS and len(payload) >= 2:
                    (ultra_cm,) = struct.unpack("<H", payload[:2])
                    if ultra_cm > 0:
                        obstacle_range = ultra_cm / 100.0

            fsm.update(target, obstacle_range)

            v, w = fsm.controller(
                bearing=target["bearing"] if target else None,
                dist_hint=target["dist_m"] if target else None,
            )

            now = time.time()
            if now >= next_send:
                state_id = STATE_ID.get(fsm.state, STATE_ID["WAIT"])
                payload = struct.pack("<Bff", state_id, float(v), float(w))
                link.send(CMD, payload)

                # optional, non-blocking peek for ACK:
                # ack = link.recv()
                # if ack and ack[0] == (CMD | ACK_MASK): pass

                next_send = now + send_dt

    except KeyboardInterrupt:
        pass
    finally:
        try:
            payload = struct.pack("<Bff", STATE_ID["WAIT"], 0.0, 0.0)
            link.send(CMD, payload)
        except Exception:
            pass

if __name__ == "__main__":
    main()
