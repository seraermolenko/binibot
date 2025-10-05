# binibot RPi orchestrator: webcam → YOLOv8 + ByteTrack → FSM → UART → ESP32
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

    # Normalize camera source
    cam = vcfg.get("camera_index", 0)
    if isinstance(cam, str) and cam.startswith("/dev/video"):
        source = cam
    else:
        try:
            source = int(cam)
        except (TypeError, ValueError):
            source = 0

    # Initialize modules
    tracker = YoloByteTrack(
        model_path=vcfg.get("model_path", "yolov8n.pt"),
        imgsz=vcfg.get("imgsz", 416),
        conf=vcfg.get("conf", 0.6),
        camera_index=source,
        tracker_cfg=vcfg.get("tracker_cfg", "bytetrack.yaml"),
    )

    fsm = binibotFSM(**cfg.get("fsm", {}))
    link = SerialLink(**cfg.get("comms", {}))

    send_dt = 1.0 / cfg["comms"].get("send_hz", 10)
    next_send = 0.0

    while True:
        # --- YOLO + ByteTrack step ---
        raw = None
        for tracks in tracker.stream(src=source):
            raw = select_target(tracks)
            break

        target = bbox_to_target(
            raw,
            vcfg.get("width", 640),
            vcfg.get("height", 480),
        ) if raw else None

        # --- Ultrasonic sensor read from ESP32 ---
        obstacle_range = None
        evt = link.recv()
        if evt:
            typ, payload = evt
            if typ == STATUS and len(payload) >= 2:
                (ultra_cm,) = struct.unpack("<H", payload[:2])
                if ultra_cm > 0:
                    obstacle_range = ultra_cm / 100.0

        # --- Update FSM and motor commands ---
        fsm.update(target, obstacle_range)
        v, w = fsm.controller(
            bearing=target["bearing"] if target else None,
            dist_hint=target["dist_m"] if target else None,
        )

        # --- Send periodic velocity command to ESP32 ---
        now = time.time()
        if now >= next_send:
            state_id = STATE_ID.get(fsm.state, STATE_ID["WAIT"])
            payload = struct.pack("<Bff", state_id, float(v), float(w))
            link.send(CMD, payload)
            next_send = now + send_dt

        # non-blocking peek for ACK (optional)
        # ack = link.recv()
        # if ack and ack[0] == (CMD | ACK_MASK):
        #     pass

    # graceful stop
    try:
        payload = struct.pack("<Bff", STATE_ID["WAIT"], 0.0, 0.0)
        link.send(CMD, payload)
    except Exception:
        pass


if __name__ == "__main__":
    main()
