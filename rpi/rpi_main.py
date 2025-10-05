# binibot RPi orchestrator: webcam → YOLOv8+ByteTrack → FSM → UDP → ESP32

import time
import yaml
from typing import Optional, Dict, Any, List

from yolo_bytrack import YoloByteTrack
from rpi.fsm import binibotFSM
from rpi.udp_link import UdpLink

def select_target(tracks: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Pick a person to follow. Policy: largest bbox area (closest)."""
    if not tracks:
        return None
    people = [t for t in tracks if t.get("cls") == 0]
    if not people:
        return None
    people.sort(key=lambda t: (t["xyxy"][3]-t["xyxy"][1]) * (t["xyxy"][2]-t["xyxy"][0]), reverse=True)
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
    wait_duration = fcfg.get("still_bored_after_s", 5.0)
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

    # --- Comms (RPi ↔ ESP32) ---
    ccfg = cfg.get("comms", {})
    link = UdpLink(
        esp_ip=ccfg.get("esp_ip", "192.168.1.80"),
        tx_port=ccfg.get("tx_port", 5005),   # Pi → ESP
        rx_port=ccfg.get("rx_port", 5006),   # ESP → Pi
    )
    send_hz = ccfg.get("send_hz", 20)
    send_dt = 1.0 / max(1, send_hz)
    next_send = 0.0

    try:
        for tracks in tracker.stream(src=vcfg.get("camera_index", 0)):
            target = select_target(tracks)

            resp = link.recv_telemetry()
            us_ranges_m = None

            if resp and "status" in resp:
                ultra_raw = resp["status"].get("ultra_cm")
                if ultra_raw:
                    us_ranges_m = [x / 100.0 for x in ultra_raw]

                # NOTE: 4 SENSORS POST TESTING
                # if ultra_raw and len(ultra_raw) == 4:
                #     us_ranges_m = [x / 100.0 for x in ultra_raw]
                # else:
                #     us_ranges_m = (None, None, None, None)

            fsm.update(target, us_ranges_m)
            
            v, w = fsm.controller(
                bearing=target["bearing"] if target else None,
                dist_hint=target["dist_m"] if target else None
            )


            now = time.time()
            if now >= next_send:
                link.send_cmd(fsm.state, v, w)
                next_send = now + send_dt

            # print(f"{state} v={v:.2f} w={w:.2f} obs={obstacle_range}") # debug

    except KeyboardInterrupt:
        pass
    finally:
        link.send_cmd("WAIT", 0.0, 0.0)

if __name__ == "__main__":
    main()
