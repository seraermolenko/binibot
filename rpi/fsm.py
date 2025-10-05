# Finite State Machine
# Minimal behavior FSM for binibot with stop-at-distance + still-boredom
# binibot behavior FSM:
# EXPLORE (wander until target) → FOLLOW (pursue target) → WAIT (at stop distance, 5s) ; AVOID only when not in WAIT
import time, random
from typing import Optional, Dict, Any, Tuple

class binibotFSM:
    def __init__(
        self,
        conf_min: float = 0.35,
        lost_timeout: float = 1.5,
        avoid_dist: float = 0.40,
        avoid_clear: float = 0.8,
        follow_stop_dist_m: float = 1.0,   # via bbox proxy
        wait_duration_s: float = 5.0,      
        bearing_deadband: float = 0.08,    # how centered before considering "stopped"
        kp_ang: float = 1.8,
        kp_lin: float = 0.35,
        max_speed: float = 0.35,
        frame_w: int = 640,
        frame_h: int = 480,
    ):
        self.CONF_MIN     = conf_min
        self.LOST_TIMEOUT = lost_timeout
        self.AVOID_DIST   = avoid_dist
        self.AVOID_CLEAR  = avoid_clear
        self.STOP_DIST    = follow_stop_dist_m
        self.WAIT_DUR     = wait_duration_s
        self.DEADBAND     = bearing_deadband
        self.Kp_ang       = kp_ang
        self.Kp_lin       = kp_lin
        self.V_MAX        = max_speed
        self.W            = frame_w
        self.H            = frame_h

        self.state = "EXPLORE"
        self.enter_t = time.time()
        self.prev_state = "EXPLORE"

        self.last_seen_t = 0.0
        self._wait_started: Optional[float] = None

    def _change(self, new_state: str):
        if new_state != self.state:
            if new_state == "AVOID":
                self.prev_state = self.state
            self.state = new_state
            self.enter_t = time.time()
            if new_state != "WAIT":
                self._wait_started = None

    def _bearing_and_dist(self, target: Dict[str, Any]) -> Tuple[Optional[float], Optional[float], float]:
        if not target:
            return None, None, 0.0
        (x1,y1,x2,y2) = target["xyxy"]
        cx = (x1 + x2) / 2.0
        bearing = (cx - self.W/2) / (self.W/2) 
        box_h = max(1.0, (y2 - y1))
        # crode proxy/taller box -> closer
        dist_hint = max(0.5, min(3.0, 1.6 * (self.H / box_h)))
        conf = float(target.get("conf", 0.0))
        return bearing, dist_hint, conf

    def update(self, target: Optional[Dict[str, Any]], obstacle_range_m: Optional[float]):
        now = time.time()

        obstacle = (self.state != "WAIT") and (obstacle_range_m is not None) and (obstacle_range_m < self.AVOID_DIST)

        bearing, dist_hint, conf = self._bearing_and_dist(target) if target else (None, None, 0.0)
        if target and conf >= self.CONF_MIN:
            self.last_seen_t = now
        seen_recently = (now - self.last_seen_t) < self.LOST_TIMEOUT
        high_conf = conf >= self.CONF_MIN

        if obstacle:
            self._change("AVOID")
        else:
            if self.state == "AVOID":
                if (now - self.enter_t) > self.AVOID_CLEAR:
                    self._change(self.prev_state if seen_recently else "EXPLORE")

            elif self.state == "EXPLORE":
                if seen_recently and high_conf:
                    self._change("FOLLOW")

            elif self.state == "FOLLOW":
                if not seen_recently:                       
                    self._change("EXPLORE")
                else:
                    if (dist_hint is not None) and (bearing is not None):
                        if (dist_hint <= self.STOP_DIST) and (abs(bearing) <= self.DEADBAND):
                            self._change("WAIT")
                            self._wait_started = now

            elif self.state == "WAIT":
                if self._wait_started is None:
                    self._wait_started = now
                if (now - self._wait_started) >= self.WAIT_DUR:
                    self._change("EXPLORE")


        if self.state == "EXPLORE":
            v = 0.15
            w = random.uniform(-0.6, 0.6)

        elif self.state == "FOLLOW":
            if (bearing is None) or (dist_hint is None):
                v, w = 0.0, 0.0
            else:
                w = self.Kp_ang * (-bearing)
                v_base = self.Kp_lin * max(0.0, dist_hint - self.STOP_DIST)
                v = max(0.0, min(self.V_MAX, v_base - abs(w)*0.05))

        elif self.state == "WAIT":
            v, w = 0.0, 0.0

        elif self.state == "AVOID":
            v = -0.15
            w = 0.8 if random.random() < 0.5 else -0.8

        else:
            v, w = 0.0, 0.0

        return self.state, float(v), float(w)
