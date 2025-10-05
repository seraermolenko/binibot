import time
from fsm import binibotFSM  

def mk_target(conf=0.9, bearing=0.0, dist=2.0):
    return {"conf": conf, "bearing": bearing, "dist_m": dist}

def test_explore_to_follow():
    fsm = binibotFSM()
    assert fsm.state == "EXPLORE"
    fsm.update(target=mk_target(bearing=0.1, dist=2.0), obstacle_range_m=None)
    v, w = fsm.controller(bearing=0.1, dist_hint=2.0)
    assert fsm.state in ("FOLLOW", "EXPLORE") 
    assert v >= 0.0

def test_follow_to_wait_and_back():
    fsm = binibotFSM(follow_stop_dist_m=1.0, wait_duration_s=0.2, bearing_deadband=0.1)
    fsm.update(target=mk_target(bearing=0.0, dist=2.0), obstacle_range_m=None)
    fsm.update(target=mk_target(bearing=0.05, dist=0.9), obstacle_range_m=None)
    assert fsm.state in ("FOLLOW", "WAIT")
    t0 = time.time()
    while time.time() - t0 < 0.3:
        fsm.update(target=None, obstacle_range_m=10.0)
        time.sleep(0.05)
    assert fsm.state == "EXPLORE"

def test_avoid_ignored_in_wait():
    fsm = binibotFSM(wait_duration_s=0.2)
    fsm.state = "WAIT"
    fsm._wait_started = time.time()
    fsm.update(target=None, obstacle_range_m=0.1)
    assert fsm.state == "WAIT"
