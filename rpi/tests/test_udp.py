import time
from udp_link import UdpLink

def test_udp_ack():
    link = UdpLink(esp_ip="127.0.0.1", tx_port=5005, rx_port=5006)
    ok, resp = link.send_cmd_with_ack("follow", 0.2, -0.1)
    assert ok is True
    assert resp["status"]["ultra_cm"] == 42
    # also test fire-and-forget path
    link.send_cmd("explore", 0.0, 0.5)
    time.sleep(0.05)
    # optional: poll telemetry (fake_esp doesn’t push unsolicited, so likely None)
    tel = link.recv_telemetry()
    assert tel is None or "status" in tel
