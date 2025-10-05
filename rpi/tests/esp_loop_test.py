# dummy esp 

import json, socket

ESP_LISTEN = ("127.0.0.1", 5005) 
PI_ADDR    = ("127.0.0.1", 5006) 

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(ESP_LISTEN)
    while True:
        data, addr = sock.recvfrom(1500)
        msg = json.loads(data.decode("utf-8"))
        resp = {
            "ack_seq": msg.get("seq"),
            "t_ms_echo": msg.get("t_ms"),
            "status": {"battery_v": 8.0, "ultra_cm": 42, "wheel_l": 0.3, "wheel_r": 0.3}
        }
        out = json.dumps(resp).encode("utf-8")
        sock.sendto(out, PI_ADDR)

if __name__ == "__main__":
    main()
