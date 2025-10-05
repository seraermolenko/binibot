#  send/recv sockets

# NOTE: ESP -> PI is 5005 -> tx -> transmit -> JSON ACK + telemetry over UDP
# NOTE: PI -> ESP is 5006 -> rx -> recive ->  small JSON control messages over UDP
# NOTE: esp is listening on tx_port. Our rpi will send will send a UDP packet to (esp_ip, tx_port) to tell esp commands
# NOTE: 1500 bytes for Ethernet MTU

# The rpi will be listenitng to the esp for acknolwedgments, if we have time. I think it will be good for us to uild a lightweight reliability layer
# because UDP doesnt send any ACk/no guarntee/most unreliable, it would be good to re-send the msg if the motor didnt get the command

# NOTE: v is linear velocity and w is angular velocity

RETRY_MS  = 80     # per-try wait for ACK
MAX_RETRY = 3     
MTU_BYTES = 1500   # recv buffer

import socket, json, time

class UdpLink: 
    """
    Pi <-> ESP UDP link
    """
    def __init__(self, esp_ip, tx_port=5005, rx_port=5006):
        self.tx_addr = (esp_ip, tx_port) # storign where to send the addresess

        # UDP sockets 
        # NOTE: datagram -> not connection-based
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx.bind(("0.0.0.0", rx_port))
        self.rx.setblocking(False)

        self.seq = 0

    def _now_ms(self):
        return int(time.time() * 1000)
    
    def _build_cmd(self, state, v, w):
        self.seq += 1
        return {
            "t_ms": self._now_ms(),
            "seq": self.seq,
            "state": state,
            "cmd": {"v": float(v), "w": float(w)}
        }

    def send_cmd(self, state, v, w):
        """
        No ack, in case we want to test with this instead
        """
        msg = self._build_cmd(state, v, w)
        self.tx.sendto(json.dumps(msg).encode("utf-8"), self.tx_addr)

    def send_cmd_with_ack(self, state, v, w):
        """
        Send a with ACK from ESP.
        Returns (ok, resp) where:
          - ok   = True if ACK for this seq was received
          - resp = telemetry dict from ESP (or None if no ACK)
        """
        msg = self._build_cmd(state, v, w)
        payload = json.dumps(msg).encode("utf-8")

        tries = 0
        while tries < MAX_RETRY:

            self.tx.sendto(payload, self.tx_addr)

            # polling rx
            t0 = time.time()
            while (time.time() - t0) * 1000 < RETRY_MS:
                try:
                    data, _ = self.rx.recvfrom(MTU_BYTES)
                    resp = json.loads(data.decode("utf-8"))
                except BlockingIOError:
                    time.sleep(0.002)  # brief yield
                    continue
                except json.JSONDecodeError:
                    continue

                # {"ack_seq": <int>, "t_ms_echo": <int>, "status": {...}}
                if resp.get("ack_seq") == msg["seq"]:
                    return True, resp

            tries += 1

        return False, None

    def recv_telemetry(self):
        """
        Non-blocking: grab latest telemetry/unsolicited messages from ESP if any.
        Returns dict or None.
        """
        try:
            data, _ = self.rx.recvfrom(MTU_BYTES)
            return json.loads(data.decode("utf-8"))
        except BlockingIOError:
            return None
        except json.JSONDecodeError:
            return None