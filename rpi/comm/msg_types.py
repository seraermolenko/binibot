AI_RESULT = 0x01
PING      = 0x02
ACK_MASK  = 0x80
CMD      = 0x10   # Pi → ESP: command payload <Bff> = (state_id, v, w)
STATUS   = 0x20   # ESP → Pi: status payload <H>   = (ultra_cm)

STATE_ID = {
    "WAIT":    0,
    "FOLLOW":  1,
    "EXPLORE": 2,
    "AVOID":   3,
}