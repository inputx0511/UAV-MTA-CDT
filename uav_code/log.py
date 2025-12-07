#!/usr/bin/env python3
import time
from pymavlink import mavutil

#RC_CHANNELS : 65
#GLOBAL_POSITION_INT: 33
def request_message(m, msg_id, hz = 5.0):
    interval_us = int(1_000_000 / hz)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0,0,0,0,0
    )

def stream_data(m,f):
    msg = m.recv_match(type=["RC_CHANNELS", "GLOBAL_POSITION_INT", "STATUSTEXT"], blocking=False)
    if msg is None:
        return None,None
    msg_type = msg.get_type()
    ch6 = None      
    alt_m = None    
    if msg_type == "STATUSTEXT":
        f.write(f"[FCU] {msg.text}\n")
    if msg_type == "RC_CHANNELS":
        ch6 = getattr(msg, "chan6_raw", 0)
    if msg_type == "GLOBAL_POSITION_INT":
        alt_m = round(msg.relative_alt / 1000.0,3)
    f.write(f"{ch6:4d},{alt_m}\n")
    return ch6,alt_m

def stop_stream(m,f):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        0, 0, 0, 0, 0, 0
    )
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        0, 0, 0, 0, 0, 0
    )
    f.close()    

def log_file():
    filename = time.strftime("logs/%Y_%m_%d_%H_%M.csv")
    f = open(filename, "w")
    return f

def main():
    PORT = "/dev/ttyUSB0"
    BAUD = 57600
    f = log_file()
    f.write(f"Connecting to {PORT} @ {BAUD}...\n")
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    m.wait_heartbeat()
    f.write(f"HEARTBEAT: {m.target_system}, {m.target_component}")

    request_message(m, 65, hz=5.0)
    request_message(m, 33, hz=5.0)

    try:
        while True:
            stream_data(m,f)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_stream(m,f)
        m.close()

if __name__ == "__main__":
    main()