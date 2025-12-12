#!/usr/bin/env python3
from pymavlink import mavutil
import time
import detect
from state import state
from flight_controller import FlightController

def connect(port="/dev/ttyUSB0", baud=57600):
    filename = time.strftime("logs/%Y_%m_%d_%H_%M.csv")
    f = open(filename, "w")
    f.write(f"Connecting to {port} @ {baud}...")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    f.write(f"HEARTBEAT received: {m.target_system}, {m.target_component}")
    return m, f

def wait_to_takeoff():
    st.start_stream()
    while True:
        snap = st.poll()
        if snap is not None:
            t_ms, ch6, alt, roll, pitch, yaw = snap
            if ch6 == 2000:
                break
        time.sleep(0.05)

def takeoff():
    fc.arm()
    time.sleep(1.5)
    fc.takeoff(4) #cất cánh lên 4m

if __name__ =="__main__":
    try:
        m, f = connect()
        st = state(m, f)
        fc = FlightController(m, f)
        fc.set_mode_guided()
        wait_to_takeoff()
        takeoff()
        reached_alt = False
        t0 = time.time()
        while time.time() - t0 < 30:
            detect.detect()
            snap = st.poll()
            if snap is not None:
                t_ms, ch6, alt, roll, pitch, yaw = snap
                if not reached_alt and alt >= 4:
                    reached_alt = True

            if reached_alt:
                fc.send_body_velocity(0,0,0)

        fc.land()
        
    except Exception as e:
        f.write(f"Lỗi xảy ra: {e}")
    finally:
        fc.land()
        st.stop_stream
        detect.close()
        m.close()