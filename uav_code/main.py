#!/usr/bin/env python3
from pymavlink import mavutil
import time
import socket
import json
from state import state
from flight_controller import FlightController

# ================= Socket config =================
UDP_IP = "127.0.0.1"
MAIN_PORT = 9000
DETECT_PORT = 10000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
detect_addr = (UDP_IP, DETECT_PORT)
sock.bind(("0.0.0.0", MAIN_PORT))
sock.setblocking(False)

def send_mode(mode: str, repeat=5, interval_s=0.1):
    pkt = {
        "type": "mode",
        "mode": mode
    }
    data = json.dumps(pkt).encode("utf-8")
    for _ in range(repeat):
        sock.sendto(data, detect_addr)
        time.sleep(interval_s)

def send_altitude(altitude: float, period_s=0.1):
    if not hasattr(send_altitude, "last_t"):
        send_altitude.last_t = 0.0

    t_now = time.time()
    if t_now - send_altitude.last_t < period_s:
        return

    send_altitude.last_t = t_now
    pkt = {
        "type": "altitude",
        "altitude": float(altitude)
    }
    sock.sendto(json.dumps(pkt).encode("utf-8"), detect_addr)


def recv_udp():
    if not hasattr(recv_udp, "last_msg"):
        recv_udp.last_msg = None
        recv_udp.last_t = 0.0
        recv_udp.dt = 0.0

    try:
        data, _ = sock.recvfrom(256)
    except (BlockingIOError, OSError):
        return recv_udp.last_msg, recv_udp.dt, False

    try:
        recv_udp.last_msg = json.loads(data.decode("utf-8"))
    except Exception:
        return recv_udp.last_msg, recv_udp.dt, False

    t_now = time.time()
    recv_udp.dt = t_now - recv_udp.last_t
    recv_udp.last_t = t_now
    return recv_udp.last_msg, recv_udp.dt, True #Có dữ liệu mới cập nhật


def connect(port="/dev/ttyUSB0", baud=57600):
    filename = time.strftime("logs/%Y_%m_%d_%H_%M.csv")
    f = open(filename, "w")
    f.write(f"Connecting to {port} @ {baud}...\n")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    f.write(f"HEARTBEAT received: {m.target_system}, {m.target_component}\n")
    return m, f
def hover():
    t0 = time.time()
    while time.time() - t0 < 3:
        fc.send_body_velocity(0,0,0)
        time.sleep(0.02)

def wait_to_takeoff():
    st.start_stream()
    is_start = True
    while True:
        snap = st.poll()
        pkt, _, _ = recv_udp()
        if pkt is not None and pkt.get("seq", 0) == -1 and is_start:
            fc.set_mode_guided()
            is_start = False
        if snap is not None:
            t_ms, ch6, alt, roll, pitch, yaw = snap
            if ch6 == 2000:
                send_mode("Take off")
                break
        time.sleep(0.05)

def takeoff(alt):
    fc.arm()
    time.sleep(1.5)
    fc.takeoff(alt)
    while True:
        snap = st.poll()
        if snap is not None:
            t_ms, ch6, dz, roll, pitch, yaw = snap
            send_altitude(dz)
            if dz >= 0.95*alt:
                send_mode("Following")
                time.sleep(1)
                break
        # fc.heading_hold()
        time.sleep(0.05)
    hover()

def landing():
    send_mode("Landing")
    fc.land()
    t_land_start = time.time()
    LAND_TIMEOUT = 15.0

    while True:
        snap = st.poll()
        if snap is not None:
            t_ms, ch6, dz, roll, pitch, yaw = snap
            send_altitude(dz)
            if dz <= 0.5:
                break

        if time.time() - t_land_start > LAND_TIMEOUT:
            break

        time.sleep(0.05)

if __name__ == "__main__":
    try:
        m, f = connect()
        st = state(m, f)
        fc = FlightController(m, f)
        wait_to_takeoff()
        takeoff(6) #cất cánh lên 6m
        vx = vy = vz = 0.0
        t0 = time.time()
        while time.time() - t0 < 60: #Thời gian detect
            pkt, dt, d_flag = recv_udp()
            snap = st.poll()
            if snap is not None:
                t_ms, ch6, dz, roll, pitch, yaw = snap
                send_altitude(dz)
            
            if d_flag:
                vx, vy, vz = fc.pid_commute(pkt["dx"], pkt["dy"], pkt["dxp"], pkt["dyp"], dz, dt, pkt["found"])
            if dz > 1:
                fc.send_body_velocity(vx,vy,vz)
            else:
                fc.send_body_velocity(vx,vy,0)
            if pkt["dx"] < 0.5 and pkt["dy"] < 0.5 and dz < 1:
                break
        # t0 = time.time()
        # while time.time() - t0 < 3:
        #     fc.send_body_velocity(0,0,0)
        #     time.sleep(0.02)
        
    except Exception as e:
        f.write(f"Lỗi xảy ra: {e}")
    finally:
        landing()

        st.stop_stream()
        m.close()
        send_mode("Stop")