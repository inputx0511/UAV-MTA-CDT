#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyTHS1"
BAUDRATE = 38400
TIMEOUT = 0.2
CMD = bytes([0xA5, 0x5A, 0x03, 0x00, 0xFC])
ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
print(f"[OK] Opened {PORT} @ {BAUDRATE}")
ser.write(CMD)
ser.flush()

def read_distance():

    if not hasattr(read_distance, "buf"):
        read_distance.buf = bytearray()
        read_distance.distance = 0.0

    global ser
    # đọc tất cả byte đang có
    n = ser.in_waiting
    if n <= 0:
        return read_distance.distance

    read_distance.buf += ser.read(n)
    buf = read_distance.buf

    while len(buf) >= 8:
        # tìm header
        if buf[0] != 0xB4:
            buf.pop(0)
            continue

        frame = buf[:8]

        # check byte thứ 3
        if frame[2] != 0x03:
            buf.pop(0)
            continue

        # checksum XOR
        cs = 0xB4
        for i in range(1, 7):
            cs ^= frame[i]

        if cs != frame[7]:
            buf.pop(0)
            continue

        # OK → parse distance
        read_distance.distance = 1.0*((frame[5] << 8) | frame[6])/1000
        print(f"{time.time()}, {read_distance.distance}")

        # bỏ frame đã xử lý
        del buf[:8]
        return read_distance.distance
    
    return read_distance.distance
