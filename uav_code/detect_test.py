import cv2
from ultralytics import YOLO
import time
import socket
import json
import numpy as np

# ================= Model / Camera =================
model = YOLO("models/yolov8_n.pt")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

video_name = time.strftime("videos/%Y_%m_%d_%H_%M.mp4")
out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*"mp4v"), 16.0, (480, 640))
dummy = np.zeros((480, 640, 3), dtype=np.uint8)
_ = model.predict(dummy,conf=0.4,imgsz=320,device="cuda",half=True,verbose=False)

def detect():
    if not hasattr(detect, "frame_id"):
        detect.frame_id = 0
        detect.prev_t = 0.0
        detect.fps_now = 0
        detect.seq = 0

    dxc = dyc = None
    best_conf = None
    obj_cx = obj_cy = None
    bbox = None

    ret, frame = cap.read()
    if not ret:
        pkt = {
            "seq": detect.seq,
            "found": False,
            "dx": 0.0,
            "dy": 0.0,
            "conf": 0.0,
        }
        return pkt
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    H, W = 640, 480
    cam_cx, cam_cy = W / 2.0, H / 2.0
    r = model.predict(frame, conf=0.4, imgsz=320, device="cuda", half=True, verbose=False)[0]
    b = r.boxes

    found = False
    if b is not None and len(b) > 0:
        found = True
        best_conf = float(b.conf[0].item())
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        bbox = (x1, y1, x2, y2)

        obj_cx = (x1 + x2) / 2.0
        obj_cy = (y1 + y2) / 2.0

        dyc = obj_cx - cam_cx
        dxc = cam_cy - obj_cy
    else:
        best_conf = 0.0
        dxc = 0.0
        dyc = 0.0

    # Tính fps
    detect.frame_id += 1
    if detect.frame_id == 1:
        detect.prev_t = time.time()
    elif detect.frame_id == 10:
        t_now = time.time()
        detect.fps_now = int(9 / (t_now - detect.prev_t))
        detect.frame_id = 0

    # Vẽ chú thích
    annotated = frame
    cv2.putText(annotated, f"FPS:{detect.fps_now}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.drawMarker(annotated, (int(cam_cx), int(cam_cy)), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (200, 0, 0), 2)
        cv2.circle(annotated, (int(obj_cx), int(obj_cy)), 4, (0, 0, 255), -1)
        cv2.line(annotated, (int(cam_cx), int(cam_cy)), (int(obj_cx), int(obj_cy)), (0, 255, 255), 2)

        cv2.putText(annotated, f"dx={dxc:.1f}px dy={dyc:.1f}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(annotated, f"conf={best_conf:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    out.write(annotated)

    pkt = {
        "seq": detect.seq,
        "found": found,
        "dx": float(dxc),
        "dy": float(dyc),
        "conf": round(float(best_conf),2),
    }
    detect.seq += 1
    return pkt

def close():
    cap.release()
    out.release()

if __name__ == "__main__":
    try:
        t0 = time.time()
        while time.time() - t0 < 15:
            detect()


    except: 
        pass
    finally:
        close()
