import cv2
from ultralytics import YOLO
import time

# ================= Model / Camera =================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

video_name = time.strftime("videos/%Y_%m_%d_%H_%M.mp4")
out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (480, 640))

def detect():
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # Vẽ chú thích
    # annotated = frame
    # cv2.drawMarker(annotated, (int(1280/2), int(720/2)), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)


    out.write(frame)


def close():
    cap.release()
    out.release()

if __name__ == "__main__":
    try:
        t0 = time.time()
        while time.time() - t0 < 180:
            detect()

    except: 
        pass
    finally:
        close()