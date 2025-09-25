import cv2
import numpy as np
import time
from ultralytics import YOLO
from collections import deque, Counter
import websocket

# ===== WebSocket settings =====
ESP32_IP = "192.168.137.10"
ESP32_PORT = 81
ws = websocket.WebSocket()
try:
    ws.connect(f"ws://{ESP32_IP}:{ESP32_PORT}/")
    print("✅ Connected to ESP32")
except Exception as e:
    print("❌ WebSocket connection failed:", e)
    ws = None

# ===== Load YOLO model =====
model = YOLO("Training&Testing/best.pt")

# ===== Video source =====
video_url = "http://10.194.52.27:4747/video"
cap = cv2.VideoCapture(video_url)

# ===== Smoothing histories =====
history_len = 5
cx_history = deque(maxlen=history_len)
instruction_history = deque(maxlen=history_len)

# ===== Red circle detection =====
def detect_red_circles(frame, x1, y1, x2, y2):
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return None, None, None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                          cv2.inRange(hsv, lower_red2, upper_red2))
    # Morphological filtering to remove noise
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, None

    # Only largest contour
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < 50:  # smaller min area
        return None, None, None

    (cx, cy), radius = cv2.minEnclosingCircle(c)
    cx_abs, cy_abs = int(cx) + x1, int(cy) + y1

    cv2.circle(frame, (cx_abs, cy_abs), int(radius), (0, 0, 255), 2)
    cv2.circle(frame, (cx_abs, cy_abs), 5, (255, 0, 255), -1)
    return cx_abs, cy_abs, area

# ===== Main loop =====
prev_time = time.time()
red_detected_stable_frames = 0
red_stability_threshold = 2  # fewer frames for faster response
last_send_time = 0
send_interval = 0.25  # seconds
last_command_sent = None
hysteresis = 40  # deadzone around center

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        continue

    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame_h, frame_w = frame.shape[:2]
    frame_center_x = frame_w // 2

    move_instruction = "NO BOX DETECTED"
    target_used = "NONE"
    distance_estimate = None

    results = model(frame, conf=0.5, verbose=False)

    box_cx, box_cy = None, None
    red_cx, red_cy = None, None
    area_red = None

    # Process YOLO boxes
    for result in results:
        for box, cls_id in zip(result.boxes.xyxy, result.boxes.cls):
            label_name = model.names[int(cls_id)]
            if label_name == "BOX":
                x1, y1, x2, y2 = map(int, box)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(frame_w, x2), min(frame_h, y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                box_cx, box_cy = (x1 + x2) // 2, (y1 + y2) // 2
                red_cx, red_cy, area_red = detect_red_circles(frame, x1, y1, x2, y2)

    # Decide which target to use
    if red_cx is not None and area_red > 50:
        red_detected_stable_frames += 1
        if red_detected_stable_frames >= red_stability_threshold:
            target_cx, target_cy = red_cx, red_cy
            target_used = "RED CIRCLE"
        else:
            target_cx, target_cy = box_cx, box_cy
            target_used = "BOX"
    elif box_cx is not None:
        target_cx, target_cy = box_cx, box_cy
        target_used = "BOX"
        red_detected_stable_frames = 0
    else:
        target_cx, target_cy = None, None
        target_used = "NONE"
        red_detected_stable_frames = 0

    # Smooth target position
    if target_cx is not None:
        cx_history.append(target_cx)
        target_cx_s = int(np.mean(cx_history))

        # Hysteresis deadzone
        if target_cx_s < frame_center_x - hysteresis:
            move_instruction = "MOVE LEFT"
            send_val = 4
        elif target_cx_s > frame_center_x + hysteresis:
            move_instruction = "MOVE RIGHT"
            send_val = 3
        else:
            move_instruction = "FORWARD"
            send_val = 1

        # Stop if red circle very close
        if target_used == "RED CIRCLE" and area_red is not None and area_red > 5000:
            move_instruction = "STOP"
            send_val = 0

        # Send command to ESP32 if changed & after interval
        now = time.time()
        if ws and (send_val != last_command_sent) and (now - last_send_time > send_interval):
            try:
                ws.send(str(send_val))
                last_send_time = now
                last_command_sent = send_val
            except Exception as e:
                print("WebSocket send failed:", e)

        cv2.circle(frame, (target_cx_s, target_cy), 7, (0, 255, 255), -1)
        cv2.putText(frame, f"Target: {target_used}", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    # Smooth instructions for display
    instruction_history.append(move_instruction)
    move_instruction_smooth = Counter(instruction_history).most_common(1)[0][0]
    cv2.putText(frame, f"Move: {move_instruction_smooth}", (30, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 2)

    # FPS display
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    cv2.putText(frame, f"FPS: {int(fps)}", (30, frame_h - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.imshow("BOX+RED Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if ws:
    ws.close()
