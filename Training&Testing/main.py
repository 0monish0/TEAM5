import cv2
import numpy as np
import time
from ultralytics import YOLO
from collections import deque

# Load YOLO model
model = YOLO("Training&Testing/best.pt")

# Video URL
video_url = "http://10.194.52.27:4747/video"
cap = cv2.VideoCapture(video_url)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

# Moving average history
history_len = 5  # number of frames to average
cx_history = deque(maxlen=history_len)
cy_history = deque(maxlen=history_len)
area_history = deque(maxlen=history_len)

def detect_red_circles(frame, x1, y1, x2, y2):
    """Detect all red circles inside YOLO box, return largest circle center+area"""
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return None, None, None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Red ranges
    lower_red1 = np.array([0, 100, 80])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 80])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    mask = cv2.medianBlur(mask, 5)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, None

    largest_circle = None
    largest_area = 0
    cx_full, cy_full = None, None

    for c in contours:
        area = cv2.contourArea(c)
        if area < 100:  # filter small noise
            continue

        (cx, cy), radius = cv2.minEnclosingCircle(c)
        cx, cy, radius = int(cx), int(cy), int(radius)

        cx_abs = cx + x1
        cy_abs = cy + y1

        # Draw every red circle
        cv2.circle(frame, (cx_abs, cy_abs), radius, (0, 255, 0), 2)

        if area > largest_area:
            largest_area = area
            largest_circle = (cx_abs, cy_abs, radius)

    if largest_circle:
        cx_full, cy_full, r = largest_circle
        # Mark center of largest circle
        cv2.circle(frame, (cx_full, cy_full), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"Area: {int(largest_area)}",
                    (cx_full - 40, cy_full - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return cx_full, cy_full, largest_area


# For FPS calculation
prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Apply 90-degree clockwise rotation
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    frame_h, frame_w = frame.shape[:2]
    frame_center_x = frame_w // 2
    move_instruction = "NO BOX DETECTED"
    largest_area_logged = None

    # Run YOLO
    results = model(frame, conf=0.5, verbose=False)

    for result in results:
        for box, cls_id in zip(result.boxes.xyxy, result.boxes.cls):
            label_name = model.names[int(cls_id)]
            if label_name == "BOX":
                x1, y1, x2, y2 = map(int, box)

                # Clamp ROI coords (avoid empty slices)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(frame_w, x2), min(frame_h, y2)

                # Draw YOLO box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, "BOX", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                # Detect red circles inside
                cx_red, cy_red, area = detect_red_circles(frame, x1, y1, x2, y2)

                if cx_red is not None:
                    # Store in history
                    cx_history.append(cx_red)
                    cy_history.append(cy_red)
                    area_history.append(area)

                    # Compute smoothed values
                    cx_smoothed = int(np.mean(cx_history))
                    cy_smoothed = int(np.mean(cy_history))
                    area_smoothed = np.mean(area_history)

                    tolerance = frame_w // 10  # ~10% width
                    if cx_smoothed < frame_center_x - tolerance:
                        move_instruction = "MOVE LEFT"
                    elif cx_smoothed > frame_center_x + tolerance:
                        move_instruction = "MOVE RIGHT"
                    else:
                        move_instruction = "FORWARD"

                    if area_smoothed > 5000:  # too close
                        move_instruction = "STOP"

                    largest_area_logged = area_smoothed

    # Show movement instruction
    cv2.putText(frame, move_instruction, (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 3)

    # FPS calculation (smoothed)
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    cv2.putText(frame, f"FPS: {int(fps)}", (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    if largest_area_logged is not None:
        print(f"Red circle area (smoothed): {int(largest_area_logged)}")

    cv2.imshow("YOLOv11 + Red Circle Detection (Smoothed)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
