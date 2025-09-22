import cv2
from ultralytics import YOLO

# Load trained model
model = YOLO("Training&Testing/best.pt")  

# Video URL (DroidCam / IP Camera)
video_url = "http://10.194.52.27:4747/video"
cap = cv2.VideoCapture(video_url)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO detection
    results = model.predict(frame, conf=0.5)

    # Frame center (for alignment check)
    frame_h, frame_w = frame.shape[:2]
    frame_center_x = frame_w // 2

    move_instruction = "NO BOX DETECTED"

    for result in results:
        for box, cls_id in zip(result.boxes.xyxy, result.boxes.cls):
            label_name = model.names[int(cls_id)]
            if label_name == "BOX":
                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, box)
                cx = (x1 + x2) // 2  # center x of detected box
                cy = (y1 + y2) // 2

                # Draw detection
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"{label_name}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # Decide movement instruction
                tolerance = 50  # pixels allowed deviation
                if cx < frame_center_x - tolerance:
                    move_instruction = "MOVE LEFT"
                elif cx > frame_center_x + tolerance:
                    move_instruction = "MOVE RIGHT"
                else:
                    move_instruction = "FORWARD"

    # Show movement instruction on screen
    cv2.putText(frame, move_instruction, (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 3)

    cv2.imshow("YOLOv11 Object Detection with Instructions", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
