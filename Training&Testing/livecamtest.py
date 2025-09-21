from ultralytics import YOLO
import cv2

# Load your trained YOLO model (replace with your path)
model = YOLO("model/best.pt")  

# Open camera (0 = default webcam / Pi camera)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO inference
    results = model(frame, stream=True)

    # Draw detections
    for r in results:
        for box in r.boxes:
            # Get bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])  # confidence
            cls = int(box.cls[0])      # class id
            label = model.names[cls]   # should be "target"

            # Draw rectangle
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

    # Show the live video with detections
    cv2.imshow("YOLO Detection", frame)

    # Quit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
