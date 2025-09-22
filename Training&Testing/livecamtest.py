import cv2
from ultralytics import YOLO  # YOLOv8+ API also works for YOLOv11 models

# Load your trained YOLO model
model = YOLO("Training&Testing/best.pt")  # path to your YOLOv11n model

# Video URL
video_url = "http://10.194.52.27:4747/video"  # replace with your DroidCam or stream URL
cap = cv2.VideoCapture(video_url)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Run object detection
    results = model.predict(frame, conf=0.5)  # conf=0.5 is confidence threshold

    # Draw boxes for objects labeled 'BOX'
    for result in results:
        boxes = result.boxes  # get detected boxes
        for box, cls_id in zip(boxes.xyxy, boxes.cls):
            label_name = model.names[int(cls_id)]
            if label_name == "BOX":
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # red box
                cv2.putText(frame, label_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    cv2.imshow("YOLOv11 Object Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
