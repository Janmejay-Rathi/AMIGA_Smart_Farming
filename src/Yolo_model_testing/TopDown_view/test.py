import cv2
from ultralytics import YOLO

# Load YOLOv8 model (make sure to replace with your custom model path if needed)
model = YOLO("best.pt")  # Use yolov8s.pt, yolov8m.pt, etc., if desired

# Open webcam feed
cap = cv2.VideoCapture(2)  # 0 is usually the default webcam

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLOv8 inference on the frame
    results = model(frame, conf=0.5)

    # Visualize results on the frame
    annotated_frame = results[0].plot()

    # Print detected classes
    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        print(f"Detected: {model.names[cls_id]} with confidence {conf:.2f}")

    # Show the output
    cv2.imshow("YOLOv8 Webcam Detection", annotated_frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
