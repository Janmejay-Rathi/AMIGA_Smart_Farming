import cv2
import math
import RPi.GPIO as GPIO
from ultralytics import YOLO

# Setup GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(22, GPIO.OUT)  # Green light
GPIO.setup(24, GPIO.OUT)  # Yellow light
GPIO.setup(26, GPIO.OUT)  # Red light
GPIO.setup(35, GPIO.OUT)  # Buzzer

# Start webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Model
model = YOLO("yolo-Weights/yolov8n.pt")

# Object classes
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"]

while True:
    success, img = cap.read()
    if not success:
        print("Failed to capture image")
        break
    
    # Divide the image into front and rear view
    h, w, _ = img.shape
    front_view = img[0:h//2, :, :]  # Top half for front camera
    rear_view = img[h//2:, :, :]    # Bottom half for rear camera
    
    # Use only the front view for detection
    results = model(front_view, stream=True)

    # Coordinates
    detected_person = False  # To track if a person is detected

    for r in results:
        boxes = r.boxes

        for box in boxes:
            # Bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Convert to int values

            # Calculate the width of the bounding box
            width = x2 - x1

            # Get the class index
            cls = int(box.cls[0])

            # Only process if the detected class is 'person' (class index 0)
            if classNames[cls] == "person":
                # Mark person detected
                detected_person = True
                
                # Put box in cam
                cv2.rectangle(front_view, (x1, y1), (x2, y2), (255, 0, 255), 3)

                # Confidence
                confidence = math.ceil((box.conf[0] * 100)) / 100
                print("Confidence --->", confidence)

                # Object details
                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.8  # Reduce the font size for better fitting
                color = (255, 0, 0)
                thickness = 2

                cv2.putText(front_view, classNames[cls], org, font, fontScale, color, thickness)

                # Threshold-based actions based on the bounding box width
                if width <= 30:
                    GPIO.output(22, GPIO.HIGH)  # Green light
                    GPIO.output(24, GPIO.LOW)   # Yellow light off
                    GPIO.output(26, GPIO.LOW)   # Red light off
                    GPIO.output(35, GPIO.LOW)   # Buzzer off
                elif 30 < width <= 80:
                    GPIO.output(22, GPIO.LOW)   # Green light off
                    GPIO.output(24, GPIO.HIGH)  # Yellow light
                    GPIO.output(26, GPIO.LOW)   # Red light off
                    GPIO.output(35, GPIO.LOW)   # Buzzer off
                else:
                    GPIO.output(22, GPIO.LOW)   # Green light off
                    GPIO.output(24, GPIO.LOW)   # Yellow light off
                    GPIO.output(26, GPIO.HIGH)  # Red light
                    GPIO.output(35, GPIO.HIGH)  # Buzzer

    # If no human detected, run green light
    if not detected_person:
        GPIO.output(22, GPIO.HIGH)  # Green light
        GPIO.output(24, GPIO.LOW)   # Yellow light off
        GPIO.output(26, GPIO.LOW)   # Red light off
        GPIO.output(35, GPIO.LOW)   # Buzzer off

    # Resize the window to fit the image size
    cv2.namedWindow('front_view', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('front_view', 800, 600)  # Resize to fit the display (adjust as needed)

    # Show front view with detected persons
    cv2.imshow('front_view', front_view)

    # Press 'q' to exit
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
