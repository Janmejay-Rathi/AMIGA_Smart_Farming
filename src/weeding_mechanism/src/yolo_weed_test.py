import depthai as dai
import cv2
from ultralytics import YOLO
import numpy as np

# Load the YOLO model
model = YOLO('best.pt')

# Create DepthAI pipeline for OAK-D
pipeline = dai.Pipeline()

# Camera calibration parameters
camera_matrix = np.array([[565.24788784, 0.0, 609.00543687],
                          [0.0, 564.44266352, 344.98979659],
                          [0.0, 0.0, 1.0]])
dist_coeffs = np.array([-0.310698084, 0.121052216, 0.000180668097, 0.0000485008167, -0.0234607907])

# Create the color camera node
cam = pipeline.create(dai.node.ColorCamera)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)
cam.setFps(30)

# Stream video to host
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.video.link(xout.input)

# Start device
with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=8, blocking=False)

    while True:
        frame = q.get().getCvFrame()

        # Undistort the frame using camera matrix and distortion coefficients
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Run YOLO on the undistorted frame
        results = model(undistorted_frame)

        for result in results:
            boxes = result.boxes.xyxy
            confidences = result.boxes.conf
            class_ids = result.boxes.cls

            for i in range(len(boxes)):
                x1, y1, x2, y2 = boxes[i]
                x_center, y_center = int((x1+x2)//2), int((y1+y2)//2)
                confidence = confidences[i]
                class_id = class_ids[i]

                if confidence > 0.5:
                    print(f"Detection: Class {int(class_id)}, Confidence {confidence:.2f}, BBox: ({x1}, {y1}), ({x2}, {y2})")

                    cv2.rectangle(undistorted_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                    cv2.circle(undistorted_frame, (x_center, y_center), 4, (0, 0, 255), -1)
                    cv2.putText(undistorted_frame, f"({x_center},{y_center})", (x_center + 5, y_center - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    cv2.putText(undistorted_frame, f"Conf: {confidence:.2f}", (int(x1), int(y1)-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.imshow("Undistorted OAK-D Feed with YOLO", undistorted_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
