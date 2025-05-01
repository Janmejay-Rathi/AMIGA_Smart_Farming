#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from ultralytics import YOLO

class HumanDetector:
    def __init__(self, human_model_path):
        self.human_model_path = human_model_path  # Path to the YOLO model
        self.bridge = CvBridge()  # ROS <-> OpenCV bridge
        self.image_subscriber = rospy.Subscriber("/oak0/rgb", CompressedImage, self.image_callback)
        self.image_publisher = rospy.Publisher("/human_detection_output", Float64, queue_size=1)  # Optional publisher if needed
        self.conf_threshold = 0.5  # Confidence threshold for human detection
        self.load_model()

    def load_model(self):
        """Load the YOLO model."""
        self.human_model = YOLO(self.human_model_path)  # Load the pretrained YOLOv8 model

    def image_callback(self, ros_image):
        """Callback function to process the incoming image."""
        np_arr = np.frombuffer(ros_image.data, np.uint8)  # Convert ROS image to numpy array
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode the image
        self.detect_humans_and_display(image)

    def detect_humans_and_display(self, image):
        """Detect humans using YOLO and display the image."""
        results = self.human_model(image, conf=self.conf_threshold)  # Run detection
        for result in results[0].boxes:
            box = result.xyxy[0].cpu().numpy()  # Get bounding box coordinates
            cls = int(result.cls[0].cpu().numpy())  # Get the class id

            if cls == 0:  # Class ID 0 for 'person' in COCO dataset
                x1, y1, x2, y2 = box
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Draw bounding box

        # Display the image with detections
        image = cv2.resize(image,(1280,720))
        cv2.imshow("Human Detection", image)
        cv2.waitKey(1)  # Refresh the image window

    def main(self):
        rospy.spin()  # Keep the ROS node running

if __name__ == '__main__':
    rospy.init_node("human_detector_node", anonymous=True)

    # Initialize the detector with the YOLO model path
    detector = HumanDetector(human_model_path="/home/jurathi2/catkin_ws/src/navigation_pkg/src/line_follow/yolov8n.pt")  # Replace with the correct path to your YOLO model
    detector.main()
