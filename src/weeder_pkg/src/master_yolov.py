#!/usr/bin/env python3
# master_oop.py

import rospy
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image, CompressedImage
import torch
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import pandas


class DynamixelMasterNode:
    def __init__(self):
        # Parameters for control
        self.robot_speed = 1.0  # Speed of the robot
        self.distance_from_weed = 3.0  # Distance from the weed

        # Load YOLO model
        # model_path = '/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/line_follow/lab_leaf_weed.pt'  # Replace with actual model path
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

        self.model = YOLO("/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/line_follow/lab_leaf_weed.pt") 

        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # ROS publishers
        self.weed_pub = rospy.Publisher('/weed_detected', Bool, queue_size=1)
        self.rotation_time_pub = rospy.Publisher('/rotation_trigger_time', Float32, queue_size=1)

        # ROS subscriber
        rospy.Subscriber('/oak1/rgb', CompressedImage, self.image_callback)
        print('asasas')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        # print("asasas")
        # frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  
        frame = cv2.resize(frame, (640, 360))
        h, w = frame.shape[:2]
        crop = 60
        frame[:,w-crop:w,:] = np.zeros((h,crop,3))
        cv2.imshow("frame",frame)
        cv2.waitKey(1)

        # Run YOLO model on the frame
        results = self.model(frame, conf = 0.5)
        # detected_classes = results.pandas().xyxy[0]['name'].values
        # detected_classes = results.xyxy[0][:,-1].tolist()
        detected_classes = [self.model.names[int(cls)] for cls in results[0].boxes.cls]

        # Detect weed and publish appropriate messages
        if 'horseradish' in detected_classes:
            rospy.loginfo("Weed detected, publishing control messages.")

            # Publish weed detection flag
            self.weed_pub.publish(True)

            # Calculate and publish rotation trigger time
            rotation_trigger_time = self.distance_from_weed / self.robot_speed
            self.rotation_time_pub.publish(rotation_trigger_time)
        else:
            self.weed_pub.publish(False)  # No weed detected

    def run(self):
        rospy.init_node('master_oop', anonymous=True)
        rospy.loginfo("Dynamixel master node started.")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DynamixelMasterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
