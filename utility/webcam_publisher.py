#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def webcam_publisher():
    # Initialize the node
    rospy.init_node('webcam_publisher', anonymous=True)

    # Create a VideoCapture object to access the webcam
    cap = cv2.VideoCapture(2)  # '0' for the default webcam, change if needed

    if not cap.isOpened():
        rospy.logerr("Could not open webcam!")
        return

    # Set the frame width and height
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Create a CvBridge object to convert OpenCV images to ROS images
    bridge = CvBridge()

    # Create a publisher to publish the webcam data to a topic
    image_pub = rospy.Publisher('/webcam/image_raw', Image, queue_size=10)

    rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz

    while not rospy.is_shutdown():
        # Capture a frame from the webcam
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Failed to grab frame!")
            break

        # Convert the frame to a ROS Image message
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the image on the /webcam/image_raw topic
        image_pub.publish(ros_image)

        # Sleep to maintain the publishing rate
        rate.sleep()

    # Release the webcam when done
    cap.release()

if __name__ == '__main__':
    try:
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass
