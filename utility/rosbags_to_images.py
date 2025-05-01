import time
import math
import numpy as np
import cv2
import rospy
import os
import pathlib

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import argparse

# Argument parsing
parser = argparse.ArgumentParser()
parser.add_argument('--output_name', '-o', type=str, required=True)
args = parser.parse_args()

# Initialize CvBridge and other variables
bridge = CvBridge()

# Change the output directory path where you want the images to be saved
# OUTPUT_DIR = "/home/jurathi2/catkin_ws/bags/Test_Area_Setup/DownwardsCam/images"
OUTPUT_DIR = "/home/jurathi2/catkin_ws/bags/Test_Area_Setup/BackCam_weeds/24th March/imgs"

pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
should_exit = False
cnt = 0

def img_callback_compressed_image(data):
    global cnt
    global should_exit
    try:
        # Check if the message is a CompressedImage
        if isinstance(data, CompressedImage):
            # Decode the compressed image data
            np_arr = np.frombuffer(data.data, np.uint8)  # Convert byte data to numpy array
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode the image
        else:
            # If it's a regular Image message, use CvBridge to convert to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        
    except CvBridgeError as e:
        print(f"Error: {e}")
        return

    cnt += 1
    freq = 30  # Save every 1st frame
    if cnt % freq == 0:
        output_path = os.path.join(OUTPUT_DIR, '{}_{}.jpg'.format(args.output_name, cnt // freq))
        print(f"Saving image to {output_path}")
        cv2.imwrite(output_path, cv_image)

    if cnt // freq >= 1000:
        should_exit = True

def img_callback_normal(data):
    global cnt
    global should_exit
    try:
        # Convert the ROS Image message to an OpenCV format
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        
    except CvBridgeError as e:
        print(f"Error: {e}")
        return

    cnt += 1
    freq = 20  # Save every 1st frame
    if cnt % freq == 0:
        output_path = os.path.join(OUTPUT_DIR, '{}_{}.jpg'.format(args.output_name, cnt // freq))
        print(f"Saving image to {output_path}")
        cv2.imwrite(output_path, cv_image)

    if cnt // freq >= 1000:
        should_exit = True

if __name__ == '__main__':    
    rospy.init_node('lanenet_node', anonymous=True)

    # Subscribe to the compressed image topic
    # For Downwards Camera
    # sub_image = rospy.Subscriber('/oak1/rgb', img_callback_compressed_image, img_callback, queue_size=1)
    # For Front Camera
    # sub_image = rospy.Subscriber('/oak0/rgb', Compressimg_callback_compressed_imageedImage, img_callback, queue_size=10)
    # For Weeding Camera
    sub_image = rospy.Subscriber('/oakd_weed/image_raw', Image, img_callback_normal, queue_size=10)

    while not should_exit and not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
    
    print("Exiting...")

