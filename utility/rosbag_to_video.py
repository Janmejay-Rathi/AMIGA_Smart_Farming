#!/usr/bin/env python

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# ==== CONFIG ====
bag_file = "a.bag"
image_topic = "/oakd_weed/image_raw"  
output_video = "output.mp4"
fps = 30  # frames per second

# Initialize bridge and video writer
bridge = CvBridge()
out = None

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if out is None:
                height, width = cv_img.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

            out.write(cv_img)

        except Exception as e:
            print(f"Error converting frame: {e}")

if out:
    out.release()
    print("Video saved:", output_video)
