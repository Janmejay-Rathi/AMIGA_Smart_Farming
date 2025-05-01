#!/usr/bin/env python

import rospy
import cv2
import depthai as dai
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def oakd_publisher():
    rospy.init_node('oak_weed_publisher', anonymous=True)
    
    # Create a DepthAI pipeline
    pipeline = dai.Pipeline()

    # Define an RGB camera node
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setFps(30) 

    # Define an XLink output node to get frames to the host
    xout_video = pipeline.create(dai.node.XLinkOut)
    xout_video.setStreamName("video")
    cam_rgb.preview.link(xout_video.input)

    # Start the pipeline
    with dai.Device(pipeline) as device:
        video_queue = device.getOutputQueue(name="video", maxSize=40, blocking=False)
        bridge = CvBridge()
        image_pub = rospy.Publisher('/oakd_weed/image_raw', Image, queue_size=40)
        rate = rospy.Rate(30)  # 10 Hz

        rospy.loginfo("OAK-D Camera publisher started...")

        while not rospy.is_shutdown():
            in_video = video_queue.get()  # Get frame from OAK-D
            frame = in_video.getCvFrame()  # Convert to OpenCV format
            
            if frame is None:
                rospy.logerr("Failed to grab frame from OAK-D!")
                continue

            # Convert OpenCV image to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the image
            image_pub.publish(ros_image)

            rate.sleep()

if __name__ == '__main__':
    try:
        oakd_publisher()
    except rospy.ROSInterruptException:
        pass
