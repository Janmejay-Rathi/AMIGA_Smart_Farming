#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

rospy.init_node('wheel_commander')

# Publishers to each wheel's velocity controller
pubs = {
    "br": rospy.Publisher('/br_wheel_joint_velocity_controller/command', Float64, queue_size=1),
    "bl": rospy.Publisher('/bl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
    "fl": rospy.Publisher('/fl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
    "fr": rospy.Publisher('/fr_wheel_joint_velocity_controller/command', Float64, queue_size=1),
}

# Default speeds with runtime-overridable parameters
default_speeds = {
    "br": rospy.get_param("~br_speed", 0.0),
    "bl": rospy.get_param("~bl_speed", 7.0),
    "fl": rospy.get_param("~fl_speed", 7.0),
    "fr": rospy.get_param("~fr_speed", 0.0),
}

rate = rospy.Rate(10)  # Hz

while not rospy.is_shutdown():
    for wheel, pub in pubs.items():
        speed = rospy.get_param("~" + wheel + "_speed", default_speeds[wheel])
        pub.publish(speed)
    rate.sleep()

