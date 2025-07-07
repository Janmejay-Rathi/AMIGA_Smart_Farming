#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

rospy.init_node('wheel_commander')

pubs = {
    "lf": rospy.Publisher('/br_wheel_joint_velocity_controller/command', Float64, queue_size=1),
    "rf": rospy.Publisher('/bl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
    "lr": rospy.Publisher('/fl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
    "rr": rospy.Publisher('/fr_wheel_joint_velocity_controller/command', Float64, queue_size=1),
}

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    for pub in pubs.values():
        pub.publish(2.0)  # rad/s velocity
    rate.sleep()
