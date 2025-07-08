#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('turning_commander')

    # Publishers for each wheel
    pubs = {
        "br": rospy.Publisher('/br_wheel_joint_velocity_controller/command', Float64, queue_size=1),
        "bl": rospy.Publisher('/bl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
        "fl": rospy.Publisher('/fl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
        "fr": rospy.Publisher('/fr_wheel_joint_velocity_controller/command', Float64, queue_size=1),
    }

    # Define turning speed (rad/s)
    turning_speed = 1.0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Left wheels forward
        pubs["bl"].publish(turning_speed)
        pubs["fl"].publish(turning_speed)

        # Right wheels backward
        pubs["br"].publish(-turning_speed)
        pubs["fr"].publish(-turning_speed)

        rate.sleep()

if __name__ == '__main__':
    main()
