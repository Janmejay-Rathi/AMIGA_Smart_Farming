#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped

def main():
    rospy.init_node('straight_cmd_vel_publisher')
    pub = rospy.Publisher('/amiga/cmd_vel', TwistStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "robot"
        msg.twist.linear.x = 5.2   # Forward speed in m/s
        msg.twist.angular.z = 2.0  # No rotation (straight)

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
