#!/usr/bin/env python3
# Master.py
import rospy
from std_msgs.msg import Float32

class DynamixelMasterNode:
    def input_node():
        rospy.init_node('Master', anonymous=True)
        pub = rospy.Publisher('/dynamixel_angle', Float32, queue_size=10)

        while not rospy.is_shutdown():
            try:
                user_input = input("Enter angle increment (degrees, -360 to 360) or 'q' to quit: ")
                if user_input.lower() == 'q':
                    rospy.loginfo("Exiting...")
                    break

                angle = float(user_input)
                if -360 <= angle <= 360:
                    pub.publish(angle)
                    rospy.loginfo(f"Published angle increment: {angle}")
                else:
                    rospy.logwarn("Please enter a valid angle increment between -360 and 360.")
            except ValueError:
                rospy.logwarn("Invalid input. Please enter a valid number.")

if __name__ == '__main__':
    try:
        DynamixelMasterNode.input_node()
    except rospy.ROSInterruptException:
        pass
