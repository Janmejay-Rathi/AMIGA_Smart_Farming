#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

class TwistToWheelController:
    def __init__(self):
        rospy.init_node('twist_to_wheels')

        # Robot physical parameters
        wheel_radius = 0.1      # meters
        wheel_base = 0.5        # distance between left and right wheels (meters)

        self.L = wheel_base
        self.R = wheel_radius

        # Publishers to each wheel
        self.pubs = {
            "lf": rospy.Publisher('/br_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "rf": rospy.Publisher('/bl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "lr": rospy.Publisher('/fl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "rr": rospy.Publisher('/fr_wheel_joint_velocity_controller/command', Float64, queue_size=1),
        }

        rospy.Subscriber('/amiga/cmd_vel', TwistStamped, self.cmd_callback)
        rospy.spin()

    def cmd_callback(self, msg):
        linear_vel = msg.twist.linear.x
        angular_vel = msg.twist.angular.z

        # Differential drive kinematics
        v_left = (linear_vel - angular_vel * self.L / 2.0) / self.R
        v_right = (linear_vel + angular_vel * self.L / 2.0) / self.R

        # Publish to each wheel (assuming left = lf+lr, right = rf+rr)
        self.pubs['lf'].publish(v_right)
        self.pubs['rf'].publish(v_left)
        self.pubs['lr'].publish(v_right)
        self.pubs['rr'].publish(v_left)

if __name__ == '__main__':
    TwistToWheelController()
