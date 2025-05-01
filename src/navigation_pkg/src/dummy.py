#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class HeadingAnglePlotter:
    def __init__(self, topic_name="/canbus/twist"):
        # Initialize ROS node
        rospy.init_node('heading_angle_plotter', anonymous=True)
        
        # Initialize subscriber
        self.topic_name = topic_name
        self.subscriber = rospy.Subscriber(self.topic_name, TwistStamped, self.twist_callback)
        
        # Initialize data storage for time and angle
        self.time_data = []
        self.angle_data = []
        self.angle2_data = []
        self.vel_data_x = []
        self.vel_data_y = []
        self.start_time = None
        self.prev_time = 0
        # self.start_time = 0
        self.time_step = 1/15.8
        self.angle2 = 0
        # Initialize the plot
        plt.figure()
        self.ani = FuncAnimation(plt.gcf(), self.plot_heading, interval=10)
        plt.show()

    def twist_callback(self, twist_stamped_msg):
        # Extract linear velocities
        
        linear_x = twist_stamped_msg.twist.linear.x
        linear_y = twist_stamped_msg.twist.linear.y
        angular_z = twist_stamped_msg.twist.angular.z

        # Calculate heading angle using atan2
        angle = math.atan2(linear_y, linear_x)

        
        # Use the timestamp from the message header
        if self.start_time is None:
            self.start_time = twist_stamped_msg.header.stamp.to_sec()
        self.current_time = twist_stamped_msg.header.stamp.to_sec() - self.start_time
        self.time_step = self.current_time - self.prev_time
        print("step:",self.time_step)
        self.angle2 += angular_z * self.time_step
        print("Angle: ", math.degrees(self.angle2))
        
        # Record time and angle for plotting
        self.time_data.append(self.current_time)
        self.angle_data.append(math.degrees(angle))  # Convert to degrees for easier interpretation
        self.angle2_data.append(math.degrees(self.angle2))  # Convert to degrees for easier interpretation
        self.vel_data_x.append(linear_x)
        self.vel_data_y.append(linear_y)
        self.prev_time = self.current_time

    def plot_heading(self, i):
        # Clear the plot to update
        plt.cla()
        plt.plot(self.time_data, self.angle2_data, label="Heading Angle (degrees)")
        plt.xlabel("Time (s)")
        plt.ylabel("Heading Angle (degrees)")
        plt.title("Heading Angle Over Time")
        plt.legend(loc="upper right")
        plt.grid()

    def run(self):
        # Keep the ROS node alive
        rospy.spin()

if __name__ == "__main__":
    plotter = HeadingAnglePlotter(topic_name="/canbus/twist")
    plotter.run()
