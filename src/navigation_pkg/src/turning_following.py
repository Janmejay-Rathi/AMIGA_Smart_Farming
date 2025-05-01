#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path

class PathFollower:
    def __init__(self):
        # Subscriber and Publisher
        self.gps_xy_sub = rospy.Subscriber('/gps_xy_loc', Float32MultiArray, self.gps_xy_callback)
        self.waypoints_turning_sub = rospy.Subscriber('/waypoints_turning', Path, self.load_global_waypointsCB)
        
        # Publisher for local waypoints
        self.local_waypoints_pub = rospy.Publisher('/local_waypoints_turning', Path, queue_size=10)

        # Initialize variables
        self.current_position = None
        self.previous_position = None
        self.current_orientation = 0.0  # Initial heading
        self.waypoints = []  # List of waypoints as (x, y) tuples

    def load_global_waypointsCB(self, msg):
        # Clear waypoints and load new ones from the Path message
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        rospy.loginfo(f"Loaded global waypoints: {self.waypoints}")

    def gps_xy_callback(self, msg):
        # Update the robot's current position using the incoming GPS data
        x, y = msg.data[0], msg.data[1]
        new_position = np.array([x, y])

        # Update heading if we have a previous position
        if self.current_position is not None:
            dx, dy = new_position - self.current_position
            self.current_orientation = np.arctan2(dy, dx)  # Estimate heading based on GPS movement

        # Update positions
        self.previous_position = self.current_position
        self.current_position = new_position

    def convert_to_robot_frame(self, waypoint):
        if self.current_position is None:
            return None

        # Convert global waypoint (x, y) to robot's local frame
        dx, dy = waypoint[0] - self.current_position[0], waypoint[1] - self.current_position[1]

        # Rotation matrix for current orientation
        cos_theta = np.cos(-self.current_orientation)
        sin_theta = np.sin(-self.current_orientation)
        x_local = cos_theta * dx - sin_theta * dy
        y_local = sin_theta * dx + cos_theta * dy

        return np.array([x_local, y_local])

    def get_local_waypoints(self):
        # Convert global waypoints to local robot frame
        local_waypoints = [self.convert_to_robot_frame(wp) for wp in self.waypoints]
        return [wp for wp in local_waypoints if wp is not None]

    def publish_local_waypoints(self, local_waypoints):
        # Create a Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "base_link"  # Use "base_link" for local coordinates

        # Convert each waypoint to a PoseStamped and add to path_msg
        for waypoint in local_waypoints:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)

        # Publish the Path message with local waypoints
        self.local_waypoints_pub.publish(path_msg)
        rospy.loginfo("Published local waypoints as a Path message")

    def follow_path(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.current_position is None:
                continue  # Wait until GPS data is available

            # Convert waypoints to the robot's local frame
            local_waypoints = self.get_local_waypoints()

            # Publish the local waypoints
            self.publish_local_waypoints(local_waypoints)

            rate.sleep()

def main():
    rospy.init_node('path_follower', anonymous=True)
    path_follower = PathFollower()
    path_follower.follow_path()

if __name__ == "__main__":
    main()
