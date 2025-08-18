#!/usr/bin/env python
# This ROS node generates a sequence of waypoints based on detected row distance and heading angle.
# The waypoints are published as a Path message for the robot to follow.

import rospy
from std_msgs.msg import Float64, Float64MultiArray     # For receiving heading and distance values
import numpy as np                                      # For numerical operations
from nav_msgs.msg import Path                           # Path message to represent sequence of waypoints
from geometry_msgs.msg import PoseStamped               # PoseStamped for individual waypoints


class Waypoints():
    def __init__(self):
        """
        Initializes the Waypoints generator node:
        - Subscribes to robot heading angle and row distance topics
        - Publishes waypoints in the form of a Path message
        """
        # Subscriber: receives heading angle offset (from row detector node)
        self.angle_offset_sub = rospy.Subscriber('/robot_heading_angle', Float64, self.angleCB, queue_size=1)
        
        # Publisher: publishes generated waypoints as a Path message
        self.waypoints_pub = rospy.Publisher('/waypoints_in_line', Path, queue_size=1)
        
        # Subscriber: receives distance offset (lateral distance from plant row center)
        self.distance_sub = rospy.Subscriber('row_distance', Float64, self.distanceCB, queue_size=1)
        
        self.rate = rospy.Rate(10)   # Define loop rate (10 Hz, currently unused)
        rospy.spin()                 # Keep node alive and responsive to callbacks

    def angleCB(self, msg):
        """
        Callback function for heading angle.
        Stores the received angle and generates waypoints whenever a new angle is received.
        """
        self.angle_offset = msg.data
        self.generate_waypoints()   # Trigger waypoint generation after receiving angle
        # print("Angle received")   # Debugging statement (commented out)

    def distanceCB(self, msg):
        """
        Callback function for lateral row distance.
        Stores the received distance offset for later use in waypoint calculation.
        """
        self.distance_offset = msg.data

    def get_waypoints_on_line(self, spacing=0.1):
        """
        Generates waypoints along a straight line at a constant distance offset.
        Args:
            spacing: Distance between consecutive waypoints (not directly used here).
        Returns:
            List of waypoints [(x, y), ...] in the row coordinate frame.
        """
        scale = 0.3 / 139   # Conversion factor: meters per pixel (camera calibration scaling)
        
        # Convert pixel-based distance from row detector into real-world distance
        distance_offset = self.distance_offset * scale
        
        # Define x-coordinates for waypoints (from 0 to 5 meters, total 10 points)
        start, stop, num = 0, 5, 10
        x_pts = np.linspace(start, stop, num)
        
        # Generate waypoints: x increases linearly, y is constant offset
        waypoints = [(x, distance_offset) for x in x_pts]
        waypoints_final = waypoints
        
        return waypoints_final
    
    def transform_waypoints_to_robot_frame(self, waypoints):
        """
        Transforms waypoints from world/row frame to robot frame using heading angle.
        Args:
            waypoints: List of (x, y) points in the row frame.
        Returns:
            Transformed waypoints in the robot's base frame.
        """
        # Convert heading angle from degrees to radians
        self.theta = np.deg2rad(self.angle_offset)
        
        # Define 2D rotation matrix
        rotation_matrix = np.array([[np.cos(self.theta), -np.sin(self.theta)], 
                                    [np.sin(self.theta),  np.cos(self.theta)]])
        
        # Apply rotation transformation to all waypoints
        transformed_waypoints = []
        for waypoint in waypoints:
            transformed_wp = np.dot(rotation_matrix, np.array(waypoint))
            transformed_waypoints.append(tuple(transformed_wp))
        
        return transformed_waypoints
    
    def generate_waypoints(self):
        """
        Generates a Path message containing waypoints in the robot frame
        and publishes it to the '/waypoints_in_line' topic.
        """
        # Step 1: Generate base waypoints in row frame
        wps = self.get_waypoints_on_line()
        
        # Step 2: Transform them to robot's base frame
        wps_robot_frame = self.transform_waypoints_to_robot_frame(wps)

        # Step 3: Create a Path message
        path = Path()
        path.header.frame_id = 'base_footprint'   # Waypoints relative to robot base frame

        # Step 4: Fill Path message with waypoints
        for waypoint in wps_robot_frame:
            pose = PoseStamped()
            x_rotated, y_rotated = waypoint
            pose.pose.position.x = x_rotated
            pose.pose.position.y = y_rotated

            # Orientation is set to identity quaternion (no rotation)
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            # Append waypoint to path
            path.poses.append(pose)

        # Debugging: print generated reference waypoints
        print("Reference wps:", wps_robot_frame)

        # Step 5: Publish Path message
        self.waypoints_pub.publish(path)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node("Wps_generator_node")
    
    # Create Waypoints generator instance
    WPS_gen = Waypoints()
    # Node will run continuously due to rospy.spin() inside class
