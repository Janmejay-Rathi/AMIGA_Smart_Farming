#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

class Turn:
    def __init__(self, gps, d1=50, d2=100, radius=None, num_waypoints=20):
        self.gps = gps  # Starting GPS coordinates (point A)
        self.d1 = d1  # Vertical distance from A to B and from C to D
        self.d2 = d2  # Horizontal distance between A and D (along x-axis)
        self.radius = radius if radius else d2 / 2  # The radius of the semicircle
        self.num_waypoints = num_waypoints
        self.current_gps_position = gps  # Initialize with starting position
        self.actual_gps_position = (0, 1)  # To store actual GPS position from topic
        self.all_waypoints = []  # Store all waypoints generated

        # ROS publisher for waypoints in Path message format
        self.path_pub = rospy.Publisher('/waypoints_turning_path', Path, queue_size=10)
        self.gps_sub = rospy.Subscriber('/gps_xy_loc', Float32MultiArray, self.actual_gps_positionCB, queue_size=1)

        # Set up the plotting
        plt.ion()  # Interactive mode on
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Filtered Waypoints")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_xlim(-200, 200)  # Set x limits based on expected GPS coordinates
        self.ax.set_ylim(-200, 200)  # Set y limits based on expected GPS coordinates

    def actual_gps_positionCB(self, msg):
        # Update the actual GPS position of the robot
        self.actual_gps_position = (msg.data[0], msg.data[1])

    def move_vertical(self, distance):
        new_position = (self.current_gps_position[0], self.current_gps_position[1] - distance)
        self.current_gps_position = new_position
        return new_position

    def generate_semicircle_waypoints(self, center_x, center_y):
        waypoints = []
        start_angle = 180  # Start from point B
        end_angle = 0  # End at point C
        angles = np.linspace(np.radians(start_angle), np.radians(end_angle), self.num_waypoints)

        for angle in angles:
            x = center_x + self.radius * np.cos(angle)
            y = center_y + self.radius * np.sin(angle)
            waypoints.append((x, y))
        return waypoints

    def filter_waypoints_ahead(self):
        # Only return waypoints that are ahead of the actual GPS position
        if self.actual_gps_position is None:
            return []

        def is_ahead(waypoint):
            # Calculate squared distance to avoid sqrt for efficiency
            dist_sq = (waypoint[0] - self.actual_gps_position[0]) ** 2 + \
                      (waypoint[1] - self.actual_gps_position[1]) ** 2
            return dist_sq > 1.0  # Change this threshold as needed for "ahead" distance

        return [wp for wp in self.all_waypoints if is_ahead(wp)]

    def publish_waypoints_as_path(self):
        # Filter waypoints to only those ahead of the robot
        waypoints_ahead = self.filter_waypoints_ahead()

        # Create a Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"  # Set the appropriate frame

        # Convert waypoints to PoseStamped and add to path_msg
        for waypoint in waypoints_ahead:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)

        # Publish the path message
        self.path_pub.publish(path_msg)
        rospy.loginfo(f"Published {len(path_msg.poses)} waypoints as a Path message")

        # Plot filtered waypoints in real-time
        self.plot_filtered_waypoints(waypoints_ahead)

    def plot_filtered_waypoints(self, waypoints_ahead):
        # Clear the axes
        self.ax.clear()
        self.ax.set_title("Filtered Waypoints")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_xlim(-200, 200)  # Set x limits based on expected GPS coordinates
        self.ax.set_ylim(-200, 200)  # Set y limits based on expected GPS coordinates

        # Plot waypoints
        if waypoints_ahead:
            x_vals, y_vals = zip(*waypoints_ahead)
            self.ax.plot(x_vals, y_vals, 'bo-', label='Waypoints Ahead')
            self.ax.legend()

        plt.pause(0.1)  # Pause to update the plot

    def perform_turn(self):
        # Step 1: Start at point A and move vertically to point B
        initial_position = self.gps
        point_B = self.move_vertical(-self.d1)
        rospy.loginfo(f"Moved vertically to point B: {point_B}")

        # Step 2: Calculate the center of the semicircle (center is horizontally between B and C)
        center_x = point_B[0] + self.d2 / 2
        center_y = point_B[1]
        rospy.loginfo(f"Center of semicircle: ({center_x}, {center_y})")

        # Step 3: Generate waypoints along the semicircle
        semicircle_waypoints = self.generate_semicircle_waypoints(center_x, center_y)
        rospy.loginfo("Generated semicircle waypoints")

        # Step 4: Move to point C (final waypoint in the semicircle)
        point_C = semicircle_waypoints[-1]
        rospy.loginfo(f"Reached point C: {point_C}")

        # Step 5: Move vertically again from point C to point D
        self.current_gps_position = point_C
        point_D = self.move_vertical(self.d1)
        rospy.loginfo(f"Moved vertically to point D: {point_D}")

        # Combine all waypoints for the entire path and store in `self.all_waypoints`
        self.all_waypoints = [initial_position, point_B] + semicircle_waypoints + [point_C, point_D]

    def execute_turn_and_publish(self):
        # Execute turn maneuver once to generate all waypoints
        self.perform_turn()
        rate = rospy.Rate(1)  # Publish at 1 Hz
        while not rospy.is_shutdown():
            # Publish only waypoints ahead of the robot
            self.publish_waypoints_as_path()
            rate.sleep()

def main():
    rospy.init_node('turn_waypoints_generator', anonymous=True)

    buffer_dist = 2
    row_row_width = 3 * 32 * 0.0254
    num_waypoints = 50

    # Initialize turn maneuver with starting GPS coordinates (0, 0)
    turn = Turn(gps=(0, 0), d1=buffer_dist, d2=row_row_width, radius=None, num_waypoints=num_waypoints)
    turn.execute_turn_and_publish()

if __name__ == '__main__':
    main()
