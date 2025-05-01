#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class Waypoints(Node):
    def __init__(self):
        super().__init__('wps_generator_node')
        # Subscribe to the robot heading angle
        self.angle_offset_sub = self.create_subscription(
            Float64,
            '/robot_heading_angle',
            self.angleCB,
            1
        )
        # Subscribe to the row distance
        self.distance_sub = self.create_subscription(
            Float64,
            'row_distance',
            self.distanceCB,
            1
        )
        # Publisher for the generated waypoints path
        self.waypoints_pub = self.create_publisher(
            Path,
            '/waypoints_in_line',
            1
        )

        # Initialize offsets to zero to avoid attribute errors
        self.angle_offset = 0.0
        self.distance_offset = 0.0

    def angleCB(self, msg: Float64):
        """
        Callback for heading angle updates.
        """
        self.angle_offset = msg.data
        self.generate_waypoints()

    def distanceCB(self, msg: Float64):
        """
        Callback for distance updates.
        """
        self.distance_offset = msg.data

    def get_waypoints_on_line(self, spacing=0.1):
        """
        Generate a list of waypoints along a straight line offset by the detected row distance.
        """
        scale = 0.3 / 139.0  # in m/pixel
        distance_offset = self.distance_offset * scale
        start, stop, num = 0.0, 5.0, 10
        x_pts = np.linspace(start, stop, num)
        waypoints = [(x, distance_offset) for x in x_pts]
        return waypoints

    def transform_waypoints_to_robot_frame(self, waypoints):
        """
        Rotate the waypoints by the current heading angle to express them in the robot frame.
        """
        theta = np.deg2rad(self.angle_offset)
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        transformed = []
        for wp in waypoints:
            transformed_wp = R.dot(np.array(wp))
            transformed.append((transformed_wp[0], transformed_wp[1]))
        return transformed

    def generate_waypoints(self):
        """
        Generate, transform, and publish the Path message containing waypoints.
        """
        wps = self.get_waypoints_on_line()
        wps_robot_frame = self.transform_waypoints_to_robot_frame(wps)

        path = Path()
        path.header.frame_id = 'base_footprint'
        # Optionally set timestamp:
        # path.header.stamp = self.get_clock().now().to_msg()

        for x_r, y_r in wps_robot_frame:
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            # pose.header.stamp = path.header.stamp  # if using timestamps
            pose.pose.position.x = x_r
            pose.pose.position.y = y_r
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.get_logger().info(f'Reference wps: {wps_robot_frame}')
        self.waypoints_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
