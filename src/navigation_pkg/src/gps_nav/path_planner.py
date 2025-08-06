#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import os
import math

class WaypointGenerator:
    def __init__(self):
        rospy.init_node('waypoint_generator_node')
        self.robot_name = 'amiga_model'
        self.goal_x = 9.35
        self.goal_y = 7.08
        self.end_pose_angle = 0.0
        self.goal_pose = {'x': -self.goal_x, 'y': self.goal_y, 'yaw': np.deg2rad(self.end_pose_angle)}
        self.waypoint_spacing = 1.0
        self.got_pose = False
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback)
        # rospy.Timer(rospy.Duration(5.0), self.timer_callback)
        self.wait_for_pose_and_publish()
        rospy.loginfo("Waypoint Generator Node Started")
        rospy.spin()

    def model_state_callback(self, msg):
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            pose: Pose = msg.pose[idx]
            orientation = pose.orientation
            self.current_pose = {
                'x': -pose.position.x,
                'y': pose.position.y,
                'yaw': self.quaternion_to_yaw(orientation)
            }
            self.got_pose = True

    def wait_for_pose_and_publish(self):
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for robot pose...")
        while not rospy.is_shutdown() and not self.got_pose:
            rate.sleep()

        rospy.loginfo("Generating and publishing waypoints...")
        waypoints = self.generate_waypoints(self.current_pose, self.goal_pose)
        self.plot_waypoints(waypoints, self.current_pose, self.goal_pose)
        self.publish_path(waypoints)
        rospy.loginfo("Waypoints published and plotted.")

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    # def timer_callback(self, event):
    #     if not self.got_pose:
    #         rospy.loginfo("Waiting for robot pose...")
    #         return

    #     # rospy.loginfo("Generating and publishing waypoints...")
    #     waypoints = self.generate_waypoints(self.current_pose, self.goal_pose)
    #     self.plot_waypoints(waypoints, self.current_pose, self.goal_pose)
    #     self.publish_path(waypoints)

    def generate_waypoints(self, start, goal):
        waypoints = []
        approach_offset = 1.2
        large_approach_offset = 3.5
        extend_offset = 1.7

        # Define pre-goal and post-goal
        pre_goal = goal
        post_goal = goal
        if self.end_pose_angle == 0.0 and goal['y'] >= 0.0:
            pre_goal = {'x': goal['x'], 'y': goal['y'] - approach_offset, 'yaw': goal['yaw']}
            post_goal = {'x': goal['x'], 'y': goal['y'] + extend_offset, 'yaw': goal['yaw']}
        elif self.end_pose_angle == 180.0 and goal['y'] >= 0.0:
            pre_goal = {'x': goal['x'], 'y': goal['y'] + large_approach_offset, 'yaw': goal['yaw']}
            post_goal = {'x': goal['x'], 'y': goal['y'] - extend_offset, 'yaw': goal['yaw']}
        elif self.end_pose_angle == 0.0 and goal['y'] < 0.0:
            pre_goal = {'x': goal['x'], 'y': goal['y'] - large_approach_offset, 'yaw': goal['yaw']}
            post_goal = {'x': goal['x'], 'y': goal['y'] + extend_offset, 'yaw': goal['yaw']}
        elif self.end_pose_angle == 180.0 and goal['y'] < 0.0:
            pre_goal = {'x': goal['x'], 'y': goal['y'] + approach_offset, 'yaw': goal['yaw']}
            post_goal = {'x': goal['x'], 'y': goal['y'] - extend_offset, 'yaw': goal['yaw']}

        # Initial and adjusted departure orientation
        wp0 = {'x': start['x'], 'y': start['y'], 'yaw': start['yaw']}
        wp1 = {'x': start['x'], 'y': start['y'], 'yaw': self.compute_heading(start, pre_goal)}
        waypoints.extend([wp0])

        # Interpolated midpoints
        heading = wp1['yaw']
        dx = pre_goal['x'] - wp1['x']
        dy = pre_goal['y'] - wp1['y']
        distance = math.hypot(dx, dy)
        num_segments = max(1, int(distance / self.waypoint_spacing))

        for i in range(1, num_segments):
            ratio = i / num_segments
            xi = wp1['x'] + dx * ratio
            yi = wp1['y'] + dy * ratio
            waypoints.append({'x': xi, 'y': yi, 'yaw': heading})

        # Pre-arrival adjustment
        wp2 = {'x': pre_goal['x'], 'y': pre_goal['y'], 'yaw': heading}
        wp3 = pre_goal
        waypoints.extend([wp2, wp3])

        # waypoints.extend([goal, post_goal])

        # Interpolated alignment points
        heading_a = pre_goal['yaw']
        dx_a = post_goal['x'] - pre_goal['x']
        dy_a = post_goal['y'] - pre_goal['y']
        num_segments_a = 5

        for k in range(1, num_segments_a):
            ratio_a = k / num_segments_a
            xi_a = wp3['x'] + dx_a * ratio_a
            yi_a = wp3['y'] + dy_a * ratio_a
            waypoints.append({'x': xi_a, 'y': yi_a, 'yaw': heading_a})

        return waypoints

    def compute_heading(self, start, goal):
        dx = goal['x'] - start['x']
        dy = goal['y'] - start['y']
        return np.arctan2(dx, dy)  # yaw=0 is +Y

    # def plot_waypoints(self, waypoints, start, goal):
    #     xs = [wp['x'] for wp in waypoints]
    #     ys = [wp['y'] for wp in waypoints]
    #     yaws = [wp['yaw'] for wp in waypoints]

    #     plt.figure()
    #     plt.plot(xs, ys, 'k:', label='Path')
    #     plt.plot(xs, ys, 'ko')  # black dots

    #     arrow_length = 0.3  # same as before
    #     for i, (x, y, yaw) in enumerate(zip(xs, ys, yaws)):
    #         dx = arrow_length * np.sin(yaw)
    #         dy = arrow_length * np.cos(yaw)

    #         if i == 0:
    #             arrow_color = 'green'  # Start arrow
    #         elif i == len(xs) - 1:
    #             arrow_color = 'red'    # Goal arrow
    #         else:
    #             arrow_color = 'blue'   # Intermediate arrows

    #         plt.arrow(x, y, dx, dy, head_width=0.1, color=arrow_color)

    #     plt.plot(start['x'], start['y'], 'go', markersize=10, label='Start')
    #     plt.plot(goal['x'], goal['y'], 'ro', markersize=10, label='Goal')
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     plt.grid(True)
    #     plt.legend()
    #     plt.axis('equal')

    #     # === Auto-padding to avoid arrow clipping ===
    #     pad = 0.5
    #     x_all = [wp['x'] for wp in waypoints]
    #     y_all = [wp['y'] for wp in waypoints]
    #     plt.xlim(min(x_all) - pad, max(x_all) + pad)
    #     plt.ylim(min(y_all) - pad, max(y_all) + pad)

    #     # === Save plot ===
    #     save_path = '/home/ken/AMIGA_onboard/src/navigation_pkg/data/planned_path.png'
    #     os.makedirs(os.path.dirname(save_path), exist_ok=True)
    #     plt.savefig(save_path)
    #     plt.close()

    def plot_waypoints(self, waypoints, start, goal):
        xs = [wp['x'] for wp in waypoints]
        ys = [wp['y'] for wp in waypoints]
        yaws = [wp['yaw'] for wp in waypoints]

        plt.figure()
        plt.plot(xs, ys, 'k:', label='Path')
        plt.plot(xs, ys, 'ko')  # black dots

        arrow_length = 0.3
        for i, (x, y, yaw) in enumerate(zip(xs, ys, yaws)):
            dx = arrow_length * np.sin(yaw)
            dy = arrow_length * np.cos(yaw)

            if i == 0:
                arrow_color = 'green'
            elif i == len(xs) - 3:
                arrow_color = 'red'
            else:
                arrow_color = 'blue'

            plt.arrow(x, y, dx, dy, head_width=0.1, color=arrow_color)

        plt.plot(start['x'], start['y'], 'go', markersize=10, label='Start')
        plt.plot(goal['x'], goal['y'], 'ro', markersize=10, label='Goal')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')

        # === Auto-padding and axis scaling ===
        pad = 1.0
        x_all = [wp['x'] for wp in waypoints]
        y_all = [wp['y'] for wp in waypoints]
        plt.xlim(min(x_all) - pad, max(x_all) + pad)
        plt.ylim(min(y_all) - pad, max(y_all) + pad)

        # Reverse X axis
        plt.gca().invert_xaxis()

        save_path = '/home/ken/AMIGA_onboard/src/navigation_pkg/data/planned_path.png'
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path)
        plt.close()


    def publish_path(self, waypoints):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.0
            q = self.yaw_to_quaternion(wp['yaw'])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def yaw_to_quaternion(self, yaw):
        qx = 0
        qy = 0
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        return (qx, qy, qz, qw)

if __name__ == '__main__':
    try:
        WaypointGenerator()
    except rospy.ROSInterruptException:
        pass
