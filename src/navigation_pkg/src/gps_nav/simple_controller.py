#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower_node')

        self.robot_name = 'amiga_model'
        self.cmd_pub = rospy.Publisher('/amiga/cmd_vel', TwistStamped, queue_size=1)

        self.current_pose = None
        self.current_path = []
        self.executing_path = False
        self.logged_path_ids = set()

        # Stores the last measured distance to the current waypoint.  This is
        # used in the control loop to detect when the robot is moving away
        # from a waypoint (i.e. potentially overshooting it). When None, it
        # indicates that a new waypoint has just become active and the
        # distance measurement should be initialised on the next iteration.
        self.last_wp_distance = None

        rospy.Subscriber('/planned_path', Path, self.path_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback)
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("Waypoint Follower Node Started")
        rospy.spin()

    def model_state_callback(self, msg):
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            pose = msg.pose[idx]
            orientation = pose.orientation
            yaw = self.quaternion_to_yaw(orientation)
            self.current_pose = {
                'x': -pose.position.x,
                'y': pose.position.y,
                'yaw': yaw
            }

    def quaternion_to_yaw(self, q):
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def path_callback(self, msg: Path):
        if self.executing_path:
            # rospy.loginfo("Currently executing path. Ignoring new path.")
            return

        path_id = tuple((p.pose.position.x, p.pose.position.y) for p in msg.poses)
        if path_id not in self.logged_path_ids:
            rospy.loginfo("Received new path with %d waypoints:" % len(msg.poses))
            for i, p in enumerate(msg.poses):
                x = p.pose.position.x
                y = p.pose.position.y
                yaw = self.quaternion_to_yaw(p.pose.orientation)
                rospy.loginfo("  Waypoint %d: x=%.2f, y=%.2f, yaw=%.2f deg" % (i, x, y, math.degrees(yaw)))
            self.logged_path_ids.add(path_id)

        self.current_path = msg.poses
        self.executing_path = True
        self.current_wp_index = 0
        # Reset the last waypoint distance whenever a new path is received
        self.last_wp_distance = None

    def control_loop(self, event):
        if not self.executing_path or self.current_pose is None:
            return
        
        if self.current_wp_index >= len(self.current_path):
            rospy.loginfo("Reached final goal pose. Path complete.")
            self.executing_path = False
            # Reset distance tracking for next path
            self.last_wp_distance = None
            self.publish_velocity(0.0, 0.0)
            return

        total_points = len(self.current_path)
        target_pose = self.current_path[self.current_wp_index].pose
        # Compute relative position and yaw error to the target waypoint
        dx = target_pose.position.x - self.current_pose['x']
        dy = target_pose.position.y - self.current_pose['y']
        distance = math.hypot(dx, dy)
        target_yaw = self.quaternion_to_yaw(target_pose.orientation)
        yaw_error = self.normalize_angle(target_yaw - self.current_pose['yaw'])

        # Initialise last_wp_distance for a new waypoint
        if self.last_wp_distance is None:
            self.last_wp_distance = distance

        # ------------------------------------------------------------------
        # Overshoot detection logic
        #
        # If we have a previous waypoint to reference (i.e., current_wp_index > 0),
        # compute the vector from the previous waypoint to the current one and
        # the vector from the current waypoint to the robot. When the dot product
        # of these two vectors is positive and the distance to the waypoint
        # increases compared to the previous iteration, the robot has likely
        # passed the waypoint without entering its acceptance radius.  In that
        # case the waypoint is skipped.  If the two consecutive waypoints share
        # exactly the same position, the segment vector becomes zero and the dot
        # product test is meaningless.  For that special case we instead look
        # for the robot moving away from the waypoint by more than a small
        # margin while still having a significant yaw error.  This prevents the
        # robot from spinning indefinitely on in‑place waypoints when it
        # accidentally moves away.
        if self.current_wp_index > 0:
            prev_pose = self.current_path[self.current_wp_index - 1].pose
            seg_x = target_pose.position.x - prev_pose.position.x
            seg_y = target_pose.position.y - prev_pose.position.y
            rel_x = self.current_pose['x'] - target_pose.position.x
            rel_y = self.current_pose['y'] - target_pose.position.y
            if abs(seg_x) < 1e-6 and abs(seg_y) < 1e-6:
                # Same position as previous waypoint; use distance trend check
                overshoot_margin = 0.05  # 5 cm increase indicates divergence
                if distance > self.last_wp_distance + overshoot_margin and abs(yaw_error) > 0.02:
                    rospy.logwarn(
                        "Waypoint %d appears to be missed (identical position). Skipping to the next waypoint." % self.current_wp_index
                    )
                    self.current_wp_index += 1
                    self.last_wp_distance = None
                    return
            else:
                # Standard finish‑line test using dot product
                dot = seg_x * rel_x + seg_y * rel_y
                if dot > 0 and distance > self.last_wp_distance:
                    rospy.logwarn(
                        "Waypoint %d overshot. Crossing finish line – skipping to the next waypoint." % self.current_wp_index
                    )
                    self.current_wp_index += 1
                    self.last_wp_distance = None
                    return

        # Update last_wp_distance for next iteration
        self.last_wp_distance = distance

        # Control logic
        linear_speed = 0.2 if distance > 0.05 else 0.0
        angular_speed = 1.5 * yaw_error

        # If yaw error is large, rotate in place until aligned
        if abs(yaw_error) >= 0.02:
            linear_speed = 0.0
            print("zero movement. adjust angle first")

        # Publish velocity command
        self.publish_velocity(linear_speed, angular_speed)

        # If distance to final goal is small enough, we can stop
        final_pose = self.current_path[len(self.current_path) - 1].pose
        dx_final = final_pose.position.x - self.current_pose['x']
        dy_final = final_pose.position.y - self.current_pose['y']
        distance_final = math.hypot(dx_final, dy_final)
        if distance_final < 0.22 and abs(yaw_error) < 0.02:
            self.publish_velocity(0.0, 0.0)
            # Mark all waypoints complete
            self.current_wp_index = len(self.current_path)
            # Reset last_wp_distance for next path
            self.last_wp_distance = None
            rospy.loginfo("Final goal achieved without all waypoints. Complete Stop")

        # Otherwise, Log and advance when close enough
        if distance < 0.18 and abs(yaw_error) < 0.02:
            rospy.loginfo(
                "Reached waypoint %d of %d: x=%.2f, y=%.2f, yaw=%.2f deg" %
                (self.current_wp_index, total_points - 1,
                target_pose.position.x, target_pose.position.y,
                math.degrees(target_yaw))
            )

            # Come to complete stop when reached Pre-arrival waypoint
            if self.current_wp_index == (len(self.current_path) - 6):
                self.publish_velocity(0.0, 0.0)
                rospy.loginfo("Pre-arrival waypoint reached. Stop and adjust")
                rospy.sleep(1.0)

             # If this is the final goal
            if self.current_wp_index == (len(self.current_path) - 1) and abs(yaw_error) < 0.02:
                self.publish_velocity(0.0, 0.0)
                rospy.loginfo("Final goal achieved. Complete Stop")

            self.current_wp_index += 1

            if self.current_wp_index < total_points:
                next_pose = self.current_path[self.current_wp_index].pose
                next_yaw = self.quaternion_to_yaw(next_pose.orientation)
                rospy.loginfo(
                    "Approaching waypoint %d of %d: x=%.2f, y=%.2f, yaw=%.2f deg" %
                    (self.current_wp_index, total_points - 1,
                    next_pose.position.x, next_pose.position.y,
                    math.degrees(next_yaw))
                )

    def publish_velocity(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

if __name__ == '__main__':
    try:
        WaypointFollower()
    except rospy.ROSInterruptException:
        pass
