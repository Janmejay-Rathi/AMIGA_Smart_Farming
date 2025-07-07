#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

class TwistToWheelController:
    def __init__(self):
        rospy.init_node('twist_to_wheels')

        # Robot physical parameters
        wheel_radius = 0.3   # meters
        wheel_base = 1.0     # meters (distance between left and right wheels)

        self.L = wheel_base
        self.R = wheel_radius

        # Publishers mapped to correct wheel sides
        self.pubs = {
            "bl": rospy.Publisher('/bl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "fl": rospy.Publisher('/fl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "br": rospy.Publisher('/br_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "fr": rospy.Publisher('/fr_wheel_joint_velocity_controller/command', Float64, queue_size=1),
        }

        # Storage for latest command velocities
        self.commanded_velocities = {
            'bl_wheel_joint': 0.0,
            'fl_wheel_joint': 0.0,
            'br_wheel_joint': 0.0,
            'fr_wheel_joint': 0.0,
        }

        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0

        # Subscribers
        rospy.Subscriber('/amiga/cmd_vel', TwistStamped, self.cmd_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        rospy.spin()

    def cmd_callback(self, msg):
        linear_vel = msg.twist.linear.x
        angular_vel = msg.twist.angular.z * 35

        self.latest_linear_x = linear_vel
        self.latest_angular_z = angular_vel

        # Differential drive kinematics
        v_left = (linear_vel - angular_vel * self.L / 2.0) / self.R
        v_right = (linear_vel + angular_vel * self.L / 2.0) / self.R

        # Update commanded values
        self.commanded_velocities['bl_wheel_joint'] = v_left
        self.commanded_velocities['fl_wheel_joint'] = v_left
        self.commanded_velocities['br_wheel_joint'] = v_right
        self.commanded_velocities['fr_wheel_joint'] = v_right

        # Publish commands to correct wheels
        self.pubs['bl'].publish(v_left)
        self.pubs['fl'].publish(v_left)
        self.pubs['br'].publish(v_right)
        self.pubs['fr'].publish(v_right)

    def joint_state_callback(self, msg):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        output_lines = []

        # Log command input
        output_lines.append(f"Subscribed linear.x: {self.latest_linear_x:.3f}, angular.z: {self.latest_angular_z:.3f}\n")

        actual_velocities = {}

        for joint, commanded in self.commanded_velocities.items():
            if joint in name_to_index:
                actual = msg.velocity[name_to_index[joint]]
                actual_velocities[joint] = actual
                error = abs(commanded - actual)
                output_lines.append(f"{joint} -> Commanded: {commanded:.3f}, Actual: {actual:.3f}, |Error|: {error:.3f}")

        # Determine turning direction based on actual wheel velocities
        if all(j in actual_velocities for j in ['fl_wheel_joint', 'fr_wheel_joint', 'bl_wheel_joint', 'br_wheel_joint']):
            left_avg = (actual_velocities['fl_wheel_joint'] + actual_velocities['bl_wheel_joint']) / 2.0
            right_avg = (actual_velocities['fr_wheel_joint'] + actual_velocities['br_wheel_joint']) / 2.0
            diff = left_avg - right_avg

            output_lines.append("")
            if diff > 0.01:
                output_lines.append("🔄 Actual motion: TURNING LEFT")
            elif diff < -0.01:
                output_lines.append("🔁 Actual motion: TURNING RIGHT")
            else:
                output_lines.append("⬆️ Actual motion: DRIVING STRAIGHT")

            fl_fr_diff = actual_velocities['fl_wheel_joint'] - actual_velocities['fr_wheel_joint']
            bl_br_diff = actual_velocities['bl_wheel_joint'] - actual_velocities['br_wheel_joint']

            output_lines.append(f"fl - fr = {fl_fr_diff:.3f}   (Front left vs right)")
            output_lines.append(f"bl - br = {bl_br_diff:.3f}   (Back left vs right)")

        rospy.loginfo("\n" + "\n".join(output_lines) + "\n" + "-" * 50)

if __name__ == '__main__':
    TwistToWheelController()
