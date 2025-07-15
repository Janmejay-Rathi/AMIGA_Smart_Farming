#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
import atexit

class TwistToWheelController:
    def __init__(self):
        rospy.init_node('twist_to_wheels')

        # Robot physical parameters
        wheel_radius = 0.216   # meters
        wheel_base = 0.81     # meters (distance between left and right wheels)

        self.L = wheel_base
        self.R = wheel_radius
        self.model_name = 'amiga_model'

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
        self.boosted_angular_z = 0.0

        self.latest_gazebo_linear = 0.0
        self.latest_gazebo_angular = 0.0

        self.gazebo_linear_history = []
        self.target_linear_history = []
        self.gazebo_angular_history = []
        self.target_angular_history = []

        # Subscribers
        rospy.Subscriber('/amiga/cmd_vel', TwistStamped, self.cmd_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        rospy.on_shutdown(self.plot_linear_velocity_history)
        rospy.on_shutdown(self.plot_angular_velocity_history)

        rospy.spin()

    def model_states_callback(self, msg):
        if self.model_name in msg.name:
            index = msg.name.index(self.model_name)
            twist = msg.twist[index]
            self.latest_gazebo_linear = (twist.linear.x ** 2 + twist.linear.y ** 2) ** 0.5
            self.latest_gazebo_angular = twist.angular.z

    def boost_power(self, x):
        max_cap = 2.5
        abs_x = abs(x)
        # If x is small, do not scale
        if abs_x < 0.01:
            return x
        
        # k0 = 2.181  # from curve fitting
        # n0 = 0.754  # from curve fitting

        k = 2.181  # from curve fitting
        n = 0.754  # from curve fitting

        a = k / (abs_x ** n)
        # a = 15
        y = a * abs_x
        y_capped = min(y, max_cap)
        return y_capped if x >= 0 else -y_capped
    
    def cmd_callback(self, msg):
        linear_vel = (msg.twist.linear.x ** 2 + msg.twist.linear.y ** 2) ** 0.5
        angular_vel = msg.twist.angular.z

        self.latest_linear_x = linear_vel
        self.latest_angular_z = angular_vel
        self.boosted_angular_z = self.boost_power(angular_vel)

        # Differential drive kinematics
        v_left = (linear_vel - self.boosted_angular_z * self.L / 2.0) / self.R
        v_right = (linear_vel + self.boosted_angular_z * self.L / 2.0) / self.R

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
        output_lines.append(f"Subscribed linear.x: {self.latest_linear_x:.3f}, angular.z: {self.latest_angular_z:.3f}, boosted_angular.z: {self.boosted_angular_z:.3f}\n")

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
                output_lines.append("🔁 Actual motion: TURNING RIGHT")
            elif diff < -0.01:
                output_lines.append("🔄 Actual motion: TURNING LEFT")
            else:
                output_lines.append("⬆️  Actual motion: DRIVING STRAIGHT")

            fl_fr_diff = actual_velocities['fl_wheel_joint'] - actual_velocities['fr_wheel_joint']
            bl_br_diff = actual_velocities['bl_wheel_joint'] - actual_velocities['br_wheel_joint']

            output_lines.append(f"fl - fr = {fl_fr_diff:.3f}   (Front left vs right)")
            output_lines.append(f"bl - br = {bl_br_diff:.3f}   (Back left vs right)")

        output_lines.append("")

        epsilon = 1e-3  # to prevent division by zero or small denominators

        lin_err = abs(self.latest_gazebo_linear - self.latest_linear_x)
        ang_err = abs(self.latest_gazebo_angular - self.latest_angular_z)

        lin_pct = 0.0 if abs(self.latest_linear_x) < epsilon and abs(self.latest_gazebo_linear) < epsilon else 100.0 * lin_err / (abs(self.latest_linear_x) + epsilon)
        ang_pct = 0.0 if abs(self.latest_angular_z) < epsilon and abs(self.latest_gazebo_angular) < epsilon else 100.0 * ang_err / (abs(self.latest_angular_z) + epsilon)

        output_lines.append(f"Gazebo actual linear.x: {self.latest_gazebo_linear:.3f} vs Target: {self.latest_linear_x:.3f}   | Error: {lin_err:.3f} ({lin_pct:.1f}%)")
        output_lines.append(f"Gazebo actual angular.z: {self.latest_gazebo_angular:.3f} vs Target: {self.latest_angular_z:.3f} vs Commanded: {self.boosted_angular_z:.3f}   | Error: {ang_err:.3f} ({ang_pct:.1f}%)")

        self.gazebo_linear_history.append(self.latest_gazebo_linear)
        self.target_linear_history.append(self.latest_linear_x)
        self.gazebo_angular_history.append(self.latest_gazebo_angular)
        self.target_angular_history.append(self.latest_angular_z)

        rospy.loginfo("\n" + "\n".join(output_lines) + "\n" + "-" * 50)

    def plot_linear_velocity_history(self):
        if not self.gazebo_linear_history or not self.target_linear_history:
            return
        script_dir = os.path.dirname(os.path.realpath(__file__))
        parent_dir = os.path.dirname(script_dir)
        data_dir = os.path.join(parent_dir, 'data')
        plt.figure()
        plt.plot(self.gazebo_linear_history, label='Actual Linear Velocity (m/s)')
        plt.plot(self.target_linear_history, label='Target Linear Velocity (m/s)', linestyle='--')
        plt.xlabel('Time Step')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Actual vs Target Linear Velocity')
        plt.legend()
        plt.grid(True)
        plt.savefig(data_dir + '/linear_velocity_comparison.png')
        plt.close()
        print("Figure (linear velocity) has been saved")

    def plot_angular_velocity_history(self):
        if not self.gazebo_angular_history or not self.target_angular_history:
            return
        script_dir = os.path.dirname(os.path.realpath(__file__))
        parent_dir = os.path.dirname(script_dir)
        data_dir = os.path.join(parent_dir, 'data')
        plt.figure()
        plt.plot(self.gazebo_angular_history, label='Actual Angular Velocity (rad/s)')
        plt.plot(self.target_angular_history, label='Target Angular Velocity (rad/s)', linestyle=':')
        plt.xlabel('Time Step')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.title('Actual vs Target Angular Velocity')
        plt.legend()
        plt.grid(True)
        plt.savefig(data_dir + '/angular_velocity_comparison.png')
        plt.close()
        print("Figure (angular velocity) has been saved")

if __name__ == '__main__':
    TwistToWheelController()
