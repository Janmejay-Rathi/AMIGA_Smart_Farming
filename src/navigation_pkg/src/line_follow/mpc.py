#!/usr/bin/env python3
"""
Model Predictive Control (MPC) for robot navigation with ROS integration.
This script:
- Subscribes to waypoints (nav_msgs/Path)
- Solves an MPC optimization problem to follow the waypoints
- Publishes velocity commands (geometry_msgs/TwistStamped) to the robot
- Saves trajectory visualization plots for debugging
"""

import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path


class MPCController:
    def __init__(self, robot_dims, v, N, dt, omega_max):
        """
        Initialize the MPC controller parameters.
        
        Args:
            robot_dims (tuple): (length, width, height) of the robot (currently unused).
            v (float): Linear velocity (constant forward speed of the robot).
            N (int): Horizon length (how many steps into the future MPC predicts).
            dt (float): Time step for prediction (seconds).
            omega_max (float): Maximum allowable angular velocity (rad/s).
        """
        # Robot motion parameters
        self.v_current = v
        self.robot_dims = robot_dims
        self.N = N
        self.dt = dt
        self.omega_max = omega_max

        # Storage for waypoints
        self.relative_waypoints = None
        self.waypoints = []

        # ROS loop frequency (10 Hz)
        self.rate_hz = 10
        self.rate = rospy.Rate(self.rate_hz)

        # Publisher: send velocity commands to robot
        self.cmd_pub = rospy.Publisher("/amiga/cmd_vel", TwistStamped, queue_size=10)

        # Subscriber: listen to waypoints (Path message)
        self.subscriber = rospy.Subscriber('/waypoints_in_line', Path, self.callback, queue_size=1)

        rospy.sleep(1)  # Allow publisher setup time
        rospy.spin()    # Keep node running until shutdown

    def callback(self, msg):
        """
        ROS subscriber callback.
        Called whenever a new Path message is received on `/waypoints_in_line`.

        Args:
            msg (nav_msgs/Path): Contains a list of waypoint poses.
        """
        self.waypoints = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            waypoint = (x, y)
            self.waypoints.append(waypoint)

        # Trigger MPC control whenever new waypoints are received
        self.main()

    def vehicle_dynamics(self, x, y, theta, v, omega):
        """
        Simple vehicle dynamics model using small-angle approximation.
        
        Args:
            x, y (float): Current position
            theta (float): Current heading (radians)
            v (float): Linear velocity (m/s)
            omega (float): Angular velocity (rad/s)

        Returns:
            (x_next, y_next, theta_next): Predicted next state
        """
        x_next = x + v * self.dt               # cos(theta) ≈ 1
        y_next = y + v * theta * self.dt       # sin(theta) ≈ theta
        theta_next = theta + omega * self.dt   # heading update
        return x_next, y_next, theta_next

    def solve_mpc(self, v_current, omega_current):
        """
        Solve MPC optimization problem to compute optimal angular velocity sequence.
        
        Args:
            v_current (float): Linear velocity of the robot.
            omega_current (float): Current angular velocity (not directly used).

        Returns:
            np.ndarray: Optimal angular velocity sequence over prediction horizon.
        """
        if len(self.waypoints) > 0:
            # Convert waypoints to numpy array
            self.relative_waypoints = np.array(self.waypoints)

            # Decision variable: angular velocity sequence
            omega = cp.Variable(self.N)

            # Initialize cost and constraints
            cost = 0
            constraints = []

            # Initial state (robot starts at origin in relative frame)
            x, y, theta = 0, 0, 0
            self.trajectory_x = [x]
            self.trajectory_y = [y]

            # Build MPC problem across the prediction horizon
            for t in range(self.N):
                # Predict next state
                x_next, y_next, theta_next = self.vehicle_dynamics(x, y, theta, v_current, omega[t])

                # Error to corresponding waypoint
                waypoint_x, waypoint_y = self.relative_waypoints[t]
                cost += cp.square(x_next - waypoint_x) + cp.square(y_next - waypoint_y)

                # Update state
                x, y, theta = x_next, y_next, theta_next
                self.trajectory_x.append(x_next)
                self.trajectory_y.append(y_next)

                # Constraint: omega must stay within robot’s physical limits
                constraints += [cp.abs(omega[t]) <= self.omega_max]

            # Solve optimization problem
            problem = cp.Problem(cp.Minimize(cost), constraints)
            problem.solve()

            # Extract optimal angular velocities
            optimal_omega = omega.value
            print("Optimal steering angular velocities (omega):", optimal_omega)

            # Convert predicted trajectory to float values for plotting
            self.trajectory_x = [float(val.value) if hasattr(val, 'value') else float(val) for val in self.trajectory_x]
            self.trajectory_y = [float(val.value) if hasattr(val, 'value') else float(val) for val in self.trajectory_y]

        else:
            # If no waypoints are available, set angular velocity to zero
            optimal_omega = np.zeros(self.N)

        return optimal_omega

    def visualize(self):
        """
        Save a plot showing waypoints vs MPC predicted trajectory.
        Useful for debugging path following performance.
        """
        plt.figure(figsize=(10, 6))

        # Plot waypoints (red)
        plt.plot(self.relative_waypoints[:, 0], self.relative_waypoints[:, 1], 'ro-', label='Waypoints')

        # Plot predicted trajectory (blue)
        plt.plot(self.trajectory_x, self.trajectory_y, 'bo-', label='MPC Predicted Trajectory')

        # Formatting
        plt.title('MPC Control: Vehicle Trajectory vs Waypoints')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid(True)

        # Save figure instead of showing (for logging/debugging)
        plt.savefig('/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/mpc_path.jpg')

    def main(self):
        """
        Main MPC execution loop.
        Called whenever new waypoints are received.
        - Runs MPC optimization
        - Publishes velocity command
        - Logs and visualizes trajectory
        """
        self.omega_current = 0.0  # Placeholder (not directly used in solve_mpc)

        # Solve for optimal angular velocities
        optimal_omega = self.solve_mpc(self.v_current, self.omega_current)

        # Construct and publish velocity command
        if optimal_omega is not None:
            twist = TwistStamped()
            twist.header.frame_id = "robot"
            twist.twist.linear.x = self.v_current                # Constant forward velocity
            twist.twist.angular.z = optimal_omega[0]             # Apply first angular velocity in sequence

            # Publish command
            self.cmd_pub.publish(twist)
            rospy.loginfo(f"Published Twist: linear.x = {twist.twist.linear.x}, angular.z = {twist.twist.angular.z}")

            # Save visualization
            self.visualize()

        # Maintain loop frequency
        self.rate.sleep()


if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('mpc_controller', anonymous=True)
    try:
        # Vehicle parameters
        robot_dims = (53, 20, 23.25)   # Robot size (not used in current dynamics)
        v_current = 0.2                # Constant linear velocity (m/s)
        omega_current = 0.3            # Initial angular velocity (rad/s)
        N = 20                         # Prediction horizon (steps)
        dt = 0.5                       # Time step (s)
        omega_max = 0.2                # Max angular velocity (rad/s)
        rate_hz = 20                   # Control loop frequency (Hz)

        # Initialize MPC Controller (subscribes & starts running)
        mpc_controller = MPCController(robot_dims, v_current, N, dt, omega_max)

    except rospy.ROSInterruptException:
        pass
