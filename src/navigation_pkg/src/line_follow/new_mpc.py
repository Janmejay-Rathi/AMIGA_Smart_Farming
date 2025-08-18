#!/usr/bin/env python

"""
MPC Controller Node (ROS1, Python, CasADi)

This script implements a Model Predictive Controller (MPC) for path tracking.
It subscribes to waypoints, computes optimal linear and angular velocities, and
publishes them as TwistStamped commands to control a mobile robot.

Key Features:
- Uses CasADi for optimization with nonlinear dynamics.
- Handles linear velocity and angular velocity as control inputs.
- Receives path waypoints from ROS and computes optimal controls to follow.
- Publishes velocity commands at a fixed frequency.
"""

import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import casadi as ca


class MPCController:
    def __init__(self):

        # =============================
        # MPC Parameters
        # =============================
        self.N = 10        # Prediction horizon (number of future steps to optimize over)
        self.dt = 1.5      # Time step for discretization [s]
        self.L = 0.71      # Wheelbase of the vehicle [m] (not used here since omega is input)
        self.ang_max = 0.2 # Maximum angular velocity [rad/s]
        self.vel_max = 0.2 # Maximum linear velocity [m/s]

        # =============================
        # Define symbolic state variables
        # =============================
        x = ca.MX.sym('x')        # Position X
        y = ca.MX.sym('y')        # Position Y
        theta = ca.MX.sym('theta')# Heading angle (yaw)

        # Control inputs
        v = ca.MX.sym('v')        # Linear velocity
        omega = ca.MX.sym('omega')# Angular velocity

        # State and control vectors
        state = ca.vertcat(x, y, theta)
        controls = ca.vertcat(v, omega)

        # =============================
        # System dynamics (unicycle model)
        # =============================
        rhs = ca.vertcat(
            v * ca.cos(theta),  # dx/dt
            v * ca.sin(theta),  # dy/dt
            omega               # dtheta/dt
        )

        # CasADi function for system dynamics
        self.f = ca.Function('f', [state, controls], [rhs])
        
        # =============================
        # ROS Communication Setup
        # =============================
        self.relative_waypoints = None
        self.waypoints = []

        self.rate_hz = 20              # Publish rate [Hz]
        self.rate = rospy.Rate(self.rate_hz)

        # Subscribing to robot's current velocity (from CAN bus)
        self.amiga_curr_vel_sub = rospy.Subscriber(
            "/canbus/twist", TwistStamped, self.velocity_callback, queue_size=1
        )

        # Publishing velocity commands to the robot
        self.cmd_pub = rospy.Publisher(
            "/amiga/cmd_vel", TwistStamped, queue_size=10
        )

        # Subscribing to waypoints (as a Path message)
        self.subscriber = rospy.Subscriber(
            '/waypoints_in_line', Path, self.callback, queue_size=1
        )

        # Give ROS publishers time to initialize
        rospy.sleep(1)

        # Keep node running
        rospy.spin()

    # =============================
    # ROS Callback: Waypoints
    # =============================
    def callback(self, msg):
        """
        Callback function to receive waypoints from /waypoints_in_line.
        Extracts (x,y) positions and stores them for MPC.
        """
        self.waypoints = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            waypoint = (x, y)
            self.waypoints.append(waypoint)

        # Store waypoints for relative reference
        self.relative_waypoints = self.waypoints

        # Run MPC after receiving waypoints
        self.main()
    
    # =============================
    # ROS Callback: Current Velocity
    # =============================
    def velocity_callback(self, msg):
        """
        Callback to update the current linear and angular velocity of the robot.
        """
        self.current_velocity = msg.twist.linear.x
        self.current_ang_vel = msg.twist.angular.z

    # =============================
    # MPC Solver
    # =============================
    def mpc_controller_solve(self, ref_path, x0):
        """
        Solves the MPC optimization problem for path tracking.

        :param ref_path: Reference waypoints [(x, y), ...]
        :param x0: Initial state [x, y, theta]
        :return: Optimal velocity and angular velocity arrays
        """
        opti = ca.Opti()

        # Decision variables
        X = opti.variable(3, self.N + 1)  # States: [x, y, theta] over horizon
        U = opti.variable(2, self.N)      # Controls: [v, omega] over horizon
        
        # Objective function (tracking cost + control effort cost)
        cost = 0
        Q = np.diag([50, 50, 0])   # Weight matrix for state error
        R = np.diag([2, 10])       # Weight matrix for control effort
        
        for k in range(self.N):
            ref_x, ref_y = ref_path[k]   # Reference waypoint
            ref_theta = 0               # Assume flat heading
            state_error = X[:, k] - ca.vertcat(ref_x, ref_y, ref_theta)

            # Penalize deviation from reference
            cost += ca.mtimes([state_error.T, Q, state_error])

            # Penalize control effort
            cost += ca.mtimes([U[:, k].T, R, U[:, k]])
        
        opti.minimize(cost)
        
        # Dynamics constraints
        for k in range(self.N):
            x_next = X[:, k] + self.dt * self.f(X[:, k], U[:, k])
            opti.subject_to(X[:, k + 1] == x_next)
        
        # Initial condition
        opti.subject_to(X[:, 0] == x0)
        
        # Control input constraints
        opti.subject_to(opti.bounded(0, U[0, :], self.vel_max))       # Velocity limits
        opti.subject_to(opti.bounded(-self.ang_max, U[1, :], self.ang_max)) # Angular velocity limits

        # Solver options
        p_opts = {"expand": True}
        s_opts = {"print_level": 0, "sb": "yes",
                  "print_timing_statistics": "no",
                  "print_user_options": "no"}
        
        opti.solver("ipopt", p_opts, s_opts)
        
        # Solve optimization problem
        sol = opti.solve()
        
        return sol.value(U)  # Return [v, omega] sequences

    # =============================
    # Trajectory Visualization
    # =============================
    def visualize(self, v, omega, waypoints):
        """
        Visualizes the predicted trajectory vs. reference waypoints.
        Saves the plot to a .jpg file.
        """
        plt.figure(figsize=(10, 6))

        # Initialize robot state
        x, y, theta = 0, 0, 0
        trajectory_x = [x]
        trajectory_y = [y]

        # Simulate trajectory with chosen controls
        for t in range(self.N):
            x, y, theta = self.vehicle_dynamics(x, y, theta, v[t], omega[t])
            trajectory_x.append(x)
            trajectory_y.append(y)

        # Plot predicted trajectory
        plt.plot(trajectory_x, trajectory_y, 'bo-', label='MPC Predicted Trajectory')

        # Plot reference waypoints
        waypoints = np.array(waypoints)
        plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoints')

        plt.legend()
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("MPC Path Tracking")
        plt.savefig('/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/mpc_path.jpg')

    # =============================
    # Main MPC Execution
    # =============================
    def main(self):
        """
        Main function to run MPC once waypoints are received.
        Computes optimal velocities and publishes them as Twist messages.
        """
        x0 = [0, 0, 0]  # Initial state (assuming robot starts at origin)
        ref_path = self.waypoints

        # Solve MPC optimization
        u = self.mpc_controller_solve(ref_path, x0)
        optimal_vel, optimal_omega = u[0, :], u[1, :]

        # Create TwistStamped message
        twist = TwistStamped()
        twist.header.frame_id = "robot"

        if optimal_omega is not None:
            # Apply first control input in sequence
            twist.twist.linear.x = optimal_vel[0]
            twist.twist.angular.z = optimal_omega[0]

            # Publish velocity command
            self.cmd_pub.publish(twist)
            rospy.loginfo(
                f"Published Twist: linear.x = {twist.twist.linear.x}, angular.z = {twist.twist.angular.z}"
            )
        
        # Maintain loop rate
        self.rate.sleep()


# =============================
# Main Entry Point
# =============================
if __name__ == "__main__":
    rospy.init_node('mpc_controller', anonymous=True)
    try:
        mpc_controller = MPCController()
    except rospy.ROSInterruptException:
        pass
