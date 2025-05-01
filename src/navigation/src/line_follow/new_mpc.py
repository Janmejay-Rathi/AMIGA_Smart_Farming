#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
import time


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # MPC Parameters
        self.N = 10         # Prediction horizon
        self.dt = 1.5       # Time step
        self.L = 0.71       # Wheelbase of the vehicle
        self.ang_max = 0.2
        self.vel_max = 0.2

        # Define state variables
        x     = ca.MX.sym('x')     # Position X
        y     = ca.MX.sym('y')     # Position Y
        theta = ca.MX.sym('theta') # Heading angle

        # Control inputs
        v     = ca.MX.sym('v')     # Linear Velocity
        omega = ca.MX.sym('omega') # Angular velocity

        # State and control vectors
        state    = ca.vertcat(x, y, theta)
        controls = ca.vertcat(v, omega)

        # System dynamics
        rhs = ca.vertcat(
            v * ca.cos(theta),  # x dot
            v * ca.sin(theta),  # y dot
            omega               # theta dot
        )

        # CasADi function for propagation
        self.f = ca.Function('f', [state, controls], [rhs])

        # Storage for incoming waypoints & rate
        self.relative_waypoints = None
        self.waypoints = []
        self.rate_hz = 20
        self.rate = self.create_rate(self.rate_hz)

        # Subscribers / Publishers
        self.amiga_curr_vel_sub = self.create_subscription(
            TwistStamped,
            '/canbus/twist',
            self.velocity_callback,
            1
        )
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/amiga/cmd_vel',
            10
        )
        self.waypoint_sub = self.create_subscription(
            Path,
            '/waypoints_in_line',
            self.callback,
            1
        )

        # Give publishers a moment to connect
        time.sleep(1)

    def callback(self, msg: Path):
        # Extract (x,y) from each PoseStamped
        self.waypoints = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.waypoints.append((x, y))
        self.relative_waypoints = self.waypoints
        self.main()

    def velocity_callback(self, msg: TwistStamped):
        self.current_velocity = msg.twist.linear.x
        self.current_ang_vel  = msg.twist.angular.z

    def mpc_controller_solve(self, ref_path, x0):
        opti = ca.Opti()
        X = opti.variable(3, self.N + 1)  # States over horizon
        U = opti.variable(2, self.N)      # Controls over horizon

        # Cost setup
        cost = 0
        Q = np.diag([50, 50, 0])
        R = np.diag([2, 10])
        for k in range(self.N):
            ref_x, ref_y = ref_path[k]
            ref_theta = 0
            err = X[:, k] - ca.vertcat(ref_x, ref_y, ref_theta)
            cost += ca.mtimes([err.T, Q, err])
            cost += ca.mtimes([U[:, k].T, R, U[:, k]])
        opti.minimize(cost)

        # Dynamics constraints
        for k in range(self.N):
            x_next = X[:, k] + self.dt * self.f(X[:, k], U[:, k])
            opti.subject_to(X[:, k + 1] == x_next)

        # Initial state constraint
        opti.subject_to(X[:, 0] == x0)

        # Control bounds
        opti.subject_to(opti.bounded(0,           U[0, :], self.vel_max))
        opti.subject_to(opti.bounded(-self.ang_max, U[1, :], self.ang_max))

        # Solver options
        p_opts = {"expand": True}
        s_opts = {
            "print_level": 0,
            "sb": "yes",
            "print_timing_statistics": "no",
            "print_user_options": "no"
        }
        opti.solver("ipopt", p_opts, s_opts)

        sol = opti.solve()
        return sol.value(U)

    def visualize(self, v, omega, waypoints):
        plt.figure(figsize=(10, 6))
        x, y, theta = 0, 0, 0
        traj_x = [x]
        traj_y = [y]

        for t in range(self.N):
            # same call as original (assumes vehicle_dynamics is defined elsewhere)
            x, y, theta = self.vehicle_dynamics(x, y, theta, v[t], omega[t])
            traj_x.append(x)
            traj_y.append(y)

        plt.plot(traj_x, traj_y, 'bo-', label='MPC Predicted Trajectory')
        way_np = np.array(waypoints)
        plt.plot(way_np[:, 0], way_np[:, 1], 'ro-', label='Waypoints')
        plt.savefig('/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/mpc_path.jpg')

    def main(self):
        x0 = [0, 0, 0]
        ref_path = self.waypoints
        U = self.mpc_controller_solve(ref_path, x0)
        optimal_vel, optimal_omega = U[0, :], U[1, :]

        twist = TwistStamped()
        twist.header.frame_id = 'robot'

        if optimal_omega is not None:
            twist = TwistStamped()
            twist.header.frame_id = 'robot'
            twist.twist.linear.x  = optimal_vel[0]
            twist.twist.angular.z = optimal_omega[0]
            self.cmd_pub.publish(twist)
            self.get_logger().info(
                f"Published Twist: linear.x={twist.twist.linear.x}, angular.z={twist.twist.angular.z}"
            )

        # maintain loop rate
        self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
