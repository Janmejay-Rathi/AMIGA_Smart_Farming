#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import casadi as ca

class MPCController:
    def __init__(self):

        # MPC Parameters
        self.N = 10  # Prediction horizon
        self.dt = 1.5  # Time step
        self.L = 0.71  # Wheelbase of the vehicle
        self.ang_max = 0.2
        self.vel_max = 0.2

        # Define state variables
        x = ca.MX.sym('x')   # Position X
        y = ca.MX.sym('y')   # Position Y
        theta = ca.MX.sym('theta')  # Heading angle

        # Control inputs
        v = ca.MX.sym('v')  # Linear Velocity
        omega = ca.MX.sym('omega')  # Angular velocity

        # State and control vectors
        state = ca.vertcat(x, y, theta)
        controls = ca.vertcat(v, omega)  # Controls: linear velocity and angular velocity

        # System dynamics (motion model with angular velocity control)
        rhs = ca.vertcat(
            v * ca.cos(theta),  # x dot
            v * ca.sin(theta),  # y dot
            omega  # theta dot (angular velocity directly controls the angular velocity)
        )

        # Create a function for state propagation
        self.f = ca.Function('f', [state, controls], [rhs])
        
        self.relative_waypoints = None
        self.waypoints = []
        self.rate_hz = 20  # Frequency in Hz (sending commands at 10 Hz)
        self.rate = rospy.Rate(self.rate_hz)

        self.amiga_curr_vel_sub = rospy.Subscriber("/canbus/twist", TwistStamped, self.velocity_callback,queue_size=1)
        self.cmd_pub = rospy.Publisher("/amiga/cmd_vel", TwistStamped, queue_size=10)
        # Subscribe to the /waypoint topic, expecting messages of type geometry_msgs/Point
        self.subscriber = rospy.Subscriber('/waypoints_in_line', Path, self.callback,queue_size=1)


        rospy.sleep(1)  # Allow time for the publisher to set up
        rospy.spin()

    def callback(self, msg):
        path = msg
        # print("CBas")
        self.waypoints = []
        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = 0.5
            # Store the received waypoint in the waypoints list
            waypoint = (x,y)
            self.waypoints.append(waypoint)
        # rospy.loginfo(f"Received waypoint: {self.waypoints}")
        self.relative_waypoints = self.waypoints
        self.main()
    
    def velocity_callback(self, msg):
        self.current_velocity = msg.twist.linear.x
        self.current_ang_vel = msg.twist.angular.z


    def mpc_controller_solve(self,ref_path, x0):
        """
        Model Predictive Control (MPC) for path tracking.
        :param ref_path: List of reference waypoints [(x, y, theta_ref)]
        :param x0: Initial state [x, y, theta]
        :return: Optimal velocity and angular velocity
        """
        opti = ca.Opti()
        # Decision variables
        X = opti.variable(3, self.N + 1)  # States [x, y, theta]
        U = opti.variable(2, self.N)  # Controls [v, omega]
        
        # Objective function (cost)
        cost = 0
        Q = np.diag([50, 50, 0])  # State error cost
        R = np.diag([2,10])  # Control effort cost
        
        for k in range(self.N):
            ref_x, ref_y = ref_path[k]
            ref_theta = 0
            state_error = X[:, k] - ca.vertcat(ref_x, ref_y, ref_theta)
            cost += ca.mtimes([state_error.T, Q, state_error])  # State error cost
            cost += ca.mtimes([U[:, k].T, R, U[:, k]])  # Control effort cost

            # print(f"Cost: {cost}")
        
        # Terminal Cost
        # ref_x, ref_y = ref_path[self.N]  # Or ref_path[N] if length matches
        # ref_theta = 0
        # terminal_error = X[:, self.N] - ca.vertcat(ref_x, ref_y, ref_theta)
        # cost += ca.mtimes([terminal_error.T, Q, terminal_error])
        
        opti.minimize(cost)
        
        # Constraints
        for k in range(self.N):
            x_next = X[:, k] + self.dt * self.f(X[:, k], U[:, k])  # State propagation
            opti.subject_to(X[:, k + 1] == x_next)
        
        # Initial condition constraint
        opti.subject_to(X[:, 0] == x0)
        
        # Control limits
        opti.subject_to(opti.bounded(0, U[0, :], self.vel_max))  # Vel [min, max] m/s
        opti.subject_to(opti.bounded(-self.ang_max, U[1, :], self.ang_max))  # Angular velocity [min, max] rad/s

        p_opts = {"expand": True}  # CasADi-level options
        s_opts = {
            "print_level": 0,       # IPOPT prints almost nothing
            "sb": "yes",            # Suppress IPOPT banner
            "print_timing_statistics": "no",
            "print_user_options": "no"}
        
        # Solver settings
        opti.solver("ipopt", p_opts, s_opts)
        
        # Solve optimization problem
        sol = opti.solve()
        
        # Return first control action (velocity and angular velocity)
        return sol.value(U)



    def visualize(self, v, omega, waypoints):
        # Visualize the predicted trajectory compared to the waypoints
        plt.figure(figsize=(10, 6))

        # Starting state
        x, y, theta = 0, 0, 0
        trajectory_x = [x]
        trajectory_y = [y]

        # Apply dynamics and simulate the trajectory
        for t in range(self.N):
            x, y, theta = self.vehicle_dynamics(x, y, theta, v[t], omega[t])
            trajectory_x.append(x)
            trajectory_y.append(y)

        # Plot the predicted trajectory
        plt.plot(trajectory_x, trajectory_y, 'bo-', label='MPC Predicted Trajectory')

        # Convert waypoints to numpy array
        waypoints = np.array(waypoints)
        plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoints')
        plt.savefig('/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/mpc_path.jpg')

    def main(self):

        x0 = [0,0,0]
        ref_path = self.waypoints
        u = self.mpc_controller_solve(ref_path, x0)
        optimal_vel, optimal_omega = u[0,:] , u[1,:]

        twist = TwistStamped()
        twist.header.frame_id = "robot"

        if optimal_omega is not None:
            # The first angular velocity will be used for the next time step
            # Create a Twist message to publish the velocities
            twist = TwistStamped()
            twist.header.frame_id = "robot"
            twist.twist.linear.x = optimal_vel[0]  # Constant linear velocity 0.2 m/s
            twist.twist.angular.z = optimal_omega[0]  # Use the first angular velocity from the optimization 
            # twist.twist.angular.z = 0.0

            # Publish the velocity command to the robot
            self.cmd_pub.publish(twist)
            rospy.loginfo(f"Published Twist message: linear.x = {twist.twist.linear.x}, angular.z = {twist.twist.angular.z}")
            # self.visualize(optimal_vel,optimal_omega,waypoints)
        
        # Sleep to maintain the loop rate
        self.rate.sleep()
        


if __name__ == "__main__":
    # Initialize ROS publisher
    rospy.init_node('mpc_controller', anonymous=True)
    try:
        # Initialize MPC controller
        mpc_controller = MPCController()

    except rospy.ROSInterruptException:
        pass