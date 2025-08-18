import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path

class MPCController:
    def __init__(self, robot_dims, v, N, dt, omega_max):
        """
        Initialize the MPC Controller.
        
        Args:
            robot_dims (tuple): Dimensions of the robot (length, width, height).
            v (float): Constant linear velocity (m/s).
            N (int): Prediction horizon (number of future steps).
            dt (float): Time step (s).
            omega_max (float): Maximum angular velocity (rad/s).
        """
        self.v_current = v
        self.robot_dims = robot_dims
        self.N = N
        self.dt = dt
        self.omega_max = omega_max
        self.relative_waypoints = None
        self.waypoints = []
        self.rate_hz = 10  # Control loop frequency
        self.rate = rospy.Rate(self.rate_hz)

        # Publisher: sends velocity commands to robot
        self.cmd_pub = rospy.Publisher("/amiga/cmd_vel", TwistStamped, queue_size=10)

        # Subscriber: listens for waypoints (as nav_msgs/Path messages)
        self.subscriber = rospy.Subscriber('/waypoints_in_line', Path, self.callback, queue_size=1)

        # Allow publisher to initialize
        rospy.sleep(1)

        # Keep node alive and responsive
        rospy.spin()

    def callback(self, msg):
        """
        Callback function when new waypoints are received.
        Args:
            msg (Path): Path message containing waypoints.
        """
        path = msg
        self.waypoints = []
        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            waypoint = (x, y)
            self.waypoints.append(waypoint)
        # Call the main control loop
        self.main()

    def vehicle_dynamics(self, x, y, theta, v, omega):
        """
        Vehicle dynamics model (small-angle approximation).
        Args:
            x, y, theta (float): Current state.
            v (float): Linear velocity.
            omega (float): Angular velocity.
        Returns:
            (x_next, y_next, theta_next): Predicted next state.
        """
        x_next = x + v * self.dt       # x update
        y_next = y + v * theta * self.dt  # y update with small-angle approx
        theta_next = theta + omega * self.dt
        return x_next, y_next, theta_next

    def solve_mpc(self, v_current, omega_current):
        """
        Solve the MPC optimization to compute optimal angular velocities.

        Args:
            v_current (float): Current linear velocity.
            omega_current (float): Current angular velocity (unused here).

        Returns:
            np.ndarray: Optimal angular velocity sequence for the horizon.
        """
        if len(self.waypoints) > 0:
            # Convert waypoints to numpy array
            self.relative_waypoints = np.array(self.waypoints)

            # Define decision variable: angular velocity sequence
            omega = cp.Variable(self.N)

            # Define objective (cost function) and constraints
            cost = 0
            constraints = []

            # Initialize robot state at origin
            x, y, theta = 0, 0, 0
            self.trajectory_x = [x]
            self.trajectory_y = [y]

            # Build optimization problem over horizon
            for t in range(self.N):
                # Predict next state
                x_next, y_next, theta_next = self.vehicle_dynamics(x, y, theta, v_current, omega[t])

                # Penalize deviation from desired waypoint
                waypoint_x, waypoint_y = self.relative_waypoints[t]
                cost += cp.square(x_next - waypoint_x) + cp.square(y_next - waypoint_y)

                # Update state for next iteration
                x, y, theta = x_next, y_next, theta_next
                self.trajectory_x.append(x_next)
                self.trajectory_y.append(y_next)

                # Add angular velocity constraints
                constraints += [cp.abs(omega[t]) <= self.omega_max]

            # Solve optimization problem
            problem = cp.Problem(cp.Minimize(cost), constraints)
            problem.solve()

            # Retrieve solution
            optimal_omega = omega.value
            print("Optimal steering angular velocities (omega):", optimal_omega)

            # Ensure trajectory values are floats
            self.trajectory_x = [float(val.value) if hasattr(val, 'value') else float(val) for val in self.trajectory_x]
            self.trajectory_y = [float(val.value) if hasattr(val, 'value') else float(val) for val in self.trajectory_y]

        else:
            # If no waypoints, output zeros
            optimal_omega = np.zeros(self.N)

        return optimal_omega

    def visualize(self):
        """
        Visualize waypoints and predicted trajectory.
        Saves the plot as an image file.
        """
        plt.figure(figsize=(10, 6))

        # Plot waypoints
        plt.plot(self.relative_waypoints[:, 0], self.relative_waypoints[:, 1], 'ro-', label='Waypoints')

        # Plot predicted trajectory
        plt.plot(self.trajectory_x, self.trajectory_y, 'bo-', label='MPC Predicted Trajectory')

        # Labels and legend
        plt.title('MPC Control: Vehicle Trajectory vs Waypoints')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid(True)

        # Save visualization
        plt.savefig('/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/mpc_path.jpg')

    def main(self):
        """
        Main control function:
        - Solves MPC problem.
        - Publishes velocity command.
        - Visualizes trajectory.
        """
        self.omega_current = 0.0  # Placeholder (not used in MPC)
        optimal_omega = self.solve_mpc(self.v_current, self.omega_current)

        twist = TwistStamped()
        twist.header.frame_id = "robot"

        if optimal_omega is not None:
            # Assign linear and angular velocities
            twist.twist.linear.x = self.v_current
            twist.twist.angular.z = optimal_omega[0]  # Use first control input

            # Publish command
            self.cmd_pub.publish(twist)
            rospy.loginfo(f"Published Twist: linear.x={twist.twist.linear.x}, angular.z={twist.twist.angular.z}")

            # Save trajectory visualization
            self.visualize()

        # Maintain loop rate
        self.rate.sleep()

# Main entry point
if __name__ == "__main__":
    rospy.init_node('mpc_controller', anonymous=True)
    try:
        # Define robot and MPC parameters
        robot_dims = (53, 20, 23.25)  # Robot dimensions (unused)
        v_current = 0.2  # Linear velocity (m/s)
        omega_current = 0.3  # Angular velocity (not used directly)
        N = 20  # Prediction horizon
        dt = 0.5  # Time step
        omega_max = 0.2  # Max angular velocity
        rate_hz = 20  # Loop frequency (Hz)

        # Initialize MPC controller
        mpc_controller = MPCController(robot_dims, v_current, N, dt, omega_max)

    except rospy.ROSInterruptException:
        pass
