import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path

class MPCController:
    def __init__(self, robot_dims, v, N, dt, omega_max):
        # Initialize vehicle parameters
        self.v_current = v
        self.robot_dims = robot_dims  # (length, width, height)
        self.N = N  # Horizon length
        self.dt = dt  # Time step
        self.omega_max = omega_max  # Max angular velocity
        self.relative_waypoints = None
        self.waypoints = []
        self.rate_hz = 10  # Frequency in Hz (sending commands at 10 Hz)
        self.rate = rospy.Rate(self.rate_hz)

        # print("inititalias")
        self.cmd_pub = rospy.Publisher("/amiga/cmd_vel", TwistStamped, queue_size=10)
        # Subscribe to the /waypoint topic, expecting messages of type geometry_msgs/Point
        self.subscriber = rospy.Subscriber('/local_waypoints_turning', Path, self.callback,queue_size=1)


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
        self.main()
    

    def vehicle_dynamics(self, x, y, theta, v, omega):
        # Vehicle dynamics (small angle approximation)
        x_next = x + v * self.dt  # cos(θ) ≈ 1
        y_next = y + v * theta * self.dt  # sin(θ) ≈ θ
        theta_next = theta + omega * self.dt
        return x_next, y_next, theta_next
    
    def solve_mpc(self, v_current, omega_current):
        """
        Solve for optimal angular velocity
        Input:
        - v_current: current linear velocity
        - omega_current: current angular velocity
        - waypoints: target path (list of [x, y] waypoints)

        Output:
        - optimal_omega: optimal angular velocity sequence
        
        The mpc solver will predict the future 10 points to minimize the predicted position to its corresponding waypoint.
        """
        if len(self.waypoints)>0:
            # Convert waypoints to numpy array
            self.relative_waypoints = np.array(self.waypoints)

            # Decision variable for angular velocity
            omega = cp.Variable(self.N)

            # Initialize objective and constraints
            cost = 0
            constraints = []

            # Initialize state variables
            x, y, theta = 0, 0, 0  # Start at origin
            self.trajectory_x = [x]
            self.trajectory_y = [y]

            # Build optimization problem
            for t in range(self.N):
                # Predict next state
                x_next, y_next, theta_next = self.vehicle_dynamics(x, y, theta, v_current, omega[t])

                # Calculate error from waypoints
                waypoint_x, waypoint_y = self.relative_waypoints[t]
                cost += cp.square(x_next - waypoint_x) + cp.square(y_next - waypoint_y)

                # Update state
                x, y, theta = x_next, y_next, theta_next
                self.trajectory_x.append(x_next)
                self.trajectory_y.append(y_next)

                # Add constraints for angular velocity
                constraints += [cp.abs(omega[t]) <= self.omega_max]

            # Solve the optimization problem
            problem = cp.Problem(cp.Minimize(cost), constraints)
            problem.solve()

            # Retrieve the optimal angular velocity sequence
            optimal_omega = omega.value
            print("Optimal steering angular velocities (omega):", optimal_omega)

            # Convert trajectory values to float
            self.trajectory_x = [float(val.value) if hasattr(val, 'value') else float(val) for val in self.trajectory_x]
            self.trajectory_y = [float(val.value) if hasattr(val, 'value') else float(val) for val in self.trajectory_y]

        else: 
            optimal_omega = np.zeros(self.N)

        return optimal_omega

    def visualize(self):
        # Visualize the trajectory and waypoints
        plt.figure(figsize=(10, 6))
        # print(self.relative_waypoints)
        # Plot waypoints
        plt.plot(self.relative_waypoints[:, 0], self.relative_waypoints[:, 1], 'ro-', label='Waypoints')

        # Plot predicted trajectory
        plt.plot(self.trajectory_x, self.trajectory_y, 'bo-', label='MPC Predicted Trajectory')

        # Labels and legend
        plt.title('MPC Control: Vehicle Trajectory vs Waypoints')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()

        # Show plot
        plt.grid(True)
        # plt.show()
        plt.savefig('/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/mpc_path.jpg')


    def main(self):
        self.omega_current = 0.0 #Dummy variable, not used in MPC solver
        # Solve for optimal angular velocity


        # print("ASSAS")
        optimal_omega = self.solve_mpc(self.v_current, self.omega_current)
        twist = TwistStamped()
        twist.header.frame_id = "robot"

        if optimal_omega is not None:
            # The first angular velocity will be used for the next time step
            # Create a Twist message to publish the velocities
            twist = TwistStamped()
            twist.header.frame_id = "robot"
            twist.twist.linear.x = v_current  # Constant linear velocity 0.2 m/s
            twist.twist.angular.z = optimal_omega[0]  # Use the first angular velocity from the optimization 
            # twist.twist.angular.z = 0.0

            # Publish the velocity command to the robot
            self.cmd_pub.publish(twist)
            rospy.loginfo(f"Published Twist message: linear.x = {twist.twist.linear.x}, angular.z = {twist.twist.angular.z}")
            self.visualize()
        
        # Sleep to maintain the loop rate
        self.rate.sleep()
        
# Example usage
# if __name__ == "__main__":
#     # Vehicle parameters
#     robot_dims = (53, 20, 23.25)  # Length, width, height, currently not used
#     v_current = 10  # Current linear velocity
#     omega_current = 0.5  # Current angular velocity
#     N = 10  # Horizon length
#     dt = 0.1  # Time step
#     omega_max = np.pi / 4  # Max angular velocity

#     # Initialize MPC controller
#     mpc_controller = MPCController(robot_dims, v_current, N, dt, omega_max)

#     # Define waypoints
#     waypoints = [[0, 1], [1, 1], [2, 1], [3, 1], [4, 1], [5, 1], [6, 1], [7, 1], [8, 1], [9, 1]]
    
#     # Solve for optimal angular velocity
#     optimal_omega = mpc_controller.solve_mpc(v_current, omega_current, waypoints)

#     # Visualize the result
#     mpc_controller.visualize()

if __name__ == "__main__":
    # Initialize ROS publisher
    rospy.init_node('mpc_controller', anonymous=True)
    try:
        # print("Starat")
        # Vehicle parameters
        robot_dims = (53, 20, 23.25)  # Length, width, height (unused in current model)
        v_current = 0.2  # Constant linear velocity
        omega_current = 0.3  # Current angular velocity
        N = 20  # Prediction horizon
        dt = 0.5  # Time step= horizon_length/vel/N
        omega_max = 0.2  # Max angular velocity
        rate_hz = 20  # Frequency in Hz (sending commands at 10 Hz)

        # Initialize MPC controller
        mpc_controller = MPCController(robot_dims, v_current, N, dt, omega_max)
        # mpc_controller.main()

    except rospy.ROSInterruptException:
        pass