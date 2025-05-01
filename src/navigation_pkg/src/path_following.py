import numpy as np
import matplotlib.pyplot as plt
import cv2

class PathFollower:
    def __init__(self, image_width, image_height):
        self.image_width = image_width
        self.image_height = image_height
        self.image_center = (image_width // 2, image_height // 2)
        print(f"Image center is: {self.image_center}")
        self.lines = []
        self.theta = 0

    def set_lines(self, lines):
        self.lines = lines
        print(f"Lines set: {self.lines}")

    def set_theta(self, theta_value):
        self.theta = theta_value
        print(f"Theta value is set to: {np.degrees(self.theta)} degrees")

    def find_nearest_line(self):
        min_distance = float('inf')
        nearest_line = None
        for line in self.lines:
            slope, intercept = line
            # Calculate distance from the center of the image (x = image_center[0]) to the line y = mx + b
            distance = abs(slope * self.image_center[0] - self.image_center[1] + intercept) / np.sqrt(slope**2 + 1)
            if distance < min_distance:
                min_distance = distance
                nearest_line = line
        return nearest_line

    def get_waypoints_on_line(self, spacing= 1):
        waypoints = [(0, y) for y in range(0, 10*spacing, spacing)]
        return waypoints

    def transform_waypoints_to_robot_frame(self, waypoints):
        rotation_matrix = np.array([[np.cos(self.theta), -np.sin(self.theta)], 
                                    [np.sin(self.theta), np.cos(self.theta)]])
        transformed_waypoints = []
        for waypoint in waypoints:
            transformed_wp = np.dot(rotation_matrix, np.array(waypoint))
            transformed_waypoints.append(tuple(transformed_wp))
        return transformed_waypoints
    
    def plot_waypoints_on_image(self, image, waypoints):
        plt.imshow(image)
        plt.gca().invert_yaxis()

        x_coords = [wp[0] + self.image_center[0] for wp in waypoints]
        y_coords = [self.image_height - wp[1] for wp in waypoints]

        plt.scatter(x_coords, y_coords, color='red', label='Waypoints')
        plt.legend()
        plt.show()
            
    
    def plot_waypoints(self, original_waypoints, adjusted_waypoints):
            x_orig, y_orig = zip(*original_waypoints)
            x_adj, y_adj = zip(*adjusted_waypoints)

            plt.scatter(x_orig, y_orig, color='blue', label='Original Waypoints')

            plt.scatter(x_adj, y_adj, color='red', label='Adjusted Waypoints')

            plt.xlabel('X (Robot Local Frame)')
            plt.ylabel('Y (Robot Local Frame)')
            plt.legend()
            plt.title('Waypoints Before and After Alignment with Real Line')
            plt.grid(True)
            plt.show()
        
        
if __name__ == '__main__':
    follower = PathFollower(image_width=640, image_height=480)

    lines_from_image = [(0.1, 90), (0.15, 280), (0.2, 490), (0.05, 620)]  # Example lines

    follower.set_lines(lines_from_image)

    theta_value = np.radians(45)  # Example: theta is 10 degrees (in radians)
    follower.set_theta(theta_value)

    # Find the nearest line to the center of the image
    nearest_line = follower.find_nearest_line()
    print(f"The nearest line to the image center has slope: {nearest_line[0]} and intercept: {nearest_line[1]}")

    # Get the waypoints on the nearest line (10 waypoints with spacing=1)
    waypoints_nearest_line = follower.get_waypoints_on_line(spacing=1)
    print("\nWaypoints on the nearest line (before transformation):")
    for wp in waypoints_nearest_line:
        print(wp)

    # Transform the waypoints into the robot's reference frame using the theta value
    adjusted_waypoints = follower.transform_waypoints_to_robot_frame(waypoints_nearest_line)
    print("\nAdjusted waypoints in robot reference frame:")
    for wp in adjusted_waypoints:
        print(wp)

    follower.plot_waypoints(waypoints_nearest_line, adjusted_waypoints)