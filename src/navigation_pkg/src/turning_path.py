import numpy as np
import matplotlib.pyplot as plt

class Turn:
    def __init__(self, gps, d1=50, d2=100, radius=None, num_waypoints=20):
        self.gps = gps  # Starting GPS coordinates (point A)
        self.d1 = d1  # Vertical distance from A to B and from C to D
        self.d2 = d2  # Horizontal distance between A and D (along x-axis)
        self.radius = radius if radius else d2 / 2  # The radius of the semicircle
        self.num_waypoints = num_waypoints
        self.current_gps_position = None

    def get_gps_position(self):
        # Mock GPS position for now
        self.current_gps_position = self.gps
        print(f"Initial GPS position (A): {self.current_gps_position}")
        return self.current_gps_position

    def move_vertical(self, distance):
        new_position = (self.current_gps_position[0], self.current_gps_position[1] - distance)
        self.current_gps_position = new_position
        return new_position

    def generate_semicircle_waypoints(self, center_x, center_y):
        waypoints = []
        start_angle = 180  # Start from point B
        end_angle = 0  # End at point C
        angles = np.linspace(np.radians(start_angle), np.radians(end_angle), self.num_waypoints)

        for angle in angles:
            x = center_x + self.radius * np.cos(angle)
            y = center_y + self.radius * np.sin(angle)
            waypoints.append((x, y))
        return waypoints

    def perform_turn(self):
        # Step 1: Start at point A and move vertically to point B
        initial_position = self.get_gps_position()
        point_B = self.move_vertical(-self.d1)
        print(f"Moved vertically to point B: {point_B}")

        # Step 2: Calculate the center of the semicircle (center is horizontally between B and C)
        center_x = point_B[0] + self.d2 / 2
        center_y = point_B[1]
        print(f"Center of semicircle: ({center_x}, {center_y})")

        # Step 3: Generate waypoints along the semicircle
        semicircle_waypoints = self.generate_semicircle_waypoints(center_x, center_y)
        print("Generated semicircle waypoints:", semicircle_waypoints)

        # Step 4: Move to point C (final waypoint in the semicircle)
        point_C = semicircle_waypoints[-1]
        print(f"Reached point C: {point_C}")

        # Step 5: Move vertically again from point C to point D
        self.current_gps_position = point_C
        point_D = self.move_vertical(self.d1)
        print(f"Moved vertically to point D: {point_D}")

        return initial_position, point_B, semicircle_waypoints, point_C, point_D


turn = Turn(gps=(0, 0), d1=50, d2=100, radius=None)

initial_position, point_B, semicircle_waypoints, point_C, point_D = turn.perform_turn()

plt.figure(figsize=(10, 6))

plt.plot([initial_position[0], point_B[0]], [initial_position[1], point_B[1]], 'b-', label="Move Vertical A to B")

semicircle_x = [wp[0] for wp in semicircle_waypoints]
semicircle_y = [wp[1] for wp in semicircle_waypoints]
plt.plot(semicircle_x, semicircle_y, 'g-', label="Semicircular Turn B to C")

plt.plot([point_C[0], point_D[0]], [point_C[1], point_D[1]], 'r-', label="Move Vertical C to D")

plt.scatter(*initial_position, color='blue', label="Start (A)")
plt.scatter(*point_D, color='red', label="End (D)")

plt.text(initial_position[0], initial_position[1], 'A', fontsize=12, verticalalignment='bottom')
plt.text(point_B[0], point_B[1], 'B', fontsize=12, verticalalignment='bottom')
plt.text(point_C[0], point_C[1], 'C', fontsize=12, verticalalignment='bottom')
plt.text(point_D[0], point_D[1], 'D', fontsize=12, verticalalignment='top')

plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Robot's Path with Semicircular Turn")
plt.legend()
plt.grid(True)

plt.show()
