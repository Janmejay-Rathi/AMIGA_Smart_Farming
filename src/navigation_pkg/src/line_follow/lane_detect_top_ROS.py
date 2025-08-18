#!/usr/bin/env python
# This script detects plant rows from a camera feed using YOLOv8, processes them to compute heading and distance,
# and publishes the results as ROS topics for navigation control.

import cv2                      # OpenCV for image processing
import numpy as np              # Numpy for numerical operations
import rospy                    # ROS Python client library
from sensor_msgs.msg import CompressedImage, Image   # ROS message types for images
from std_msgs.msg import Float64                     # ROS message type for publishing heading/distance
from cv_bridge import CvBridge, CvBridgeError        # For converting ROS images <-> OpenCV images
from ultralytics import YOLO                         # YOLOv8 model for object detection
from sklearn.linear_model import ElasticNet          # ElasticNet regression for line fitting
import matplotlib.pyplot as plt                      # For plotting/debugging
import time                                          # To measure processing time

# Class that encapsulates plant row detection functionality
class PlantRowDetector:
    def __init__(self, model_path, img_save_path, mask_coords,
                 G_threshold=70, IExG_threshold=30, height_spacing=20,
                 conf_threshold=0.5, row_dist_threshold=50):
        """
        Initialize the PlantRowDetector class with parameters.
        Args:
            model_path: Path to YOLOv8 model weights
            img_save_path: Path to save processed images
            mask_coords: Coordinates for masking (unused currently)
            G_threshold: Threshold for green pixel intensity
            IExG_threshold: Threshold for IExG index (improved green index)
            height_spacing: Spacing for sampling vertical slices inside bounding boxes
            conf_threshold: Confidence threshold for YOLO detections
            row_dist_threshold: Distance threshold for detecting plant rows
        """
        self.model_path = model_path
        self.img_save_path = img_save_path
        self.G_threshold = G_threshold
        self.IExG_threshold = IExG_threshold
        self.height_spacing = height_spacing
        self.mask_coords = mask_coords
        self.counter = 1                   # Counter for saving images
        self.model = None                  # Placeholder for YOLO model
        self.conf_threshold = conf_threshold
        self.row_dist_threshold = row_dist_threshold
        self.bridge = CvBridge()           # ROS <-> OpenCV bridge

        # Subscriber: compressed RGB image from camera
        self.image_subscriber = rospy.Subscriber("/oak1/rgb", CompressedImage, self.image_callback)
        # Publisher: publishes processed image after overlay
        self.image_Publisher = rospy.Publisher("/front_cam/image", Image, queue_size=1)
        # Publisher: publishes computed heading angle
        self.heading_publisher = rospy.Publisher("/robot_heading_angle", Float64, queue_size=1)
        # Publisher: publishes distance from row centerline
        self.distance_publisher = rospy.Publisher("/row_distance", Float64, queue_size=1)

        # Camera intrinsic matrix (K) and distortion coefficients (dist)
        self.K = np.array([[1.22594758e+03, 0.00000000e+00, 9.40595698e+02],
                           [0.00000000e+00, 1.23194872e+03, 5.24056999e+02],
                           [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.dist = np.array([-0.21994376, 0.04686792, 0.00173996, -0.0007083, 0.02286507])

        self.prev_heading = None           # Store previous heading for smoothing
        self.load_model()                  # Load YOLO model

    def load_model(self):
        """Loads the YOLOv8 model using the given model path."""
        self.model = YOLO(self.model_path)

    def image_callback(self, ros_image):
        """Callback for receiving images from the camera topic."""
        start = time.time()   # Record start time for performance measurement

        # Convert compressed ROS image to numpy array
        np_arr = np.frombuffer(ros_image.data, np.uint8)
        # Decode numpy array to OpenCV image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Get image dimensions
        h, w = image.shape[:2]

        # Undistort image using camera intrinsics
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(image, self.K, self.dist, None, newcameramtx)

        # Resize image to fixed size (640x360) for processing
        image = cv2.resize(dst, (640, 360))

        # Process the image
        self.process_single_image(image)

        # Print processing time
        end = time.time()
        print("time taken:", end - start)

    def process_single_image(self, image):
        """Processes a single image to detect rows and compute heading/distance."""
        # Apply mask (currently masking whole image)
        image = self.mask_image(image)
        # Run YOLO detection and get bounding boxes
        crop_points, bounding_boxes, initial_centers = self.predict_bounding_boxes(image)
        
        # Handle special cases:
        if len(bounding_boxes) == 1:
            # If only one row detected, reuse previous slope/intercept
            rospy.loginfo("Single detection, skipping image. Publishing previous slope and intercept values....")
            self.publish_heading(self.prev_slope, self.prev_intercept)
            return
        elif len(bounding_boxes) == 0 or len(initial_centers) < 2:
            # If no rows detected, skip this frame
            rospy.loginfo("No detections, skipping image......")
            return

        # Find midpoints of green vegetation pixels inside bounding boxes
        midpoints, midpoints_list = self.find_midpoints(image, crop_points, bounding_boxes)

        # Fit a line using ElasticNet regression on detected centers
        slope, intercept = self.fit_line_single(initial_centers)

        # Publish heading and distance
        self.publish_heading(slope, intercept)

        # Plot fitted line and midpoints on the image
        self.plot_single_line(slope, intercept, midpoints, image, initial_centers)

    def mask_image(self, image):
        """Masks the input image based on defined coordinates (currently keeps full image)."""
        h, w = image.shape[:2]
        self.height, self.width = h, w
        mask = np.zeros((h, w), dtype=np.uint8)   # Initialize mask
        x_start, x_end = 0, w
        mask[:, x_start:x_end] = 255              # Full horizontal mask
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        return masked_image

    def predict_bounding_boxes(self, image):
        """Runs YOLOv8 detection to find bounding boxes of plants/rows."""
        results = self.model(image, conf=self.conf_threshold)
        crop_points, bounding_boxes, initial_centers = [], [], []

        for result in results[0].boxes:     # Iterate over detected objects
            box = result.xyxy[0].cpu().numpy()     # Bounding box coordinates
            cls = int(result.cls[0].cpu().numpy()) # Class ID

            if cls == 0:  # Only process class=0 (plant row)
                x1, y1, x2, y2 = box
                w, h = x2 - x1, y2 - y1
                cx, cy = int(x1 + w // 2), int(y1 + h // 2)   # Center point
                crop_points.append((cx, cy))
                bounding_boxes.append((int(w), int(h)))
                initial_centers.append([cx, cy])
        
        print(initial_centers)   # Debugging: print detected centers
        return crop_points, bounding_boxes, initial_centers

    def find_midpoints(self, image, crop_points, bounding_boxes):
        """Finds midpoints of green vegetation pixels inside each bounding box."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        midpoints, midpoints_list = [], []

        # Loop through each bounding box and its center
        for (cx, cy), (w, h) in zip(crop_points, bounding_boxes):
            top_left_x = cx - w // 2
            top_left_y = cy - h // 2
            midpoints2 = []

            # Crop the bounding box region
            crop_img = image_rgb[top_left_y:top_left_y + h, top_left_x:top_left_x + w]

            # Divide into horizontal slices and detect green pixels
            for y in range(0, crop_img.shape[0], self.height_spacing):
                slice_img = crop_img[y:y + self.height_spacing, :]
                R, G, B = slice_img[:, :, 0], slice_img[:, :, 1], slice_img[:, :, 2]
                IExG = 2 * G - R - B     # Excess green index
                green_mask = (G > self.G_threshold) | (IExG > self.IExG_threshold)
                green_pixels = np.column_stack(np.where(green_mask))

                if len(green_pixels) > 0:
                    # Compute average (midpoint) of green pixels
                    midpoint = green_pixels.mean(axis=0)
                    midpoints.append((int(midpoint[1]) + top_left_x, int(midpoint[0]) + top_left_y + y))
                    midpoints2.append([int(midpoint[1]) + top_left_x, int(midpoint[0]) + top_left_y + y])
            midpoints_list.append(np.array(midpoints2))

        return midpoints, midpoints_list

    def fit_line_single(self, midpts):
        """Fits a straight line through points using ElasticNet regression."""
        midpts = np.array(midpts)
        x_pts = midpts[:, 0].reshape(-1, 1)
        y_pts = midpts[:, 1]

        if len(x_pts) > 1:
            model = ElasticNet(alpha=0.1, l1_ratio=0.5)  # Regularized regression
            model.fit(x_pts, y_pts)
            slope, intercept = model.coef_[0], model.intercept_
        else:
            slope, intercept = 0, 0   # Default if insufficient points
        return slope, intercept

    def publish_heading(self, slope, intercept):
        """Publishes computed heading angle and lateral distance to ROS topics."""
        heading = np.arctan(slope)                   # Compute angle in radians
        heading_deg = np.rad2deg(heading) * (-1)     # Convert to degrees, invert sign

        # Compute perpendicular distance from image centerline to detected line
        x_center, y_center = self.width // 2, self.height // 2
        distance = (y_center - slope * x_center - intercept) / np.sqrt(1 + slope ** 2)

        # Cap heading angle to avoid extreme values
        heading_cutoff = 20
        if abs(heading_deg) > heading_cutoff:
            heading_deg = heading_deg / abs(heading_deg) * heading_cutoff

        # Publish results
        self.heading_publisher.publish(Float64(heading_deg))
        self.distance_publisher.publish(Float64(distance))

        rospy.loginfo(f"Heading (deg): {heading_deg}, Distance: {distance}")

        # Store values for future use
        self.prev_heading, self.prev_distance = heading_deg, distance
        self.prev_slope, self.prev_intercept = slope, intercept

    def plot_midpoints(self, image, midpoints):
        """Draws detected midpoints on the image (for debugging)."""
        for point in midpoints:
            cv2.circle(image, point, 5, (0, 0, 255), -1)

    def plot_single_line(self, slope, intercept, midpoints, image_rgb, initial_centers):
        """Plots fitted line and midpoints onto the image and publishes the visualization."""
        height, width, _ = image_rgb.shape
        y_values = np.linspace(0, height, 10000)              # Sample y values
        x_values = (y_values - intercept) / slope             # Compute x from line equation

        # Filter points within image boundaries
        valid_idx = np.where((0 <= x_values) & (x_values <= width) & (0 <= y_values) & (y_values <= height))
        valid_x = x_values[valid_idx]
        valid_y = y_values[valid_idx]

        # Draw points representing the line
        for i in range(len(valid_x)):
            x = int(valid_x[i])
            y = int(valid_y[i])
            cv2.circle(image_rgb, (x, y), 5, (255, 0, 0), -1)

        # Convert OpenCV image to ROS Image message and publish
        bridge = CvBridge()
        try:
            image_msg = bridge.cv2_to_imgmsg(image_rgb, encoding="bgr8")
            self.image_Publisher.publish(image_msg)
        except CvBridgeError as e:
            print(e)

        # Show image locally in a window (for debugging)
        cv2.imshow("img", image_rgb)
        cv2.waitKey(1)

        self.counter += 1   # Increment image counter

    def main(self):
        """Keeps the ROS node running and listening for images."""
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("plant_row_detector", anonymous=True)   # Initialize ROS node

    # Initialize the detector with model path and parameters
    detector = PlantRowDetector(
        model_path="/home/jurathi2/catkin_ws/src/navigation_pkg/src/line_follow/best.pt",
        img_save_path='/home/jurathi2/catkin_ws/src/navigation_pkg/src/line_follow/',
        mask_coords=(150, 550),
        conf_threshold=0.5,
        height_spacing=10,
        row_dist_threshold=50
    )

    # Start processing
    detector.main()
