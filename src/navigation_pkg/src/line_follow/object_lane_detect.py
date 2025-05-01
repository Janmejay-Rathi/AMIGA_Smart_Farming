#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from ultralytics import YOLO
from sklearn.linear_model import ElasticNet
import time

class PlantRowDetector:
    def __init__(self, horseradish_model_path, human_model_path, img_save_path, mask_coords, G_threshold=70, IExG_threshold=30, height_spacing=20, conf_threshold=0.5, row_dist_threshold=50):
        # Path to custom horseradish detection model and YOLOv8 human detection model
        self.horseradish_model_path = horseradish_model_path
        self.human_model_path = human_model_path
        self.img_save_path = img_save_path
        self.G_threshold = G_threshold
        self.IExG_threshold = IExG_threshold
        self.height_spacing = height_spacing
        self.mask_coords = mask_coords
        self.counter = 1
        self.horseradish_model = None
        self.human_model = None
        self.conf_threshold = conf_threshold
        self.row_dist_threshold = row_dist_threshold
        self.bridge = CvBridge()  # ROS <-> OpenCV bridge
        self.image_subscriber_oak1 = rospy.Subscriber("/oak1/rgb", CompressedImage, self.image_callback_oak1)
        self.image_subscriber_oak0 = rospy.Subscriber("/oak0/rgb", CompressedImage, self.image_callback_oak0)
        self.image_Publisher = rospy.Publisher("/front_cam/image", Image, queue_size=1)
        self.heading_publisher = rospy.Publisher("/robot_heading_angle", Float64, queue_size=1)
        self.distance_publisher = rospy.Publisher("/row_distance", Float64, queue_size=1)
        self.K = np.array([[1.22594758e+03 ,0.00000000e+00 ,9.40595698e+02],
                           [0.00000000e+00 ,1.23194872e+03 ,5.24056999e+02],
                           [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])
        self.dist = np.array([-0.21994376 , 0.04686792 , 0.00173996 ,-0.0007083 ,  0.02286507])
        self.prev_heading = None
        self.load_models()

    def load_models(self):
        """Load both YOLOv8 models."""
        self.horseradish_model = YOLO(self.horseradish_model_path)  # Custom model for horseradish detection
        self.human_model = YOLO(self.human_model_path)  # Pretrained YOLOv8 model for human detection

    def image_callback_oak1(self, ros_image):
        """Callback for receiving images from oak1/rgb."""
        start = time.time()
        np_arr = np.frombuffer(ros_image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode the compressed image
        h, w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(image, self.K, self.dist, None, newcameramtx)
        image = cv2.resize(dst, (640, 360))
        self.process_single_image(image)
        end = time.time()
        print("time taken:", end-start)

    def image_callback_oak0(self, ros_image):
        """Callback for receiving images from oak0/rgb."""
        np_arr = np.frombuffer(ros_image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.detect_humans_and_display(image)

    def detect_humans_and_display(self, image):
        """Detect humans using YOLO and display the image."""
        results = self.human_model(image, conf=self.conf_threshold)
        human_detected = False
        for result in results[0].boxes:
            box = result.xyxy[0].cpu().numpy()
            cls = int(result.cls[0].cpu().numpy())
            if cls == 0:  # Class ID 0 for 'person' in COCO dataset
                x1, y1, x2, y2 = box
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Draw bounding box
                human_detected = True

        # Display the image
        # print("sizeeeee================", image.shape)
        # image = cv2.resize(image,(1280,720))
        cv2.imshow("Oak0 RGB Feed", image)
        cv2.waitKey(1)

        # Only publish heading when no human is detected
        if not human_detected:
            self.publish_heading(self.prev_slope, self.prev_intercept)

    def process_single_image(self, image):
        """Process a single image."""
        image = self.mask_image(image)
        crop_points, bounding_boxes, initial_centers = self.predict_bounding_boxes(image)

        if len(bounding_boxes) == 1:
            rospy.loginfo("Single detection, skipping image. Publishing previous slope and intercept values....")
            self.publish_heading(self.prev_slope, self.prev_intercept)
            return
        elif len(bounding_boxes) == 0 or len(initial_centers) < 2:
            rospy.loginfo("No detections, skipping image......")
            return
        
        midpoints, midpoints_list = self.find_midpoints(image, crop_points, bounding_boxes)
        slope, intercept = self.fit_line_single(initial_centers)
        self.publish_heading(slope, intercept)
        self.plot_single_line(slope, intercept, midpoints, image, initial_centers)

    def mask_image(self, image):
        """Masks the input image based on defined coordinates."""
        h, w = image.shape[:2]
        self.height, self.width = h, w
        mask = np.zeros((h, w), dtype=np.uint8)
        x_start, x_end = 0, w
        mask[:, x_start:x_end] = 255
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        return masked_image

    def predict_bounding_boxes(self, image):
        """Uses YOLOv8 to predict bounding boxes."""
        results = self.horseradish_model(image, conf=self.conf_threshold)
        crop_points, bounding_boxes, initial_centers = [], [], []

        for result in results[0].boxes:
            box = result.xyxy[0].cpu().numpy()
            cls = int(result.cls[0].cpu().numpy())
            
            if cls == 0:  # Detect horseradish class (Assuming it's '0')
                x1, y1, x2, y2 = box
                w, h = x2 - x1, y2 - y1
                cx, cy = int(x1 + w // 2), int(y1 + h // 2)
                crop_points.append((cx, cy))
                bounding_boxes.append((int(w), int(h)))
                initial_centers.append([cx, cy])

        return crop_points, bounding_boxes, initial_centers

    def find_midpoints(self, image, crop_points, bounding_boxes):
        """Find midpoints within bounding boxes."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        midpoints, midpoints_list = [], []

        for (cx, cy), (w, h) in zip(crop_points, bounding_boxes):
            top_left_x = cx - w // 2
            top_left_y = cy - h // 2
            midpoints2 = []

            crop_img = image_rgb[top_left_y:top_left_y + h, top_left_x:top_left_x + w]
            for y in range(0, crop_img.shape[0], self.height_spacing):
                slice_img = crop_img[y:y + self.height_spacing, :]
                R, G, B = slice_img[:, :, 0], slice_img[:, :, 1], slice_img[:, :, 2]
                IExG = 2 * G - R - B
                green_mask = (G > self.G_threshold) | (IExG > self.IExG_threshold)
                green_pixels = np.column_stack(np.where(green_mask))
                if len(green_pixels) > 0:
                    midpoint = green_pixels.mean(axis=0)
                    midpoints.append((int(midpoint[1]) + top_left_x, int(midpoint[0]) + top_left_y + y))
                    midpoints2.append([int(midpoint[1]) + top_left_x, int(midpoint[0]) + top_left_y + y])
            midpoints_list.append(np.array(midpoints2))

        return midpoints, midpoints_list

    def fit_line_single(self, midpts):
        """Fits a line through midpoints using ElasticNet."""
        midpts = np.array(midpts)
        x_pts = midpts[:, 0].reshape(-1, 1)
        y_pts = midpts[:, 1]

        if len(x_pts) > 1:
            model = ElasticNet(alpha=0.1, l1_ratio=0.5)
            model.fit(x_pts, y_pts)
            slope, intercept = model.coef_[0], model.intercept_
        else:
            slope, intercept = 0, 0
        return slope, intercept

    def publish_heading(self, slope, intercept):
        """Publishes the heading angle and distance to ROS topics."""
        heading = np.arctan(slope)
        heading_deg = np.rad2deg(heading) * (-1)

        x_center, y_center = self.width // 2, self.height // 2
        distance = (y_center - slope * x_center - intercept) / np.sqrt(1 + slope ** 2)

        heading_cutoff = 20
        if abs(heading_deg) > heading_cutoff:
            heading_deg = heading_deg / abs(heading_deg) * heading_cutoff

        self.heading_publisher.publish(Float64(heading_deg))
        self.distance_publisher.publish(Float64(distance))

        rospy.loginfo(f"Heading (deg): {heading_deg}, Distance: {distance}")
        self.prev_heading, self.prev_distance = heading_deg, distance
        self.prev_slope, self.prev_intercept = slope, intercept

    def plot_single_line(self, slope, intercept, midpoints, image_rgb, initial_centers):
        height, width, _ = image_rgb.shape
        y_values = np.linspace(0, height, 10000)
        x_values = (y_values - intercept) / slope
        valid_idx = np.where((0 <= x_values) & (x_values <= width) & (0 <= y_values) & (y_values <= height))

        valid_x = x_values[valid_idx]
        valid_y = y_values[valid_idx]

        for i in range(len(valid_x)):
            x = int(valid_x[i])
            y = int(valid_y[i])
            cv2.circle(image_rgb, (x, y), 5, (255, 0, 0), -1)

        bridge = CvBridge()
        try:
            image_msg = bridge.cv2_to_imgmsg(image_rgb, encoding="bgr8")
            self.image_Publisher.publish(image_msg)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("img", image_rgb)
        cv2.waitKey(1)

    def main(self):
        rospy.spin()  # Keep the node alive and listening for incoming images

if __name__ == '__main__':
    rospy.init_node("plant_row_detector", anonymous=True)

    # Initialize the detector with both models
    detector = PlantRowDetector(
        horseradish_model_path="/home/jurathi2/catkin_ws/src/navigation_pkg/src/line_follow/best.pt",  # Update with correct path
        human_model_path="/home/jurathi2/catkin_ws/src/navigation_pkg/src/line_follow/yolov8n.pt",  # Update with correct path
        img_save_path='/home/jurathi2/catkin_ws/src/navigation_pkg/src/line_follow/',
        mask_coords=(150, 550),
        conf_threshold=0.5,
        height_spacing=10,
        row_dist_threshold=50
    )

    detector.main()