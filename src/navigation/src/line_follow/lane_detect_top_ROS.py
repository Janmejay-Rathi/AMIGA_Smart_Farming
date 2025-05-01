#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from sklearn.linear_model import ElasticNet
import matplotlib.pyplot as plt
import time

class PlantRowDetector(Node):
    def __init__(self,
                 model_path: str,
                 img_save_path: str,
                 mask_coords: tuple,
                 G_threshold: int = 70,
                 IExG_threshold: int = 30,
                 height_spacing: int = 20,
                 conf_threshold: float = 0.5,
                 row_dist_threshold: int = 50):
        super().__init__('plant_row_detector')

        # Parameters
        self.model_path = model_path
        self.img_save_path = img_save_path
        self.mask_coords = mask_coords
        self.G_threshold = G_threshold
        self.IExG_threshold = IExG_threshold
        self.height_spacing = height_spacing
        self.conf_threshold = conf_threshold
        self.row_dist_threshold = row_dist_threshold

        # CV bridge
        self.bridge = CvBridge()
        self.counter = 1

        # Camera intrinsics & distortion
        self.K = np.array([
            [1.22594758e+03, 0.0,            9.40595698e+02],
            [0.0,            1.23194872e+03, 5.24056999e+02],
            [0.0,            0.0,            1.0]
        ])
        self.dist = np.array([
            -0.21994376, 0.04686792, 0.00173996,
            -0.0007083,  0.02286507
        ])

        # State for previous fit
        self.prev_heading = None
        self.prev_slope = 0.0
        self.prev_intercept = 0.0

        # Publishers & subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/oak1/rgb',
            self.image_callback,
            1
        )
        self.image_pub = self.create_publisher(
            Image,
            '/front_cam/image',
            1
        )
        self.heading_pub = self.create_publisher(
            Float64,
            '/robot_heading_angle',
            1
        )
        self.distance_pub = self.create_publisher(
            Float64,
            '/row_distance',
            1
        )

        # Load YOLO model
        self.load_model()

        # Give ROS time to connect publishers
        time.sleep(1.0)

    def load_model(self):
        """Loads the YOLOv8 model."""
        self.model = YOLO(self.model_path)

    def image_callback(self, msg: CompressedImage):
        """Callback for incoming compressed images."""
        start = time.time()
        # Decode image
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Undistort & resize
        h, w = img.shape[:2]
        newcam_mtx, _ = cv2.getOptimalNewCameraMatrix(
            self.K, self.dist, (w, h), 1, (w, h)
        )
        dst = cv2.undistort(img, self.K, self.dist, None, newcam_mtx)
        image = cv2.resize(dst, (640, 360))

        # Process
        self.process_single_image(image)

        elapsed = time.time() - start
        self.get_logger().info(f"Image processed in {elapsed:.3f}s")

    def process_single_image(self, image: np.ndarray):
        """Runs the full pipeline on one frame."""
        masked = self.mask_image(image)
        crop_pts, bboxes, centers = self.predict_bounding_boxes(masked)

        if len(bboxes) == 1:
            self.get_logger().info(
                "Single detection; publishing previous heading/distance"
            )
            self.publish_heading(self.prev_slope, self.prev_intercept)
            return
        if len(bboxes) == 0 or len(centers) < 2:
            self.get_logger().info("No valid detections; skipping frame")
            return

        midpts, _ = self.find_midpoints(masked, crop_pts, bboxes)
        slope, intercept = self.fit_line_single(centers)
        self.publish_heading(slope, intercept)
        self.plot_single_line(slope, intercept, midpts, image, centers)

    def mask_image(self, image: np.ndarray) -> np.ndarray:
        """Apply a rectangular mask over the image."""
        h, w = image.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[:, :] = 255
        return cv2.bitwise_and(image, image, mask=mask)

    def predict_bounding_boxes(self, image: np.ndarray):
        """Run YOLO and return centers & box sizes for class 0."""
        results = self.model(image, conf=self.conf_threshold)
        crop_pts, bboxes, centers = [], [], []
        for box in results[0].boxes:
            coords = box.xyxy[0].cpu().numpy()
            cls_id = int(box.cls[0].cpu().numpy())
            if cls_id == 0:
                x1, y1, x2, y2 = coords
                w, h = x2 - x1, y2 - y1
                cx, cy = int(x1 + w/2), int(y1 + h/2)
                crop_pts.append((cx, cy))
                bboxes.append((int(w), int(h)))
                centers.append([cx, cy])
        return crop_pts, bboxes, centers

    def find_midpoints(self, image, crop_pts, bboxes):
        """Within each box, find green midpoints at slices."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        midpts, midpts_list = [], []
        for (cx, cy), (w, h) in zip(crop_pts, bboxes):
            tlx, tly = cx - w//2, cy - h//2
            pts2 = []
            for y in range(0, h, self.height_spacing):
                slice_ = image_rgb[tly+y:tly+y+self.height_spacing, tlx:tlx+w]
                R, G, B = slice_[:,:,0], slice_[:,:,1], slice_[:,:,2]
                IExG = 2*G - R - B
                mask = (G > self.G_threshold) | (IExG > self.IExG_threshold)
                pix = np.column_stack(np.where(mask))
                if pix.size:
                    mp = pix.mean(axis=0)
                    xp = int(mp[1]) + tlx
                    yp = int(mp[0]) + tly + y
                    midpts.append((xp, yp))
                    pts2.append([xp, yp])
            midpts_list.append(np.array(pts2))
        return midpts, midpts_list

    def fit_line_single(self, pts):
        """Fit line y = m x + b via ElasticNet."""
        arr = np.array(pts)
        X = arr[:,0].reshape(-1,1)
        y = arr[:,1]
        if X.shape[0] > 1:
            model = ElasticNet(alpha=0.1, l1_ratio=0.5)
            model.fit(X, y)
            return float(model.coef_[0]), float(model.intercept_)
        return 0.0, 0.0

    def publish_heading(self, slope, intercept):
        """Compute & publish heading (°) and lateral distance (m)."""
        heading = np.arctan(slope)
        heading_deg = -np.rad2deg(heading)
        h, w = 360, 640
        dist = (h/2 - slope*(w/2) - intercept) / np.sqrt(1 + slope**2)

        # clamp
        heading_deg = max(min(heading_deg, 20.0), -20.0)

        self.heading_pub.publish(Float64(data=heading_deg))
        self.distance_pub.publish(Float64(data=dist))
        self.get_logger().info(
            f"Heading: {heading_deg:.2f}°, Distance: {dist:.2f}m"
        )

        self.prev_slope = slope
        self.prev_intercept = intercept

    def plot_single_line(self, slope, intercept, pts, image, centers):
        """Overlay fitted line & display/publish image."""
        h, w, _ = image.shape
        ys = np.linspace(0, h, 1000)
        xs = (ys - intercept) / slope
        valid = (xs>=0)&(xs<=w)
        for x, y in zip(xs[valid], ys[valid]):
            cv2.circle(image, (int(x), int(y)), 3, (255,0,0), -1)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

        cv2.imshow("Line Fit", image)
        cv2.waitKey(1)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PlantRowDetector(
        model_path="/home/jetson-amiga/ros2_ws/src/navigation/src/line_follow/best.pt",
        img_save_path="/home/jetson-amiga/ros2_ws/src/navigation/src/line_follow",
        mask_coords=(150, 550),
        conf_threshold=0.5,
        height_spacing=10,
        row_dist_threshold=50
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
