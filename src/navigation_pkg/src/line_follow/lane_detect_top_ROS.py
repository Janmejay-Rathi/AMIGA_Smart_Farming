#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from ultralytics import YOLO
from sklearn.linear_model import ElasticNet
import matplotlib.pyplot as plt
import time
from cv_bridge import CvBridge, CvBridgeError

class PlantRowDetector:
    def __init__(self, model_path, img_save_path, mask_coords, G_threshold=70, IExG_threshold=30, height_spacing=20, conf_threshold=0.5, row_dist_threshold=50):
        self.model_path = model_path
        self.img_save_path =img_save_path
        self.G_threshold = G_threshold
        self.IExG_threshold = IExG_threshold
        self.height_spacing = height_spacing
        self.mask_coords = mask_coords
        self.counter = 1
        self.model = None
        self.conf_threshold = conf_threshold
        self.row_dist_threshold = row_dist_threshold
        self.bridge = CvBridge()  # ROS <-> OpenCV bridge
        self.image_subscriber = rospy.Subscriber("/oak1/rgb", CompressedImage, self.image_callback)
        self.image_Publisher = rospy.Publisher("/front_cam/image", Image, queue_size=1)
        self.heading_publisher = rospy.Publisher("/robot_heading_angle", Float64, queue_size=1)
        self.distance_publisher = rospy.Publisher("/row_distance", Float64, queue_size=1)
        self.K =  np.array([[1.22594758e+03 ,0.00000000e+00 ,9.40595698e+02],
            [0.00000000e+00 ,1.23194872e+03 ,5.24056999e+02],
            [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])
        self.dist = np.array([-0.21994376 , 0.04686792 , 0.00173996 ,-0.0007083 ,  0.02286507])
        self.prev_heading = None
        self.load_model()

    def load_model(self):
        """Loads the YOLOv8 model."""
        self.model = YOLO(self.model_path)

    def image_callback(self, ros_image):
        start = time.time()
        # self.image_Publisher.publish(ros_image)
        """Callback for receiving images from the compressed image topic."""
        np_arr = np.frombuffer(ros_image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode the compressed image
        h, w = image.shape[:2]
        # print(h,w)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 1, (w, h))

        dst = cv2.undistort(image, self.K, self.dist,None, newcameramtx)
        image = cv2.resize(dst, (640, 360))

        # cv2.imshow("img",image)
        # cv2.waitKey(1)
        # cv2.destroyAllWindows()


        self.process_single_image(image)
        end = time.time()
        print("time taken:",end-start)

    def process_single_image(self, image):
        """Process a single image."""
        image = self.mask_image(image)
        crop_points, bounding_boxes, initial_centers = self.predict_bounding_boxes(image)
        
        if len(bounding_boxes)==1:
            rospy.loginfo("Single detection, skipping image. Publishing previous slope and intercept values....")
            self.publish_heading(self.prev_slope, self.prev_intercept) 
            return

        elif len(bounding_boxes) == 0 or len(initial_centers)<2:
            rospy.loginfo("No detections, skipping image......")
            return
        
        midpoints, midpoints_list = self.find_midpoints(image, crop_points, bounding_boxes)
        # self.plot_midpoints(image, midpoints)

        slope, intercept = self.fit_line_single(initial_centers)
        self.publish_heading(slope, intercept)
        self.plot_single_line(slope, intercept, midpoints, image,initial_centers)
        

    def mask_image(self, image):
        """Masks the input image based on defined coordinates."""
        h, w = image.shape[:2]
        self.height, self.width = h,w
        mask = np.zeros((h, w), dtype=np.uint8)
        x_start, x_end = 0, w
        mask[:, x_start:x_end] = 255
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        return masked_image

    def predict_bounding_boxes(self, image):
        """Uses YOLOv8 to predict bounding boxes."""
        results = self.model(image, conf=self.conf_threshold)
        crop_points, bounding_boxes, initial_centers = [], [], []

        for result in results[0].boxes:
            box = result.xyxy[0].cpu().numpy()
            cls = int(result.cls[0].cpu().numpy())
            
            if cls == 0:
                x1, y1, x2, y2 = box
                w, h = x2 - x1, y2 - y1
                cx, cy = int(x1 + w // 2), int(y1 + h // 2)
                crop_points.append((cx, cy))
                bounding_boxes.append((int(w), int(h)))
                initial_centers.append([cx, cy])
        
        print(initial_centers)

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
        # distance = -distance


        heading_cutoff = 20
        if abs(heading_deg)>heading_cutoff:
            heading_deg = heading_deg/abs(heading_deg)*heading_cutoff

        self.heading_publisher.publish(Float64(heading_deg))
        self.distance_publisher.publish(Float64(distance))


        # if self.prev_heading is None:
        #     self.prev_heading = heading_deg

        # #This is a check so that we dont have big changes in the heading angle
        # if abs(heading_deg/self.prev_heading) < 1.0: 
        #     heading_cutoff = 20
        #     if abs(heading_deg)>heading_cutoff:
        #         heading_deg = heading_deg/abs(heading_deg)*heading_cutoff

        #     self.heading_publisher.publish(Float64(heading_deg))
        #     self.distance_publisher.publish(Float64(distance))
        # else:
        #     heading_deg = self.prev_heading
        #     heading_cutoff = 20
        #     if abs(self.prev_heading)>heading_cutoff:
        #         heading_deg = heading_deg/abs(heading_deg)*heading_cutoff

        #     self.heading_publisher.publish(Float64(heading_deg))
        #     self.distance_publisher.publish(Float64(distance))
        
        rospy.loginfo(f"Heading (deg): {heading_deg}, Distance: {distance}")
        self.prev_heading,self.prev_distance = heading_deg, distance
        self.prev_slope, self.prev_intercept = slope,intercept

    def plot_midpoints(self, image, midpoints):
        """Plots midpoints on the image."""
        for point in midpoints:
            cv2.circle(image, point, 5, (0, 0, 255), -1)
        # plt.imshow(image)
        # plt.show()

    def plot_single_line(self,slope, intercept, midpoints, image_rgb,initial_centers):
        height,width,_ = image_rgb.shape
        y_values = np.linspace(0, height, 10000)
        x_values = (y_values - intercept) / slope
        valid_idx = np.where((0 <= x_values) & (x_values <= width) & (0 <= y_values) & (y_values <= height))
        # print(f"Line{counter} y = {slope}*x + {intercept} ")
        
        # plt.plot(x_values[valid_idx], y_values[valid_idx], 'w-')

        # for pt in list(zip(x_values[valid_idx],y_values[valid_idx])):
        #     cv2.circle(image_rgb,pt,5,(0,0,255),-1)
                
        valid_x = x_values[valid_idx]
        valid_y = y_values[valid_idx]
        # # for point in midpoints:
        # #     cv2.circle(image_rgb, point, 5, (255, 0, 0), -1)
        for i in range(len(valid_x)):
            x = int(valid_x[i])
            y = int(valid_y[i])
            # print(x,y)
            cv2.circle(image_rgb, (x,y), 5, (255, 0, 0), -1)

        bridge = CvBridge()
        # cv2.imwrite("/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/img.jpg",image_rgb)

        try:
            image_msg = bridge.cv2_to_imgmsg(image_rgb, encoding="bgr8")
            self.image_Publisher.publish(image_msg)
            # rate.sleep()
        except CvBridgeError as e:
            print(e)

        # print("size==============", image_rgb.shape)
        # image_rgb = cv2.resize(image_rgb, (1280,720))
        cv2.imshow("img",image_rgb)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()
        # plt.imshow(image_rgb)
        # plt.savefig(self.img_save_path + f'/img_{self.counter}.jpg')
        # plt.show()
        # plt.close()
        
        self.counter+=1

    def main(self):
        rospy.spin()  # Keep the node alive and listening for incoming images

if __name__ == '__main__':
    rospy.init_node("plant_row_detector", anonymous=True)

    # Initialize the detector
    detector = PlantRowDetector(
        model_path="/home/jurathi2/catkin_ws/src/navigation_pkg/src/line_follow/best.pt",
        img_save_path='/home/jurathi2/catkin_ws/src/navigation_pkg/src/line_follow/',
        mask_coords=(150, 550),
        conf_threshold=0.5,
        height_spacing=10,
        row_dist_threshold=50
    )

    detector.main()