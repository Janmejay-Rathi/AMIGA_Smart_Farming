import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import depthai as dai
import cv2
import Jetson.GPIO as GPIO
from time import sleep
import numpy as np

# Parameters
x_parts = 1
thresh = 130  # Threshold to check the difference
thresh2 = 150  # Threshold to check the difference
weeder_tip1 = 0  # Example constant value for the weeder tip
weeder_tip2 = 40 
extension_duration = 0.09
retraction_duration = 0.09

# Jetson GPIO Connections 
RELAY_EXTEND_ARM1 = 18  # Extending the cylinder
RELAY_RETRACT_ARM1 = 16  # Retracting the cylinder

RELAY_EXTEND_ARM2 = 35  # Extending the cylinder
RELAY_RETRACT_ARM2 = 37  # Retracting the cylinder


def pneumatic_control_extend():
    # Logic to extend the pneumatic system
    print(f"Extending pneumatic system for pin 1")
    GPIO.output(RELAY_EXTEND_ARM1, GPIO.LOW)
    sleep(extension_duration)
    GPIO.output(RELAY_EXTEND_ARM1, GPIO.HIGH)

def pneumatic_control_retract():
    # Logic to retract the pneumatic system
    print(f"Retracting pneumatic system for pin 1")
    GPIO.output(RELAY_RETRACT_ARM1, GPIO.LOW)
    sleep(retraction_duration)
    GPIO.output(RELAY_RETRACT_ARM1, GPIO.HIGH)

def pneumatic_control_extend2():
    # Logic to extend the pneumatic system
    print(f"Extending pneumatic system for pin 1")
    GPIO.output(RELAY_EXTEND_ARM2, GPIO.LOW)
    sleep(extension_duration)
    GPIO.output(RELAY_EXTEND_ARM2, GPIO.HIGH)

def pneumatic_control_retract2():
    # Logic to retract the pneumatic system
    print(f"Retracting pneumatic system for pin 1")
    GPIO.output(RELAY_RETRACT_ARM2, GPIO.LOW)
    sleep(retraction_duration)
    GPIO.output(RELAY_RETRACT_ARM2, GPIO.HIGH)


class OakDCameraSubscriber(Node):
    def __init__(self):
        super().__init__('oakd_camera_subscriber')

        # ROS2 Subscriber setup to receive image messages
        self.subscription = self.create_subscription(
            Image,
            '/oakd/image_raw',  # Topic name (adjust as per your topic)
            self.image_callback,
            10
        )

        # Initialize CvBridge to convert ROS image messages to OpenCV format
        self.bridge = CvBridge()

        # Camera calibration parameters
        self.camera_matrix = np.array([[565.24788784, 0.0, 609.00543687],
                                [0.0, 564.44266352, 344.98979659],
                                [0.0, 0.0, 1.0]])

        self.dist_coeffs = np.array([[-0.310698084, 0.121052216, 0.000180668097, 0.0000485008167, -0.0234607907]])

        # Image size from OAK-D: 1280x720 (W x H)
        self.image_size = (1280, 720)

        self.image = None
        self.clicked_point = None

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display the image
        self.image = cv_image
        cv2.imshow(self.window_name, self.image)

        # Print the size of the image (height, width, channels)
        print(f"Image Size: {self.image.shape}")

        # Wait for a key press (1ms delay) to continue processing
        cv2.waitKey(1)

class WeedDetectionNode(Node):
    def __init__(self):
        super().__init__('weed_detection_node')

        # Load YOLOv8 model
        self.model = YOLO('best.pt')

        self.bridge = CvBridge()

        # DepthAI pipeline setup for Oak-D camera
        self.pipeline = dai.Pipeline()

        # Create the color camera node
        self.cam = self.pipeline.create(dai.node.ColorCamera)
        self.cam.setFps(30)

        # Create an output stream for the camera
        self.xout = self.pipeline.create(dai.node.XLinkOut)
        self.cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        self.xout.setStreamName("video")
        self.cam.video.link(self.xout.input)

        # Create and start the DepthAI device with the pipeline
        self.device = dai.Device(self.pipeline)

        # Create ROS2 node instance
        self.oakd_camera_subscriber = OakDCameraSubscriber()

        # Create a publisher to send the image to the ROS2 topic
        self.pub = self.oakd_camera_subscriber.create_publisher(Image, '/oakd/image_raw', 10)

        # Define the bridge to convert DepthAI frame to ROS Image
        self.bridge = CvBridge()

        # Retrieve the output queue to get frames from the Oak-D camera
        self.q = self.device.getOutputQueue(name="video", maxSize=8, blocking=False)

    def run_detection_on_frame(self):
        while rclpy.ok():
            # Get the latest frame from the Oak-D camera
            frame = self.q.get()  # Blocking call

            # Convert the frame to OpenCV format
            frame_data = frame.getCvFrame()

            # Compute optimal new camera matrix
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.oakd_camera_subscriber.camera_matrix, self.oakd_camera_subscriber.dist_coeffs, self.oakd_camera_subscriber.image_size, alpha=1, newImgSize=self.oakd_camera_subscriber.image_size)

            undistorted_frame = cv2.undistort(frame_data, self.oakd_camera_subscriber.camera_matrix, self.oakd_camera_subscriber.dist_coeffs, None, new_camera_matrix)

            # Optional: crop using ROI to remove black borders
            x, y, w, h = roi
            undistorted_cropped = undistorted_frame[y:y+h, x:x+w]

            # Resize cropped image back to original for display (optional)
            undistorted_resized = cv2.resize(undistorted_cropped, (1280, 720))

            # Run YOLOv8 detection on the frame
            results = self.model(undistorted_resized, conf = 0.70)[0]

            # Process detections and draw bounding boxes
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf
                midpoint = (y1 + y2) // 2  # Midpoint of the bounding box
                x_midpoint = (x1 + x2) // 2  # Midpoint of the bounding box

                distance1 = abs(weeder_tip1 - midpoint)
                distance2 = abs(weeder_tip2 - midpoint)


                if distance1 < thresh : 
                    print(f"Detection: Conf: {confidence.item():.2f}, BBox: ({x1}, {y1}), ({x2}, {y2})")
                    # Draw bounding box on the frame
                    cv2.rectangle(undistorted_resized, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(undistorted_resized, f"Conf: {confidence.item():.2f}", (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    pneumatic_control_extend()  # Example of extending the pneumatic control
                    sleep(2)
                    pneumatic_control_retract()
                    print("----------one cycle of weed removal completed---------")

                if distance2 < thresh2 and x_midpoint > 900: 
                    print(f"Detection: Conf: {confidence.item():.2f}, BBox: ({x1}, {y1}), ({x2}, {y2})")
                    # Draw bounding box on the frame
                    cv2.rectangle(undistorted_resized, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(undistorted_resized, f"Conf: {confidence.item():.2f}", (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    pneumatic_control_extend2()  # Example of extending the pneumatic control
                    sleep(2)
                    pneumatic_control_retract2()
                    print("----------one cycle of weed removal completed---------")


            # Convert the frame to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame_data, encoding="bgr8")

            # Publish the image to the topic
            self.pub.publish(ros_image)

            # Display the frame with bounding boxes
            cv2.imshow("Oak-D Camera Feed with YOLO", undistorted_resized)

            # Press 'q' to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()


def main(args=None):
    # Initialize ROS2 node
    rclpy.init(args=args)

    # Create WeedDetectionNode instance
    weed_detection_node = WeedDetectionNode()

    try:
        weed_detection_node.run_detection_on_frame()
    except KeyboardInterrupt:
        pass
    finally:
        weed_detection_node.device.close()
        rclpy.shutdown()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(RELAY_EXTEND_ARM1, GPIO.OUT)
    GPIO.setup(RELAY_RETRACT_ARM1, GPIO.OUT)
    GPIO.setup(RELAY_EXTEND_ARM2, GPIO.OUT)
    GPIO.setup(RELAY_RETRACT_ARM2, GPIO.OUT)
    main()
