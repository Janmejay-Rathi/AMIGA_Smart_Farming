import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import threading
import depthai as dai
import cv2
import Jetson.GPIO as GPIO
from time import sleep

# Parameters
x_parts = 4
scale_factor = 10   # Find the saling factor
thresh = 5  # Set the threshold value for activate time
weeder_tip = 10 * scale_factor # Set the weeder tip point in meters from the bottom end of image
extension_duration = 0.5
retraction_duration = 0.5

# Jetson GPIO Connections 
RELAY_EXTEND_ARM1 = 16  # Extending the cylinder
RELAY_RETRACT_ARM1 = 18  # Retracting the cylinder

def pneumatic_control_extend(pin_number: int):
    # Logic to extend the pneumatic system
    print(f"Extending pneumatic system for pin {pin_number}")
    GPIO.output(RELAY_EXTEND_ARM1, GPIO.LOW)
    sleep(extension_duration)
    GPIO.output(RELAY_EXTEND_ARM1, GPIO.HIGH)

def pneumatic_control_retract(pin_number: int):
    # Logic to retract the pneumatic system
    print(f"Retracting pneumatic system for pin {pin_number}")
    GPIO.output(RELAY_RETRACT_ARM1, GPIO.LOW)
    sleep(retraction_duration)
    GPIO.output(RELAY_RETRACT_ARM1, GPIO.HIGH)

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

        # Set the camera resolution to 720P (since 1080P might not be supported)
        self.cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720P)

        # Create an output stream for the camera
        self.xout = self.pipeline.create(dai.node.XLinkOut)
        self.xout.setStreamName("video")
        self.cam.video.link(self.xout.input)

        # Create and start the DepthAI device with the pipeline
        self.device = dai.Device(self.pipeline)

        # Create ROS2 node instance
        oakd_camera_subscriber = OakDCameraSubscriber()

        # Create a publisher to send the image to the ROS2 topic
        self.pub = oakd_camera_subscriber.create_publisher(Image, '/oakd/image_raw', 10)

        # Define the bridge to convert DepthAI frame to ROS Image
        self.bridge = CvBridge()

        # Retrieve the output queue to get frames from the Oak-D camera
        self.q = self.device.getOutputQueue(name="video", maxSize=8, blocking=False)

    def run_detection_on_segment(self, image_segment, part_index):
        # Run YOLOv8 detection on the image segment
        results = self.model(image_segment, verbose=False)[0]

        # Process detections and check if they are within threshold
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            midpoint = (x1 + x2) // 2  # Midpoint of the bounding box
            distance = abs(weeder_tip - midpoint)

            # If the distance is less than the threshold, append the bounding box to the respective list
            if distance < thresh:
                # Add bounding box to the corresponding list based on x-coordinate
                self.detections[part_index].append({
                    'bbox': (x1, y1, x2, y2),
                    'midpoint': midpoint,
                    'distance': distance
                })

                # Trigger pneumatic control based on the detection
                pin_number = part_index  # Use part index as a pin number for control
                pneumatic_control_extend(pin_number)  # Example of extending the pneumatic control
                sleep(4)
                pneumatic_control_retract(pin_number)
                print("----------one cycle of weed removal completed---------")

                # Draw bounding box on the frame
                cv2.rectangle(image_segment, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(image_segment, f"Conf: {box.conf.item():.2f}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Log the detections in the part
        if self.detections[part_index]:
            self.get_logger().info(f"Detections in part {part_index}: {self.detections[part_index]}")

    def image_callback(self, msg):
        try:
            # Get the latest frame from DepthAI camera
            frame = self.q.get().getCvFrame()
        except Exception as e:
            self.get_logger().error(f"Error while getting frame from DepthAI: {e}")
            return

        height, width, _ = frame.shape
        segment_width = width // x_parts

        # Process each frame and divide it into segments
        for i in range(x_parts):
            x_start = i * segment_width
            x_end = width if i == x_parts - 1 else (i + 1) * segment_width
            roi = frame[:, x_start:x_end]
            threading.Thread(target=self.run_detection_on_segment, args=(roi, i)).start()

def main(args=None):
    # Initialize ROS2 node
    rclpy.init(args=args)
    # Create WeedDetectionNode instance
    weed_detection_node = WeedDetectionNode()

    try:
        # Start processing the frames and detection logic
        rclpy.spin(weed_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Proper shutdown
        weed_detection_node.device.close()
        rclpy.shutdown()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(RELAY_EXTEND_ARM1, GPIO.OUT)
    GPIO.setup(RELAY_RETRACT_ARM1, GPIO.OUT)
    main()
