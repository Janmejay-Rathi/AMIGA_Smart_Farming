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
import numpy as np

class PneumaticControl:
    def __init__(self):
        # Image params
        self.x_parts = 1  # Number of segments
        self.thresh = 160  # Activation threshold
        self.weeder_tip = 0  # Tip Y position in pixels (top of image)
        self.extension_duration = 0.5
        self.retraction_duration = 0.5
        self.default_time = 1

        # Jetson GPIO pin mappings (BOARD mode)
        self.RELAY_EXTEND_ARM1 = 16
        self.RELAY_RETRACT_ARM1 = 18
        # Add more arms here if needed...

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RELAY_EXTEND_ARM1, GPIO.OUT)
        GPIO.setup(self.RELAY_RETRACT_ARM1, GPIO.OUT)
        # Add setup for more pins if using more pistons

    def pneumatic_control_extend(self, pin_number: int):
        print(f"Extending arm {pin_number}")
        pin = getattr(self, f'RELAY_EXTEND_ARM{pin_number}')
        GPIO.output(pin, GPIO.LOW)
        sleep(self.extension_duration)
        GPIO.output(pin, GPIO.HIGH)

    def pneumatic_control_retract(self, pin_number: int):
        print(f"Retracting arm {pin_number}")
        pin = getattr(self, f'RELAY_RETRACT_ARM{pin_number}')
        GPIO.output(pin, GPIO.LOW)
        sleep(self.retraction_duration)
        GPIO.output(pin, GPIO.HIGH)

    def default_pneumatic_position(self):
        print("Resetting all pistons...")
        for i in range(1, self.x_parts + 1):
            pin = getattr(self, f'RELAY_RETRACT_ARM{i}')
            GPIO.output(pin, GPIO.LOW)
            sleep(self.default_time)
            GPIO.output(pin, GPIO.HIGH)

class WeedDetectionNode(Node):
    def __init__(self):
        super().__init__('weed_detection_node')

        # Load YOLOv8 model
        self.model = YOLO('best.pt')

        # ROS2 Publisher setup (optional, can be used for debugging/viewing)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/oakd/image_raw', 10)

        # Pneumatic controller instance
        self.pc = PneumaticControl()

        # DepthAI camera setup
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.ColorCamera)
        cam.setFps(30)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("video")
        cam.video.link(xout.input)

        self.device = dai.Device(self.pipeline)
        self.q = self.device.getOutputQueue(name="video", maxSize=8, blocking=False)

        # Start the detection loop in a background thread
        threading.Thread(target=self.run_detection_loop, daemon=True).start()

    def run_detection_loop(self):
        while rclpy.ok():
            try:
                frame = self.q.get().getCvFrame()
            except Exception as e:
                self.get_logger().error(f"Error retrieving frame: {e}")
                continue

            height, width, _ = frame.shape
            segment_width = width // self.pc.x_parts
            self.detections = [[] for _ in range(self.pc.x_parts)]

            threads = []

            for i in range(self.pc.x_parts):
                x_start = i * segment_width
                x_end = width if i == self.pc.x_parts - 1 else (i + 1) * segment_width
                roi = frame[:, x_start:x_end]
                thread = threading.Thread(target=self.run_detection_on_segment, args=(roi, i))
                thread.start()
                threads.append(thread)

            # Wait for all threads to complete
            for thread in threads:
                thread.join()

            # (Optional) publish raw frame to ROS for monitoring
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

            # Show the original feed
            cv2.imshow("OAK-D Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    def run_detection_on_segment(self, image_segment, part_index):
        results = self.model(image_segment, verbose=False)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            midpoint = (y1 + y2) // 2  # Vertical midpoint
            distance = abs(midpoint - self.pc.weeder_tip)

            if distance < self.pc.thresh:
                self.detections[part_index].append({
                    'bbox': (x1, y1, x2, y2),
                    'midpoint': midpoint,
                    'distance': distance
                })

                # Draw bounding box
                cv2.rectangle(image_segment, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(image_segment, f"Conf: {box.conf.item():.2f}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # Trigger pneumatic mechanism for this part
                self.pc.pneumatic_control_extend(part_index + 1)
                sleep(4)  # Simulated delay
                self.pc.pneumatic_control_retract(part_index + 1)

                print("---------- One cycle of weed removal completed ----------")

        if self.detections[part_index]:
            self.get_logger().info(f"Detections in part {part_index + 1}: {self.detections[part_index]}")

def main(args=None):
    rclpy.init(args=args)
    node = WeedDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.device.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
