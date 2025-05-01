import depthai as dai
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

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

        # Window for displaying the image
        self.window_name = 'Oak-D Camera Image'
        cv2.namedWindow(self.window_name)

        # Mouse callback to capture coordinates when clicking on the image
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

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

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button click
            self.clicked_point = (x, y)
            print(f"Clicked at: x={x}, y={y}")

def main(args=None):
    # Initialize ROS2 node
    rclpy.init(args=args)

    # Setup DepthAI pipeline for Oak-D camera
    pipeline = dai.Pipeline()

    # Create a camera node and configure it for color stream
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setFps(30)

    # Create an output stream for the image
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    cam.video.link(xout.input)

    # Create and start the DepthAI device with the pipeline
    device = dai.Device(pipeline)

    # Create ROS2 node instance
    oakd_camera_subscriber = OakDCameraSubscriber()

    # Create a publisher to send the image to the ROS2 topic
    pub = oakd_camera_subscriber.create_publisher(Image, '/oakd/image_raw', 10)

    # Define the bridge to convert DepthAI frame to ROS Image
    bridge = CvBridge()

    # Retrieve the output queue to get frames from the Oak-D camera
    q = device.getOutputQueue(name="video", maxSize=8, blocking=False)

    # ROS2 spin for handling subscriber events
    while rclpy.ok():
        # Get the latest frame from the Oak-D camera
        frame = q.get()  # Blocking call

        # Convert the frame to OpenCV format
        frame_data = frame.getCvFrame()

        # Convert OpenCV image to ROS Image message
        ros_image = bridge.cv2_to_imgmsg(frame_data, encoding="bgr8")

        # Publish the image to the topic
        pub.publish(ros_image)

        rclpy.spin_once(oakd_camera_subscriber)

    # Cleanup and shutdown
    oakd_camera_subscriber.destroy_node()
    rclpy.shutdown()
    device.close()

    # Close OpenCV window
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
