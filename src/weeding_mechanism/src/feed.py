import depthai as dai
import cv2
import numpy as np

# Camera calibration results (replace with your actual values)
camera_matrix = np.array([[565.24788784, 0.0, 609.00543687],
                          [0.0, 564.44266352, 344.98979659],
                          [0.0, 0.0, 1.0]])

dist_coeffs = np.array([[-0.310698084, 0.121052216, 0.000180668097, 0.0000485008167, -0.0234607907]])

# Image size from OAK-D: 1280x720 (W x H)
image_size = (1280, 720)

# Compute optimal new camera matrix 
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
    camera_matrix, dist_coeffs, image_size, alpha=1, newImgSize=image_size)

# Initialize DepthAI pipeline
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
cam.setFps(30)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.video.link(xout.input)

print("[INFO] Starting OAK-D camera stream with undistortion...")

with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=8, blocking=False)

    while True:
        frame = q.get().getCvFrame()

        # Undistort frame
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

        # Optional: crop using ROI to remove black borders
        x, y, w, h = roi
        undistorted_cropped = undistorted[y:y+h, x:x+w]

        # Resize cropped image back to original for display (optional)
        undistorted_resized = cv2.resize(undistorted_cropped, (1280, 720))

        # Show original and undistorted side-by-side
        combined = np.hstack((frame, undistorted_resized))
        # combined = cv2.resize(combined, (1280,1280))
        cv2.imshow("Original (Left) | Undistorted (Right)", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
