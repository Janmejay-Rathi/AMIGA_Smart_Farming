import depthai as dai
import cv2
import numpy as np
import os
from datetime import datetime
# import yaml  # Uncomment if you want to save calibration results to a file

# Calibration settings
CHECKERBOARD = (8, 6)  # Number of inner corners (columns, rows)
square_size = 2.2  # Real-world square size in cm

# Output directory for captured images
output_dir = "calibration_images"
os.makedirs(output_dir, exist_ok=True)

# Create DepthAI pipeline
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
cam.setFps(30)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.video.link(xout.input)

captured_images = []

print("[INFO] Starting OAK-D camera stream...")
with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=8, blocking=False)

    while True:
        frame = q.get().getCvFrame()
        display_frame = frame.copy()

        cv2.putText(display_frame, "Press 'c' to capture, 'e' to calibrate, 'q' to quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("OAK-D Camera", display_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            # Save captured image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
            filename = os.path.join(output_dir, f"image_{timestamp}.jpg")
            cv2.imwrite(filename, frame)
            captured_images.append(frame)
            print(f"[CAPTURED] {filename}")

        elif key == ord('e'):
            print("[INFO] Starting calibration...")
            obj_points = []
            img_points = []

            # Prepare object points grid (same for all images)
            objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * square_size

            valid_images = 0
            for idx, img in enumerate(captured_images):
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

                display = img.copy()
                if ret:
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    img_points.append(corners2)
                    obj_points.append(objp)
                    valid_images += 1

                    cv2.drawChessboardCorners(display, CHECKERBOARD, corners2, ret)
                    print(f"[OK] Detected corners in image {idx + 1}")
                else:
                    print(f"[SKIPPED] Could not detect corners in image {idx + 1}")

                cv2.imshow("Checkerboard Detection", display)
                cv2.waitKey(500)

            cv2.destroyWindow("Checkerboard Detection")

            if valid_images >= 5:
                ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                    obj_points, img_points, gray.shape[::-1], None, None)

                print("\n[RESULT] Camera Calibration Successful:")
                print("Camera Matrix:\n", camera_matrix)
                print("Distortion Coefficients:\n", dist_coeffs)

                # Uncomment below to save to file
                """
                data = {
                    'camera_matrix': camera_matrix.tolist(),
                    'dist_coeff': dist_coeffs.tolist()
                }
                with open("calib.yaml", "w") as f:
                    yaml.dump(data, f)
                print("[SAVED] Calibration data saved to 'calib.yaml'")
                """

            else:
                print("[ERROR] Not enough valid images for calibration. Need at least 5.")
            break

        elif key == ord('q'):
            break

    cv2.destroyAllWindows()
