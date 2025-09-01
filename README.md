# Weeding Mechanism (ROS 2)

This package contains scripts for controlling and testing **weeding mechanisms** in the AMIGA Smart Farming project.  
Although some scripts currently use ROS 2 publishers (e.g., for the OAK-D camera), this is not strictly required at this stage.  

### Why ROS 2?
If your pipeline is as simple as:  
**OAK-D camera → YOLO detection → GPIO actuation**,  
then a single Python script with **DepthAI + YOLO + Jetson.GPIO + OpenCV** is sufficient and lighter.  

ROS 2 becomes useful if:
- You need multiple modules working together (e.g., perception + actuation + navigation).  
- You want future scalability and modularity.  
- You want to use ROS tools for debugging and visualization.  
- You plan to integrate the weeding mechanism with navigation or other robotic systems.  

Using ROS 2 adds some overhead (nodes, publishers/subscribers, QoS), but enables greater flexibility for future expansions.

---
## 📂 Folder Structure
```
AMIGA_Smart_Farming/
├── build/
├── install/
├── log/
├── src/
│   └── weeding_mechanism/
│       └── src/
│           ├── Final_dual_arm_basic_weeding.py
│           ├── best.pt
│           ├── cam_calibration.py
│           ├── camtest.py
│           ├── feed.py
│           ├── gpio_test.py
│           ├── pressure_test.py
│           ├── weeding_test.py
│           ├── weeding_test_final.py
│           └── yolo_weed_test.py
├── Weeding-mechanism_CircuitDiagram.png
└── README.md

```


---

## 📜 File Descriptions

- **`Final_dual_arm_basic_weeding.py`**  
  Main script for testing **dual-arm weeding mechanism** activation.

- **`weeding_test_final.py`**  
  Script for **single-arm weeding mechanism** activation and testing.

- **`yolo_weed_test.py`**  
  Runs the trained YOLOv8 model on camera input to detect weeds.  
  Used to verify **model accuracy** and performance before integration.

- **`best.pt`**  
  The trained **YOLOv8 model weights** for detecting artificial weeds (testing phase).

- **`cam_calibration.py`**  
  Script to calibrate the OAK-D camera using the **OpenCV checkerboard method**.

- **`camtest.py`**  
  Simple test to verify if **camera calibration was successful**.

- **`feed.py`**  
  Publishes OAK-D camera feed into ROS 2 as a topic (though this is not strictly required at this stage).

- **`pressure_test.py`**  
  Tests **different pneumatic cylinder pressure levels** for actuation control.

---

## 🚀 Usage

> ⚠️ Before running scripts, ensure you have installed dependencies:  
> - ROS 2 (Humble/Foxy or compatible)  
> - DepthAI  
> - OpenCV  
> - Ultralytics YOLOv8  
> - Jetson.GPIO (for Jetson-based GPIO control)


## 🌱 Weeding Mechanism Package

The `weeding_mechanism` package provides scripts for testing and running the robotic weeding system, including dual-arm control, YOLO-based weed detection, GPIO testing, and camera utilities.

---

### Clone the Repository (Jetson Orin Branch)
```bash
git clone -b JetsonOrin https://github.com/Janmejay-Rathi/AMIGA_Smart_Farming.git
cd AMIGA_Smart_Farming
git checkout JetsonOrin
```

#### 1. Final Dual Arm Basic Weeding
Main script to run the dual-arm weeding mechanism.  
```bash
cd src/weeding_mechanism/src/
python3 Final_dual_arm_basic_weeding.py
```

## 🚗 Navigation Package (ROS 2 – Jetson Orin Branch)
The `navigation_pkg` in this branch is a **ROS 2 migrated version** of the original `navigation_pkg` from the `DellG16` branch (ROS 1).  
It provides the navigation pipeline for lane detection, waypoint following, and MPC-based control.

### ⚠️ **Disclaimer:**  
This package is a **work-in-progress ROS 2 migration**.  
Due to how the `amiga-ros2-bridge` was developed, there may be **latency issues (~5 seconds delay)** in the navigation pipeline.  
This can cause the robot’s response to commands to be slightly delayed. Expect possible bugs and use with caution.

---

### 📥 Clone the Repository (Jetson Orin Branch)
```bash
git clone -b JetsonOrin https://github.com/Janmejay-Rathi/AMIGA_Smart_Farming.git
cd AMIGA_Smart_Farming
git checkout JetsonOrin
```

### ▶️ Running the Navigation Pipeline

Unlike the ROS 1 version, there is no single `.launch` file for the navigation stack in ROS 2.  
You need to **launch the bridge nodes first**, followed by the navigation scripts in the correct order.

---

#### Building and sourcing the ROS2 workspace
```bash
colcon build
source install/setup.bash
```

#### Launch the `amiga-ros2-bridge` nodes through different terminals individually
```bash
ros2 launch amiga-ros2-bridge amiga_stream.launch.py
ros2 launch amiga-ros2-bridge twist_control.launch.py
```

#### Launch the nodes for ROS2 migrated vision based navigation through different terminals individually
```bash
ros2 run navigation_pkg lane_detect_top_ROS.py
ros2 run navigation_pkg waypoints_lines_follow.py
ros2 run navigation_pkg new_mpc.py
```

---
# Credits

Author: Janmejay Rathi <br>
Email: janmejayrathi123@gmail.com <br>
Website: https://www.janmejayrathi.me

