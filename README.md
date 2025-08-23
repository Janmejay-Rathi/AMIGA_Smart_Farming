# AMIGA Smart Farming

This repository contains ROS packages, utility scripts, and configurations developed for the **AMIGA Smart Farming Project**.  
It integrates camera systems, navigation stacks, and data collection tools to enable autonomous lane following, weed detection, and dataset management.

---

## 📂 Repository Structure


---

## 🛠 Utility Folder

The `utility/` folder provides standalone scripts and helper tools for setup, debugging, and data processing.

- **`IP_Config.py`**  
  Configure IP settings (useful for network setup with ROS nodes and external hardware).

- **`OAK-D W PoE Camera Setup_.docx`**  
  Step-by-step documentation for setting up the OAK-D W PoE camera without the canbus.

- **`cameraCheck.py`**  
  Verifies if the OAK-D camera is connected & checks their availability.

- **`oakd_weed.py`**  
  Script for weed detection camera check without jetson.

- **`rosbag_to_video.py`**  
  Converts a recorded ROS bag into a playable video file.

- **`rosbags_to_images.py`**  
  Extracts image frames from a ROS bag and saves them as individual image files.

---

## 🚗 Navigation Package

The `navigation_pkg` provides launch files for navigation-related tasks.

### Launch Files

- **`lane_follow_stack.launch`**  
  Brings up the lane following stack, including required perception, planning, and control nodes.

### Running the Launch File

1. Clone repository and checkout the DellG16
```
git clone -b DellG16 https://github.com/Janmejay-Rathi/AMIGA_Smart_Farming.git
cd AMIGA_Smart_Farming-DellG16
git checkout DellG16
```

2. Build the workspace
```
catkin_make
```

3. Source the workspace
```
source devel/setup.bash
```

4. Launch the navigation files for autonomous navigation
```bash
roslaunch navigation_pkg lane_follow_stack.launch

```
---
# Credits

Author: Janmejay Rathi <br>
Email: janmejayrathi123@gmail.com <br>
Website: https://www.janmejayrathi.me
