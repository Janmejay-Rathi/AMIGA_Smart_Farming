# AMIGA_SMART_FARMING (Simulation Package)

## Prerequisites

Before running the Amiga Smart Farming Simulation, ensure your system meets the following requirements:

### Operating System
- Ubuntu 20.04 LTS (Focal Fossa)

### ROS
- ROS Noetic Ninjemys (compatible with Ubuntu 20.04)
- Make sure your ROS environment is properly sourced:

```bash
source /opt/ros/noetic/setup.bash
```


### System Requirements for Gazebo Simulation

To smoothly run the Amiga Smart Farming simulation in Gazebo, your system should meet the following minimum specifications:

- **CPU:** Quad-core processor (Intel i5/Ryzen 5 or better)  
- **RAM:** 16 GB or more  
- **GPU:** NVIDIA GPU with at least 4 GB VRAM (for Gazebo GUI and rendering)  
- **Storage:** 20 GB free disk space (for simulation and dependencies)  
- **Additional Software:** `ros-noetic-desktop-full` package installed for full ROS + Gazebo support

---

1. Clone repository and checkout the simulation branch
```
git clone https://github.com/Janmejay-Rathi/AMIGA_Smart_Farming.git
cd ~/AMIGA_Smart_Farming
git checkout simulation
```

2. Make the workspace
```
catkin_make
```

3. Source the workspace
```
source devel/setup.bash
```

4. Run the standard simulation

```
roslaunch amiga_sim sim.launch
```

# Credits

Author: Ken Chen (kenken4016@gmail.com)
