import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jetson-amiga/ros2_ws/install/amiga_ros2_bridge'
