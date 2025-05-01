#!/usr/bin/env python3
# slave_oop.py

import rospy
from std_msgs.msg import Bool, Float32
from dynamixel_sdk import *  # Import the Dynamixel SDK
import time


class DynamixelSlaveNode:
    def __init__(self):
        # Dynamixel motor configuration
        self.DXL_ID = 1
        self.BAUDRATE = 57600
        self.DEVICENAME = '/dev/ttyUSB0'
        self.PROTOCOL_VERSION = 2.0
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # Control variables
        self.angle_to_rotate = 512  # Motor steps to rotate (1/8 rotation)
        self.timer = 5  # Seconds before rotating back
        self.flag = False  # Indicates weed detection
        self.rotation_trigger_time = 0  # Calculated time for rotation

        # Timers
        self.timer1_start = None  # Start time for timer1
        self.timer2_start = None  # Start time for timer2

        # Initialize Dynamixel SDK handlers
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port and set baudrate
        if not self.portHandler.openPort():
            rospy.logerr("Failed to open the port.")
            exit()
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            rospy.logerr("Failed to set baudrate.")
            exit()

        # Enable torque
        self.enable_torque()

        # ROS subscribers
        rospy.Subscriber('/weed_detected', Bool, self.weed_detected_callback)
        rospy.Subscriber('/rotation_trigger_time', Float32, self.rotation_trigger_time_callback)

    def enable_torque(self):
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

    def set_motor_position(self, position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, position
        )
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(f"Set position failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            rospy.logerr(f"Error setting position: {self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            rospy.loginfo(f"Motor set to position: {position}")

    def rotate_motor_clockwise(self):
        rospy.loginfo("Rotating motor clockwise.")
        self.set_motor_position(self.angle_to_rotate)

    def rotate_motor_counterclockwise(self):
        rospy.loginfo("Rotating motor counterclockwise.")
        self.set_motor_position(-self.angle_to_rotate)

    def weed_detected_callback(self, msg):
        self.flag = msg.data
        if self.flag:
            rospy.loginfo("Weed detected signal received.")
            self.timer1_start = time.time()  # Start timer1 when weed is detected

    def rotation_trigger_time_callback(self, msg):
        self.rotation_trigger_time = msg.data

    def motor_control_logic(self):
        while not rospy.is_shutdown():
            if self.flag and self.timer1_start:
                elapsed_time1 = time.time() - self.timer1_start

                # Check if it’s time to rotate clockwise
                if elapsed_time1 >= self.rotation_trigger_time:
                    self.rotate_motor_clockwise()
                    self.timer2_start = time.time()  # Start timer2 after clockwise rotation

                    # Wait until timer2 reaches the specified `timer` duration
                    while time.time() - self.timer2_start < self.timer:
                        rospy.sleep(0.1)

                    # Rotate motor back counterclockwise
                    self.rotate_motor_counterclockwise()

                    # Reset the flag and timers after rotation
                    self.flag = False
                    self.timer1_start = None
                    self.timer2_start = None
                    rospy.loginfo("Motor rotation sequence complete, reset timers.")

    def run(self):
        rospy.init_node('slave_oop', anonymous=True)
        rospy.loginfo("Dynamixel slave node started, awaiting weed detection signals.")
        self.motor_control_logic()

    def __del__(self):
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.portHandler.closePort()
        rospy.loginfo("Port closed.")


if __name__ == '__main__':
    try:
        node = DynamixelSlaveNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
