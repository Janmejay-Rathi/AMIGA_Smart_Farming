#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Waypoints():
    def __init__(self):
        self.angle_offset_sub = rospy.Subscriber('/robot_heading_angle', Float64, self.angleCB, queue_size=1)
        self.waypoints_pub = rospy.Publisher('/waypoints_in_line',Path,queue_size=1)
        self.distance_sub = rospy.Subscriber('row_distance',Float64, self.distanceCB,queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.spin()

    def angleCB(self, msg):
        self.angle_offset = msg.data
        self.generate_waypoints()
        # print("Angle recieved")

    def distanceCB(self,msg):
        self.distance_offset = msg.data

    def get_waypoints_on_line(self, spacing= 0.1):
        scale = 0.3/139 #in m/pixel
        distance_offset = self.distance_offset*scale  #Change this to actual real world distance offset by converting the pixel distance from the distanceCB to realworld distance using cam transformations
        start,stop,num = 0,5,10
        x_pts = np.linspace(start,stop,num)
        waypoints = [(x,distance_offset) for x in x_pts]
        waypoints_final = waypoints
        # waypoints_final.append(waypoints)
         
        return waypoints_final
    
    def transform_waypoints_to_robot_frame(self, waypoints):
        self.theta = np.deg2rad(self.angle_offset)
        rotation_matrix = np.array([[np.cos(self.theta), -np.sin(self.theta)], 
                                    [np.sin(self.theta), np.cos(self.theta)]])
        transformed_waypoints = []
        for waypoint in waypoints:
            transformed_wp = np.dot(rotation_matrix, np.array(waypoint))
            transformed_waypoints.append(tuple(transformed_wp))
        return transformed_waypoints
    
    def generate_waypoints(self):
        wps = self.get_waypoints_on_line()
        wps_robot_frame = self.transform_waypoints_to_robot_frame(wps)

        path = Path()
        path.header.frame_id = 'base_footprint'

        for waypoint in wps_robot_frame:
            pose = PoseStamped()
            x_rotated,y_rotated = waypoint
            pose.pose.position.x = x_rotated
            pose.pose.position.y = y_rotated 

            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            path.poses.append(pose)
        print("Reference wps:",wps_robot_frame)
        self.waypoints_pub.publish(path)
        # self.rate.sleep()
        # waypoints_pub = []
        # for pose in path.poses:
        #     x = pose.pose.position.x
        #     y = pose.pose.position.y
        #     z = 0.5
        #     # Store the received waypoint in the waypoints list
        #     waypoint = (x,y,z)
        #     waypoints_pub.append(waypoint)
        # rospy.loginfo(f"Received waypoint: {waypoints_pub}")
    # def main(self):
        

if __name__ == '__main__':
    rospy.init_node("Wps_generator_node")
    WPS_gen = Waypoints()
    # WPS_gen.main()

    


