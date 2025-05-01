#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from pyproj import Proj, Transformer
# from geographiclib.geodesic import Geodesic
# from geographiclib.utm import UTM
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

# Initialize global variables for storing points
x_points = []
y_points = []

class GPSPlotter:
    def __init__(self):
        # Subscriber to NavSatFix topic
        rospy.Subscriber('/gps/pvt', NavSatFix, self.navsatfix_callback)
        self.gps_xy_loc_pub = rospy.Publisher('/gps_xy_loc',Float32MultiArray,queue_size=1)
        self.starter = 0
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32633", always_xy=True)  # WGS84 to UTM

    
    def navsatfix_callback(self, msg):
        latitude, longitude, altitude = msg.latitude, msg.longitude, msg.altitude
        x, y = self.latlon_to_utm(latitude, longitude)
        if self.starter==0:
            self.x_offset = x
            self.y_offset = y
            x = 0
            y = 0
            # self.heading_angle = np.atan()
            self.current_x = x
            self.current_y = y
            self.starter = 1
        else:
            x = x - self.x_offset
            y = y - self.y_offset

        # print(current_y_offset,current_x_offset)
        x_final = -x
        y_final = y
        x_points.append(x_final)
        y_points.append(y_final)
        print("Current x,y: ",x_final,y_final)

        msg = Float32MultiArray()
        msg.data = [x_final,y_final]
        self.gps_xy_loc_pub.publish(msg)

        self.plot_trajectory()

    def latlon_to_utm(self, latitude, longitude):
        # Use UTM projection (WGS84)
        proj_utm = Proj(proj='utm', zone=15, ellps='WGS84', preserve_units=False) #zone 16 or 15
        x, y = proj_utm(longitude, latitude)
        return x, y
    
    # def latlon_to_utm(self, latitude, longitude):
    #     # Automatically handle UTM zone conversion
    #     utm = UTM()
    #     u = utm.Forward(latitude, longitude)
    #     return u['easting'], u['northing']

    # def latlon_to_utm(self, latitude, longitude):
    #     # Convert latitude, longitude to UTM coordinates
    #     x, y = self.transformer.transform(longitude, latitude)  # lon, lat order
    #     return x, y

    def plot_trajectory(self):
        plt.plot(y_points, x_points, marker='o')
        plt.title("GPS Trajectory")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.grid(True)
        plt.pause(0.001)  # For dynamic update
        plt.draw()
        plt.savefig('/home/cosmos/catkin_ws_amiga/src/navigation_pkg/src/gps.jpg')

def main():
    rospy.init_node('gps_plotter', anonymous=True)
    gps_plotter = GPSPlotter()
    
    # Keep the program alive
    rospy.spin()

if __name__ == '__main__':
    main()

#1st run - Ushape: front-right-right
# length: 20-25ft
# width: 6-8 ft

#2nd run - Ushape: front-right-right
#l: 10-11 ft
#w: 5-6 ft

#3rd run - Lshape: front and then right
#l: 25 ft
#w: 19.5 ft