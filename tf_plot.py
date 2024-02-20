#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage

import utm

def convert_utm_to_latlon(x, y, zone_number, zone_letter):
    lat, lon = utm.to_latlon(x, y, zone_number, zone_letter)
    return lat, lon

odom_x = []
odom_y = []
count = 0

def tf_callback(data):
    global odom_x, odom_y, latitude, longitude
    global init_x, init_y, count
    # for transform in data.transforms:
    #     if count == 0:
    #         init_x, init_y = transform.transform.translation.x, transform.transform.translation.y
    #         count += 1
    #     if transform.header.frame_id == "earth":
    #         odom_x.append(transform.transform.translation.x - init_x)
    #         odom_y.append(transform.transform.translation.y - init_y)
        
    #     print(odom_x[-1], odom_y[-1])
    
    for transform in data.transforms:

        if transform.header.frame_id == "earth":
            # odom_x.append(transform.transform.translation.x - init_x)
            # odom_y.append(transform.transform.translation.y - init_y)
        
            print(transform.transform.translation.x, transform.transform.translation.y)

def plot_trajectory():
    plt.figure()
    plt.title("Odom Trajectory")
    plt.plot(odom_x, odom_y, label='Odom')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    rospy.init_node('odom_trajectory_plotter', anonymous=True)
    rospy.Subscriber("/tf", TFMessage, tf_callback)

    rospy.spin()
    plot_trajectory()