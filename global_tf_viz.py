#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage

import utm
from mtt_msgs.msg import FollowTargetInfo, TargetCandidate

def convert_utm_to_latlon(x, y, zone_number, zone_letter):
    lat, lon = utm.to_latlon(x, y, zone_number, zone_letter)
    return lat, lon

odom_x = []
odom_y = []
count = 0

def target_info_callback(data):
    global odom_x, odom_y, latitude, longitude
    global init_x, init_y, count
    # import pdb; pdb.set_trace()
    # if count == 0:
    #     init_x, init_y = data.pose.position.x, data.pose.position.y
    #     count += 1
    # odom_x.append(data.pose.position.x - init_x)
    # odom_y.append(data.pose.position.y - init_y)
    # init_x, init_y = 319733.341874, 3985798.20831
    odom_x.append(data.pose.position.x)
    odom_y.append(data.pose.position.y)
    # odom_x.append(data.pose.position.x - init_x)
    # odom_y.append(data.pose.position.y - init_y)
    print(odom_x[-1], odom_y[-1])
    
    # plt.cla()  # 현재 축을 지웁니다.
    # plt.plot(odom_x, odom_y, label='Odom Trajectory')
    # plt.legend()
    # plt.draw()
    # plt.pause(0.1)  # 짧은 일시 중지로 GUI 이벤트 처리
    
    # if transform.header.frame_id == "earth":
    # if transform.child_frame_id == "map":
    #     # latitude, longitude = convert_utm_to_latlon(transform.transform.translation.x, transform.transform.translation.y, 52, 'N')
    #     latitude, longitude = transform.transform.translation.x, transform.transform.translation.y
    #     odom_x.append(latitude)
    #     odom_y.append(longitude)
    #     print(latitude, longitude)
        # import pdb; pdb.set_trace()
    # plot_trajectory()

def plot_trajectory():
    plt.figure()
    plt.title("Odom Trajectory")
    plt.plot(odom_x, odom_y, label='Odom')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.show()

def update_plot():
    plt.cla()  # 현재 축을 지웁니다.
    plt.plot(odom_x, odom_y, label='Odom Trajectory')
    plt.legend()
    plt.draw()
    plt.pause(0.01)  # GUI 이벤트 처리를 위한 짧은 일시 중지

if __name__ == '__main__':
    rospy.init_node('odom_trajectory_plotter', anonymous=True)
    # rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.Subscriber("/arion/mtt/core/follow_target_info", FollowTargetInfo, target_info_callback)

    plt.ion()  # 대화형 모드 활성화
    plt.figure()
    plt.title("Odom Trajectory")
    plt.xlabel("X")
    plt.ylabel("Y")
    
    # 메인 스레드에서 그래프를 주기적으로 업데이트
    while not rospy.is_shutdown():
        update_plot()

    plt.ioff()  # 대화형 모드 비활성화
    plt.show()  # 최종 그래프 보여주기