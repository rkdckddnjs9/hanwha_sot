#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import ros_numpy

def point_cloud_callback(msg):
    # PointCloud2 메시지를 numpy 배열로 변환
    #cloud_points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
    # cloud_points = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    
    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

    # Z축 값에 -1을 곱함
    # modified_points = [(p[0], p[1], (p[2] * -1)-1.6, p[3]) for p in cloud_points]
    
    # 필터링 조건 설정
    # x_cond = (cloud_array['x'] >= 0) & (cloud_array['x'] <= 57)
    # y_cond = (cloud_array['y'] >= -46) & (cloud_array['y'] <= 46)
    # # z_cond = (cloud_array['z'] >= -2) & (cloud_array['z'] <= 4)
    # z_cond = (cloud_array['z'] >= -3) & (cloud_array['z'] <= 3)
    
    
    x_cond = (cloud_array['x'] >= 0) & (cloud_array['x'] <= 57)
    y_cond = (cloud_array['y'] >= -46) & (cloud_array['y'] <= 46)
    # z_cond = (cloud_array['z'] >= -2) & (cloud_array['z'] <= 4)
    z_cond = (cloud_array['z'] >= -20) & (cloud_array['z'] <= 20)

    # 조건에 맞는 포인트만 필터링
    filtered_array = cloud_array[x_cond & y_cond & z_cond]

    # Z축 값에 -1을 곱함
    filtered_array['z'] *= -1
    # filtered_array['z'] += -1.6
    # filtered_array['z'] += -2.0
    

    
    # fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #           PointField('y', 4, PointField.FLOAT32, 1),
    #           PointField('z', 8, PointField.FLOAT32, 1),
    #           PointField('intensity', 12, PointField.FLOAT32, 1)]

    # 수정된 데이터로 새 PointCloud2 메시지 생성
    header = msg.header
    # modified_cloud = pc2.create_cloud_xyz32(header, modified_points)
    # modified_cloud = pc2.create_cloud(header, fields, modified_points)
    modified_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(filtered_array, stamp=rospy.Time.now(), frame_id=msg.header.frame_id)

    # 수정된 PointCloud2 메시지를 publish
    pub.publish(modified_cloud)

if __name__ == '__main__':
    rospy.init_node('invert_z_axis_point_cloud')

    # PointCloud2 메시지를 구독
    rospy.Subscriber("/ouster/points", PointCloud2, point_cloud_callback)

    # 수정된 PointCloud2 메시지를 publish할 퍼블리셔 생성
    pub = rospy.Publisher("/ouster/points_flip", PointCloud2, queue_size=1)

    rospy.spin()