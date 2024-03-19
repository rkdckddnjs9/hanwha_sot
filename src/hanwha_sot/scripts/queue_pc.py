#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import ros_numpy
import queue
import threading
class PointCloudProcessor:
    def __init__(self):
        self.queue = queue.Queue()
        self.thread = threading.Thread(target=self.process_queue)
        self.thread.daemon = True
        self.thread.start()

    def process_queue(self):
        while not rospy.is_shutdown():
            msg = self.queue.get()
            self.receive_from_ros_custom(msg)
            self.queue.task_done()

    def receive_from_ros_custom(self, msg): #self.point_cloud_range = [0, -44.8, -3, 57.6, 44.8, 3]
        cloud_array = np.copy(ros_numpy.point_cloud2.pointcloud2_to_array(msg))
        cloud_array['z'] *= -1
        cloud_array['y'] *= -1
        
        x_cond = (cloud_array['x'] >= 0) & (cloud_array['x'] <= 57.599)
        y_cond = (cloud_array['y'] >= -44.799) & (cloud_array['y'] <= 44.799)
        z_cond = (cloud_array['z'] >= -1.999) & (cloud_array['z'] <= 3.999)
        # z_cond = (cloud_array['z'] >= -3) & (cloud_array['z'] <= 3)

        filtered_array = cloud_array[x_cond & y_cond & z_cond]

        
        # modified_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(filtered_array, stamp=rospy.Time.now(), frame_id=msg.header.frame_id)
        modified_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(filtered_array, frame_id=msg.header.frame_id)
        pub.publish(modified_cloud)

    def callback(self, msg):
        self.queue.put(msg)
        
if __name__ == '__main__':
    rospy.init_node('invert_yz_axis_point_cloud')
    processor = PointCloudProcessor()
    rospy.Subscriber("/ouster/points", PointCloud2, processor.callback)

    pub = rospy.Publisher("/arion/mtt/core/points_flip", PointCloud2, queue_size=1)
    
    rospy.loginfo("set up point cloud preprocessor")

    rospy.spin()
