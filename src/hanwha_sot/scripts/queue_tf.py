#!/usr/bin/env python
import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
import queue
import threading
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from hanwha_utils import *

class TFProcessor:
    def __init__(self):
        self.queue = queue.Queue()
        self.thread = threading.Thread(target=self.process_queue)
        self.thread.daemon = True
        self.thread.start()
        self.count = 0
        self.earth2map = np.eye(4)
        self.odom2base = np.eye(4)
        self.sensor2lidar = np.eye(4)
        self.odom_flag = True
        self.odom_roll_list = []
        self.odom_yaw_list = []
        self.odom_pitch_list = []
        
        self.earth2map_publisher = rospy.Publisher("/arion/mtt/core/earth2map", Float64MultiArray, queue_size=10)
        self.map2odom_publisher = rospy.Publisher("/arion/mtt/core/map2odom", Float64MultiArray, queue_size=10)
        self.odom2base_publisher = rospy.Publisher("/arion/mtt/core/odom2base", Float64MultiArray, queue_size=10)
        
        self.odom_roll_list_publisher = rospy.Publisher("/arion/mtt/core/odom_roll_list", Float64MultiArray, queue_size=10)
        self.odom_yaw_list_publisher = rospy.Publisher("/arion/mtt/core/odom_yaw_list", Float64MultiArray, queue_size=10)
        self.odom_pitch_list_publisher = rospy.Publisher("/arion/mtt/core/odom_pitch_list", Float64MultiArray, queue_size=10)
    
    def publish_earth2map(self):
        # self.map2odom publish
        earth2map_msg = Float64MultiArray()
        earth2map_msg.layout.dim.append(MultiArrayDimension())
        earth2map_msg.layout.dim[0].size = self.earth2map.shape[0]
        earth2map_msg.layout.dim[0].stride = self.earth2map.shape[0] * self.earth2map.shape[1]
        earth2map_msg.layout.dim[0].label = "earth2map"
        earth2map_msg.data = self.earth2map.flatten().tolist()
        self.earth2map_publisher.publish(earth2map_msg) 
    
    def publish_odom2base(self):
        # self.map2odom publish
        odom2base_msg = Float64MultiArray()
        odom2base_msg.layout.dim.append(MultiArrayDimension())
        odom2base_msg.layout.dim[0].size = self.odom2base.shape[0]
        odom2base_msg.layout.dim[0].stride = self.odom2base.shape[0] * self.odom2base.shape[1]
        odom2base_msg.layout.dim[0].label = "odom2base"
        odom2base_msg.data = self.odom2base.flatten().tolist()
        self.odom2base_publisher.publish(odom2base_msg) 
        
    def publish_map2odom(self):
        # self.map2odom publish
        map2odom_msg = Float64MultiArray()
        map2odom_msg.layout.dim.append(MultiArrayDimension())
        map2odom_msg.layout.dim[0].size = self.map2odom.shape[0]
        map2odom_msg.layout.dim[0].stride = self.map2odom.shape[0] * self.map2odom.shape[1]
        map2odom_msg.layout.dim[0].label = "map2odom"
        map2odom_msg.data = self.map2odom.flatten().tolist()
        self.map2odom_publisher.publish(map2odom_msg) 
        
    def publish_roll_list(self):
        # self.odom_roll_list publish
        odom_roll_list_msg = Float64MultiArray()
        odom_roll_list_msg.layout.dim.append(MultiArrayDimension())
        odom_roll_list_msg.layout.dim[0].size = len(self.odom_roll_list)
        odom_roll_list_msg.layout.dim[0].stride = len(self.odom_roll_list)
        odom_roll_list_msg.layout.dim[0].label = "odom_roll_list"
        odom_roll_list_msg.data = self.odom_pitch_list
        self.odom_roll_list_publisher.publish(odom_roll_list_msg)
         
    def publish_pitch_list(self):
        # self.odom_yaw_list publish
        odom_pitch_list_msg = Float64MultiArray()
        odom_pitch_list_msg.layout.dim.append(MultiArrayDimension())
        odom_pitch_list_msg.layout.dim[0].size = len(self.odom_pitch_list)
        odom_pitch_list_msg.layout.dim[0].stride = len(self.odom_pitch_list)
        odom_pitch_list_msg.layout.dim[0].label = "odom_pitch_list"
        odom_pitch_list_msg.data = self.odom_pitch_list
        self.odom_pitch_list_publisher.publish(odom_pitch_list_msg)
    
    def publish_yaw_list(self):
        # self.odom_yaw_list publish
        odom_yaw_list_msg = Float64MultiArray()
        odom_yaw_list_msg.layout.dim.append(MultiArrayDimension())
        odom_yaw_list_msg.layout.dim[0].size = len(self.odom_yaw_list)
        odom_yaw_list_msg.layout.dim[0].stride = len(self.odom_yaw_list)
        odom_yaw_list_msg.layout.dim[0].label = "odom_yaw_list"
        odom_yaw_list_msg.data = self.odom_yaw_list
        self.odom_yaw_list_publisher.publish(odom_yaw_list_msg)

    def process_queue(self):
        while not rospy.is_shutdown():
            data = self.queue.get()
            self.tf_callback_3(data)
            self.queue.task_done()

    def tf_callback_3(self, data):
        for transform in data.transforms:            
            # earth -> map tf matrix
            if transform.header.frame_id == "earth":
                earth2map_t = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
                # print(earth2map_t)
                earth2map_quaternion = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
                earth2map_rot = quaternion_to_rotation_matrix(earth2map_quaternion)
                self.earth2map[:3, :3] = earth2map_rot
                self.earth2map[:3, 3] = earth2map_t
                self.publish_earth2map()

            # odom -> base_link
            elif transform.header.frame_id == "odom":
                if self.odom_flag:
                    init_odom_rt = np.eye(4)
                    init_odom_rt[:3, 3] = np.array([transform.transform.translation.x, transform.transform.translation.y, 0.])
                    init_odom_r = (
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w
                    )
                    init_pitch = quaternion_to_pitch(init_odom_r)
                    init_odom_r = quaternion_to_rotation_matrix(init_odom_r)
                    init_odom_rt[:3, :3] = init_odom_r
                    self.odom_flag=False
                odom2base_t = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
                
                odom2base_quaternion = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
                odom2base_rot = quaternion_to_rotation_matrix(odom2base_quaternion)
                self.odom2base[:3, :3] = odom2base_rot
                self.odom2base[:3, 3] = odom2base_t    
                # print("odom2base_t : {}".format(odom2base_t))
                if len(self.odom_yaw_list) < 6:
                    self.odom_roll_list.append(quaternion_to_roll(odom2base_quaternion))
                    self.odom_yaw_list.append(quaternion_to_yaw(odom2base_quaternion))
                    self.odom_pitch_list.append(quaternion_to_pitch(odom2base_quaternion))
                else:
                    self.odom_roll_list.append(quaternion_to_roll(odom2base_quaternion))
                    self.odom_yaw_list.append(quaternion_to_yaw(odom2base_quaternion))
                    self.odom_pitch_list.append(quaternion_to_pitch(odom2base_quaternion))
                    self.odom_roll_list.pop(0)
                    self.odom_yaw_list.pop(0)
                    self.odom_pitch_list.pop(0)
                self.publish_odom2base()
                self.publish_roll_list()
                self.publish_yaw_list()
                self.publish_pitch_list()
                # tf_pub.publish(TFMessage([transform]))

    def callback(self, msg):
        self.queue.put(msg)

if __name__ == '__main__':
    rospy.init_node('tf_processor_node')
    tf_processor = TFProcessor()
    rospy.Subscriber("/tf", TFMessage, tf_processor.callback)

    rospy.loginfo("set up tf processor")
    rospy.spin()
