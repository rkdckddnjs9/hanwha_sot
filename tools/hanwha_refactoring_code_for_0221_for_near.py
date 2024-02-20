#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Some codes are modified from the OpenPCDet.
"""

import os
import glob
import datetime
import argparse
from pathlib import Path
from collections import defaultdict
import sys

import numpy as np

import torch
import torch.nn as nn
from livoxdetection.models.ld_base_v1 import LD_base
import pdb
import time
import importlib
import math
from pyquaternion import Quaternion

import copy
import rospy
import ros_numpy
import std_msgs.msg
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import String, Int32, Float32, Int8
from geometry_msgs.msg import PoseStamped, Point
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
import cv2

# custom message
from mtt_msgs.msg import FollowTargetInfo, TargetCandidate

#==== for custom utils =====================
sys.path.insert(0, "/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
sys.path.append("/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
# importlib.import_module("/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
from pub_tracker import PubTracker, NUSCENES_TRACKING_NAMES, greedy_assignment
from hanwha_utils import *
#==========================

def tf_callback_3(data):
    global count, earth2map, map2odom, odom2base, imu2os_sensor, sensor2lidar, earth2os_sensor, init_odom_rt, odom_flag, odom_yaw_list, odom_pitch_list, lidar_z_list, init_pitch, tf_time
    # for global to os_sensor : earth -> map -> odom -> base_link -> imu_link -> os_sensor -> os_lidar
    # base_link2imu_link : translation: [0,0,0], quaternion:[0,0,0,1]
    # imu_link2os_sensor

    for transform in data.transforms:
        tf_time = transform.header.stamp.secs
        if count == 0:
            count += 1
            # tf_static define
            # imu_link -> os_sensor
            imu2os_sensor = np.eye(4)
            odom2base = np.eye(4)
            earth2map = np.eye(4)
            # map -> odom : translation: [0,0,0], quaternion:[0,0,0,1]  <- skip
            map2odom = np.eye(4)
            sensor2lidar = np.eye(4)
            
            imu2os_sensor_t = np.array([1.4, 0.0, -0.37])
            # imu2os_sensor_quaternion = (0., 0., 0., 1.) # real_value: (1.0, 0., 0., 6.12323399574e-17) <- out points are preprocessed!!
            # imu2os_sensor_rot = quaternion_to_rotation_matrix(imu2os_sensor_quaternion)
            # imu2os_sensor[:3, :3] = imu2os_sensor_rot
            imu2os_sensor[:3, 3] = imu2os_sensor_t
        # import pdb; pdb.set_trace()
        
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
            earth2map[:3, :3] = earth2map_rot
            earth2map[:3, 3] = earth2map_t


        # odom -> base_link
        elif transform.header.frame_id == "odom":
            if odom_flag:
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
                odom_flag=False
            odom2base_t = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            
            odom2base_quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            odom2base_rot = quaternion_to_rotation_matrix(odom2base_quaternion)
            odom2base[:3, :3] = odom2base_rot
            odom2base[:3, 3] = odom2base_t    
            
            if len(odom_yaw_list) < 6:
                odom_yaw_list.append(quaternion_to_yaw(odom2base_quaternion))
                odom_pitch_list.append(quaternion_to_pitch(odom2base_quaternion))
            else:
                odom_yaw_list.append(quaternion_to_yaw(odom2base_quaternion))
                odom_pitch_list.append(quaternion_to_pitch(odom2base_quaternion))
                odom_yaw_list.pop(0)
                odom_pitch_list.pop(0)
            
            # tf_pub.publish(TFMessage([transform]))
        
        elif transform.header.frame_id == "os_sensor":
            
            sensor2lidar_t = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            # print(earth2map_t)
            sensor2lidar_quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            sensor2lidar_rot = quaternion_to_rotation_matrix(sensor2lidar_quaternion)
            sensor2lidar[:3, :3] = sensor2lidar_rot
            sensor2lidar[:3, 3] = sensor2lidar_t
            
            if len(lidar_z_list) < 4:
                lidar_z_list.append(sensor2lidar_t)
            else:
                lidar_z_list.append(sensor2lidar_t)
                lidar_z_list.pop(0)
    
    earth2os_sensor = earth2map @ map2odom @ odom2base @ imu2os_sensor @ sensor2lidar
    
    # print("============================")
    # print("earth2map :\n {} \n".format(earth2map))
    # print("============================")
    # print("map2odom :\n {} \n".format(map2odom))
    # print("============================")
    # print("odom2base :\n {} \n".format(odom2base))
    # print("============================")
    # print("imu2os_sensor :\n {} \n".format(imu2os_sensor))
    # print("============================")
    # print("sensor2lidar :\n {} \n".format(sensor2lidar))
    # print("============================")
    # print("earth2os_sensor :\n {} \n".format(earth2os_sensor))
    # print("============================")
    
    
    return earth2os_sensor 

def selection_callback(data):
    global selected_id
    selected_id = int(data.data)  
 
def pub_target_info(data, global_box):
    custom_msg = FollowTargetInfo()
    custom_msg.header.stamp = rospy.Time.now()
    custom_msg.id = Int32(data['pred_id'].item())  #Int32(1)
    custom_msg.class_ = Int8(data['pred_labels'].item())
    custom_msg.score = Float32(data['pred_scores'].item())
    
    box = global_box.tolist()
    quat = yaw2quaternion(box[-1])
    custom_msg.pose.position = Point(box[0], box[1], box[2])
    custom_msg.pose.orientation.x = quat[0]
    custom_msg.pose.orientation.y = quat[1]
    custom_msg.pose.orientation.z = quat[2]
    custom_msg.pose.orientation.w = quat[3]
    
    custom_msg.size = Point(box[3], box[4], box[5])
    custom_msg.velocity = Point(0.0, 0.0, 0.0)

    return custom_msg

def pub_candidate_info(data, global_box):
    custom_msg = TargetCandidate()
    custom_msg.header.stamp = rospy.Time.now()
    
    custom_msg.id_1 = Int32(data['pred_id'][0].item())  #Int32(1)
    custom_msg.class_1 = Int8(data['pred_labels'][0].item())
    custom_msg.position_1 = Point(global_box[0][0], global_box[0][1], global_box[0][2])
    
    custom_msg.id_2 = Int32(data['pred_id'][1].item())  #Int32(1)
    custom_msg.class_2 = Int8(data['pred_labels'][1].item())
    custom_msg.position_2 = Point(global_box[1][0], global_box[1][1], global_box[1][2])
    
    custom_msg.id_3 = Int32(data['pred_id'][2].item())  #Int32(1)
    custom_msg.class_3 = Int8(data['pred_labels'][2].item())
    custom_msg.position_3 = Point(global_box[2][0], global_box[2][1], global_box[2][2])
    
    custom_msg.id_4 = Int32(data['pred_id'][3].item())  #Int32(1)
    custom_msg.class_4 = Int8(data['pred_labels'][3].item())
    custom_msg.position_4 = Point(global_box[3][0], global_box[3][1], global_box[3][2])
    
    custom_msg.id_5 = Int32(data['pred_id'][4].item())  #Int32(1)
    custom_msg.class_5 = Int8(data['pred_labels'][4].item())
    custom_msg.position_5 = Point(global_box[4][0], global_box[4][1], global_box[4][2])

    return custom_msg

def transform_pose_stamped(box):
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "livox_frame"

    x,y,z,w = yaw2quaternion(box[-1])
    pose_stamped.pose.position.x = box[0]
    pose_stamped.pose.position.y = box[1]
    pose_stamped.pose.position.z = box[2]

    pose_stamped.pose.orientation.x = x
    pose_stamped.pose.orientation.y = y
    pose_stamped.pose.orientation.z = z
    pose_stamped.pose.orientation.w = w
    return pose_stamped

# from vis_ros import ROS_MODULE
from vis_ros_hanwha import ROS_MODULE
ros_vis = ROS_MODULE()
last_box_num = 0
last_gtbox_num = 0

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')

    parser.add_argument('--pt', type=str, default=None, help='checkpoint to start from')

    # centerpoint track ==============
    parser.add_argument("--max_age", type=int, default=50) # mot and sot history memory
    parser.add_argument("--hungarian", action='store_true')
    # ==========================
    
    # ==================================
    parser.add_argument("--max_occlusion", type=int, default=40) # occlusion frame (40 = 4s)
    parser.add_argument("--moving_interval", type=int, default=5)
    parser.add_argument("--fps_distance", type=int, default=4) # farthest_point_sampling # for near object (# of points so about near object == large computation of clustering)
    parser.add_argument("--max_ransac_dist", type=int, default=30)
    parser.add_argument("--min_ransac_dist", type=int, default=10)
    parser.add_argument("--max_transfer_dist", type=int, default=5) # transfer box distance for ped
    parser.add_argument("--max_car_transfer_dist", type=int, default=20)
    parser.add_argument("--num_candidate", type=int, default=5)
    # ==================================
    
    args = parser.parse_args()
    return args

class ros_demo():
    def __init__(self, model, args=None):
        self.args = args
        self.model = model
        self.starter, self.ender = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)

        self.offset_angle = 0
        self.offset_ground = 0.0

        self.point_cloud_range = [0, -44.8, -3, 57.6, 44.8, 3]
        self.mot_tracker = PubTracker(max_age=args.max_age, hungarian=args.hungarian)
        self.sot_tracker = PubTracker(max_age=args.max_age, hungarian=args.hungarian)

    def receive_from_ros(self, msg):
        pc = ros_numpy.numpify(msg)
        points_list = np.zeros((pc.shape[0], 4))
        points_list[:, 0] = copy.deepcopy(np.float32(pc['x']))
        points_list[:, 1] = copy.deepcopy(np.float32(pc['y']))
        points_list[:, 2] = copy.deepcopy(np.float32(pc['z']))
        points_list[:, 3] = copy.deepcopy(np.float32(pc['intensity']))

        # preprocess 
        points_list[:, 2] += points_list[:, 0] * np.tan(self.offset_angle / 180. * np.pi) + self.offset_ground
        rviz_points = copy.deepcopy(points_list)
        points_list = mask_points_out_of_range(points_list, self.point_cloud_range)

        input_dict = {
                'points': points_list,
                'points_rviz': rviz_points
                }

        data_dict = input_dict
        return data_dict
    
    def receive_from_ros_custom(self, msg):
        # pc = ros_numpy.numpify(msg)
        
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        x_cond = (cloud_array['x'] >= 0) & (cloud_array['x'] <= 57)
        y_cond = (cloud_array['y'] >= -46) & (cloud_array['y'] <= 46)
        # z_cond = (cloud_array['z'] >= -2) & (cloud_array['z'] <= 4)
        z_cond = (cloud_array['z'] >= -20) & (cloud_array['z'] <= 20)

        # 조건에 맞는 포인트만 필터링
        filtered_array = cloud_array[x_cond & y_cond & z_cond]

        # Z축 값에 -1을 곱함
        filtered_array['z'] *= -1
        
        # pc = ros_numpy.numpify(filtered_array)
        points_list = np.zeros((filtered_array.shape[0], 4))
        points_list[:, 0] = copy.deepcopy(np.float32(filtered_array['x']))
        points_list[:, 1] = copy.deepcopy(np.float32(filtered_array['y']))
        points_list[:, 2] = copy.deepcopy(np.float32(filtered_array['z']))
        points_list[:, 3] = copy.deepcopy(np.float32(filtered_array['intensity']))

        # preprocess 
        points_list[:, 2] += points_list[:, 0] * np.tan(self.offset_angle / 180. * np.pi) + self.offset_ground
        rviz_points = copy.deepcopy(points_list)
        points_list = mask_points_out_of_range(points_list, self.point_cloud_range)

        input_dict = {
                'points': points_list,
                'points_rviz': rviz_points
                }

        data_dict = input_dict
        return data_dict
    
    @staticmethod
    def load_data_to_gpu(batch_dict):
        for key, val in batch_dict.items():
            if not isinstance(val, np.ndarray):
                continue
            else:
                batch_dict[key] = torch.from_numpy(val).float().cuda()

    @staticmethod
    def collate_batch(batch_list, _unused=False):
        data_dict = defaultdict(list)
        for cur_sample in batch_list:
            for key, val in cur_sample.items():
                data_dict[key].append(val)
        batch_size = len(batch_list)
        ret = {}
        for key, val in data_dict.items():
            if key in ['points']:
                coors = []
                for i, coor in enumerate(val):
                    coor_pad = np.pad(coor, ((0, 0), (1, 0)), mode='constant', constant_values=i)
                    coors.append(coor_pad)
                ret[key] = np.concatenate(coors, axis=0)
        ret['batch_size'] = batch_size
        return ret
    
    def numpy_to_imgmsg(self,np_array, encoding='bgr8'):
        
        ros_image = Image()
        
        ros_image.header.stamp = rospy.Time.now()  
        ros_image.height = np_array.shape[0]  
        ros_image.width = np_array.shape[1]
        ros_image.encoding = encoding  
        ros_image.is_bigendian = False 
        ros_image.step = np_array.shape[1] * np_array.shape[2]  
        ros_image.data = np_array.tostring()  
        np_array =[]
        return ros_image

    def online_inference(self, msg):
        global selected_id, _pred_dicts_, temp_selected_id, slc_label, sot_list, count, mask_dist, index2label, ocl_count, cluster_count
        global last_box_num, pred_dicts_sot, init_h, temp_
        # start = time.time()
        torch.cuda.synchronize()
        self.starter.record()
        # data_dict = self.receive_from_ros(msg)
        data_dict = self.receive_from_ros_custom(msg)
        data_infer = ros_demo.collate_batch([data_dict])
        ros_demo.load_data_to_gpu(data_infer)
        mot_time_lag = 1
        sot_time_lag = 1
        pc_viz=False
        
        self.model.eval()
        with torch.no_grad(): 
            pred_dicts = self.model.forward(data_infer)
            # pdb.set_trace()
        pred_dicts[0]['pred_labels'] = torch.where(pred_dicts[0]['pred_labels']==3, 2, pred_dicts[0]['pred_labels']) #cyclist to pedestrian
        
        # data_infer, pred_dicts = ROS_MODULE.gpu2cpu_w_id(data_infer, pred_dicts)
        data_infer, pred_dicts = ROS_MODULE.gpu2cpu(data_infer, pred_dicts)
        
        if selected_id == None:
            if pred_dicts[0]['pred_labels'].shape[0] != 0:
                # =================== mot =======================================
                pred_for_track = transform_pred(pred_dicts[0])
                outputs_mot = self.mot_tracker.step_centertrack(pred_for_track, mot_time_lag)
                pred_dicts_mot = transform_track_np(outputs_mot)
                # ===============================================================
                
                # ==================== sot ======================================
                outputs_sot = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                pred_dicts_sot = transform_track_np(outputs_sot)
                # ===============================================================
                _pred_dicts_ = pred_dicts_mot
                pc_viz=False
                # ===================== MTT msgs publish ===========================================
                # pdb.set_trace()
                if _pred_dicts_[0]['pred_boxes'].shape[0] >= args.num_candidate:
                    pub_dicts = _pred_dicts_.copy()
                    pub_dicts[0]['pred_boxes'] = pub_dicts[0]['pred_boxes'][:args.num_candidate, :]
                    pub_dicts[0]['pred_scores'] = pub_dicts[0]['pred_scores'][:args.num_candidate]
                    pub_dicts[0]['pred_labels'] = pub_dicts[0]['pred_labels'][:args.num_candidate]
                    pub_dicts[0]['pred_id'] = pub_dicts[0]['pred_id'][:args.num_candidate]
                else:
                    # import pdb; pdb.set_trace()
                    pub_dicts = _pred_dicts_.copy()
                    temp_box = np.zeros((5, 7))
                    temp_box[:pub_dicts[0]['pred_boxes'].shape[0], :] = pub_dicts[0]['pred_boxes']
                    temp_score = np.zeros((5,))
                    temp_score[:pub_dicts[0]['pred_scores'].shape[0], ...] = pub_dicts[0]['pred_scores']
                    temp_label = np.zeros((5,), dtype=np.int32)
                    temp_label[:pub_dicts[0]['pred_labels'].shape[0], ...] = pub_dicts[0]['pred_labels']
                    temp_id = np.zeros((5,), dtype=np.int32)
                    temp_id[:pub_dicts[0]['pred_id'].shape[0], ...] = pub_dicts[0]['pred_id']
                    
                    pub_dicts[0]['pred_boxes'] = temp_box
                    pub_dicts[0]['pred_scores'] = temp_score
                    pub_dicts[0]['pred_labels'] = temp_label
                    pub_dicts[0]['pred_id'] = temp_id
                # import pdb; pdb.set_trace()
                # rotated_box = pub_dicts[0]['pred_boxes'].copy()
                # homogeneous_center = np.hstack([rotated_box.copy()[:, :3],np.ones((rotated_box.shape[0],1))])
                # homogeneous_center = (np.linalg.inv(earth2os_sensor) @ homogeneous_center.T).T
                # rotated_box[:, :3] = homogeneous_center[:, :3]
                rotated_box = pub_dicts[0]['pred_boxes'].copy()[:, :3]
                candidate_msg = pub_candidate_info(pub_dicts[0], rotated_box)
                candidate_info_pub.publish(candidate_msg)
                # =======================================================================================
            else:
                pc_viz=True
                _pred_dicts_ = pred_dicts
            
        elif selected_id == -1: # 사용 X
            # _pred_dicts_ = pred_dicts_mot
            if pred_dicts[0]['pred_labels'].shape[0] != 0:
                # =================== mot =======================================
                pred_for_track = transform_pred(pred_dicts[0])
                outputs_mot = self.mot_tracker.step_centertrack(pred_for_track, mot_time_lag)
                pred_dicts_mot = transform_track_np(outputs_mot)
                # ===============================================================

                # ==================== sot ======================================
                outputs_sot = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                pred_dicts_sot = transform_track_np(outputs_sot)
                # ===============================================================
                _pred_dicts_ = pred_dicts_mot
                pc_viz=False
                 # ===================== MTT msgs publish ===========================================
                # pdb.set_trace()
                if _pred_dicts_[0]['pred_boxes'].shape[0] >= args.num_candidate:
                    pub_dicts = _pred_dicts_.copy()
                    pub_dicts[0]['pred_boxes'] = pub_dicts[0]['pred_boxes'][:args.num_candidate, :]
                    pub_dicts[0]['pred_scores'] = pub_dicts[0]['pred_scores'][:args.num_candidate]
                    pub_dicts[0]['pred_labels'] = pub_dicts[0]['pred_labels'][:args.num_candidate]
                    pub_dicts[0]['pred_id'] = pub_dicts[0]['pred_id'][:args.num_candidate]
                else:
                    # import pdb; pdb.set_trace()
                    pub_dicts = _pred_dicts_.copy()
                    temp_box = np.zeros((5, 7))
                    temp_box[:pub_dicts[0]['pred_boxes'].shape[0], :] = pub_dicts[0]['pred_boxes']
                    temp_score = np.zeros((5,))
                    temp_score[:pub_dicts[0]['pred_scores'].shape[0], ...] = pub_dicts[0]['pred_scores']
                    temp_label = np.zeros((5,), dtype=np.int32)
                    temp_label[:pub_dicts[0]['pred_labels'].shape[0], ...] = pub_dicts[0]['pred_labels']
                    temp_id = np.zeros((5,), dtype=np.int32)
                    temp_id[:pub_dicts[0]['pred_id'].shape[0], ...] = pub_dicts[0]['pred_id']
                    
                    pub_dicts[0]['pred_boxes'] = temp_box
                    pub_dicts[0]['pred_scores'] = temp_score
                    pub_dicts[0]['pred_labels'] = temp_label
                    pub_dicts[0]['pred_id'] = temp_id
                # import pdb; pdb.set_trace()
                # rotated_box = pub_dicts[0]['pred_boxes'].copy()
                # homogeneous_center = np.hstack([rotated_box.copy()[:, :3],np.ones((rotated_box.shape[0],1))])
                # homogeneous_center = (np.linalg.inv(earth2os_sensor) @ homogeneous_center.T).T
                # rotated_box[:, :3] = homogeneous_center[:, :3]
                rotated_box = pub_dicts[0]['pred_boxes'].copy()[:, :3]
                candidate_msg = pub_candidate_info(pub_dicts[0], rotated_box)
                candidate_info_pub.publish(candidate_msg)
                # =======================================================================================
            else:
                pc_viz=True
                _pred_dicts_ = pred_dicts
            
        else: #추종 시작
            # import pdb; pdb.set_trace()
            if cluster_count ==0:
                slc_idx = np.where(pred_dicts_sot[0]['pred_id'] == selected_id)[0][0]
                # slc_label = pred_dicts_sot[0]['pred_id'][slc_idx]
                slc_label = pred_dicts_sot[0]['pred_labels'][slc_idx] # 1: vehicle
                x,y,z  = pred_dicts_sot[0]['pred_boxes'][slc_idx][np.newaxis, ...][0][:3]
                init_h = pred_dicts_sot[0]['pred_boxes'][slc_idx][np.newaxis, ...][0][5]
            
                if len(sot_list) < 10:
                    sot_list.append(np.array([[x,y,z]]))
                else:
                    sot_list.append(np.array([[x,y,z]]))
                    sot_list.pop(0)
                cluster_count += 1
            else:
                if slc_label != 1:
                    x,y,z = sot_list[-1][0]
                else:
                    pass
                  
            if slc_label == 1:   #차량 처리 및 전이
                # =================== mot =======================================
                pred_for_track = transform_pred(pred_dicts[0])
                outputs_mot = self.mot_tracker.step_centertrack(pred_for_track, mot_time_lag)
                pred_dicts_mot = transform_track_np(outputs_mot)
                # ===============================================================
                try:
                    slc_idx = np.where(pred_dicts_mot[0]['pred_id'] == selected_id)[0][0]
                    # slc_label = pred_dicts_mot[0]['pred_id'][slc_idx]
                    pred_dicts[0]['pred_boxes'] = pred_dicts_mot[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                    pred_dicts[0]['pred_scores'] = pred_dicts_mot[0]['pred_scores'][slc_idx][np.newaxis, ...]
                    pred_dicts[0]['pred_labels'] = pred_dicts_mot[0]['pred_labels'][slc_idx][np.newaxis, ...]
                    pred_dicts[0]['pred_id'] = pred_dicts_mot[0]['pred_id'][slc_idx][np.newaxis, ...]
                    _pred_dicts_ = pred_dicts
                    if len(sot_list) < 10:
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                    else:
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                        sot_list.pop(0)
                    ocl_count = 0
                except: #이전 결과 추가해주는 것임
                    _pred_dicts_ = _pred_dicts_
                    ocl_count += 1
                    
                    if len(sot_list) < 10:
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                    else:
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                        sot_list.pop(0)
                
                print("ocl_count : {}".format(ocl_count))    
                if ocl_count >= args.max_occlusion:
                    # import pdb; pdb.set_trace()
                    # new object search
                    prev_label = slc_label #_pred_dicts_[0]['pred_labels']
                    prev_obj = sot_list[-1].copy()
                    
                    label_mask = (pred_dicts[0]['pred_labels']==prev_label)
                    if label_mask.sum() != 0: # no same cls obj in current detection result 
                        pred_dicts[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][label_mask]
                        pred_dicts[0]['pred_scores'] = pred_dicts[0]['pred_scores'][label_mask]
                        pred_dicts[0]['pred_labels'] = pred_dicts[0]['pred_labels'][label_mask]
                        
                        dist_mask =  get_target_distance_v2(prev_obj, pred_dicts[0]['pred_boxes']) #distance value return
                        min_dist_idx = np.where(dist_mask[0]==dist_mask.min())[0].item()
                        dist_mask = dist_mask[0] < args.max_car_transfer_dist
                        if dist_mask[min_dist_idx] == True:
                            temp_ = [{}]
                            temp_[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][dist_mask]
                            temp_[0]['pred_scores'] = np.ones(pred_dicts[0]['pred_boxes'].shape[0])[dist_mask]
                            temp_[0]['pred_labels'] = np.full(pred_dicts[0]['pred_boxes'].shape[0], slc_label)[dist_mask]
                            
                            pred_for_track = transform_pred(temp_[0])
                            outputs = self.mot_tracker.step_centertrack(pred_for_track, mot_time_lag)
                            temp_ = transform_track_np(outputs)
                            
                            slc_idx_ = np.isclose(pred_dicts[0]['pred_boxes'][dist_mask][:,np.newaxis, :], temp_[0]['pred_boxes'], atol=1e-6).all(axis=2)
                            slc_idx = np.where(slc_idx_[0]==True)[0][0]
                            slc_label = temp_[0]['pred_labels'][slc_idx]
                            temp_[0]['pred_boxes'] = temp_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                            temp_[0]['pred_scores'] = temp_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                            temp_[0]['pred_labels'] = temp_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                            temp_[0]['pred_id'] = temp_[0]['pred_id'][slc_idx][np.newaxis, ...]
                            
                            if len(sot_list) < 10:
                                sot_list.append(np.array([temp_[0]['pred_boxes'][0][:3]]))
                            else:
                                sot_list.append(np.array([temp_[0]['pred_boxes'][0][:3]]))
                                sot_list.pop(0)
                            # import pdb; pdb.set_trace()
                            _pred_dicts_ = temp_
                            selected_id = _pred_dicts_[0]['pred_id'].item()
                            ocl_count = 0
                        else:
                            ocl_count += 1
                            pc_viz=True
                    else: # same cls obj in current detection result 
                        ocl_count += 1
            else:
                # import pdb; pdb.set_trace()
                # base yaw angle compensation !
                # odom topic average hz : 50 Hz -> delta theta * args.moving_interval(default = 5)
                prev_yaw = odom_yaw_list[-2]
                cur_yaw = odom_yaw_list[-1]
                var_yaw = cur_yaw - prev_yaw
                rot_m = yaw_to_rotation_matrix(var_yaw*args.moving_interval)
                x, y, z = rot_m @ np.array([x,y,z])
                
                prev_pitch = odom_pitch_list[-2]
                cur_pitch = odom_pitch_list[-1]
                var_pitch = cur_pitch - prev_pitch
                rot_m = pitch_to_rotation_matrix(var_pitch*args.moving_interval)
                x, y, z = rot_m @ np.array([x,y,z])
                
                prev_z = lidar_z_list[-2][-1]
                cur_z = lidar_z_list[-1][-1]
                var_z = cur_z - prev_z
                z += var_z
                
                if len(sot_list) > 1:
                    prev_x, prev_y, prev_z = sot_list[-2][0]
                    vx, vy, vz = x - prev_x, y - prev_y, z - prev_z
                    if abs(vx) > 0.3 or abs(vy) > 0.3:
                        x += vx*0.35
                        y += vy*0.35
                        # x += vx*0.4
                        # y += vy*0.4
                else:
                    vx, vy, vz = 0, 0, 0
                
                z = np.mean(np.array(sot_list)[:, :, 2])
                obj_dist = np.sqrt(np.array(x**2+y**2))
                if obj_dist <1.5:
                    box_z_filter = -0.4
                else:
                    box_z_filter = update_dynamic_z_filter_base(sot_list=sot_list)
                print("object center : {} {} {}".format(x,y,z))
                print("obj_dist,  box_z_filter : {} {}".format(obj_dist, box_z_filter))
                
                if ocl_count < args.max_occlusion:
                    try:
                        # if tf_time > 1697522031:
                        #     import pdb; pdb.set_trace()
                        #     print(tf_time)
                        #     # msg.header.stamp.secs
                        cut_range = np.array([x-0.36, y-0.36, z-0.4, x+0.36, y+0.36, z+0.4])
                        temp_pc = mask_points_out_of_range_2(data_infer['points'][:, 1:4], cut_range)

                        if obj_dist < args.fps_distance:
                            start = time.time()
                            temp_pc = farthest_point_sampling_optimized(temp_pc, num_samples=100)
                            
                        
                        # clusters = hdbscan_cluster_obstacles_2(temp_pc)
                        if np.sqrt(np.array(x**2+y**2)) < args.max_ransac_dist and np.sqrt(np.array(x**2+y**2)) > args.min_ransac_dist:
                            start = time.time()
                            temp_pc = remove_ground_points(temp_pc)

                        clusters = cluster_obstacles_2(temp_pc, eps=0.3, min_samples=5)
                        ocl_count = 0
                        bbox_list = calculate_bounding_boxes(clusters)
                        bbox_list = bbox_list[(bbox_list[:, 3]<1.3), :] # l filtering
                        bbox_list = bbox_list[(bbox_list[:, 4]<1.3), :] # w filtering
                        bbox_list = bbox_list[(bbox_list[:, 5]>0.1), :] # h filtering
                        bbox_list = bbox_list[(bbox_list[:, 2]>box_z_filter), :]
                        bbox_list = bbox_list[nms_without_scores(bbox_list, 0.9)[:10]]
                        
                        # # for distance mask
                        dist_mask =  get_target_distance(sot_list[-1].copy(), bbox_list)
                        temp_ = [{}]
                        temp_[0]['pred_boxes'] = bbox_list[dist_mask]
                        temp_[0]['pred_scores'] = np.ones(bbox_list.shape[0])[dist_mask]
                        temp_[0]['pred_labels'] = np.full(bbox_list.shape[0], slc_label)[dist_mask]
                        
                        pred_for_track = transform_pred(temp_[0])
                        outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                        temp_ = transform_track_np(outputs)

                    except: # fully occlusion
                        # ocl_count += 1
                        print("fully occlusion! : {}, {}".format(ocl_count, 30))
                        try:
                            # cut_range = np.array([x-0.7, y-0.7, z-0.4, x+0.7, y+0.7, z+0.5])
                            cut_range = np.array([x-1., y-1., z-0.4, x+1., y+1., z+0.5])
                            temp_pc = mask_points_out_of_range_2(data_infer['points'][:, 1:4], cut_range)
                            if np.sqrt(np.array(x**2+y**2)) < args.fps_distance:
                                start = time.time()
                                temp_pc = farthest_point_sampling_optimized(temp_pc, num_samples=200)
                                print("FPS sampling time :", time.time() - start)
                                
                            clusters = cluster_obstacles_2(temp_pc, eps=0.3, min_samples=5)
                            bbox_list = calculate_bounding_boxes(clusters)
                            # bbox_list = bbox_list[(bbox_list[:, 2]<1.3), :] #center_z filtering
                            bbox_list = bbox_list[(bbox_list[:, 3]<1.3), :] # l filtering
                            bbox_list = bbox_list[(bbox_list[:, 4]<1.3), :] # w filtering
                            bbox_list = bbox_list[(bbox_list[:, 5]>0.1), :] # h filtering
                            bbox_list = bbox_list[(bbox_list[:, 2]>box_z_filter), :]
                            bbox_list = bbox_list[nms_without_scores(bbox_list, 0.9)[:10]]
                            # bbox_list[:, 2] += var_z
                            temp_box = [np.mean(np.array(sot_list)[-3:][:, 0, 0]), np.mean(np.array(sot_list)[-3:][:, 0, 1]), np.mean(np.array(sot_list)[-3:][:, 0, 2])]
                            # dist_mask =  get_target_distance(sot_list[-1].copy(), bbox_list)
                            dist_mask =  get_target_distance(sot_list[-1].copy(), bbox_list)
                            temp_ = [{}]
                            temp_[0]['pred_boxes'] = bbox_list[dist_mask]
                            temp_[0]['pred_scores'] = np.ones(bbox_list.shape[0])[dist_mask]
                            temp_[0]['pred_labels'] = np.full(bbox_list.shape[0], slc_label)[dist_mask]
                            pred_for_track = transform_pred(temp_[0])
                            outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                            temp_ = transform_track_np(outputs)
                            
                            slc_idx = np.where(temp_[0]['pred_id'] == selected_id)[0][0]
                            slc_label = temp_[0]['pred_labels'][slc_idx]
                            temp_[0]['pred_boxes'] = temp_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                            temp_[0]['pred_scores'] = temp_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                            temp_[0]['pred_labels'] = temp_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                            temp_[0]['pred_id'] = temp_[0]['pred_id'][slc_idx][np.newaxis, ...]

                            _pred_dicts_ = temp_
                            
                        except:  
                            ocl_count += 1
                            temp_ = _pred_dicts_
                            pred_for_track = transform_pred(temp_[0])
                            outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                            temp_ = transform_track_np(outputs)
                    
                    slc_idx = np.where(temp_[0]['pred_id'] == selected_id)[0][0]
                    # slc_label = temp_[0]['pred_id'][slc_idx]
                    slc_label = temp_[0]['pred_labels'][slc_idx]
                    temp_[0]['pred_boxes'] = temp_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                    temp_[0]['pred_scores'] = temp_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                    temp_[0]['pred_labels'] = temp_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                    temp_[0]['pred_id'] = temp_[0]['pred_id'][slc_idx][np.newaxis, ...]
                    
                    _pred_dicts_ = temp_

                    if len(sot_list) < 10:
                        sot_list.append(np.array([temp_[0]['pred_boxes'][0][:3]]))
                    else:
                        sot_list.append(np.array([temp_[0]['pred_boxes'][0][:3]]))
                        sot_list.pop(0)

                else: # occlusion!!
                    # import pdb; pdb.set_trace()
                    _pred_dicts_ = _pred_dicts_ # prev
                    print("occlusion !! : {}/{}".format(ocl_count, args.max_occlusion))

                    try: # if no object in current frame
                        label_mask = pred_dicts[0]['pred_labels']==slc_label
                        if label_mask.sum() != 0:
                            pred_dicts[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][label_mask]
                            pred_dicts[0]['pred_scores'] = pred_dicts[0]['pred_scores'][label_mask]
                            pred_dicts[0]['pred_labels'] = pred_dicts[0]['pred_labels'][label_mask]
                            
                            dist_mask =  get_target_distance_v2(sot_list[-1].copy(), pred_dicts[0]['pred_boxes']) #distance value return
                            min_dist_idx = np.where(dist_mask[0]==dist_mask.min())[0].item()
                            dist_mask = dist_mask[0] < args.max_transfer_dist
                            if dist_mask[min_dist_idx] == True:
                                temp_ = [{}]
                                # import pdb; pdb.set_trace()
                                temp_[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][dist_mask]
                                temp_[0]['pred_scores'] = np.ones(pred_dicts[0]['pred_boxes'].shape[0])[dist_mask]
                                temp_[0]['pred_labels'] = np.full(pred_dicts[0]['pred_boxes'].shape[0], slc_label)[dist_mask]
                                
                                # import pdb; pdb.set_trace()
                                pred_for_track = transform_pred(temp_[0])
                                outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                                temp_ = transform_track_np(outputs)
                                
                                slc_idx_ = np.isclose(pred_dicts[0]['pred_boxes'][dist_mask][:,np.newaxis, :], temp_[0]['pred_boxes'], atol=1e-6).all(axis=2)
                                slc_idx = np.where(slc_idx_[0]==True)[0][0]
                                slc_label = temp_[0]['pred_labels'][slc_idx]
                                temp_[0]['pred_boxes'] = temp_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                                temp_[0]['pred_scores'] = temp_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                                temp_[0]['pred_labels'] = temp_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                                temp_[0]['pred_id'] = temp_[0]['pred_id'][slc_idx][np.newaxis, ...]
                                
                                _pred_dicts_ = temp_
                                if len(sot_list) < 10:
                                    sot_list.append(np.array([temp_[0]['pred_boxes'][0][:3]]))
                                else:
                                    sot_list.append(np.array([temp_[0]['pred_boxes'][0][:3]]))
                                    sot_list.pop(0)
                                selected_id = _pred_dicts_[0]['pred_id'].item()
                                ocl_count = 0
                            else:
                                ocl_count += 1
                                pc_viz=True
                        else:
                            ocl_count += 1
                            pc_viz=True
                    except:
                        pc_viz=True

            # local to global transformation ==================================
            # pdb.set_trace()
            rotated_box = _pred_dicts_[0]['pred_boxes'][0].copy()
            homogeneous_center = np.array([0., 0., 0., 1.0])
            homogeneous_center[:3] = rotated_box[:3].copy()
            homogeneous_center = (np.linalg.inv(earth2os_sensor) @ homogeneous_center.T).T
            rotated_box[:3] = homogeneous_center[:3]
            target_msg = pub_target_info(_pred_dicts_[0], rotated_box)
            target_info_pub.publish(target_msg)
            
            # ================================================================
            
        # global last_box_num
        if pc_viz == False:
            if selected_id != None:
                if ocl_count ==0:
                    # 임시 박스 h 변경 
                    _pred_dicts_[0]['pred_boxes'][0, 5] = 1.0
            last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=_pred_dicts_, last_box_num=last_box_num, class_id=_pred_dicts_[0]['pred_id'])
        else:
            ros_vis.ros_print_2(data_dict['points_rviz'][:, 0:4])
        
        # print("time :", time.time() - start)
        self.ender.record()
        torch.cuda.synchronize()
        curr_latency = self.starter.elapsed_time(self.ender)
        if curr_latency >=90.:
            print('det_time(ms): ', curr_latency)

if __name__ == '__main__':
    args = parse_config()
    model = LD_base()

    checkpoint = torch.load(args.pt, map_location=torch.device('cpu'))  
    model.load_state_dict({k.replace('module.', ''):v for k, v in checkpoint['model_state_dict'].items()})
    model.cuda()
    
    # =========== for SOT tracking =====================================
    global count, sot_list, index2label, ocl_count, cluster_count, odom_yaw_list, odom_pitch_list, lidar_z_list, tf_time
    cluster_count=0
    index2label = {0:'Vehicle', 1:'Pedestrian', 2:'Something'}
    ocl_count = 0
    count = 0
    sot_list = []
    selected_id = None
    odom_yaw_list = []
    odom_pitch_list = []
    lidar_z_list = []
    

    # =================================================================
    global earth2map, map2odom, odom2base, imu2os_sensor, odom_flag, init_odom_rt, N_img
    odom_flag = True
    demo_ros = ros_demo(model, args)

    # sub = rospy.Subscriber("/ouster/points_flip", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    sub = rospy.Subscriber("/ouster/points", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    
    slc_track = rospy.Subscriber("/arion/mtt/core/follow_target_id", Int32, selection_callback)
    tf = rospy.Subscriber("/tf", TFMessage, tf_callback_3)
    # output publisher node ========================================================
    
    cls_pub = rospy.Publisher('class_pub', String, queue_size=10)
    id_pub = rospy.Publisher('id_pub', Int32, queue_size=10)
    score_pub = rospy.Publisher('score_pub', Float32, queue_size=10)
    pose_pub = rospy.Publisher('pose_pub', PoseStamped, queue_size=10)
    size_pub = rospy.Publisher('size_pub', Point, queue_size=10)
    vel_pub = rospy.Publisher('vel_pub', Point, queue_size=10)
    
    target_info_pub = rospy.Publisher('/arion/mtt/core/follow_target_info', FollowTargetInfo, queue_size=10)
    candidate_info_pub = rospy.Publisher('/arion/mtt/core/target_candidate', TargetCandidate, queue_size=10)
    
    # tf_pub = rospy.Publisher('tf_odom', TFMessage, queue_size=10)
    
    
    print("set up subscriber!")

    rospy.spin()
    
