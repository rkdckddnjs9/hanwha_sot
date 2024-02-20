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
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PoseStamped, Point
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
import cv2
from cv_bridge import CvBridge

# custom message
# from my_msgs.msg import FollowTargetInfo, TargetCandidate

#==== for custom utils =====================
sys.path.insert(0, "/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
sys.path.append("/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
# importlib.import_module("/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
from pub_tracker import PubTracker, NUSCENES_TRACKING_NAMES, greedy_assignment
from hanwha_utils import *
#==========================

def tf_callback_3(data):
    global count, earth2map, map2odom, odom2base, imu2os_sensor, sensor2lidar, earth2os_sensor, init_odom_rt, odom_flag, odom_yaw_list, odom_pitch_list, lidar_z_list, init_pitch
    # for global to os_sensor : earth -> map -> odom -> base_link -> imu_link -> os_sensor -> os_lidar
    # base_link2imu_link : translation: [0,0,0], quaternion:[0,0,0,1]
    # imu_link2os_sensor

    for transform in data.transforms:
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
    return earth2os_sensor 

def selection_callback(data):
    global selected_id
    selected_id = int(data.data)  

def signal_img_callback(data):
    global bridge, signal_img
    # signal_img = bridge.imgmsg_to_cv2(data, "32FC1") ############3 민재코드
    signal_img = ros_numpy.numpify(data)
    normalized_data = cv2.normalize(signal_img, None, 0, 255, cv2.NORM_MINMAX)
    uint8_data = normalized_data.astype(np.uint8)
    signal_img = cv2.flip(uint8_data, 0)

def nearir_img_callback(data):
    global bridge, nearir_img
    # nearir_img = bridge.imgmsg_to_cv2(data, "32FC1") ############3 민재코드, 불필요
    nearir_img = ros_numpy.numpify(data)
    normalized_data = cv2.normalize(nearir_img, None, 0, 255, cv2.NORM_MINMAX)
    uint8_data = normalized_data.astype(np.uint8)
    nearir_img = cv2.flip(uint8_data, 0)

def ref_img_callback(data):
    global model_2d, bridge, ref_img
    # ref_img = bridge.imgmsg_to_cv2(data, "32FC1") ############3 민재코드, 불필요
    ref_img = ros_numpy.numpify(data)
    normalized_data = cv2.normalize(ref_img, None, 0, 255, cv2.NORM_MINMAX)
    uint8_data = normalized_data.astype(np.uint8)
    ref_img = cv2.flip(uint8_data, 0)

def range_img_callback(data):
    global model_2d, bridge, range_img
    # uint8_data = bridge.imgmsg_to_cv2(data, "mono8") ############3 민재코드, 불필요
    cv_img = ros_numpy.numpify(data)
    normalized_data = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
    uint8_data = normalized_data.astype(np.uint8)
    uint8_data = cv2.flip(uint8_data, 0)
    range_img = uint8_data
    
    # range_img = bridge.imgmsg_to_cv2(data, "mono8")
    # range_img = ros_numpy.numpify(data)
#####################################################################################################
######################################## 민재코드 ###################################################
#####################################################################################################
### 3D 박스 레이블
# 1 = Vehicle, 2 = Pedestrian
### 2D 박스 레이블
# coco데이터셋 80(1번 ~ 80번)개 레이블
# 3번 Car는 Vehicle로, 나머지 레이블은 Pedestrian으로 생각하고 코드 구성
### IoU 시각화
# 2D 박스와 겹치는 3Dto2D박스가 없는 경우, 두 박스 모두 시각화 하지 않음
# 2D박스가 3Dto2D 박스 내부에 있는 경우 IoU가 1이기 때문에 이상한 박스가 매칭되어 시각화 될 수 있음

def find_best_pairs_with_same_class(iou_matrix, class_ids_a, class_ids_b, box2d, box3dto2d, iter):
    """
    iou_matrix: An N x M matrix of IoU scores.
    class_ids_a: An N x 1 array of class IDs for boxesA.
    class_ids_b: An M x 1 array of class IDs for boxesB.
    """
    # imgees = cv2.imread(f'./2D3D_vis/ori_imgs/range_img_{iter}.png')
    # green_color = (0,255,0)
    # red_color = (0,0,255)
    # for best_idx in range(len(box2d)):
    #     best_2d_ = box2d[best_idx].int()
    #     cv2.rectangle(imgees, tuple(best_2d_[:2].tolist()), tuple(best_2d_[2:].tolist()),red_color)
    #     cv2.imwrite(f'./2D3D_vis/2D3D_imgs/range_img_{iter}.png', imgees)
        # import pdb; pdb.set_trace()
    
    # 가장 높은 IoU 값을 가진 인덱스 찾기 (N개의 박스 각각에 대해)
    max_iou_indices = np.argmax(iou_matrix, axis=1)
    max_iou_values = np.max(iou_matrix, axis=1)

    # 결과를 저장할 리스트 초기화
    selected_pairs = [] # (N박스 번호, M박스 번호, IoU값) # IoU값 순서로 sorting한 뒤 top-5개 뽑으면 되겠다.

    
    # # 각 N개 박스에 대해 반복
    # selected_N = []
    # selected_M = []
    # selected_IoU = []
    
    # # iou_threshold = 0.1
    # # iou_matrix[iou_matrix<iou_threshold] = 0
    # class_ids_a[class_ids_a == 1] = 10
    # class_ids_a[class_ids_a == 2] = 1 # vehicle
    # class_ids_a[class_ids_a == 0] = 2 # person
    # for N_idx in range(len(class_ids_a)):
    #     # 최대 IoU 값을 가진 박스의 인덱스와 클래스 ID
    #     if max_iou_values[N_idx] == 0:
    #         # 3Dto2D와 IoU가 없는 2D는 시각화 안함
    #         continue
    #     M_idx = max_iou_indices[N_idx]
    #     if class_ids_a[N_idx]  == class_ids_b[M_idx]:
    #         # 2D가 차 또는 사람이고 3D도 차 또는 사람이면 그냥 비주얼라이즈
    #         selected_IoU.append(max_iou_values[N_idx])
    #         selected_N.append(N_idx)
    #         selected_M.append(M_idx)
    #     else:
    #         # 3D는 차 또는 사람인데, 2D는 전혀다른 레이블이면 IoU가 큰거 비주얼라이즈
    #         import pdb; pdb.set_trace()
    #         # 하나는 vehicle, 하나는 pedestrian으로 예측한 것의 IoU가 클 경우
    #         # 클래스 ID가 다르면, 동일한 클래스를 가진 다른 박스 찾기
    #         # 해당 클래스와 동일한 모든 B 박스의 인덱스를 찾음
    #         same_class_indices = np.where(class_ids_b == class_ids_a[N_idx])[0]
    #         # 동일한 클래스를 가진 박스들 중에서 최대 IoU 값을 가진 박스 찾기
    #         if len(same_class_indices) > 0:
    #             # IoU 값 중 최대값을 가진 인덱스 찾기
    #             ious = iou_matrix[N_idx, same_class_indices]
    #             if ious.max() == 0.0:
    #                 continue
    #             M_idx = same_class_indices[np.argmax(ious)]
    #             selected_IoU.append(ious.max())
    #             selected_N.append(N_idx)
    #             selected_M.append(M_idx)
    
    
    # 각 N개 박스에 대해 반복
    selected_N = []
    selected_M = []
    selected_IoU = []
    for N_idx in range(len(class_ids_a)):
        if max_iou_values[N_idx] == 0:
            # 3Dto2D와 IoU가 없는 2D는 시각화 안함
            continue
        M_idx = max_iou_indices[N_idx]
        selected_IoU.append(max_iou_values[N_idx])
        selected_N.append(N_idx)
        selected_M.append(M_idx)
        

    return np.array(selected_N), np.array(selected_M), np.array(selected_IoU)


def bb_intersection_over_union_vectorized(boxesA, boxesB):
    """
    Vectorized computation of IoU between each pair of two sets of boxes, boxesA and boxesB.
    """
    # Expand dimensions to support broadcasting for intersection computation
    boxesA_expanded = np.expand_dims(boxesA, axis=1)  # Shape will be [N, 1, 4]
    boxesB_expanded = np.expand_dims(boxesB, axis=0)  # Shape will be [1, M, 4]

    # Compute the intersection coordinates
    xA = np.maximum(boxesA_expanded[..., 0], boxesB_expanded[..., 0])
    yA = np.maximum(boxesA_expanded[..., 1], boxesB_expanded[..., 1])
    xB = np.minimum(boxesA_expanded[..., 2], boxesB_expanded[..., 2])
    yB = np.minimum(boxesA_expanded[..., 3], boxesB_expanded[..., 3])

    # Compute the area of intersection rectangle
    interArea = np.maximum(0, xB - xA + 1) * np.maximum(0, yB - yA + 1)

    # Compute the area of both the prediction and ground-truth rectangles
    boxAArea = (boxesA_expanded[..., 2] - boxesA_expanded[..., 0] + 1) * (boxesA_expanded[..., 3] - boxesA_expanded[..., 1] + 1)
    boxBArea = (boxesB_expanded[..., 2] - boxesB_expanded[..., 0] + 1) * (boxesB_expanded[..., 3] - boxesB_expanded[..., 1] + 1)

    # Compute the intersection over union by taking the intersection area and dividing it by the sum of prediction + ground-truth areas - the intersection area
    iou = interArea / (boxAArea + boxBArea - interArea)

    return iou


def calculate_3d_box_corners(box3d):
    x, y, z, l, w, h, heading = box3d
    # 3D 상자의 꼭지점을 추출하기 위해 상자의 회전 행렬을 계산합니다.
    rotation_matrix = np.array([
        [np.cos(heading), -np.sin(heading), 0],
        [np.sin(heading), np.cos(heading), 0],
        [0, 0, 1]
    ])

    # 3D 상자의 꼭지점을 계산합니다.
    half_l = l / 2
    half_w = w / 2
    half_h = h / 2

    # 3D 상자의 꼭지점 좌표를 계산합니다.
    box_corners = np.array([
        [-half_l, -half_w, -half_h],
        [half_l, -half_w, -half_h],
        [half_l, half_w, -half_h],
        [-half_l, half_w, -half_h],
        [-half_l, -half_w, half_h],
        [half_l, -half_w, half_h],
        [half_l, half_w, half_h],
        [-half_l, half_w, half_h]
    ])

    # 회전 행렬을 적용하여 3D 상자의 꼭지점을 회전시킵니다.
    rotated_box_corners = np.dot(box_corners, rotation_matrix.T)

    # 중심 좌표 (x, y, z)를 추가하여 최종 3D 상자의 꼭지점 좌표를 얻습니다.
    final_box_corners = rotated_box_corners + np.array([x, y, z])

    return final_box_corners


def transform_3d_to_2d(box3ds):
    vertical_span = {
        'start_angle': 22.5,
        'end_angle': -22.5,
        'step': 0.3515625
    }
    
    # horizontal_span = {
    #     'start_angle': -180,
    #     'end_angle': 180,
    #     'step': 0.3515625
    # }
    
    horizontal_span = {
        'start_angle': -90,
        'end_angle': 90,
        'step': 0.3515625/2
    }
    
    box3dto2d = np.zeros((box3ds.shape[0], 4), dtype=np.float32)
    for idx, box3d in enumerate(box3ds):
        box_corner = calculate_3d_box_corners(box3d) #[8,3]
        theta = np.degrees(np.arctan2(box_corner[:, 2], np.sqrt(box_corner[:, 0]**2 + box_corner[:, 1]**2)))
        phi = np.degrees(np.arctan2(box_corner[:, 1], box_corner[:, 0]))

        vertical_indices = ((vertical_span['start_angle'] - theta) / vertical_span['step']).astype(int)
        horizontal_indices = ((phi - horizontal_span['start_angle']) / horizontal_span['step']).astype(int)
        
        horizontal_indices = 1024 - horizontal_indices #for flip
        
        lt = (horizontal_indices.min(), vertical_indices.min())
        rb = (horizontal_indices.max(), vertical_indices.max())
        ltrb = (horizontal_indices.min(), vertical_indices.min(), horizontal_indices.max(), vertical_indices.max())
        box3dto2d[idx] = ltrb
        
    # cv2.rectangle(range_view_image, lt, rb, 255, 2)
    return box3dto2d
#####################################################################################################
######################################## 민재코드 ###################################################
#####################################################################################################



from vis_ros import ROS_MODULE
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
    parser.add_argument("--max_transfer_dist", type=int, default=5) # transfer box distance
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
        self.bridge = CvBridge()
        self.iter = 0  ############## 민재코드
        self.eta = 0 ############## 민재코드

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
        global last_box_num, pred_dicts_sot, init_h, temp_, img_pub, nearir_img, range_img, signal_img, ref_img
        # start = time.time()
        torch.cuda.synchronize()
        self.starter.record()
        data_dict = self.receive_from_ros(msg)
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
                # import pdb; pdb.set_trace()
                # cv2.imwrite(f'./2D3D_vis/ori_imgs/range_img_{self.iter}.png', nearir_img) ############# 민재코드
                
                nearir_img_ = nearir_img.copy()
                nearir_img_ = cv2.applyColorMap(nearir_img_,cv2.COLORMAP_COOL)
                img_stack = np.vstack([nearir_img[np.newaxis,...],range_img[np.newaxis,...],ref_img[np.newaxis,...]])
                
                # img_stack = cv2.flip(img_stack, 0)
                # result_2d_1 = model_2d(signal_img)
                result_2d_1 = model_2d(img_stack)
                
                ####################### 민재코드 #######################
                st = time.time()
                box3d = pred_dicts[0]['pred_boxes'] # (x, y, z, w, ?, ?, rot)
                box3d_cls = pred_dicts[0]['pred_labels'].reshape(-1, 1) # (class_id)
                box2d = result_2d_1.pred[0].cpu() # (xmin, ymin, xmax, ymax, ????, class_id)
                box2d_cls = box2d[:, 5:6]
                # transform_pc2range
                box3dto2d = transform_3d_to_2d(box3ds=box3d)
                # compute IoU matrix using the vectorized approach
                iou_matrix_vectorized = bb_intersection_over_union_vectorized(box2d[:, :4], box3dto2d)
                if iou_matrix_vectorized.shape[0] != 0:
                    best_N, best_M, best_IoU = find_best_pairs_with_same_class(iou_matrix_vectorized, box2d_cls, box3d_cls, box2d[:, :4], box3dto2d, self.iter)
                    # if self.iter==132:
                        # import pdb; pdb.set_trace()
                    # 내림차순 정렬 인덱스, top-5개
                    sort_idx = np.argsort(best_IoU)[::-1][:5] 
                    
                    # sort_idx에 따라 best_N, best_M, best_IoU 정렬
                    best_N_sorted = best_N[sort_idx]
                    best_M_sorted = best_M[sort_idx]
                    best_IoU_sorted = best_IoU[sort_idx]
                    
                    best_2d = box2d[:, :4][best_N_sorted]
                    best_3dto2d = box3dto2d[best_M_sorted]
                    imgees = nearir_img_ #cv2.imread(f'./2D3D_vis/ori_imgs/range_img_{self.iter}.png')
                    green_color = (0,255,0)
                    red_color = (0,0,255)
                    for best_idx in range(len(best_IoU)):
                        best_2d_ = best_2d[best_idx].int()
                        best_3dto2d_ = best_3dto2d[best_idx].astype(int)
                        cv2.rectangle(imgees, tuple(best_2d_[:2].tolist()), tuple(best_2d_[2:].tolist()),red_color)
                        cv2.rectangle(imgees, tuple(best_3dto2d_[:2].tolist()), tuple(best_3dto2d_[2:].tolist()), green_color)
                        # cv2.imwrite(f'./2D3D_vis/2D3D_imgs/range_img_{self.iter}.png', imgees)
                        # import pdb; pdb.set_trace()
                else:
                    # 매칭된 박스가 없을땐 그냥 기존 이미지 저장
                    # cv2.imwrite(f'./2D3D_vis/2D3D_imgs/range_img_{self.iter}.png', nearir_img)
                    pass
                self.iter += 1
                
                ####################### 민재코드 #######################

                
                # print(signal_img.mean())
                # result_2d_1 = model_2d(img)
                print(result_2d_1.pred[0].shape)
                # result_2d_2 = model_2d(uint8_data)
                # result_2d = torch.cat((result_2d_1.pred[0], result_2d_2.pred[0]), dim=0)
                # print(result_2d_1.pred[0].shape, result_2d_2.pred[0])
                if result_2d_1.pred[0].shape[0] >= 1:
                    # pdb.set_trace()
                    # print("=============fuck : {} ".format(nearir_img_.mean()))
                    # pdb.set_trace()

                    # nearir_img_ = cv2.applyColorMap(nearir_img_,cv2.COLORMAP_COOL)
                    # range_pred = result_2d_1.pred[0]
                    # for i, box in enumerate(range_pred):
                    #     x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3]) 
                    #     cv2.rectangle(nearir_img_, (x1, y1), (x2, y2), (0, 255, 0), 2) #128 1024
                    
                    # for_pub_img = self.bridge.cv2_to_imgmsg(nearir_img_, "bgr8") ######################## 민재코드 (주석처리함)
                    for_pub_img = self.numpy_to_imgmsg(nearir_img_) ######################## 민재코드
                    for_pub_img.header.frame_id = 'livox_frame'
                    for_pub_img.header.stamp = rospy.Time.now()
                    img_pub.publish(for_pub_img) 
                    # # h_fov = (-180, 180)  # 수평 FOV
                    # h_fov = (-90, 90)  # 수평 FOV
                    # # h_fov = (270, 90)
                    # # h_fov = (0, 360)
                    # v_fov = (-45, 45)  # 수직 FOV
                    # h_angles, v_angles = calculate_angles(range_img.shape[0], range_img.shape[1], h_fov, v_fov)
                    # box_2d_to_3d = bboxes_to_3d_coordinates(range_img, range_pred, h_angles, v_angles)
  
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
                        
                if ocl_count >= args.max_occlusion:
                    # import pdb; pdb.set_trace()
                    # new object search
                    prev_label = slc_label #_pred_dicts_[0]['pred_labels']
                    prev_obj = sot_list[-1].copy()
                    
                    slc_label_ = (pred_dicts_mot[0]['pred_labels']==prev_label)
                    if slc_label_.sum() == 0: # no same cls obj in current detection result 
                        pass
                    else: # same cls obj in current detection result 
                        mask_dist = get_target_distance(prev_obj, pred_dicts_mot[0]['pred_boxes'])
                        mask_dist = mask_dist*slc_label_
                        temp_selected_id = pred_dicts_mot[0]['pred_id'][mask_dist].reshape(-1)
                        selected_id = temp_selected_id
                        
                        # slc_idx update
                        slc_idx = np.where(pred_dicts_mot[0]['pred_id'] == temp_selected_id)[0][0]
                        pred_dicts_mot[0]['pred_boxes'] = pred_dicts_mot[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                        pred_dicts_mot[0]['pred_scores'] = pred_dicts_mot[0]['pred_scores'][slc_idx][np.newaxis, ...]
                        pred_dicts_mot[0]['pred_labels'] = pred_dicts_mot[0]['pred_labels'][slc_idx][np.newaxis, ...]
                        pred_dicts_mot[0]['pred_id'] = pred_dicts_mot[0]['pred_id'][slc_idx][np.newaxis, ...]
                        _pred_dicts_ = pred_dicts_mot
                        
                        sot_list = []
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                
                       
            else:
                # base yaw angle compensation !
                # odom topic average hz : 50 Hz -> delta theta * args.moving_interval(default = 5)
                prev_yaw = odom_yaw_list[-2]
                cur_yaw = odom_yaw_list[-1]
                var_yaw = cur_yaw - prev_yaw
                rot_m = yaw_to_rotation_matrix(var_yaw*args.moving_interval)
                x, y, z = rot_m @ np.array([x,y,z])
                
                # var_yaw = (odom_yaw_list[-1]-odom_yaw_list[-2]) + (odom_yaw_list[-2]-odom_yaw_list[-3]) + (odom_yaw_list[-3]-odom_yaw_list[-4]) + (odom_yaw_list[-4]-odom_yaw_list[-5]) + (odom_yaw_list[-6]-odom_yaw_list[-5])
                # rot_m = yaw_to_rotation_matrix(var_yaw)
                # x, y, z = rot_m @ np.array([x,y,z])
                
                prev_pitch = odom_pitch_list[-2]
                cur_pitch = odom_pitch_list[-1]
                var_pitch = cur_pitch - prev_pitch
                rot_m = pitch_to_rotation_matrix(var_pitch*args.moving_interval)
                x, y, z = rot_m @ np.array([x,y,z])
                
                # var_pitch = (odom_pitch_list[-1]-odom_pitch_list[-2]) + (odom_pitch_list[-2]-odom_pitch_list[-3]) + (odom_pitch_list[-3]-odom_pitch_list[-4]) + (odom_pitch_list[-4]-odom_pitch_list[-5]) + (odom_yaw_list[-6]-odom_yaw_list[-5])
                # rot_m = pitch_to_rotation_matrix(var_pitch)
                # x, y, z = rot_m @ np.array([x,y,z])
                
                prev_z = lidar_z_list[-2][-1]
                cur_z = lidar_z_list[-1][-1]
                var_z = cur_z - prev_z
                z += var_z
                
                if ocl_count < args.max_occlusion:
                    try:
                        cut_range = np.array([x-0.36, y-0.36, z-0.4, x+0.36, y+0.36, z+0.5])
                        temp_pc = mask_points_out_of_range_2(data_infer['points'][:, 1:4], cut_range)

                        if np.sqrt(np.array(x**2+y**2)) < args.fps_distance:
                            # import pdb; pdb.set_trace()
                            start = time.time()
                            # temp_pc = farthest_point_sampling_optimized(temp_pc, num_samples=200)
                            temp_pc = farthest_point_sampling_optimized(temp_pc, num_samples=100)
                            print("FPS sampling time :", time.time() - start)
                            # tt_ = np.ones((400, 4))
                            # tt_[:, :3] = farthest_point_sampling_optimized(temp_pc, num_samples=200)
                            # ros_vis.ros_print_2(tt_)
                            
                        
                        # clusters = hdbscan_cluster_obstacles_2(temp_pc)
                        if np.sqrt(np.array(x**2+y**2)) < args.max_ransac_dist and np.sqrt(np.array(x**2+y**2)) > args.min_ransac_dist:
                            start = time.time()
                            temp_pc = remove_ground_points(temp_pc)
                            print("ground remove time :", time.time() - start)
                            
                            # tt_ = np.ones((400, 4))
                            # tt_[:, :3] = temp_pc
                            # ros_vis.ros_print_2(tt_)
                            
                        
                        
                        clusters = cluster_obstacles_2(temp_pc, eps=0.3, min_samples=5)
                        ocl_count = 0
                        bbox_list = calculate_bounding_boxes(clusters)
                        bbox_list = bbox_list[(bbox_list[:, 3]<1.3), :] # l filtering
                        bbox_list = bbox_list[(bbox_list[:, 4]<1.3), :] # w filtering
                        bbox_list = bbox_list[(bbox_list[:, 5]>0.1), :] # h filtering
                        bbox_list = bbox_list[nms_without_scores(bbox_list, 0.9)[:10]]
                        
                        # # for distance mask
                        # import pdb; pdb.set_trace()
                        dist_mask =  get_target_distance(sot_list[-1].copy(), bbox_list)
                        temp_ = [{}]
                        temp_[0]['pred_boxes'] = bbox_list[dist_mask]
                        temp_[0]['pred_scores'] = np.ones(bbox_list.shape[0])[dist_mask]
                        temp_[0]['pred_labels'] = np.full(bbox_list.shape[0], slc_label)[dist_mask]
                        
                        pred_for_track = transform_pred(temp_[0])
                        outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                        temp_ = transform_track_np(outputs)
                        # import pdb; pdb.set_trace()
                    except: # fully occlusion
                        # ocl_count += 1
                        print("fully occlusion! : {}, {}".format(ocl_count, 30))
                        try:
                            cut_range = np.array([x-0.7, y-0.7, z-0.4, x+0.7, y+0.7, z+0.5])
                            temp_pc = mask_points_out_of_range_2(data_infer['points'][:, 1:4], cut_range)
                            if np.sqrt(np.array(x**2+y**2)) < args.fps_distance:
                                start = time.time()
                                temp_pc = farthest_point_sampling_optimized(temp_pc, num_samples=200)
                                print("FPS sampling time :", time.time() - start)
                                # tt_ = np.ones((400, 4))
                                # tt_[:, :3] = farthest_point_sampling_optimized(temp_pc, num_samples=200)
                                # ros_vis.ros_print_2(tt_)
                                
                            clusters = cluster_obstacles_2(temp_pc, eps=0.3, min_samples=5)
                            bbox_list = calculate_bounding_boxes(clusters)
                            bbox_list = bbox_list[(bbox_list[:, 2]<1.3), :] #center_z filtering
                            bbox_list = bbox_list[(bbox_list[:, 3]<1.3), :] # l filtering
                            bbox_list = bbox_list[(bbox_list[:, 4]<1.3), :] # w filtering
                            bbox_list = bbox_list[(bbox_list[:, 5]>0.1), :] # h filtering
                            bbox_list = bbox_list[(bbox_list[:, 2]>-0.3), :]
                            bbox_list = bbox_list[nms_without_scores(bbox_list, 0.9)[:10]]
                            # bbox_list[:, 2] += var_z
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
                            
                            # filtering_viz
                            # import pdb; pdb.set_trace()
                            # tt_ = np.ones((temp_pc.shape[0], 4))
                            # tt_[:, :3] = temp_pc
                            # ros_vis.ros_print_2(tt_)
                            
                        except:  
                            ocl_count += 1
                            temp_ = _pred_dicts_
                            pred_for_track = transform_pred(temp_[0])
                            outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                            temp_ = transform_track_np(outputs)
                            # import pdb; pdb.set_trace()
                    # import pdb; pdb.set_trace()
                    
                    slc_idx = np.where(temp_[0]['pred_id'] == selected_id)[0][0]
                    # slc_label = temp_[0]['pred_id'][slc_idx]
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

                    _pred_dicts_ = temp_
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
                                # last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=temp_, last_box_num=last_box_num, class_id=temp_[0]['pred_id'])
                                
                                # slc_idx = np.where(temp_[0]['pred_boxes'] == pred_dicts[0]['pred_boxes'][dist_mask])[0][0]
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
            # pdb.set_trace()
            cls_pub.publish(index2label[_pred_dicts_[0]['pred_labels'].item()])
            id_pub.publish(_pred_dicts_[0]['pred_id'].item())
            score_pub.publish(_pred_dicts_[0]['pred_scores'].item())
            pose_pub.publish(transform_pose_stamped(rotated_box))
            size_pub.publish(transform_point(rotated_box[3:6]))
            # vel_pub.publish()
            # ================================================================
    

        # global last_box_num
        if pc_viz == False:
            if selected_id != None:
                if ocl_count ==0:
                    # 임시 박스 h 변경 
                    # _pred_dicts_[0]['pred_boxes'][0, 5] = 1.5
                    _pred_dicts_[0]['pred_boxes'][0, 5] = 1.0
                    _pred_dicts_[0]['pred_boxes'][0, 2] -= 0.2
            last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=_pred_dicts_, last_box_num=last_box_num, class_id=_pred_dicts_[0]['pred_id'])
        else:
            ros_vis.ros_print_2(data_dict['points_rviz'][:, 0:4])
        
        # print("time :", time.time() - start)
        self.ender.record()
        torch.cuda.synchronize()
        curr_latency = self.starter.elapsed_time(self.ender)
        print('det_time(ms): ', curr_latency)
        self.eta += curr_latency
        print('######################################### => ', self.eta)

if __name__ == '__main__':
    args = parse_config()
    model = LD_base() ### CenterPoint

    checkpoint = torch.load(args.pt, map_location=torch.device('cpu'))  
    model.load_state_dict({k.replace('module.', ''):v for k, v in checkpoint['model_state_dict'].items()})
    model.cuda()
    
    # =========== for SOT tracking =====================================
    global count, sot_list, index2label, ocl_count, cluster_count, odom_yaw_list, odom_pitch_list, lidar_z_list, bridge
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
    global earth2map, map2odom, odom2base, imu2os_sensor, odom_flag, init_odom_rt
    odom_flag = True
    demo_ros = ros_demo(model, args)
    #sub = rospy.Subscriber(
    #    "/livox/lidar", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    sub = rospy.Subscriber("/ouster/points_flip", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    
    slc_track = rospy.Subscriber("/tracking_id", String, selection_callback)
    # tf = rospy.Subscriber("/tf", TFMessage, tf_callback)
    tf = rospy.Subscriber("/tf", TFMessage, tf_callback_3)
    # output publisher node ========================================================
    
    cls_pub = rospy.Publisher('class_pub', String, queue_size=10)
    id_pub = rospy.Publisher('id_pub', Int32, queue_size=10)
    score_pub = rospy.Publisher('score_pub', Float32, queue_size=10)
    pose_pub = rospy.Publisher('pose_pub', PoseStamped, queue_size=10)
    size_pub = rospy.Publisher('size_pub', Point, queue_size=10)
    vel_pub = rospy.Publisher('vel_pub', Point, queue_size=10)
    
    # tf_pub = rospy.Publisher('tf_odom', TFMessage, queue_size=10)
    
    bridge = CvBridge()
    # model_2d = torch.hub.load("ultralytics/yolov5", "custom", "./yolov5x.pt")
    model_2d = torch.hub.load("ultralytics/yolov5", "yolov5s")
    # model_2d.amp=True
    # model_2d = torch.hub.load("ultralytics/yolov5", "custom", path="./yolov5l.engine")
    # for yolo ================================================================
    global F_img, R_img, S_img, N_img, img_pub, nearir_img, range_img, signal_img, ref_img
    F_img = rospy.Subscriber("/ouster/reflec_image", Image, ref_img_callback)
    R_img = rospy.Subscriber("/ouster/range_image", Image, range_img_callback)
    S_img = rospy.Subscriber("/ouster/signal_image", Image, signal_img_callback)
    N_img = rospy.Subscriber("/ouster/nearir_image", Image, nearir_img_callback)
    img_pub = rospy.Publisher("/ouster/range_image_with_box", Image, queue_size=10)
    # =============================================================================
    
    
    print("set up subscriber!")

    rospy.spin()
    breakpoint()