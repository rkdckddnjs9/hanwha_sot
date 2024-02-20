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

import copy
import rospy
import ros_numpy
import std_msgs.msg
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField

#==== for centerpoint tracking =====================
from scipy.spatial.transform import Rotation
# import importlib
import sys
sys.path.insert(0, "/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
sys.path.append("/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
# importlib.import_module("/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/")
from pub_tracker import PubTracker, NUSCENES_TRACKING_NAMES

def greedy_assignment(dist):
    matched_indices = []
    if dist.shape[1] == 0:
        return np.array(matched_indices, np.int32).reshape(-1, 2)
    for i in range(dist.shape[0]):
        j = dist[i].argmin()
        if dist[i][j] < 1e16:
            dist[:, j] = 1e18
            matched_indices.append([i, j])
    return np.array(matched_indices, np.int32).reshape(-1, 2)

#==========================

# ================== for sot tracking ================================
from std_msgs.msg import String, Int32, Float32
def selection_callback(data):
    global selected_id
    selected_id = int(data.data)

import importlib
import math
sys.path.insert(0, "/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/STNet")
sys.path.insert(0, "/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/")
# plg_lib = importlib.import_module("/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detecti
# on/STNet/")

from STNet.utils.metrics import AverageMeter, Success, Precision
from STNet.utils.metrics import estimateOverlap, estimateAccuracy
from STNet.utils import kitti_utils
from STNet.utils.data_classes import BoundingBox, PointCloud
from STNet.utils.decode import mot_decode
from STNet.modules.stnet import STNet_Tracking
from pyquaternion import Quaternion

def heading_to_quaternion(heading_angle):
    half_angle = heading_angle / 2.0
    return (0, 0, math.sin(half_angle), math.cos(half_angle))

def quaternion_to_yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

#============================================

# for output publish================================
from geometry_msgs.msg import PoseStamped, Point
from tf2_msgs.msg import TFMessage

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
            
            if len(odom_yaw_list) < 4:
                odom_yaw_list.append(quaternion_to_yaw(odom2base_quaternion))
                odom_pitch_list.append(quaternion_to_pitch(odom2base_quaternion))
            else:
                odom_yaw_list.append(quaternion_to_yaw(odom2base_quaternion))
                odom_pitch_list.append(quaternion_to_pitch(odom2base_quaternion))
                odom_yaw_list.pop(0)
                odom_pitch_list.pop(0)
        
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

def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz, xw, yw, zw = x * y, x * z, y * z, x * w, y * w, z * w

    rotation_matrix = np.array([
        [1 - 2 * (yy + zz), 2 * (xy - zw),     2 * (xz + yw)],
        [2 * (xy + zw),     1 - 2 * (xx + zz), 2 * (yz - xw)],
        [2 * (xz - yw),     2 * (yz + xw),     1 - 2 * (xx + yy)]
    ])

    return rotation_matrix

def quaternion_to_yaw(quaternion):
    """
    Convert a quaternion into yaw (rotation around z-axis).
    """
    x, y, z, w = quaternion
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw

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

def transform_point(data):
    point = Point()
    point.x = data[0]
    point.y = data[1]
    point.z = data[2]
    return point

def yaw2quaternion(yaw):
    x = 0
    y = 0
    z = math.sin(yaw / 2)
    w = math.cos(yaw / 2)
    return (x, y, z, w)

def yaw_to_rotation_matrix(yaw):
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    
    rotation_matrix = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    
    return rotation_matrix

def quaternion_to_pitch(quaternion):
    """
    Convert a quaternion into a pitch angle.
    Pitch is the rotation around the y-axis in radians.
    """
    x, y, z, w = quaternion
    
    # Compute pitch from the quaternion
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2  # Clamp to 1.0 if out of range due to numerical error
    t2 = -1.0 if t2 < -1.0 else t2  # Clamp to -1.0 if out of range due to numerical error
    pitch = np.arcsin(t2)
    
    return pitch

def pitch_to_rotation_matrix(pitch):
    
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)

    # Rotation matrix around the y-axis
    rotation_matrix = np.array([
        [cos_pitch, 0, sin_pitch],
        [0, 1, 0],
        [-sin_pitch, 0, cos_pitch]
    ])
    return rotation_matrix
#====================================================


from vis_ros import ROS_MODULE
ros_vis = ROS_MODULE()
last_box_num = 0
last_gtbox_num = 0

def mask_points_out_of_range(pc, pc_range):
    pc_range = np.array(pc_range)
    pc_range[3:6] -= 0.01  #np -> cuda .999999 = 1.0
    mask_x = (pc[:, 0] > pc_range[0]) & (pc[:, 0] < pc_range[3])
    mask_y = (pc[:, 1] > pc_range[1]) & (pc[:, 1] < pc_range[4])
    mask_z = (pc[:, 2] > pc_range[2]) & (pc[:, 2] < pc_range[5])
    mask = mask_x & mask_y & mask_z
    pc = pc[mask]
    return pc

def check_numpy_to_torch(x):
    if isinstance(x, np.ndarray):
        return torch.from_numpy(x).float(), True
    return x, False

def rotate_points_along_z(points, angle):
    """
    Args:
        points: (B, N, 3 + C)
        angle: (B), angle along z-axis, angle increases x ==> y
    Returns:

    """
    points, is_numpy = check_numpy_to_torch(points)
    angle, _ = check_numpy_to_torch(angle)

    cosa = torch.cos(angle)
    sina = torch.sin(angle)
    zeros = angle.new_zeros(points.shape[0])
    ones = angle.new_ones(points.shape[0])
    rot_matrix = torch.stack((
        cosa,  sina, zeros,
        -sina, cosa, zeros,
        zeros, zeros, ones
    ), dim=1).view(-1, 3, 3).float()
    points_rot = torch.matmul(points[:, :, 0:3], rot_matrix)
    points_rot = torch.cat((points_rot, points[:, :, 3:]), dim=-1)
    return points_rot.numpy() if is_numpy else points_rot

def boxes_to_corners_3d(boxes3d):
    """
    Args:
        boxes3d:  (N, 7) [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center
    Returns:
    """
    boxes3d, is_numpy = check_numpy_to_torch(boxes3d)

    template = boxes3d.new_tensor((
        [1, 1, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1],
        [1, 1, 1], [1, -1, 1], [-1, -1, 1], [-1, 1, 1],
    )) / 2

    corners3d = boxes3d[:, None, 3:6].repeat(1, 8, 1) * template[None, :, :]
    corners3d = rotate_points_along_z(corners3d.view(-1, 8, 3), boxes3d[:, 6]).view(-1, 8, 3)
    corners3d += boxes3d[:, None, 0:3]

    return corners3d.numpy() if is_numpy else corners3d

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')

    parser.add_argument('--pt', type=str, default=None, help='checkpoint to start from')

    # centerpoint track ==============
    # parser.add_argument("--max_age", type=int, default=300)
    parser.add_argument("--max_age", type=int, default=50)
    # parser.add_argument("--max_age", type=int, default=5)
    parser.add_argument("--hungarian", action='store_true')
    # ==========================
    
    args = parser.parse_args()
    return args

class ros_demo():
    def __init__(self, model, args=None):
        self.args = args
        self.model = model
        self.starter, self.ender = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)

        self.offset_angle = 0
        # self.offset_ground = 1.8 
        self.offset_ground = 0.0
        # self.point_cloud_range = [0, -44.8, -2, 224, 44.8, 4]
        # self.point_cloud_range = [0, -44.8, -2, 57.6, 44.8, 4]
        self.point_cloud_range = [0, -44.8, -3, 57.6, 44.8, 3]
        self.tracker = PubTracker(max_age=args.max_age, hungarian=args.hungarian)

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
    
    def transform_pred(self, pred):
        # pred.keys() : ['pred_boxes', 'pred_scores', 'pred_labels']
        # pred_boxes : (37,7), pred_scores : (37,), pred_labels : (37,)
        sample_token = "iechhs0fjtjw7ttioqi5skr7ipuuekqv"
        annos = []
        for i, box in enumerate(pred['pred_boxes']):
            rot = Rotation.from_euler('xyz', [0,0,box[-1].item()], degrees=True)
            try:
                nusc_anno = {
                    'sample_token': sample_token,
                    "translation": box[:3].tolist(),
                    "size": box[3:6].tolist(),
                    "rotation": rot.as_quat().tolist(),
                    #"rotation": box[-1:].tolist(),
                    "velocity": [0, 0],
                    #"velocity": box[6:8].tolist(),
                    # "detection_name": NUSCENES_TRACKING_NAMES[pred['pred_labels'][i]-1],
                    "detection_name": NUSCENES_TRACKING_NAMES[pred['pred_labels'][i]],
                    "detection_score": float(pred['pred_scores'][i]),
                    "attribute_name": 'None',
                }
            except:
                import pdb;pdb.set_trace()
            annos.append(nusc_anno)
        return annos
    
    def transform_track(self, tracks):
        annos = {
            'pred_boxes':[],
            'pred_scores':[],
            'pred_labels':[],
            'pred_id': [],
        }
        for track in tracks:
            rot = Rotation.from_quat(track['rotation'])
            rot = rot.as_euler('xyz',degrees=True)[-1]
            pred_boxes = track['translation']+track['size']+[rot]
            annos['pred_boxes'].append(pred_boxes)
            annos['pred_scores'].append(track['detection_score'])
            annos['pred_labels'].append(track['label_preds'])
            annos['pred_id'].append(track['tracking_id'])
        annos['pred_boxes']=torch.tensor(annos['pred_boxes'])
        annos['pred_scores']=torch.tensor(annos['pred_scores'])
        annos['pred_labels']=torch.tensor(annos['pred_labels'])
        annos['pred_id']=torch.tensor(annos['pred_id'])
        return [annos]
    
    def get_frame_det(self, dets_all, frame):
    	
        # get irrelevant information associated with an object, not used for associationg
        # ori_array = dets_all.reshape((-1, 1))		# orientation
        # other_array = dets_all[:, 1:7] 					# other information, e.g, 2D box, ...
        # additional_info = np.concatenate((ori_array, other_array), axis=1)		

        # get 3D box
        dets = dets_all

        # dets_frame = {'dets': dets, 'info': additional_info}
        dets_frame = {'dets': dets, 'info': dets}
        return dets_frame
    
    def get_distance(self, box):
        # box shape : N, 7
        dist = np.sqrt(box[:,0]**2 + box[:,1]**2+ box[:,2]**2)
        return dist
    
    def get_target_distance(self, target, box):
        # box shape : N, 7
        dist_mask = []
        for t_box in target:
            # import pdb; pdb.set_trace()
            # dist = np.sqrt((t_box[0] - box[:,0])**2 + (t_box[1] - box[:,1])**2+ (t_box[2] - box[:,2])**2)
            dist = np.sqrt((t_box[0] - box[:,0])**2 + (t_box[1] - box[:,1])**2)
            mask_ = np.full(dist.shape, False)
            mask_[np.argmin(dist)]=True
            dist_mask.append(mask_)
        return (np.array(dist_mask).sum(axis=0) != 0)
    
    def get_target_distance_v2(self, target, box):
        # box shape : N, 7
        dist_mask = []
        for t_box in target:
            # import pdb; pdb.set_trace()
            # dist = np.sqrt((t_box[0] - box[:,0])**2 + (t_box[1] - box[:,1])**2+ (t_box[2] - box[:,2])**2)
            dist = np.sqrt((t_box[0] - box[:,0])**2 + (t_box[1] - box[:,1])**2)
            # mask_ = np.full(dist.shape, False)
            # mask_[np.argmin(dist)]=True
            dist_mask.append(dist)
        return np.array(dist_mask)

    def online_inference(self, msg):
        global selected_id, _pred_dicts_, tracklet_nums, PCs, temp_selected_id, slc_label, sot_list, count, mask_dist, index2label, rotation_matrix, ocl_count
        torch.cuda.synchronize()
        self.starter.record()
        data_dict = self.receive_from_ros(msg)
        
        interval = 5
        input_pc = data_dict['points'][:, :3].copy() #N, 4
        prev_yaw = odom_yaw_list[-2]
        cur_yaw = odom_yaw_list[-1]
        var_yaw = cur_yaw - prev_yaw
        rot_m = yaw_to_rotation_matrix(var_yaw*interval)
        input_pc = (rot_m@input_pc.T).T
        
        prev_pitch = odom_pitch_list[-2]
        cur_pitch = odom_pitch_list[-1]
        var_pitch = cur_pitch - prev_pitch
        rot_m = pitch_to_rotation_matrix(var_pitch*interval)
        input_pc = (rot_m@input_pc.T).T
        
        prev_z = lidar_z_list[-2][-1]
        cur_z = lidar_z_list[-1][-1]
        var_z = cur_z - prev_z
        # z += var_z
        input_pc[:, 2] += var_z
        
        data_dict['points'][:, :3] = input_pc
        
        data_infer = ros_demo.collate_batch([data_dict])
        ros_demo.load_data_to_gpu(data_infer)
        time_lag = 1
        
        self.model.eval()
        with torch.no_grad(): 
            pred_dicts = self.model.forward(data_infer)
            # pdb.set_trace()
        pred_dicts[0]['pred_labels'] = torch.where(pred_dicts[0]['pred_labels']==3, 2, pred_dicts[0]['pred_labels'])
        temp_pred_dicts = pred_dicts
        
        # pdb.set_trace()
        pred_for_track = self.transform_pred(pred_dicts[0])
        outputs = self.tracker.step_centertrack(pred_for_track, time_lag)
        pred_dicts = self.transform_track(outputs)
        
        data_infer, pred_dicts = ROS_MODULE.gpu2cpu_w_id(data_infer, pred_dicts)


        pred_dicts_ = pred_dicts
        if pred_dicts[0]['pred_labels'].shape[0] == 0:
            print("No object in current frame")
            skip_flag = False
            ocl_count += 1
            # pdb.set_trace()
        else:
            skip_flag = True

        if skip_flag:
            # #  score thresholding ====================================================
            # mask_ped = ((pred_dicts_[0]['pred_scores'] >0.1) * (pred_dicts_[0]['pred_labels'] ==2)) # ped
            # mask_car = ((pred_dicts_[0]['pred_scores'] >0.3) * (pred_dicts_[0]['pred_labels'] ==1)) # car
            # mask = mask_ped + mask_car
            # pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][mask]
            # pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][mask]
            # pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][mask]
            # pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][mask]
            # # ==========================================================================
            
            # distance based filtering ==================================================
            # dist =  self.get_distance(pred_dicts_[0]['pred_boxes'])
            # mask_ped = ((dist <= 30) * (pred_dicts_[0]['pred_labels'] ==2)) # ped
            # mask_car = ((dist <= 40) * (pred_dicts_[0]['pred_labels'] ==1)) # car
            # mask = mask_ped + mask_car
            # pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][mask]
            # pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][mask]
            # pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][mask]
            # pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][mask]

            if selected_id == None:
                _pred_dicts_ = pred_dicts_
            elif selected_id == -1:
                _pred_dicts_ = pred_dicts_
            else:
                try: # target object in detection result
                    slc_idx = np.where(pred_dicts_[0]['pred_id'] == selected_id)[0][0]
                    slc_label = pred_dicts_[0]['pred_id'][slc_idx]
                    pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                    pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                    pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                    pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][slc_idx][np.newaxis, ...]
                    _pred_dicts_ = pred_dicts_
                    ocl_count=0
                    
                    if len(sot_list) < 10:
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                    else:
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                        sot_list.pop(0)
                except: #이전 결과 추가해주는 것임
                    _pred_dicts_ = _pred_dicts_
                    ocl_count += 1
                    
                    if len(sot_list) < 10:
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                    else:
                        sot_list.append(_pred_dicts_[0]['pred_boxes'])
                        sot_list.pop(0)

                # # for occlusion ====================================
                # if len(sot_list) < 10:
                #     sot_list.append(_pred_dicts_[0]['pred_boxes'])
                # else:
                #     sot_list.append(_pred_dicts_[0]['pred_boxes'])
                #     sot_list.pop()
                print('ocl_count, selected_id: ', (ocl_count, selected_id))
                
                # occlusion 1초 이상 발생 처리
                if len(sot_list) == 10 and (sot_list[0][0][:3]==sot_list[-1][0][:3]).sum() == 3:
                    # pdb.set_trace()
                    # new object search
                    prev_label = _pred_dicts_[0]['pred_labels']
                    prev_obj = sot_list[-1].copy()
                    
                    slc_label_ = (pred_dicts_[0]['pred_labels']==prev_label)
                    if slc_label_.sum() == 0: # no same cls obj in current detection result 
                        pass
                    else: # same cls obj in current detection result 
                        mask_dist = self.get_target_distance(prev_obj, pred_dicts_[0]['pred_boxes'])
                        mask_dist = mask_dist*slc_label_
                        temp_selected_id = pred_dicts_[0]['pred_id'][mask_dist].reshape(-1)
                        selected_id = temp_selected_id
                        
                        # slc_idx update
                        slc_idx = np.where(pred_dicts_[0]['pred_id'] == temp_selected_id)[0][0]
                        pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                        pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                        pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                        pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][slc_idx][np.newaxis, ...]
                        _pred_dicts_ = pred_dicts_
                        
                        sot_list = []
                        sot_list.append(pred_dicts_[0]['pred_boxes'])
                        
                # =====================================================
                
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
        

            global last_box_num
            last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=_pred_dicts_, last_box_num=last_box_num, class_id=_pred_dicts_[0]['pred_id'])
        
        else:
            ros_vis.ros_print_2(data_dict['points_rviz'][:, 0:4])
        
        self.ender.record()
        torch.cuda.synchronize()
        curr_latency = self.starter.elapsed_time(self.ender)
        print('det_time(ms): ', curr_latency)

if __name__ == '__main__':
    args = parse_config()
    model = LD_base()

    checkpoint = torch.load(args.pt, map_location=torch.device('cpu'))  
    model.load_state_dict({k.replace('module.', ''):v for k, v in checkpoint['model_state_dict'].items()})
    model.cuda()
    
    # =========== for SOT tracking =====================================
    global count, tracklet_nums, results_BBs, sot_list, index2label, ocl_count, cluster_count, odom_yaw_list, odom_pitch_list, lidar_z_list
    cluster_count=0
    index2label = {0:'Vehicle', 1:'Pedestrian', 2:'Something'}
    ocl_count = 0
    count = 0
    tracklet_nums = 0
    results_BBs = []
    PCs = []
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
    
    
    print("set up subscriber!")

    rospy.spin()
    
