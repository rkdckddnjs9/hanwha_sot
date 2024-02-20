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

def tf_callback(data):
    global odom_x, odom_y, rotation_matrix, latitude, longitude
    global init_x, init_y, count
    for transform in data.transforms:
        if count == 0:
            init_x, init_y = transform.transform.translation.x, transform.transform.translation.y
            count += 1
        if transform.header.frame_id == "odom":
            # odom_x.append(transform.transform.translation.x - init_x)
            # odom_y.append(transform.transform.translation.y - init_y)
            # odom_x = transform.transform.translation.x - init_x
            # odom_y = transform.transform.translation.y - init_y
            odom_x = transform.transform.translation.x
            odom_y = transform.transform.translation.y
            
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    # print(init_x, init_y, odom_x, odom_y)

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

def transform_pose_stamped(box):
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "livox_frame"

    x,y,z,w = yaw2quaternion(box[-1])
    # 위치(position) 설정
    pose_stamped.pose.position.x = box[0]
    pose_stamped.pose.position.y = box[1]
    pose_stamped.pose.position.z = box[2]

    # 자세(orientation) 설정
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
#====================================================

# ====== BEV feature visualization =================
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def normalize(self, image_features):
    image_features = image_features
    min = image_features.min()
    max = image_features.max()
    image_features = (image_features-min)/(max-min)
    image_features = (image_features*255)
    return image_features
    
def feat_vis_max(self, feat):
    max_image_feature = np.max(np.transpose(feat.astype("uint8"), (1,2,0)),axis=2)
    max_image_feature = cv2.applyColorMap(max_image_feature,cv2.COLORMAP_JET)
    cv2.imwrite("max_image_feature.jpg", max_image_feature)

def pc2bev(pc):
    bev_w = 448 #y
    bev_h = 224 #x
    #self.point_cloud_range = [0, -44.8, -2, 57.6, 44.8, 4]
    x_range = [0, 57.6]
    y_range = [-44.8, 44.8]
    bev_image = np.zeros((bev_h, bev_w), dtype=np.uint8)
    # 좌표 범위 필터링
    mask = (pc[:, 0] > x_range[0]) & (pc[:, 0] < x_range[1]) & (pc[:, 1] > y_range[0]) & (pc[:, 1] < y_range[1])
    filtered_points = pc[mask]

    # 좌표를 이미지 좌표로 변환
    # x_pixels = np.int32((filtered_points[:, 0] - x_range[0]) / (x_range[1] - x_range[0]) * bev_w)
    # y_pixels = np.int32((filtered_points[:, 1] - y_range[0]) / (y_range[1] - y_range[0]) * bev_h)
    x_pixels = np.int32((filtered_points[:, 0] - x_range[0]) / (x_range[1] - x_range[0]) * bev_h)
    y_pixels = np.int32((filtered_points[:, 1] - y_range[0]) / (y_range[1] - y_range[0]) * bev_w)
    
    # pdb.set_trace()
    # BEV 이미지의 중심을 기준으로 좌표 재조정
    x_pixels = np.int32(bev_h / 2 - x_pixels + 112)
    y_pixels = np.int32(bev_w / 2 - y_pixels + 224)

    # BEV 이미지의 범위를 고려한 최종 좌표
    x_pixels = np.clip(x_pixels, 0, bev_h - 1)
    y_pixels = np.clip(y_pixels, 0, bev_w - 1)

    # Intensity 값 정규화 및 BEV 이미지에 점 추가
    intensities = (filtered_points[:, 3] * 255).astype(np.uint8)
    for x, y, intensity in zip(x_pixels, y_pixels, intensities):
        # bev_image[y, x] = intensity
        bev_image[int(x), int(y)] = 255
    #    bev_image[x, y] = 255 
    return bev_image

def get_2d_bbox_from_3d(box_3d, bev_w, bev_h, x_range, y_range):
    """
    3D 박스를 BEV 이미지의 2D 박스로 변환합니다.
    """
    x, y, z, w, h, l, heading = box_3d
    cos_heading = np.cos(heading)
    sin_heading = np.sin(heading)

    # 박스의 코너 포인트 계산
    corner_x = [x + l/2, x + l/2, x - l/2, x - l/2]
    corner_y = [y + w/2, y - w/2, y - w/2, y + w/2]

    # 박스 회전 적용
    rotated_x = [cos_heading * cx - sin_heading * cy for cx, cy in zip(corner_x, corner_y)]
    rotated_y = [sin_heading * cx + cos_heading * cy for cx, cy in zip(corner_x, corner_y)]

    # BEV 이미지 좌표로 변환
    x_pixels = [(cx - x_range[0]) / (x_range[1] - x_range[0]) * bev_h for cx in rotated_x]
    y_pixels = [(cy - y_range[0]) / (y_range[1] - y_range[0]) * bev_w for cy in rotated_y]

    # 박스의 최소/최대 좌표
    min_x, max_x = min(x_pixels), max(x_pixels)
    min_y, max_y = min(y_pixels), max(y_pixels)

    return int(min_x), int(min_y), int(max_x), int(max_y)

# ======================================================================

# ================ range image transformation =========================
def transform_pc2range(pc):
    lidar_data = pc
    vertical_span = {
        'start_angle': 22.5,
        'end_angle': -22.5,
        'step': 0.3515625
    }
    
    horizontal_span = {
        'start_angle': -90,
        'end_angle': 90,
        'step': 0.3515625/2
    }
    
    # Range view image의 너비와 높이 계산
    num_vertical_samples = int((vertical_span['start_angle'] - vertical_span['end_angle']) / vertical_span['step']) + 1
    num_horizontal_samples = int((horizontal_span['end_angle'] - horizontal_span['start_angle']) / horizontal_span['step']) + 1

    # Range view image 초기화
    range_view_image = np.zeros((num_vertical_samples, num_horizontal_samples), dtype=np.uint8)

    # Lidar 데이터의 x, y, z 좌표를 추출합니다.
    x = lidar_data[:, 0]
    y = lidar_data[:, 1]
    z = lidar_data[:, 2]

    # Depth의 최소 및 최대 값을 정의하고, depth를 정규화합니다.
    min_depth = np.min(z)
    max_depth = np.max(z)

    normalized_depth = ((z - min_depth) / (max_depth - min_depth)) * 255

    # Lidar 데이터를 Range view image로 변환
    theta = np.degrees(np.arctan2(z, np.sqrt(x**2 + y**2)))
    phi = np.degrees(np.arctan2(y, x))

    vertical_indices = ((vertical_span['start_angle'] - theta) / vertical_span['step']).astype(int)
    horizontal_indices = ((phi - horizontal_span['start_angle']) / horizontal_span['step']).astype(int)

    valid_indices = (vertical_indices >= 0) & (vertical_indices < num_vertical_samples) & \
                    (horizontal_indices >= 0) & (horizontal_indices < num_horizontal_samples)

    range_view_image[vertical_indices[valid_indices], horizontal_indices[valid_indices]] = normalized_depth[valid_indices].astype(np.uint8)
    # range_view_image[vertical_indices[valid_indices], horizontal_indices[valid_indices]] = 255
    range_view_image = range_view_image[:, ::-1]

    return range_view_image

def transform_3d_to_2d(box3d):
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
    
    box_corner = calculate_3d_box_corners(box3d[0]) #[8,3]
    theta = np.degrees(np.arctan2(box_corner[:, 2], np.sqrt(box_corner[:, 0]**2 + box_corner[:, 1]**2)))
    phi = np.degrees(np.arctan2(box_corner[:, 1], box_corner[:, 0]))

    vertical_indices = ((vertical_span['start_angle'] - theta) / vertical_span['step']).astype(int)
    horizontal_indices = ((phi - horizontal_span['start_angle']) / horizontal_span['step']).astype(int)
    
    horizontal_indices = 1024 - horizontal_indices #for flip
    
    lt = (horizontal_indices.min(), vertical_indices.min())
    rb = (horizontal_indices.max(), vertical_indices.max())
    # cv2.rectangle(range_view_image, lt, rb, 255, 2)
    return lt, rb
    

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

#======================================================================================================

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
        self.offset_ground = 1.8 
        # self.point_cloud_range = [0, -44.8, -2, 224, 44.8, 4]
        self.point_cloud_range = [0, -44.8, -2, 57.6, 44.8, 4]
        self.tracker = PubTracker(max_age=args.max_age, hungarian=args.hungarian)
        self.bridge = CvBridge()
        # self.kcf_tracker = cv2.TrackerKCF_create()
        self.kcf_tracker = cv2.TrackerCSRT_create()
        

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
        global selected_id, _pred_dicts_, tracklet_nums, PCs, temp_selected_id, slc_label, sot_list, count, mask_dist, index2label, rotation_matrix, kcf_count, img_sub
        torch.cuda.synchronize()
        self.starter.record()
        data_dict = self.receive_from_ros(msg)
        data_infer = ros_demo.collate_batch([data_dict])
        ros_demo.load_data_to_gpu(data_infer)
        time_lag = 1
        
        self.model.eval()
        with torch.no_grad(): 
            pred_dicts = self.model.forward(data_infer)
            # pdb.set_trace()
            
        temp_pred_dicts = pred_dicts
        pred_for_track = self.transform_pred(pred_dicts[0])
        outputs = self.tracker.step_centertrack(pred_for_track, time_lag)
        pred_dicts = self.transform_track(outputs)
        
        data_infer, pred_dicts = ROS_MODULE.gpu2cpu_w_id(data_infer, pred_dicts)


        pred_dicts_ = pred_dicts

        #  score thresholding ====================================================
        mask_ped = ((pred_dicts_[0]['pred_scores'] >0.1) * (pred_dicts_[0]['pred_labels'] ==2)) # ped
        mask_car = ((pred_dicts_[0]['pred_scores'] >0.3) * (pred_dicts_[0]['pred_labels'] ==1)) # car
        mask = mask_ped + mask_car
        pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][mask]
        pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][mask]
        pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][mask]
        pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][mask]
        # ==========================================================================
        
        # distance based filtering ==================================================
        dist =  self.get_distance(pred_dicts_[0]['pred_boxes'])
        mask_ped = ((dist <= 30) * (pred_dicts_[0]['pred_labels'] ==2)) # ped
        mask_car = ((dist <= 40) * (pred_dicts_[0]['pred_labels'] ==1)) # car
        mask = mask_ped + mask_car
        pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][mask]
        pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][mask]
        pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][mask]
        pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][mask]
        
        if selected_id == None:
            _pred_dicts_ = pred_dicts_
        elif selected_id == -1:
            _pred_dicts_ = pred_dicts_
        else:
            slc_idx = np.where(pred_dicts_[0]['pred_id'] == selected_id)[0][0]
            slc_label = pred_dicts_[0]['pred_id'][slc_idx]
            pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
            pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][slc_idx][np.newaxis, ...]
            pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][slc_idx][np.newaxis, ...]
            pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][slc_idx][np.newaxis, ...]
            _pred_dicts_ = pred_dicts_
            
            vis_range = True
            if vis_range:
                range_view_image = transform_pc2range(pc=data_infer['points'][:, 1:])
                lt, rb = transform_3d_to_2d(box3d=_pred_dicts_[0]['pred_boxes'])
                test_ = cv2.applyColorMap(range_view_image,cv2.COLORMAP_JET)
                
                if kcf_count == 0:
                    # pdb.set_trace()
                    self.kcf_tracker.init(test_, (lt[0], lt[1], rb[0] - lt[0], rb[1] - lt[1]))
                    kcf_count += 1
                    # cv2.rectangle(test_, lt, rb, (0, 255, 0), 2) #green
                    cv2.rectangle(test_, lt, rb, 255, 1)
                else:
                    # pdb.set_trace()
                    success, new_bbox = self.kcf_tracker.update(test_)
                    print(new_bbox)
                    if success:
                        x, y, width, height = new_bbox
                        lt_ = (int(x), int(y))
                        rb_ = (int(x + width), int(y + height))

                        # Range view 이미지에 새로운 Bounding Box 그리기
                        # cv2.rectangle(test_, lt_, rb_, (255, 0, 0), 2) #blue
                        cv2.rectangle(test_, lt_, rb_, 255, 1)
                    else:
                        print("KCF fail====")
                
                # cv2.rectangle(test_, lt, rb, (0, 255, 0), 2)
                cv2.rectangle(test_, lt, rb, 50, 4)
                
                # ros_image = self.bridge.cv2_to_imgmsg(range_view_image, "mono8")
                ros_image = self.bridge.cv2_to_imgmsg(test_, "bgr8")
                # ros_image = self.bridge.cv2_to_imgmsg(test_, "8UC1")
                
                
                # pdb.set_trace()
                img_pub.publish(ros_image)
                # pdb.set_trace()
            
            
            # local to global transformation==================================
            # pdb.set_trace()
            rotated_box = _pred_dicts_[0]['pred_boxes'][0].copy()
            rotated_box[:3] = rotated_box[:3]@rotation_matrix.T
            rotated_box[0] += odom_x
            rotated_box[1] += odom_y
            cls_pub.publish(index2label[_pred_dicts_[0]['pred_labels'].item()])
            id_pub.publish(_pred_dicts_[0]['pred_id'].item())
            score_pub.publish(_pred_dicts_[0]['pred_scores'].item())
            pose_pub.publish(transform_pose_stamped(rotated_box))
            size_pub.publish(transform_point(rotated_box[3:6]))
            # vel_pub.publish()
            # ================================================================

        global last_box_num
        last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=_pred_dicts_, last_box_num=last_box_num, class_id=_pred_dicts_[0]['pred_id'])
        
        
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
    global count, tracklet_nums, results_BBs, sot_list, index2label, ocl_count, kcf_count
    index2label = {0:'Vehicle', 1:'Pedestrian', 2:'Something'}
    ocl_count = 0
    count = 0
    kcf_count = 0
    tracklet_nums = 0
    results_BBs = []
    PCs = []
    sot_list = []
    selected_id = None
    

    # =================================================================

    demo_ros = ros_demo(model, args)
    #sub = rospy.Subscriber(
    #    "/livox/lidar", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    sub = rospy.Subscriber("/ouster/points_flip", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    
    slc_track = rospy.Subscriber("/tracking_id", String, selection_callback)
    tf = rospy.Subscriber("/tf", TFMessage, tf_callback)    
    # output publisher node ========================================================
    
    cls_pub = rospy.Publisher('class_pub', String, queue_size=10)
    id_pub = rospy.Publisher('id_pub', Int32, queue_size=10)
    score_pub = rospy.Publisher('score_pub', Float32, queue_size=10)
    pose_pub = rospy.Publisher('pose_pub', PoseStamped, queue_size=10)
    size_pub = rospy.Publisher('size_pub', Point, queue_size=10)
    vel_pub = rospy.Publisher('vel_pub', Point, queue_size=10)
    
    img_pub = rospy.Publisher('bev_img_pub', Image, queue_size=10)
    
    
    print("set up subscriber!")

    rospy.spin()
    cv2.destroyAllWindows()
    
