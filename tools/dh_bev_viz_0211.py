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
sys.path.insert(0, "/home/hanwha/livox_detection/tools/")
sys.path.append("/home/hanwha/livox_detection/tools")
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
sys.path.insert(0, "/home/hanwha/livox_detection/STNet")
sys.path.insert(0, "/home/hanwha/livox_detection")

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
    mask = (pc[:, 0] > x_range[0]) & (pc[:, 0] < x_range[1]) & (pc[:, 1] > y_range[0]) & (pc[:, 1] < y_range[1])
    filtered_points = pc[mask]

    # x_pixels = np.int32((filtered_points[:, 0] - x_range[0]) / (x_range[1] - x_range[0]) * bev_w)
    # y_pixels = np.int32((filtered_points[:, 1] - y_range[0]) / (y_range[1] - y_range[0]) * bev_h)
    x_pixels = np.int32((filtered_points[:, 0] - x_range[0]) / (x_range[1] - x_range[0]) * bev_h)
    y_pixels = np.int32((filtered_points[:, 1] - y_range[0]) / (y_range[1] - y_range[0]) * bev_w)
    
    # pdb.set_trace()
    x_pixels = np.int32(bev_h / 2 - x_pixels + 112)
    y_pixels = np.int32(bev_w / 2 - y_pixels + 224)

    x_pixels = np.clip(x_pixels, 0, bev_h - 1)
    y_pixels = np.clip(y_pixels, 0, bev_w - 1)

    intensities = (filtered_points[:, 3] * 255).astype(np.uint8)
    for x, y, intensity in zip(x_pixels, y_pixels, intensities):
        # bev_image[y, x] = intensity
        bev_image[int(x), int(y)] = 255
    #    bev_image[x, y] = 255 
    return bev_image

def draw_2d_bev_box(bev_feat,box_3d):
    bev_feat = bev_feat
    x, y, z, dx, dy, dz, heading = box_3d
    bev_w = 448
    bev_h = 224
    x_range = [0,57.6]
    y_range = [-44.8,44.8]

    corners = np.array([[x+dx/2,y+dy/2],[x+dx/2,y-dy/2],[x-dx/2,y-dy/2],[x-dx/2,y+dy/2],[x+dx/2,y+dy/2]])

    cos_heading,sin_heading = np.cos(heading),np.sin(heading)
    corners_rotated = np.zeros_like(corners)
    for i, (cx, cy) in enumerate(corners):
        x_rot = x + (cx - x) * cos_heading - (cy - y) * sin_heading
        y_rot = y + (cx - x) * sin_heading + (cy - y) * cos_heading
        corners_rotated[i] = [x_rot, y_rot]

    corners_pixels_x= np.int32((corners_rotated[:, 0] - x_range[0]) / (x_range[1] - x_range[0]) * bev_h)
    corners_pixels_y = np.int32((corners_rotated[:, 1] - y_range[0]) / (y_range[1] - y_range[0]) * bev_w)

    corners_pixels_x = np.int32(bev_h / 2 - corners_pixels_x + bev_h // 2)
    corners_pixels_y = np.int32(bev_w / 2 - corners_pixels_y + bev_w // 2)

    for i in range(len(corners_pixels_x) - 1):
            bev_feat = cv2.line(bev_feat, (corners_pixels_y[i], corners_pixels_x[i]), (corners_pixels_y[i + 1], corners_pixels_x[i + 1]), (0,255,255), 1)

    return bev_feat

def kcf_tracker_init (box_3d):
    x, y, z, dx, dy, dz, heading = box_3d
    bev_w = 448
    bev_h = 224
    x_range = [0,57.6]
    y_range = [-44.8,44.8]

    corners = np.array([[x+dx/2,y+dy/2],[x+dx/2,y-dy/2],[x-dx/2,y-dy/2],[x-dx/2,y+dy/2],[x+dx/2,y+dy/2]])

    cos_heading,sin_heading = np.cos(heading),np.sin(heading)
    corners_rotated = np.zeros_like(corners)
    for i, (cx, cy) in enumerate(corners):
        x_rot = x + (cx - x) * cos_heading - (cy - y) * sin_heading
        y_rot = y + (cx - x) * sin_heading + (cy - y) * cos_heading
        corners_rotated[i] = [x_rot, y_rot]

    corners_pixels_x= np.int32((corners_rotated[:, 0] - x_range[0]) / (x_range[1] - x_range[0]) * bev_h)
    corners_pixels_y = np.int32((corners_rotated[:, 1] - y_range[0]) / (y_range[1] - y_range[0]) * bev_w)

    corners_pixels_x = np.int32(bev_h / 2 - corners_pixels_x + bev_h // 2)
    corners_pixels_y = np.int32(bev_w / 2 - corners_pixels_y + bev_w // 2)

    #for kcf tracking
    min_x = np.min(corners_pixels_y)
    max_x = np.max(corners_pixels_y)
    min_y = np.min(corners_pixels_x)
    max_y = np.max(corners_pixels_x)

    kcf_x = min_x
    kcf_y = min_y
    kcf_width = max_x-min_x
    kcf_height = max_y-min_y

    bbox_2d_for_kcf = (kcf_x,kcf_y,kcf_width,kcf_height)
    return bbox_2d_for_kcf


def get_2d_bbox_from_3d(bev_feat,box_3d):
    x, y, z, w, l, h, heading = box_3d
    bev_w = 448
    bev_h = 224
    x_range = [0,57.6]
    y_range = [-44.8,44.8]
    cos_heading = np.cos(heading)
    sin_heading = np.sin(heading)

    corner_x = [x + l/2, x + l/2, x - l/2, x - l/2]
    corner_y = [y + w/2, y - w/2, y - w/2, y + w/2]

    rotated_x = [cos_heading * cx - sin_heading * cy for cx, cy in zip(corner_x, corner_y)]
    rotated_y = [sin_heading * cx + cos_heading * cy for cx, cy in zip(corner_x, corner_y)]

    x_pixels = np.int32([(cx - x_range[0]) / (x_range[1] - x_range[0]) * bev_h for cx in rotated_x])
    y_pixels = np.int32([(cy - y_range[0]) / (y_range[1] - y_range[0]) * bev_w for cy in rotated_y])

    x_pixels = np.int32(bev_h / 2 - x_pixels + 112)
    y_pixels = np.int32(bev_w / 2 - y_pixels + 224)

    x_pixels = np.clip(x_pixels, 0, bev_h - 1)
    y_pixels = np.clip(y_pixels, 0, bev_w - 1)

    min_x, max_x = min(x_pixels), max(x_pixels)
    min_y, max_y = min(y_pixels), max(y_pixels)

    return int(min_x), int(min_y), int(max_x), int(max_y)

# ======================================================================

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

        self.kcf_tracker = cv2.TrackerCSRT_create() #add
        

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
        if box.ndim < 2 or box.size == 0:
            return np.array([]) 
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
    
    import numpy as np
    from sensor_msgs.msg import Image
    import rospy

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
        global selected_id, _pred_dicts_, tracklet_nums, PCs, temp_selected_id, slc_label, sot_list, count, mask_dist, index2label, rotation_matrix,kcf_count
        torch.cuda.synchronize()
        self.starter.record()
        data_dict = self.receive_from_ros(msg)
        data_infer = ros_demo.collate_batch([data_dict])
        ros_demo.load_data_to_gpu(data_infer)
        time_lag = 1
        pc_viz = False
        
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
        #print('box_locate',pred_dicts_[0]['pred_boxes'])
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
            #print('selected_id',selected_id)
            #print('pred_id',pred_dicts_[0]['pred_id'])
            try:
                slc_idx = np.where(pred_dicts_[0]['pred_id'] == selected_id)[0][0]
                slc_label = pred_dicts_[0]['pred_id'][slc_idx]
                pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][slc_idx][np.newaxis, ...]
                _pred_dicts_ = pred_dicts_
                pc_viz = False
            except:
                pc_viz = True
            
            # print('box_locate',pred_dicts_[0]['pred_boxes'])
            vis_bev = True
            if vis_bev:
                # pc viz
                bev_feat = pc2bev(data_infer['points'][:, 1:])
                #min_x, min_y, max_x, max_y = get_2d_bbox_from_3d(pred_dicts_[0]['pred_boxes'][0]) 기 존 
                #print('min_x, min_y, max_x, max_y',min_x, min_y, max_x, max_y)
                # pdb.set_trace()
                bbox_2d_for_kcf = kcf_tracker_init(pred_dicts_[0]['pred_boxes'][0])
                bev_feat = cv2.applyColorMap(bev_feat, cv2.COLORMAP_JET) #224, 448
                #bev_feat = draw_2d_bev_box(bev_feat,pred_dicts_[0]['pred_boxes'][0]) #add
                lt_x = bbox_2d_for_kcf[0]  
                lt_y = bbox_2d_for_kcf[1]  


                rb_x = lt_x + bbox_2d_for_kcf[2]  
                rb_y = lt_y + bbox_2d_for_kcf[3]  

                lt = (lt_x, lt_y)
                rb = (rb_x, rb_y)

                if kcf_count == 0:
                    # pdb.set_trace()
                    self.kcf_tracker.init(bev_feat, bbox_2d_for_kcf)
                    kcf_count += 1
                    cv2.rectangle(bev_feat, lt, rb, (0, 255, 255), 2)
                else:
                    # pdb.set_trace()
                    success, new_bbox = self.kcf_tracker.update(bev_feat)
                    print(new_bbox)
                    if success:
                        x, y, width, height = new_bbox
                        lt_ = (int(x), int(y))
                        rb_ = (int(x + width), int(y + height))

                        cv2.rectangle((bev_feat), lt_, rb_, (255, 0, 0), 2)
                        print('good')
                    else:
                        print("KCF fail====")

                
                    ros_image = self.numpy_to_imgmsg(bev_feat)
                
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
        if pc_viz ==False:
            last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=_pred_dicts_, last_box_num=last_box_num, class_id=_pred_dicts_[0]['pred_id'])
        elif pc_viz == True:
            ros_vis.ros_print_2(data_dict['points_rviz'][:, 0:4])
        #try 문 
        
        self.ender.record()
        torch.cuda.synchronize()
        curr_latency = self.starter.elapsed_time(self.ender)
        #print('det_time(ms): ', curr_latency)
        #print('pred_dicts_',pred_dicts_)
        #print('selected_id ',selected_id)


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
    kcf_count =0
    count = 0
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
    
