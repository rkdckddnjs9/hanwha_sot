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
from std_msgs.msg import String
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

# =========================== I o U Code =======================================================
from scipy.spatial import ConvexHull
def polygon_clip(subjectPolygon, clipPolygon):
    """ Clip a polygon with another polygon.

    Ref: https://rosettacode.org/wiki/Sutherland-Hodgman_polygon_clipping#Python

    Args:
        subjectPolygon: a list of (x,y) 2d points, any polygon.
        clipPolygon: a list of (x,y) 2d points, has to be *convex*
    Note:
        **points have to be counter-clockwise ordered**

    Return:
        a list of (x,y) vertex point for the intersection polygon.
    """
    def inside(p):
        return(cp2[0]-cp1[0])*(p[1]-cp1[1]) > (cp2[1]-cp1[1])*(p[0]-cp1[0])
    
    def computeIntersection():
        dc = [ cp1[0] - cp2[0], cp1[1] - cp2[1] ]
        dp = [ s[0] - e[0], s[1] - e[1] ]
        n1 = cp1[0] * cp2[1] - cp1[1] * cp2[0]
        n2 = s[0] * e[1] - s[1] * e[0] 
        n3 = 1.0 / (dc[0] * dp[1] - dc[1] * dp[0])
        return [(n1*dp[0] - n2*dc[0]) * n3, (n1*dp[1] - n2*dc[1]) * n3]
    
    outputList = subjectPolygon
    cp1 = clipPolygon[-1]
    
    for clipVertex in clipPolygon:
        cp2 = clipVertex
        inputList = outputList
        outputList = []
        s = inputList[-1]
    
        for subjectVertex in inputList:
            e = subjectVertex
            if inside(e):
                if not inside(s):
                    outputList.append(computeIntersection())
                outputList.append(e)
            elif inside(s):
                outputList.append(computeIntersection())
            s = e
        cp1 = cp2
        if len(outputList) == 0:
            return None
    return(outputList)

def poly_area(x,y):
    """ Ref: http://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates """
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

def convex_hull_intersection(p1, p2):
    """ Compute area of two convex hull's intersection area.
        p1,p2 are a list of (x,y) tuples of hull vertices.
        return a list of (x,y) for the intersection and its volume
    """
    inter_p = polygon_clip(p1,p2)
    if inter_p is not None:
        hull_inter = ConvexHull(inter_p)
        return inter_p, hull_inter.volume
    else:
        return None, 0.0  

def box3d_vol(corners):
    ''' corners: (8,3) no assumption on axis direction '''
    a = np.sqrt(np.sum((corners[0,:] - corners[1,:])**2))
    b = np.sqrt(np.sum((corners[1,:] - corners[2,:])**2))
    c = np.sqrt(np.sum((corners[0,:] - corners[4,:])**2))
    return a*b*c

def is_clockwise(p):
    x = p[:,0]
    y = p[:,1]
    return np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)) > 0

def box3d_iou(corners1, corners2):
    ''' Compute 3D bounding box IoU.

    Input:
        corners1: numpy array (8,3), assume up direction is negative Y
        corners2: numpy array (8,3), assume up direction is negative Y
    Output:
        iou: 3D bounding box IoU
        iou_2d: bird's eye view 2D bounding box IoU

    todo (kent): add more description on corner points' orders.
    '''
    # corner points are in counter clockwise order
    rect1 = [(corners1[i,0], corners1[i,2]) for i in range(3,-1,-1)]
    rect2 = [(corners2[i,0], corners2[i,2]) for i in range(3,-1,-1)] 
    
    area1 = poly_area(np.array(rect1)[:,0], np.array(rect1)[:,1])
    area2 = poly_area(np.array(rect2)[:,0], np.array(rect2)[:,1])
   
    inter, inter_area = convex_hull_intersection(rect1, rect2)
    iou_2d = inter_area/(area1+area2-inter_area)
    ymax = min(corners1[0,1], corners2[0,1])
    ymin = max(corners1[4,1], corners2[4,1])

    inter_vol = inter_area * max(0.0, ymax-ymin)
    
    vol1 = box3d_vol(corners1)
    vol2 = box3d_vol(corners2)
    iou = inter_vol / (vol1 + vol2 - inter_vol)
    return iou, iou_2d

def get_3d_box(box_size, heading_angle, center):
    ''' Calculate 3D bounding box corners from its parameterization.

    Input:
        box_size: tuple of (length,wide,height)
        heading_angle: rad scalar, clockwise from pos x axis
        center: tuple of (x,y,z)
    Output:
        corners_3d: numpy array of shape (8,3) for 3D box cornders
    '''
    def roty(t):
        c = np.cos(t)
        s = np.sin(t)
        return np.array([[c,  0,  s],
                         [0,  1,  0],
                         [-s, 0,  c]])

    R = roty(heading_angle)
    l,w,h = box_size
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2];
    y_corners = [h/2,h/2,h/2,h/2,-h/2,-h/2,-h/2,-h/2];
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2];
    corners_3d = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    corners_3d[0,:] = corners_3d[0,:] + center[0];
    corners_3d[1,:] = corners_3d[1,:] + center[1];
    corners_3d[2,:] = corners_3d[2,:] + center[2];
    corners_3d = np.transpose(corners_3d)
    return corners_3d
# =====================================================

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

    def online_inference(self, msg):
        global selected_id, _pred_dicts_, tracklet_nums, PCs, temp_selected_id, slc_label, sot_list
        torch.cuda.synchronize()
        self.starter.record()
        data_dict = self.receive_from_ros(msg)
        data_infer = ros_demo.collate_batch([data_dict])
        ros_demo.load_data_to_gpu(data_infer)
        time_lag = 1
        
        self.model.eval()
        with torch.no_grad(): 
            # torch.cuda.synchronize()
            # self.starter.record()
            pred_dicts = self.model.forward(data_infer)
            # pred_for_track = self.transform_pred(pred_dicts[0])
            # outputs = self.tracker.step_centertrack(pred_for_track, time_lag)
            # pdb.set_trace()
            # pred_dicts = self.transform_track(outputs)
            # pdb.set_trace()
        #     self.ender.record()
        #     torch.cuda.synchronize()
        #     curr_latency = self.starter.elapsed_time(self.ender)
        # print('det_time(ms): ', curr_latency)
        
        pred_for_track = self.transform_pred(pred_dicts[0])
        outputs = self.tracker.step_centertrack(pred_for_track, time_lag)
        pred_dicts = self.transform_track(outputs)
        
        data_infer, pred_dicts = ROS_MODULE.gpu2cpu_w_id(data_infer, pred_dicts)


        pred_dicts_ = pred_dicts

        #  score thresholding ====================================================
        mask_ped = ((pred_dicts_[0]['pred_scores'] >0.3) * (pred_dicts_[0]['pred_labels'] ==2)) # ped
        mask_car = ((pred_dicts_[0]['pred_scores'] >0.4) * (pred_dicts_[0]['pred_labels'] ==1)) # car
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
            # ===== sot =========================== 
            try:
                slc_idx = np.where(pred_dicts_[0]['pred_id'] == selected_id)[0][0]
                slc_label = pred_dicts_[0]['pred_id'][slc_idx]
                pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][slc_idx][np.newaxis, ...]
                _pred_dicts_ = pred_dicts_
                sot_list.append(pred_dicts_[0]['pred_boxes'])

            except:
                # cur_PC = PointCloud(data_infer['points'][:, 1:4].transpose(1, 0)) #.cuda()
                pre_BB = sot_list[-1]
                mask_dist = self.get_target_distance(pre_BB, pred_dicts_[0]['pred_boxes'])
                slc_label_ = (pred_dicts_[0]['pred_labels']==slc_label)
                mask_dist = mask_dist*slc_label_
                # print(mask_dist)
                temp_selected_id = pred_dicts_[0]['pred_id'][mask_dist].reshape(-1)
                # pdb.set_trace()
                
                # slc_idx = np.where(pred_dicts_[0]['pred_id'] == temp_selected_id)[0][0]
                # pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                # pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                # pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                # pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][slc_idx][np.newaxis, ...]
                # _pred_dicts_ = pred_dicts_
                # sot_list.append(pred_dicts_[0]['pred_boxes'])
                
                # pdb.set_trace()
                pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][mask_dist]
                pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][mask_dist]
                pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][mask_dist]
                pred_dicts_[0]['pred_id'] = pred_dicts_[0]['pred_id'][mask_dist]
                sot_list.append(pred_dicts_[0]['pred_boxes'])
                _pred_dicts_ = pred_dicts_

        
        global last_box_num
        # pdb.set_trace() 
        # last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts, last_box_num=last_box_num)
        # last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts, last_box_num=last_box_num, class_id=pred_dicts[0]['pred_id'])
        # pdb.set_trace()
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
    global count, tracklet_nums, results_BBs, PCs, sot_list
    count = 0
    tracklet_nums = 0
    results_BBs = []
    PCs = []
    sot_list = []
    selected_id = None
    opts = {'model_name': 'STNet',
            'which_dataset': 'KITTI',
            'train_test': 'test', 'use_tiny': False,
            'reference_BB': 'previous_result',
            'device': torch.device(type='cuda'),
            'batch_size': 64, 'n_workers': 12, 'n_epoches': 40,
            'n_gpus': 1, 'learning_rate': 0.001, 'subsample_number': 1024,
            'min_points_num': 20, 'IoU_Space': 3, 'seed': 1, 'is_completion': True,
            'n_input_feats': 0, 'use_xyz': True, 'feat_emb': 32, 'iters': 2, 'knn_num': 48,
            'offset_BB': np.array([2, 2, 1]), 'scale_BB': np.array([1, 1, 1]), 'voxel_size': torch.tensor([0.3000, 0.3000, 0.3000]),
            'xy_size': torch.tensor([0.3000, 0.3000]), 'area_extents': [-5.6, 5.6, -3.6, 3.6, -2.4, 2.4],
            'xy_area_extents': [-5.6, 5.6, -3.6, 3.6], 'downsample': 1.0, 'regress_radius': 2, 'ncols': 150,
            'db': {'data_dir': '/home/changwon/data_2/hanwha_projects/tracking/STNet/training/',
                   'val_data_dir': '/home/changwon/data_2/hanwha_projects/tracking/STNet/training/',
                   'category_name': 'Pedestrian'},
            'visual': False, 'rp_which_dataset': 'kitti', 'rp_category': 'Pedestrian', 'data_save_path': '/home/changwon/data_2/hanwha_projects/tracking/STNet/inference/full/kitti',
            'results_dir': './results/kitti_pedestrian', 'mode': False, 'model_path': '/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/STNet/results/kitti_pedestrian/netR_30.pth',
            'voxel_area': np.array([38, 24, 16], dtype=np.int32), 'scene_ground': torch.tensor([-5.6000, -3.6000, -2.4000]),
            'min_img_coord': torch.tensor([-19., -12.])}
    sot_model = STNet_Tracking(opts)
    sot_model.load_state_dict(torch.load(opts['model_path']))
    sot_model.cuda()
    sot_model.eval()
    # =================================================================

    demo_ros = ros_demo(model, args)
    #sub = rospy.Subscriber(
    #    "/livox/lidar", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    sub = rospy.Subscriber("/ouster/points_flip", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    
    slc_track = rospy.Subscriber("/tracking_id", String, selection_callback)
    print("set up subscriber!")

    rospy.spin()
    
