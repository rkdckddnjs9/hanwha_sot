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

# for tracking=================================
# libs_dir = os.path.join("/home/changwon/data_2/hanwha_projects/ROS_1123/livox_detection/", "AB3DMOT/AB3DMOT_libs")
# xinshuo_lib=os.path.join("/home/changwon/data_2/hanwha_projects/ROS_1123/livox_detection/","AB3DMOT/Xinshuo_PyToolbox/")
# xinshuo_lib=os.path.join("/home/changwon/data_2/hanwha_projects/ROS_1123/livox_detection/","AB3DMOT/Xinshuo_PyToolbox/xinshuo_io/")
# # Add the path to sys.path
# sys.path.append(libs_dir)
# sys.path.append(xinshuo_lib)

libs_dir = os.path.join("/home/changwon/data_2/hanwha_projects/ROS_1123/livox_detection/", "AB3DMOT/")
# Add the path to sys.path
sys.path.append(libs_dir)

from AB3DMOT.AB3DMOT_libs.matching import data_association
from AB3DMOT.AB3DMOT_libs.vis import vis_obj
from AB3DMOT.AB3DMOT_libs.utils import Config, get_subfolder_seq, initialize

from AB3DMOT.AB3DMOT_libs.kalman_filter import KF
#from kitti_oxts import get_ego_traj, egomotion_compensation_ID
#from kitti_oxts import load_oxts ,_poses_from_oxts
#from kitti_calib import Calibration
from AB3DMOT.AB3DMOT_libs.model import AB3DMOT
from AB3DMOT.AB3DMOT_libs.box import Box3D
from AB3DMOT.Xinshuo_PyToolbox.xinshuo_miscellaneous import get_timestring, print_log
from AB3DMOT.Xinshuo_PyToolbox.xinshuo_io import mkdir_if_missing, save_txt_file

#=============================================

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

    def online_inference(self, msg):
        global frame_id, tracker
        torch.cuda.synchronize()
        self.starter.record()
        frame_id += 1
        data_dict = self.receive_from_ros(msg)
        data_infer = ros_demo.collate_batch([data_dict])
        ros_demo.load_data_to_gpu(data_infer)
        
        self.model.eval()
        with torch.no_grad(): 
            # torch.cuda.synchronize()
            # self.starter.record()
            pred_dicts = self.model.forward(data_infer)
            # pdb.set_trace()
        #     self.ender.record()
        #     torch.cuda.synchronize()
        #     curr_latency = self.starter.elapsed_time(self.ender)
        # print('det_time(ms): ', curr_latency)
        
        data_infer, pred_dicts = ROS_MODULE.gpu2cpu(data_infer, pred_dicts)
        pred_dicts_ = pred_dicts
        pred_dicts_[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][:, [5, 4, 3, 0, 1, 2, 6]] #shape : [6, 7] #x, y, z, l, w, h, rot -> h, w, l, x, y, z, rot
        # pdb.set_trace()
        # dets_frame = self.get_frame_det(pred_dicts_, frame_id)
        # 탐지 결과를 추적기에 입력
        if pred_dicts_[0]['pred_boxes'].shape[0] > 0:
            # pdb.set_trace()
            dets_all = {'dets': pred_dicts_[0]['pred_boxes'], 'info': np.concatenate((pred_dicts_[0]['pred_scores'][:, None], pred_dicts_[0]['pred_labels'][:, None]), axis=1)}
        else:
            dets_all = {'dets': np.empty([0, 7]), 'info': np.empty([0, 1])}
            
        # score thresholding
        mask_ped = ((dets_all['info'][:, 0] >0.4) * (dets_all['info'][:, 1] ==2.)) # ped
        mask_car = ((dets_all['info'][:, 0] >0.5) * (dets_all['info'][:, 1] ==1.)) # car
        mask = mask_ped + mask_car
        dets_all['dets'] = dets_all['dets'][mask]
        dets_all['info'] = dets_all['info'][mask]
        
        # distance based filtering
        dist =  self.get_distance(dets_all['dets'])
        mask_ped = ((dist <= 30) * (dets_all['info'][:, 1] ==2.)) # ped
        mask_car = ((dist <= 40) * (dets_all['info'][:, 1] ==1.)) # car
        mask = mask_ped + mask_car
        dets_all['dets'] = dets_all['dets'][mask]
        dets_all['info'] = dets_all['info'][mask]
        
        # distance based top k selection (k=5)
        # dist =  self.get_distance(dets_all['dets'])
        # dist_mask = np.full((dist.shape), False)
        # dist_mask[np.argsort(dist)[:5]] = True
        
        # dets_all['dets'] = dets_all['dets'][dist_mask]
        # dets_all['info'] = dets_all['info'][dist_mask]
        
        # pdb.set_trace()
        results, affi = tracker.track(dets_all, frame_id, 'hanwha') # h,w,l,x,y,z,theta, ID, other info, confidence
        # pdb.set_trace()
        box_ = results[0][:, [3,4,5,2,1,0,6]]  #h,w,l,x,y,z,theta -> x,y,z,l,w,h,rot
        
        pred_dicts_[0]['pred_boxes'] = box_
        # pred_dicts_[0]['pred_scores'] = np.ones((box_.shape[0],), dtype=np.float32)
        pred_dicts_[0]['pred_scores'] = np.array(results[0][:, 8],  dtype=np.float32)
        
        # pred_dicts_[0]['pred_labels'] = np.ones((box_.shape[0],), dtype=np.int8)
        pred_dicts_[0]['pred_labels'] = np.array(results[0][:, 9], dtype=np.int32)
        pred_dicts_[0]['pred_id'] = np.array(results[0][:, 7], dtype=np.int32)
       
        global last_box_num
        # pdb.set_trace() 
        # last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts, last_box_num=last_box_num)
        last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts_, last_box_num=last_box_num, class_id=pred_dicts_[0]['pred_id'])
        
        self.ender.record()
        torch.cuda.synchronize()
        curr_latency = self.starter.elapsed_time(self.ender)
        print('det_time(ms): ', curr_latency)

if __name__ == '__main__':
    # tracking ========================================
    global frame_id, cfg, tracker
    frame_id = 0
    # load config files
    config_path = './hanwha.yml'
    cfg, settings_show = Config(config_path)
    time_str = get_timestring()
    log = os.path.join(cfg.save_root, 'log/log_%s_%s_%s.txt' % (time_str, cfg.dataset, cfg.split))
    mkdir_if_missing(log); log = open(log, 'w')
    for idx, data in enumerate(settings_show):
        print_log(data, log, display=False)
    # tracker = initialize(cfg, None, None, None, None, ['Car', 'Pedestrian', 'Cyclist'], 1, {'image': (900, 1600), 'lidar': (720, 1920)}, log_file=os.path.join(cfg.save_root, 'log/log_.txt'))
    tracker = initialize(cfg, None, None, None, None, 'Pedestrian', 1, {'image': (900, 1600), 'lidar': (720, 1920)}, log_file=log)
    # =================================================
    
    args = parse_config()
    model = LD_base()

    checkpoint = torch.load(args.pt, map_location=torch.device('cpu'))  
    model.load_state_dict({k.replace('module.', ''):v for k, v in checkpoint['model_state_dict'].items()})
    model.cuda()

    demo_ros = ros_demo(model, args)
    #sub = rospy.Subscriber(
    #    "/livox/lidar", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    sub = rospy.Subscriber("/ouster/points_flip", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    print("set up subscriber!")

    rospy.spin()
    
