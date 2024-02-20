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

import numpy as np

import torch
import torch.nn as nn
from livoxdetection.models.ld_base_v1 import LD_base
import pdb

from filterpy.kalman import KalmanFilter
import copy
import rospy
import ros_numpy
import std_msgs.msg
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField

from vis_ros import ROS_MODULE
ros_vis = ROS_MODULE()
last_box_num = 0
last_gtbox_num = 0

class KalmanFilterTracker:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=7, dim_z=7)
        self.kf.F = np.eye(7)  # 상태 전이 행렬
        self.kf.H = np.eye(7)  # 관측 행렬
        self.kf.R *= 10.       # 측정 노이즈
        self.kf.P *= 50.       # 추정 오차 공분산
        self.kf.Q *= 0.1       # 프로세스 노이즈
        self.last_update_time = 0
        self.label = None
        self.score = 0

    def update(self, measurement, label=None, score=0):
        self.kf.update(measurement)
        self.last_update_time = rospy.get_time()
        if label is not None:
            self.label = label
        if score > 0:
            self.score = score
        # self.label = label
        # self.score = score

    def predict(self):
        self.kf.predict()
        return self.kf.x[:7]  # 추정 상태 반환
    
    def get_state(self):
        return self.kf.x[:7]  # 현재 추정 상태 반환
    
    def is_active(self, timeout=5):
        """트래커가 활성 상태인지 확인합니다.
        Args:
            timeout (float): 트래커가 비활성화되기 전까지의 시간 (초).
        Returns:
            bool: 트래커가 활성 상태이면 True, 그렇지 않으면 False.
        """
        return rospy.get_time() - self.last_update_time < timeout

def mask_points_out_of_range(pc, pc_range):
    pc_range = np.array(pc_range)
    pc_range[3:6] -= 0.01  #np -> cuda .999999 = 1.0
    mask_x = (pc[:, 0] > pc_range[0]) & (pc[:, 0] < pc_range[3])
    mask_y = (pc[:, 1] > pc_range[1]) & (pc[:, 1] < pc_range[4])
    mask_z = (pc[:, 2] > pc_range[2]) & (pc[:, 2] < pc_range[5])
    mask = mask_x & mask_y & mask_z
    pc = pc[mask]
    return pc

def get_tracker_id(box, trackers, threshold=3.0):
    """
    거리 기반으로 트래커 ID를 결정하는 함수.
    
    Args:
        box (np.array): 현재 프레임에서 감지된 객체의 상태.
        trackers (dict): 현재 활성화된 트래커들.
        threshold (float): 트래커 할당을 위한 거리 임곗값.

    Returns:
        int: 할당된 트래커 ID.
    """
    min_distance = float('inf')
    assigned_tracker_id = None

    for tracker_id, tracker in trackers.items():
        # 트래커의 최신 상태 가져오기
        tracker_state = tracker.get_state()

        # 거리 계산 (Euclidean distance)
        distance = np.linalg.norm(box[:3] - tracker_state[:3])

        if distance < min_distance and distance < threshold:
            min_distance = distance
            assigned_tracker_id = tracker_id

    if assigned_tracker_id is None:
        # 새로운 트래커 ID 생성
        new_tracker_id = max(trackers.keys(), default=-1) + 1
        assigned_tracker_id = new_tracker_id

    return assigned_tracker_id

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
        
        self.trackers = defaultdict(KalmanFilterTracker)

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

    def get_distance(self, box):
        # box shape : N, 7
        dist = np.sqrt(box[:,0]**2 + box[:,1]**2+ box[:,2]**2)
        return dist

    def online_inference(self, msg):
        global cls_id
        torch.cuda.synchronize()
        self.starter.record()
        data_dict = self.receive_from_ros(msg)
        data_infer = ros_demo.collate_batch([data_dict])
        ros_demo.load_data_to_gpu(data_infer)
        
        self.model.eval()
        with torch.no_grad(): 
            # torch.cuda.synchronize()
            # self.starter.record()
            pred_dicts = self.model.forward(data_infer)
            # self.ender.record()
            # torch.cuda.synchronize()
            # curr_latency = self.starter.elapsed_time(self.ender)
        # print('det_time(ms): ', curr_latency)
        data_infer, pred_dicts = ROS_MODULE.gpu2cpu(data_infer, pred_dicts)
        
        pred_dicts_ = pred_dicts
        # score thresholding
        mask_ped = ((pred_dicts_[0]['pred_labels'] >0.4) * (pred_dicts_[0]['pred_labels'] ==2.)) # ped
        mask_car = ((pred_dicts_[0]['pred_labels'] >0.5) * (pred_dicts_[0]['pred_labels'] ==1.)) # car
        mask = mask_ped + mask_car
        pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][mask]
        pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][mask]
        pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][mask]
        
        # distance based filtering
        dist =  self.get_distance(pred_dicts_[0]['pred_boxes'])
        mask_ped = ((dist <= 30) * (pred_dicts_[0]['pred_labels'] ==2.)) # ped
        mask_car = ((dist <= 40) * (pred_dicts_[0]['pred_labels'] ==1.)) # car
        mask = mask_ped + mask_car
        pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][mask]
        pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][mask]
        pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][mask]
        
        
        
        updated_boxes = []
        updated_labels = []
        updated_scores = []
        for i, box in enumerate(pred_dicts_[0]['pred_boxes']):
            b = box
            label = pred_dicts_[0]['pred_labels'][i]
            score = pred_dicts_[0]['pred_scores'][i]
            tracker_id = get_tracker_id(box, self.trackers)
            # 박스 중심, 크기, 방향으로 상태 벡터 생성
            state = np.array([b[0], b[1], b[2], b[3], b[4], b[5], b[6]])
            if tracker_id not in self.trackers:
                # 새 트래커 생성 및 초기화
                self.trackers[tracker_id] = KalmanFilterTracker()  # KalmanTracker는 해당 트래커 클래스
                self.trackers[tracker_id].update(state, label=label, score=score)
            
            self.trackers[tracker_id].predict()
            self.trackers[tracker_id].update(state)
            updated_state = self.trackers[tracker_id].get_state()
            updated_boxes.append(updated_state)
            # updated_labels.append(self.trackers[tracker_id].label)
            # updated_scores.append(self.trackers[tracker_id].score)
            
        # 더 이상 감지되지 않는 트래커 제거
        for tracker_id in list(self.trackers.keys()):
            if not self.trackers[tracker_id].is_active():
                del self.trackers[tracker_id]
        
        for tracker_id, tracker in self.trackers.items():
            updated_state = tracker.get_state()
            updated_boxes.append(updated_state)
            updated_labels.append(self.trackers[tracker_id].label)
            updated_scores.append(self.trackers[tracker_id].score)
        
        cls_list = [id_ for id_ in self.trackers.keys()]
        pred_dicts_[0]['pred_boxes'] = np.concatenate(updated_boxes[-len(cls_list):], axis=1).T
        pred_dicts_[0]['pred_labels'] = np.array(updated_labels)
        pred_dicts_[0]['pred_scores'] = np.array(updated_scores)
        pred_dicts_[0]['pred_id'] = np.array(cls_list)
        
        self.ender.record()
        torch.cuda.synchronize()
        curr_latency = self.starter.elapsed_time(self.ender)
        print('det_time(ms): ', curr_latency)
        
        # pred_dicts[0]['pred_boxes'].shape, pred_dicts[0]['pred_id'].shape, pred_dicts[0]['pred_scores'].shape, pred_dicts[0]['pred_labels'].shape
        # pdb.set_trace()
        global last_box_num
        last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts_, last_box_num=last_box_num, class_id=pred_dicts_[0]['pred_id'])

if __name__ == '__main__':
    global cls_id, KalmanTracker
    cls_id = 0
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
    
