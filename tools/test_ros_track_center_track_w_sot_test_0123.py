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
    parser.add_argument("--max_age", type=int, default=3)
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
            dist = np.sqrt((t_box[0] - box[:,0])**2 + (t_box[1] - box[:,1])**2+ (t_box[2] - box[:,2])**2)
            mask_ = np.full(dist.shape, False)
            mask_[np.argmin(dist)]=True
            dist_mask.append(mask_)
        return (np.array(dist_mask).sum(axis=0) != 0)

    def online_inference(self, msg):
        global selected_id, _pred_dicts_, tracklet_nums, PCs
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
        mask_ped = ((pred_dicts_[0]['pred_scores'] >0.4) * (pred_dicts_[0]['pred_labels'] ==2)) # ped
        mask_car = ((pred_dicts_[0]['pred_scores'] >0.5) * (pred_dicts_[0]['pred_labels'] ==1)) # car
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
        
        # # distance based top k selection (k=5)
        # dist =  self.get_distance(dets_all['dets'])
        # dist_mask = np.full((dist.shape), False)
        # dist_mask[np.argsort(dist)[:5]] = True
        
        # dets_all['dets'] = dets_all['dets'][dist_mask]
        # dets_all['info'] = dets_all['info'][dist_mask]
        # ==================================================================================

        # pdb.set_trace()
        # _pred_dicts_ = pred_dicts_
        if selected_id == None:
            # pass
            _pred_dicts_ = pred_dicts_
        else:
            # ===== sot =========================== 
            if tracklet_nums == 0:
                # pdb.set_trace()
                slc_idx = np.where(_pred_dicts_[0]['pred_id']==selected_id)[0][0]
                
                # box3d : x,y,z,l,w,h,rot
                box3d = _pred_dicts_[0]['pred_boxes'][slc_idx]
                score = _pred_dicts_[0]['pred_scores'][slc_idx]
                label = _pred_dicts_[0]['pred_labels'][slc_idx]
                _id = _pred_dicts_[0]['pred_id'][slc_idx]
                slc_inst = BoundingBox(
                                        center=box3d[:3], #xyz
                                        size=box3d[3:6][[1,0,2]], #lwh->wlh
                                        orientation=Quaternion(heading_to_quaternion(box3d[-1])) , #yaw -> quaternion
                                        label=label,
                                        score=score,
                                        )
                results_BBs.append(slc_inst)
                PCs.append(PointCloud(data_infer['points'][:, 1:4].transpose(1, 0)))
                # PCs = PointCloud(data_infer['points'][:, 1:4].transpose(1, 0))
                tracklet_nums += 1
                
                # pdb.set_trace()
                pred_dicts_[0]['pred_boxes'] = _pred_dicts_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_scores'] = _pred_dicts_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_labels'] = _pred_dicts_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                pred_dicts_[0]['pred_id'] = _pred_dicts_[0]['pred_id'][slc_idx][np.newaxis, ...]
                _pred_dicts_ = pred_dicts_
            else:
                cur_PC = PointCloud(data_infer['points'][:, 1:4].transpose(1, 0)) #.cuda()
                pre_BB = results_BBs[-1]
                # pre_BB = results_BBs[0]
                
                # # pdb.set_trace()
                # mask_dist = self.get_target_distance(np.concatenate([pre_BB.center[np.newaxis, ...], pre_BB.wlh[np.newaxis, ...][:, [1,0,2]], np.array(quaternion_to_yaw(pre_BB.orientation.q)).reshape(1, 1)], axis=1), pred_dicts_[0]['pred_boxes'])
                
                # # pdb.set_trace()
                # box3d = pred_dicts_[0]['pred_boxes'][mask_dist].reshape(-1)
                # score = pred_dicts_[0]['pred_scores'][mask_dist].reshape(-1)
                # label = pred_dicts_[0]['pred_labels'][mask_dist].reshape(-1)
                # # _id = pred_dicts_[0]['pred_id'][mask_dist]
                # slc_inst = BoundingBox(
                #                         center=box3d[:3], #xyz
                #                         size=box3d[3:6][[1,0,2]], #lwh->wlh
                #                         orientation=Quaternion(heading_to_quaternion(box3d[-1])) , #yaw -> quaternion
                #                         label=label,
                #                         score=score,
                #                         )
                
                
                # step 3. Get the point cloud
                target_PC = kitti_utils.cropAndCenterPC(cur_PC, pre_BB, offset=opts['offset_BB'], scale=1.25, limit_area=np.array(opts['area_extents']).reshape(3, 2)) 
                # model_PC = kitti_utils.getModel([PCs[0], PCs[tracklet_nums-1]], [results_BBs[0], results_BBs[tracklet_nums-1]], scale=1.25)
                
                model_PC = kitti_utils.getModel([PCs[-1], cur_PC], [results_BBs[0], results_BBs[tracklet_nums-1]], scale=1.25)
                # model_PC = kitti_utils.getModel([PCs, cur_PC], [results_BBs[0], slc_inst], scale=1.25)
                # PCs = cur_PC
                PCs.append(cur_PC)
                # step 3.1 translate to numpy
                target_PC = target_PC.points
                model_PC = np.array(model_PC.points, dtype=np.float32)
                # step 3.2 subsample
                target_PC = kitti_utils.subsamplePC(target_PC, opts['subsample_number'])    # (M  , 3), tensor
                model_PC = kitti_utils.subsamplePC(model_PC, opts['subsample_number']//2)   # (M/2, 3), tensor
                    
                # step 4. Regression
                # pdb.set_trace()
                pred_hm, pred_loc, pred_z_axis = sot_model( model_PC.unsqueeze(0).cuda(), 
                                                        target_PC.unsqueeze(0).cuda())  
                
                # step 5. Get current bounding box
                with torch.no_grad():
                    hm = pred_hm.sigmoid_()
                    xy_img_z_ry = mot_decode(hm,pred_loc,pred_z_axis,K=1)

                xy_img_z_ry_cpu = xy_img_z_ry.squeeze(0).detach().cpu().numpy()
                xy_img_z_ry_cpu[:,:2] = (xy_img_z_ry_cpu[:,:2]+opts['min_img_coord'].numpy())*opts['xy_size'].numpy()
                estimate_box = xy_img_z_ry_cpu[0] #offset value

                box = kitti_utils.getOffsetBBtest(pre_BB, estimate_box[:4], category=opts['db']['category_name'])
                results_BBs.append(box)
                tracklet_nums += 1
                
                # pdb.set_trace()
                pred_dicts_[0]['pred_boxes'] = np.concatenate([box.center[np.newaxis, ...], box.wlh[np.newaxis, ...][:, [1,0,2]], np.array(quaternion_to_yaw(box.orientation.q)).reshape(1, 1)], axis=1)
                pred_dicts_[0]['pred_scores'] = _pred_dicts_[0]['pred_scores']
                pred_dicts_[0]['pred_labels'] = _pred_dicts_[0]['pred_labels']
                pred_dicts_[0]['pred_id'] = _pred_dicts_[0]['pred_id']
                pass

        
        global last_box_num
        # pdb.set_trace() 
        # last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts, last_box_num=last_box_num)
        # last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts, last_box_num=last_box_num, class_id=pred_dicts[0]['pred_id'])
        last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts_, last_box_num=last_box_num, class_id=pred_dicts_[0]['pred_id'])
        
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
    global count, tracklet_nums, results_BBs, PCs
    count = 0
    tracklet_nums = 0
    results_BBs = []
    PCs = []
    selected_id = None
    opts = {'model_name': 'STNet',
            'which_dataset': 'KITTI',
            'train_test': 'test', 'use_tiny': False,
            'reference_BB': 'previous_result',
            'device': torch.device(type='cuda'),
            'batch_size': 64, 'n_workers': 12, 'n_epoches': 40,
            'n_gpus': 1, 'learning_rate': 0.001, 'subsample_number': 1024,
            'min_points_num': 5, 'IoU_Space': 3, 'seed': 1, 'is_completion': True,
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
    
