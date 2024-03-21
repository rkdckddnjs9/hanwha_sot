#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import argparse
from pathlib import Path
from collections import defaultdict

import numpy as np #for yolo, pip install numpy==1.23.5

import torch
from livoxdetection.models.ld_base_v1 import LD_base
import pdb
import time

import copy
import rospy
import ros_numpy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Int32, Float64MultiArray
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge

#======== YOLO-X model======================
from yolox.data.datasets import COCO_CLASSES
from yolox.exp import get_exp
from yolox.tools.demo import *
from tools.demo import Predictor
# ==========================================

# =============custom message ==================================
from mtt_msgs.msg import FollowTargetInfo, TargetCandidate, DynamicObject, DynamicObjectList, TargetCandidateList
# ==============================================================

#==== for custom utils =========================================
from pub_tracker import PubTracker, NUSCENES_TRACKING_NAMES, greedy_assignment
from hanwha_utils import *
#===============================================================
 
from vis_ros_hanwha import ROS_MODULE
ros_vis = ROS_MODULE()
last_box_num = 0
last_gtbox_num = 0

def selection_callback(data):
    global selected_id
    selected_id = int(data.data) 
 
def pub_target_info(data, global_box):
    custom_msg = FollowTargetInfo()
    custom_msg.header.stamp = rospy.Time.now()
    custom_msg.id = int(data['pred_id'].item())  #Int32(1)
    custom_msg.classname = int(label2pub[index2label[data['pred_labels'].item()]])
    custom_msg.score = float(data['pred_scores'].item())
    
    box = global_box.tolist()
    quat = yaw2quaternion(box[-1])
    custom_msg.pose.position = Point(box[0], box[1], box[2])
    custom_msg.pose.orientation.x = quat[0]
    custom_msg.pose.orientation.y = quat[1]
    custom_msg.pose.orientation.z = quat[2]
    custom_msg.pose.orientation.w = quat[3]
    
    custom_msg.size = Point(box[3], box[4], box[5])
    # custom_msg.velocity = Point(0.0, 0.0, 0.0)
    custom_msg.velocity = Point(data['pred_velocity'][0][0], data['pred_velocity'][0][1], data['pred_velocity'][0][2])
    
    return custom_msg

def pub_candidate_info(data, global_box):
    global index2label, label2pub
    custom_msg = TargetCandidate()
    custom_msg.header.stamp = rospy.Time.now()
    
    custom_msg.id_1 = int(data['pred_id'][0].item())  #Int32(1)
    custom_msg.class_1 = int(label2pub[index2label[data['pred_labels'][0].item()]])
    custom_msg.position_1 = Point(global_box[0][0], global_box[0][1], global_box[0][2])
    
    custom_msg.id_2 = int(data['pred_id'][1].item())  #Int32(1)
    custom_msg.class_2 = int(label2pub[index2label[data['pred_labels'][1].item()]])
    custom_msg.position_2 = Point(global_box[1][0], global_box[1][1], global_box[1][2])
    
    custom_msg.id_3 = int(data['pred_id'][2].item())  #Int32(1)
    custom_msg.class_3 = int(label2pub[index2label[data['pred_labels'][2].item()]])
    custom_msg.position_3 = Point(global_box[2][0], global_box[2][1], global_box[2][2])
    
    custom_msg.id_4 = int(data['pred_id'][3].item())  #Int32(1)
    custom_msg.class_4 = int(label2pub[index2label[data['pred_labels'][3].item()]])
    custom_msg.position_4 = Point(global_box[3][0], global_box[3][1], global_box[3][2])
    
    custom_msg.id_5 = int(data['pred_id'][4].item())  #Int32(1)
    custom_msg.class_5 = int(label2pub[index2label[data['pred_labels'][4].item()]])
    custom_msg.position_5 = Point(global_box[4][0], global_box[4][1], global_box[4][2])

    return custom_msg

def pub_dyn_candidate_info(data, global_box):
    global index2label, label2pub
    pub_msg = DynamicObjectList()
    pub_msg.object = []
    # import pdb; pdb.set_trace()
    if data['pred_id'].sum() != 0:
        for i in range(data['pred_scores'].shape[0]):
            custom_msg = DynamicObject()
            # custom_msg.header.stamp = rospy.Time.now()
            
            if int(data['pred_id'][i].item()) == 0:
                continue
            else:
                custom_msg.id = int(data['pred_id'][i].item())  #Int32(1)
                custom_msg.classname = int(label2pub[index2label[data['pred_labels'][i].item()]])
                custom_msg.score = float(data['pred_scores'][i].item())
                
                box = global_box[i].tolist()
                quat = yaw2quaternion(box[-1])
                custom_msg.pose.position = Point(box[0], box[1], box[2])
                custom_msg.pose.orientation.x = quat[0]
                custom_msg.pose.orientation.y = quat[1]
                custom_msg.pose.orientation.z = quat[2]
                custom_msg.pose.orientation.w = quat[3]
                
                custom_msg.size = Point(data['pred_boxes'][i][3], data['pred_boxes'][i][4], data['pred_boxes'][i][5])
                # custom_msg.velocity = Point(0.0, 0.0, 0.0)
                custom_msg.velocity = Point(data['pred_velocity'][i][0], data['pred_velocity'][i][1], data['pred_velocity'][i][2])
                pub_msg.object.append(custom_msg)
    else:
        custom_msg = DynamicObject()
        # custom_msg.header.stamp = rospy.Time.now()
        custom_msg.id = int(data['pred_id'][0].item())  #Int32(1)
        custom_msg.classname = int(label2pub[index2label[data['pred_labels'][0].item()]])
        custom_msg.score = float(data['pred_scores'][0].item())
        box = global_box[0].tolist()
        quat = yaw2quaternion(box[-1])
        custom_msg.pose.position = Point(box[0], box[1], box[2])
        custom_msg.pose.orientation.x = quat[0]
        custom_msg.pose.orientation.y = quat[1]
        custom_msg.pose.orientation.z = quat[2]
        custom_msg.pose.orientation.w = quat[3]
        custom_msg.size = Point(data['pred_boxes'][0][3], data['pred_boxes'][0][4], data['pred_boxes'][0][5])
        custom_msg.velocity = Point(data['pred_velocity'][0][0], data['pred_velocity'][0][1], data['pred_velocity'][0][2])
        pub_msg.object.append(custom_msg)

    return pub_msg

def pub_candidate_info_2(data, global_box):
    global index2label, label2pub
    pub_msg = TargetCandidateList()
    pub_msg.candidate = []
    # import pdb; pdb.set_trace()
    if data['pred_id'].sum() != 0:
        for i in range(data['pred_scores'].shape[0]):
            custom_msg = TargetCandidate()
            # custom_msg.header.stamp = rospy.Time.now()
            # import pdb; pdb.set_trace()
            if int(data['pred_id'][i].item()) == 0:
                continue
            
            else:
                custom_msg.id = int(data['pred_id'][i].item())  #Int32(1)
                custom_msg.classname = int(label2pub[index2label[data['pred_labels'][i].item()]])
                custom_msg.position = Point(global_box[i][0], global_box[i][1], global_box[i][2])
                # custom_msg.score = float(data['pred_scores'][i].item())

                pub_msg.candidate.append(custom_msg)
    else:
        custom_msg = TargetCandidate()
        # custom_msg.header.stamp = rospy.Time.now()
        custom_msg.id = int(data['pred_id'][0].item())  #Int32(1)
        custom_msg.classname = int(label2pub[index2label[data['pred_labels'][0].item()]])
        custom_msg.position = Point(global_box[0][0], global_box[0][1], global_box[0][2])
        pub_msg.candidate.append(custom_msg)

    return pub_msg

def signal_img_callback(data):
    global bridge, signal_img
    # signal_img = bridge.imgmsg_to_cv2(data, "32FC1") ############3 민재코드
    signal_img = ros_numpy.numpify(data)

def nearir_img_callback(data):
    global bridge, nearir_img
    nearir_img = ros_numpy.numpify(data)

def ref_img_callback(data):
    global model_2d, bridge, ref_img
    ref_img = ros_numpy.numpify(data)

def range_img_callback(data):
    global model_2d, bridge, range_img
    # uint8_data = bridge.imgmsg_to_cv2(data, "mono8") ############3 민재코드, 불필요
    range_img = ros_numpy.numpify(data)

def earth2map_callback(data):
    global earth2map
    earth2map = np.array(data.data).reshape([data.layout.dim[0].size, data.layout.dim[0].size])

def odom2base_callback(data):
    global odom2base
    odom2base = np.array(data.data).reshape([data.layout.dim[0].size, data.layout.dim[0].size])

def map2odom_callback(data):
    global map2odom
    map2odom = np.array(data.data).reshape([data.layout.dim[0].size, data.layout.dim[0].size])

def odom_roll_list_callback(data):
    global odom_roll_list
    odom_roll_list = data.data 

def odom_yaw_list_callback(data):
    global odom_yaw_list
    odom_yaw_list = data.data 

def odom_pitch_list_callback(data):
    global odom_pitch_list
    odom_pitch_list = data.data 

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')

    parser.add_argument('--pt', type=str, help='checkpoint to start from')

    
    # =================================== yolox ==============================================
    parser.add_argument('--exp_file', type=str, help='exp_file')
    parser.add_argument('--yolox_model', type=str, help='yolox model ex)yolox_x')
    parser.add_argument('--yolox_pt', type=str, help='checkpoint to start from')
    parser.add_argument('--yolox_conf', type=float, default=0.3, help='confidence score thresholding')
    parser.add_argument('--yolox_nms', type=float, default=0.3, help='nms threshold')
    #=========================================================================================
    
    # centerpoint track ==============
    parser.add_argument("--max_age", type=int, default=50) # mot and sot history memory
    parser.add_argument("--hungarian", action='store_true')
    # ==========================
    
    # ================================== for sot param ==============================
    parser.add_argument("--max_occlusion", type=int, default=40) # occlusion frame (40 = 4s)
    parser.add_argument("--moving_interval", type=int, default=5)
    parser.add_argument("--fps_distance", type=int, default=6) #default = 4 # farthest_point_sampling # for near object (# of points so about near object == large computation of clustering)
    parser.add_argument("--max_ransac_dist", type=int, default=30)
    parser.add_argument("--min_ransac_dist", type=int, default=10)
    parser.add_argument("--max_transfer_dist", type=int, default=5) # transfer box distance for ped
    parser.add_argument("--max_car_transfer_dist", type=int, default=20)
    # ================================================================================
    
    # ===== defaulut mtt msgs len ==============================
    parser.add_argument("--num_candidate", type=int, default=10)
    # ==========================================================
    
    
    
    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    # import pdb; pdb.set_trace()
    if not args.pt:
        args.pt = rospy.get_param('~pt', 'default_model_path.pt')
    
    if not args.exp_file:
        args.exp_file = rospy.get_param('~exp_file', 'default_model_path.pt')
    
    if not args.yolox_model:
        args.yolox_model = rospy.get_param('~yolox_model', 'default_model_path.pt')
    
    if not args.yolox_pt:
        args.yolox_pt = rospy.get_param('~yolox_pt', 'default_model_path.pt')
        
    try:
        args.max_age = rospy.get_param('~max_age')
        args.max_occlusion = rospy.get_param('~max_occlusion')
        args.moving_interval = rospy.get_param('~moving_interval')
        args.fps_distance = rospy.get_param('~fps_distance')
        args.max_transfer_dist = rospy.get_param('~max_transfer_dist')
        args.max_car_transfer_dist = rospy.get_param('~max_car_transfer_dist')
        args.max_age = rospy.get_param('~max_age')
        args.num_candidate = rospy.get_param('~num_candidate') #for candidate 10 -> pub: 5
        args.yolox_conf = rospy.get_param('~yolox_conf')
        args.yolox_nms = rospy.get_param('~yolox_nms')
        
    except:
        pass

    #import pdb; pdb.set_trace() 
    return args

class ros_demo():
    def __init__(self, model, args=None):
        self.args = args
        self.model = model
        self.starter, self.ender = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)

        self.offset_angle = 0
        self.offset_ground = 0.0

        self.point_cloud_range = [0, -44.8, -2, 57.6, 44.8, 4]
        self.mot_tracker = PubTracker(max_age=args.max_age, hungarian=args.hungarian)
        self.sot_tracker = PubTracker(max_age=args.max_age, hungarian=args.hungarian)
        
        self.bridge = CvBridge()
        self.iter = 0

    def receive_from_ros(self, msg):
        pc = ros_numpy.numpify(msg)
        points_list = np.zeros((pc.shape[0], 4))
        points_list[:, 0] = copy.deepcopy(np.float32(pc['x']))
        points_list[:, 1] = copy.deepcopy(np.float32(pc['y']))
        points_list[:, 2] = copy.deepcopy(np.float32(pc['z']))
        points_list[:, 3] = copy.deepcopy(np.float32(pc['intensity']))

        # preprocess 
        # points_list[:, 2] += points_list[:, 0] * np.tan(self.offset_angle / 180. * np.pi) + self.offset_ground
        rviz_points = copy.deepcopy(points_list)
        # points_list = mask_points_out_of_range(points_list, self.point_cloud_range)

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
        # ros_image.data = np_array.tostring()  
        ros_image.data = np_array.tobytes()  
        np_array =[]
        return ros_image

    def online_inference(self, msg):
        global selected_id, _pred_dicts_, slc_label, sot_list, count, index2label, label2pub, ocl_count, cluster_count
        global last_box_num, pred_dicts_sot, init_h, temp_, viz_dicts, prev_pred_dicts_mot, zeor_pub_flag, horizontal_FOV, vertical_FOV

        torch.cuda.synchronize()
        self.starter.record()
        data_dict = self.receive_from_ros(msg)
        # data_dict = self.receive_from_ros_custom(msg)
        data_infer = ros_demo.collate_batch([data_dict])
        ros_demo.load_data_to_gpu(data_infer)
        temp_infer_dicts = copy.deepcopy(data_infer)
        mot_time_lag = 1
        sot_time_lag = 1
        pc_viz=False
        
        self.model.eval()
        with torch.no_grad(): 
            pred_dicts = self.model.forward(data_infer)

        pred_dicts[0]['pred_labels'] = torch.where(pred_dicts[0]['pred_labels']==3, 2, pred_dicts[0]['pred_labels']) #cyclist to pedestrian #['bg', 'Vehicle', 'Pedestrian', 'Cyclist'], 
        
        #score based filtering
        pred_dicts[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][:20, :]
        pred_dicts[0]['pred_scores'] = pred_dicts[0]['pred_scores'][:20]
        pred_dicts[0]['pred_labels'] = pred_dicts[0]['pred_labels'][:20]
        
        # data_infer, pred_dicts = ROS_MODULE.gpu2cpu_w_id(data_infer, pred_dicts)
        data_infer, pred_dicts = ROS_MODULE.gpu2cpu(data_infer, pred_dicts)
        
        # =================== mot =======================================
        pred_for_track = transform_pred(pred_dicts[0])
        outputs_mot = self.mot_tracker.step_centertrack(pred_for_track, mot_time_lag)
        pred_dicts_mot = transform_track_np(outputs_mot)
        viz_dicts = pred_dicts_mot
        # ===============================================================
        
        try:
            pred_dicts_mot = compute_velocity(prev_pred_dicts_mot, pred_dicts_mot)
        except: # if prev_pred_dicts_mot is None
            pred_dicts_mot[0]['pred_velocity'] = np.zeros((pred_dicts_mot[0]['pred_id'].shape[0],3))
        prev_pred_dicts_mot = pred_dicts_mot
                
        # ============================== yolo x ==========================================
        try:
            nearir_img_ = nearir_img.copy()
            nearir_img_ = cv2.applyColorMap(nearir_img_,cv2.COLORMAP_COOL) #for viz
            img_stack = np.vstack([nearir_img[np.newaxis,...],range_img[np.newaxis,...],ref_img[np.newaxis,...]])
            # img_stack = np.vstack([nearir_img[np.newaxis,...],nearir_img[np.newaxis,...],nearir_img[np.newaxis,...]])
            imgees = nearir_img_
            
            if nearir_img.shape == (128, 1152):
                horizontal_FOV=202.5
                vertical_FOV=45
            else:
                horizontal_FOV=180
                vertical_FOV=45
            
            # import pdb; pdb.set_trace()
            output = predictor.inference_2(img_stack)
            try:
                box2d=output.numpy()
            except:
                box2d=np.array([])
        except:
            print("img topic is empty")
            box2d=np.array([])
        # ===============================================================================      
        
        box3d = viz_dicts[0]['pred_boxes']
        box3d_cls = viz_dicts[0]['pred_labels'].reshape(-1, 1) # (class_id) # yolo_class: 1:ped, 3:car, 6:bus, 8:truck, -> 0, 2, 5, 7
        
        if box2d.shape[0] == 0 and box3d.shape[0] != 0:
            # print("yolo output is empty / centerpoint output is available")
            zeor_pub_flag = True
            
        elif box2d.shape[0] == 0 and box3d.shape[0] == 0:
            # print("yolo output is empty / centerpoint output is empty")
            zeor_pub_flag = True
            
        elif box2d.shape[0] != 0 and box3d.shape[0] == 0:
            # print("yolo output is avalilble / centerpoint output is empty")
            zeor_pub_flag = True
            
        else:
            # import pdb; pdb.set_trace()
            box2d_cls = box2d[:, 5:6]
            cls_mask = (box2d_cls == 0.)+(box2d_cls == 2.)+(box2d_cls == 5.)+(box2d_cls == 7.)
            box2d = box2d[cls_mask[:, 0]]
            box2d_cls = box2d_cls[cls_mask[:, 0]]
                        
            box3dto2d = transform_3d_to_2d_w_hw(box3ds=box3d, H=nearir_img.shape[0] ,W=nearir_img.shape[1], horizontal_FOV=horizontal_FOV, vertical_FOV=vertical_FOV)

            iou_matrix_vectorized = bb_intersection_over_union_vectorized(box2d[:, :4], box3dto2d)
            if iou_matrix_vectorized.shape[0] != 0:
                # import pdb; pdb.set_trace()
                best_N, best_M, best_IoU = find_best_pairs_with_same_class(iou_matrix_vectorized, box2d_cls, box3d_cls, box2d[:, :4], box3dto2d, self.iter, min_iou=0.3)
                # 내림차순 정렬 인덱스
                sort_idx = np.argsort(best_IoU)[::-1][:10] 
                
                # sort_idx에 따라 best_N, best_M, best_IoU 정렬
                best_N_sorted = best_N[sort_idx]
                best_M_sorted = best_M[sort_idx]
                best_IoU_sorted = best_IoU[sort_idx]
                                
                if best_M_sorted.shape[0] != 0:
                    # import pdb; pdb.set_trace()
                    zeor_pub_flag = False
                    best_2d = box2d[:, :4][best_N_sorted]
                    best_3dto2d = box3dto2d[best_M_sorted]

                    green_color = (0,255,0)
                    red_color = (0,0,255)
                    for best_idx in range(len(best_IoU_sorted)):
                        best_2d_ = best_2d[best_idx].astype(np.int32) # for yolox
                        best_3dto2d_ = best_3dto2d[best_idx].astype(int)
                        cv2.rectangle(imgees, tuple(best_2d_[:2].tolist()), tuple(best_2d_[2:].tolist()),red_color)
                        cv2.rectangle(imgees, tuple(best_3dto2d_[:2].tolist()), tuple(best_3dto2d_[2:].tolist()), green_color)

                else:
                    # print("No matched object !! ")
                    zeor_pub_flag = True
            else:
                # print("No matched object !! ")
                zeor_pub_flag = True
                
            self.iter += 1
            
        pub_dicts = [{'pred_boxes':np.zeros((args.num_candidate, 7)),
                    'pred_scores':np.zeros((args.num_candidate,)),
                    'pred_labels':np.ones((args.num_candidate,), dtype=np.int32),
                    'pred_id':np.zeros((args.num_candidate,), dtype=np.int32),
                    'pred_velocity':np.zeros((args.num_candidate,3), dtype=np.float32),
                    }]
        
        if zeor_pub_flag == True : #zero publish !!
            # pub_dicts[0]['pred_boxes'][:viz_dicts[0]['pred_boxes'][:args.num_candidate, :].shape[0], :] = viz_dicts[0]['pred_boxes'][:args.num_candidate, :]
            # pub_dicts[0]['pred_scores'][:viz_dicts[0]['pred_scores'][:args.num_candidate].shape[0]]  = viz_dicts[0]['pred_scores'][:args.num_candidate]
            # pub_dicts[0]['pred_labels'][:viz_dicts[0]['pred_labels'][:args.num_candidate].shape[0]]  = viz_dicts[0]['pred_labels'][:args.num_candidate]
            # pub_dicts[0]['pred_id'][:viz_dicts[0]['pred_id'][:args.num_candidate].shape[0]]  = viz_dicts[0]['pred_id'][:args.num_candidate]
            # pub_dicts[0]['pred_velocity'][:viz_dicts[0]['pred_velocity'][:args.num_candidate, :].shape[0], :] = viz_dicts[0]['pred_velocity'][:args.num_candidate, :]
            pass
            
        else:
            # import pdb; pdb.set_trace()
            pub_dicts[0]['pred_boxes'][:viz_dicts[0]['pred_boxes'][best_M_sorted][:args.num_candidate, :].shape[0], :] = viz_dicts[0]['pred_boxes'][best_M_sorted][:args.num_candidate, :]
            pub_dicts[0]['pred_scores'][:viz_dicts[0]['pred_scores'][best_M_sorted][:args.num_candidate].shape[0]] = viz_dicts[0]['pred_scores'][best_M_sorted][:args.num_candidate]
            pub_dicts[0]['pred_labels'][:viz_dicts[0]['pred_labels'][best_M_sorted][:args.num_candidate].shape[0]] = viz_dicts[0]['pred_labels'][best_M_sorted][:args.num_candidate]
            pub_dicts[0]['pred_id'][:viz_dicts[0]['pred_id'][best_M_sorted][:args.num_candidate].shape[0]] = viz_dicts[0]['pred_id'][best_M_sorted][:args.num_candidate]
            pub_dicts[0]['pred_velocity'][:viz_dicts[0]['pred_velocity'][best_M_sorted][:args.num_candidate].shape[0], :] = viz_dicts[0]['pred_velocity'][best_M_sorted][:args.num_candidate, :]
        
        # distance-based sorting
        dist_list = get_target_distance_v2(np.array([[0,0,0]]), pub_dicts[0]['pred_boxes'])[0]
        dist_list = [100 if ii == 0 else ii for ii in dist_list] # for no object processing
        dist_sort_idx = np.argsort(dist_list)
        pub_dicts[0]['pred_boxes'] = pub_dicts[0]['pred_boxes'][dist_sort_idx, :]
        pub_dicts[0]['pred_scores'] = pub_dicts[0]['pred_scores'][dist_sort_idx]
        pub_dicts[0]['pred_labels'] = pub_dicts[0]['pred_labels'][dist_sort_idx]
        pub_dicts[0]['pred_id'] = pub_dicts[0]['pred_id'][dist_sort_idx]
        pub_dicts[0]['pred_velocity'] = pub_dicts[0]['pred_velocity'][dist_sort_idx, :]
        
        viz_dicts = pub_dicts
        dyn_obj_dicts = pub_dicts
            
        
        if selected_id == None or selected_id == -1:
            if pred_dicts[0]['pred_labels'].shape[0] != 0:
                _pred_dicts_ = pred_dicts_mot
                pc_viz=False  
                
                # ==================== sot ======================================
                outputs_sot = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                pred_dicts_sot = transform_track_np(outputs_sot)
                # ===============================================================
                
                # ===================== MTT msgs publish ===========================================
                rotated_box = pub_dicts[0]['pred_boxes'].copy()[:, :3]

                candidate_msg = pub_candidate_info_2(pub_dicts[0], rotated_box)
                #candidate_msg = pub_candidate_info_2(pred_dicts_mot[0], pred_dicts_mot[0]['pred_boxes'])
                candidate_info_pub.publish(candidate_msg)
                
                # dynamic_msgs publish
                # dyn_msg = pub_dyn_candidate_info(pred_dicts_mot[0], pred_dicts_mot[0]['pred_boxes'])
                # print(pub_dicts[0]['pred_boxes'].shape)
                dyn_msg = pub_dyn_candidate_info(dyn_obj_dicts[0], dyn_obj_dicts[0]['pred_boxes'])
                dyn_candidate_info_pub.publish(dyn_msg)             
                # print(_pred_dicts_[0]['pred_id'].shape)   
                # ====================== MTT msgs end ========================================================
            else:
                pc_viz=True
                viz_dicts = pred_dicts
            
            sot_list = []
            cluster_count = 0
            ocl_count=0
            
        else:
            try:
                if np.where(_pred_dicts_[0]['pred_id'] == selected_id)[0].shape[0] == 0:
                    print("invalid object !!")
                    selected_id = None
                    invalid_skip_flag = True
                else:
                    invalid_skip_flag = False
            except:
                if 'pred_id' not in _pred_dicts_[0].keys():
                    print("invalid object !!")
                    selected_id = None
                    invalid_skip_flag = True
                else:
                    invalid_skip_flag = False
                    
            
            if invalid_skip_flag:
                pass
            
            else:
                slc_idx = np.where(pred_dicts_sot[0]['pred_id'] == selected_id)[0][0]

                # slc_label = pred_dicts_sot[0]['pred_id'][slc_idx]
                slc_label = pred_dicts_sot[0]['pred_labels'][slc_idx] # 1: vehicle
                if cluster_count ==0:
                    x,y,z,l,w,h  = pred_dicts_sot[0]['pred_boxes'][slc_idx][np.newaxis, ...][0][:6]
                    init_h = pred_dicts_sot[0]['pred_boxes'][slc_idx][np.newaxis, ...][0][5]
                
                    if len(sot_list) < 10:
                        sot_list.append(np.array([[x,y,z,l,w,h]]))
                    else:
                        sot_list.append(np.array([[x,y,z,l,w,h]]))
                        sot_list.pop(0)
                    cluster_count += 1
                else:
                    x,y,z,l,w,h = sot_list[-1][0]

                
                # ================ Vehicle SOT ==========================================
                if slc_label == 1:     
                    
                    prev_roll = odom_roll_list[-2]
                    cur_roll = odom_roll_list[-1]
                    var_roll = cur_roll - prev_roll
                    rot_m = yaw_to_rotation_matrix(var_roll*args.moving_interval)
                    x, y, z = rot_m @ np.array([x,y,z])
                                       
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
                    
                    if len(sot_list) > 1:
                        prev_x, prev_y, prev_z = sot_list[-2][0][:3]
                        vx, vy, vz = x - prev_x, y - prev_y, z - prev_z
                    else:
                        vx, vy, vz = 0, 0, 0
                    
                    if ocl_count < args.max_occlusion:
                        try:
                            cut_range = np.array([x-10, y-10, -2, x+10, y+10, 4])
                            temp_pc = mask_points_out_of_range_3(data_infer['points'], cut_range)
                            shift_value = temp_pc[:, 3].min()
                            temp_pc[:, 3] -= shift_value
                            temp_pc = mask_points_out_of_range_3(temp_pc, self.point_cloud_range)

                            temp_infer_dicts['points'] = torch.tensor(temp_pc).to(device=temp_infer_dicts['points'].device)
                            with torch.no_grad(): 
                                temp_pred_dicts = self.model.forward(temp_infer_dicts)
                            temp_data_infer, temp_pred_dicts = ROS_MODULE.gpu2cpu(temp_infer_dicts, temp_pred_dicts)   
                            # ros_vis.ros_print_2(-temp_infer_dicts['points'][:, 1:5])
                            # import pdb; pdb.set_trace()
                            label_mask = temp_pred_dicts[0]['pred_labels'] == 1
                            bbox_list = temp_pred_dicts[0]['pred_boxes'][label_mask]
                            bbox_list[:,2] += shift_value
                            # print(bbox_list.shape)
                            dist_mask =  get_target_distance(sot_list[-1].copy(), bbox_list)
                            temp_ = [{}]
                            temp_[0]['pred_boxes'] = bbox_list[dist_mask]
                            temp_[0]['pred_scores'] = np.ones(bbox_list.shape[0])[dist_mask]
                            temp_[0]['pred_labels'] = np.full(bbox_list.shape[0], slc_label)[dist_mask]

                            pred_for_track = transform_pred_car(temp_[0], vx, vy)
                            outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                            temp_ = transform_track_np(outputs)
                            ocl_count = 0
                        except:
                            ocl_count += 1
                            temp_ = _pred_dicts_
                            pred_for_track = transform_pred(temp_[0])
                            outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                            temp_ = transform_track_np(outputs)
                            print("fully occlusion! : {}, {}".format(ocl_count, args.max_occlusion))
                            # import pdb; pdb.set_trace()
                            
                        slc_idx = np.where(temp_[0]['pred_id'] == selected_id)[0][0]
                        slc_label = temp_[0]['pred_labels'][slc_idx]
                        # print("slc_label : {}".format(slc_label))
                        temp_[0]['pred_boxes'] = temp_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                        temp_[0]['pred_scores'] = temp_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                        temp_[0]['pred_labels'] = temp_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                        temp_[0]['pred_id'] = temp_[0]['pred_id'][slc_idx][np.newaxis, ...]
                        _pred_dicts_ = temp_

                        if len(sot_list) < 10:
                            sot_list.append([_pred_dicts_[0]['pred_boxes'][0][:6]]) #np.array([temp_[0]['pred_boxes'][0][:3]])
                        else:
                            sot_list.append([_pred_dicts_[0]['pred_boxes'][0][:6]])
                            sot_list.pop(0)
                    
                    elif ocl_count >= args.max_occlusion:
                        # import pdb; pdb.set_trace()
                        # new object search
                        prev_label = 1 # Car
                        print("max occlusion !!  :  {} / {} ".format(ocl_count, args.max_occlusion))
                        
                        if dyn_obj_dicts[0]['pred_id'][0] != 0: #no zero_pub = searching matched object!
                            label_mask = (dyn_obj_dicts[0]['pred_labels']==prev_label)
                            if label_mask.sum() != 0 and dyn_obj_dicts[0]['pred_id'][label_mask].sum() != 0: # no same cls obj in current detection result 
                                label_mask *= (dyn_obj_dicts[0]['pred_scores']>0)
                                pred_dicts[0]['pred_boxes'] = dyn_obj_dicts[0]['pred_boxes'][label_mask]
                                pred_dicts[0]['pred_scores'] = dyn_obj_dicts[0]['pred_scores'][label_mask]
                                pred_dicts[0]['pred_labels'] = dyn_obj_dicts[0]['pred_labels'][label_mask]
                                
                                dist_mask =  get_target_distance_v2(sot_list[-1].copy(), pred_dicts[0]['pred_boxes']) #distance value return
                                min_dist_idx = np.where(dist_mask[0]==dist_mask.min())[0].item()
                                dist_mask = dist_mask[0] < args.max_car_transfer_dist

                                if dist_mask[min_dist_idx] == True:
                                    temp_ = [{}]
                                    temp_[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][min_dist_idx][np.newaxis, ...]
                                    temp_[0]['pred_scores'] = pred_dicts[0]['pred_scores'][min_dist_idx][np.newaxis, ...]
                                    temp_[0]['pred_labels'] = pred_dicts[0]['pred_labels'][min_dist_idx][np.newaxis, ...]
                                    
                                    pred_for_track = transform_pred(temp_[0])
                                    outputs = self.sot_tracker.step_centertrack(pred_for_track, sot_time_lag)
                                    temp_ = transform_track_np(outputs)
                                    vx, vy, vz = 0, 0, 0
                                    
                                    slc_idx_ = np.isclose(pred_dicts[0]['pred_boxes'][dist_mask][:,np.newaxis, :], temp_[0]['pred_boxes'], atol=1e-6).all(axis=2)
                                    slc_idx = np.where(slc_idx_[0]==True)[0][0]
                                    slc_label = temp_[0]['pred_labels'][slc_idx]
                                    temp_[0]['pred_boxes'] = temp_[0]['pred_boxes'][slc_idx][np.newaxis, ...]
                                    temp_[0]['pred_scores'] = temp_[0]['pred_scores'][slc_idx][np.newaxis, ...]
                                    temp_[0]['pred_labels'] = temp_[0]['pred_labels'][slc_idx][np.newaxis, ...]
                                    temp_[0]['pred_id'] = temp_[0]['pred_id'][slc_idx][np.newaxis, ...]
                                    pred_dicts_sot = temp_
                                    _pred_dicts_ = temp_
                                        
                                    sot_list = []
                                    if len(sot_list) < 10:
                                        sot_list.append(np.array([temp_[0]['pred_boxes'][0][:6]]))
                                    else:
                                        sot_list.append(np.array([temp_[0]['pred_boxes'][0][:6]]))
                                        sot_list.pop(0)
                                        
                                    selected_id = _pred_dicts_[0]['pred_id'].item()
                                    ocl_count = 0
                                else:
                                    ocl_count += 1
                                    pc_viz=True
                            else: # same cls obj in current detection result 
                                ocl_count += 1
                        else:
                            ocl_count += 1
                else:
                    # import pdb; pdb.set_trace()
                    # base yaw angle compensation !
                    # odom topic average hz : 50 Hz -> delta theta * args.moving_interval(default = 5)
                    prev_yaw = odom_yaw_list[-2]
                    cur_yaw = odom_yaw_list[-1]
                    var_yaw = cur_yaw - prev_yaw
                    rot_m = yaw_to_rotation_matrix(-var_yaw*args.moving_interval)
                    x, y, z = rot_m @ np.array([x,y,z])
                    
                    prev_pitch = odom_pitch_list[-2]
                    cur_pitch = odom_pitch_list[-1]
                    var_pitch = cur_pitch - prev_pitch
                    rot_m = pitch_to_rotation_matrix(-var_pitch*args.moving_interval)
                    x, y, z = rot_m @ np.array([x,y,z])
                    
                    # prev_z = lidar_z_list[-2][-1]
                    # cur_z = lidar_z_list[-1][-1]
                    # var_z = cur_z - prev_z
                    # z += var_z
                    
                    if len(sot_list) > 1:
                        prev_x, prev_y, prev_z = sot_list[-2][0][:3]
                        vx, vy, vz = x - prev_x, y - prev_y, z - prev_z
                    else:
                        vx, vy, vz = 0, 0, 0
                    
                    z = np.mean(np.array(sot_list[-5:])[:, :, 2])
                    obj_dist = np.sqrt(np.array(x**2+y**2))
                    if obj_dist <3:
                        box_z_filter = -0.3
                    else:
                        box_z_filter = update_dynamic_z_filter_base(sot_list=sot_list[-5:], z_variation_threshold=0.1, min_z_filter=-0.6) #for pohang
                    # import pdb; pdb.set_trace()
                    if ocl_count < args.max_occlusion:
                        try:
                            # cut_range = np.array([x-0.36, y-0.36, z-0.4, x+0.36, y+0.36, z+0.4])
                            cut_range = np.array([x-0.36, y-0.36, -2, x+0.36, y+0.36, z+0.4])
                            temp_pc = mask_points_out_of_range_2(data_infer['points'][:, 1:4], cut_range)
                            min_z = temp_pc[:, 2].min() 
                            # print('occ min_z: ', min_z)
                            if min_z > z-0.5:
                                cut_range = np.array([x-0.36, y-0.36, z-0.4, x+0.36, y+0.36, z+0.4])
                            else:
                                min_z = (min_z + 1.8) / 2.
                                cut_range = np.array([x-0.36, y-0.36, min_z-0.4, x+0.36, y+0.36, z+0.4])
                            # print('boz_z, z, z+0.4: {} {} {}'.format(box_z_filter, min_z-0.4, z+0.4) )
                            temp_pc = mask_points_out_of_range_2(temp_pc, cut_range)
                            

                            if obj_dist < args.fps_distance:
                                # start = time.time()
                                temp_pc = farthest_point_sampling_optimized(temp_pc, num_samples=100)
                                
                            
                            # # clusters = hdbscan_cluster_obstacles_2(temp_pc)
                            # if np.sqrt(np.array(x**2+y**2)) < args.max_ransac_dist and np.sqrt(np.array(x**2+y**2)) > args.min_ransac_dist:
                            #     # start = time.time()
                            #     temp_pc = remove_ground_points(temp_pc)

                            clusters = cluster_obstacles_2(temp_pc, eps=0.3, min_samples=5)
                            ocl_count = 0
                            bbox_list = calculate_bounding_boxes(clusters)
                            # bbox_list = bbox_list[(bbox_list[:, 3]<1.3), :] # l filtering
                            # bbox_list = bbox_list[(bbox_list[:, 4]<1.3), :] # w filtering
                            bbox_list = bbox_list[(bbox_list[:, 3]<1.), :] # l filtering
                            bbox_list = bbox_list[(bbox_list[:, 4]<1.), :] # w filtering
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
                            print("fully occlusion! : {}, {}".format(ocl_count, args.max_occlusion))
                            try:
                                # cut_range = np.array([x-1., y-1., z-0.4, x+1., y+1., z+0.5])
                                cut_range = np.array([x-1., y-1., -2, x+1., y+1., 4])
                                temp_pc = mask_points_out_of_range_2(data_infer['points'][:, 1:4], cut_range)
                                # print('fully min_z: ', min_z)
                                min_z = temp_pc[:, 2].min() 
                                if min_z > z-0.5:
                                    cut_range = np.array([x-1., y-1., z-0.4, x+1., y+1., z+0.5])
                                else:
                                    min_z = (min_z + 1.8) / 2.
                                    # print('diff: ', z+0.5-box_z_filter)
                                    cut_range = np.array([x-1., y-1., min_z-0.4, x+1., y+1., z+0.5])
                                print('boz_z, z, z+0.4: {} {} {}'.format(box_z_filter, min_z-0.4, z+0.4) )
                                temp_pc = mask_points_out_of_range_2(temp_pc, cut_range)
                                
                                if np.sqrt(np.array(x**2+y**2)) < args.fps_distance:
                                    # start = time.time()
                                    temp_pc = farthest_point_sampling_optimized(temp_pc, num_samples=200)
                                    # print("FPS sampling time :", time.time() - start)
                                    
                                clusters = cluster_obstacles_2(temp_pc, eps=0.3, min_samples=5)
                                bbox_list = calculate_bounding_boxes(clusters)
                                # bbox_list = bbox_list[(bbox_list[:, 2]<1.3), :] #center_z filtering
                                # bbox_list = bbox_list[(bbox_list[:, 3]<1.3), :] # l filtering
                                # bbox_list = bbox_list[(bbox_list[:, 4]<1.3), :] # w filtering
                                bbox_list = bbox_list[(bbox_list[:, 3]<1.), :] # l filtering
                                bbox_list = bbox_list[(bbox_list[:, 4]<1.), :] # w filtering
                                bbox_list = bbox_list[(bbox_list[:, 5]>0.1), :] # h filtering
                                bbox_list = bbox_list[(bbox_list[:, 2]>box_z_filter), :]
                                bbox_list = bbox_list[nms_without_scores(bbox_list, 0.9)[:10]]         

                                # bbox_list[:, 2] += var_z
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
                                ocl_count = 0
                                
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
                            sot_list.append(np.array([temp_[0]['pred_boxes'][0][:6]]))
                        else:
                            sot_list.append(np.array([temp_[0]['pred_boxes'][0][:6]]))
                            sot_list.pop(0)

                    else: # occlusion!! # for box transfer
                        print("max count of occlusion !! : {}/{}".format(ocl_count, args.max_occlusion))

                        try: # if no object in current frame
                            # import pdb; pdb.set_trace()
                            if dyn_obj_dicts[0]['pred_id'][0] != 0:
                                label_mask = dyn_obj_dicts[0]['pred_labels']==slc_label
                                if label_mask.sum() != 0:
                                    # import pdb; pdb.set_trace()
                                    label_mask *= (dyn_obj_dicts[0]['pred_scores']>0)
                                    vx, vy, vz = 0, 0, 0
                                    pred_dicts[0]['pred_boxes'] = dyn_obj_dicts[0]['pred_boxes'][label_mask]
                                    pred_dicts[0]['pred_scores'] = dyn_obj_dicts[0]['pred_scores'][label_mask]
                                    pred_dicts[0]['pred_labels'] = dyn_obj_dicts[0]['pred_labels'][label_mask]
                                    
                                    dist_mask =  get_target_distance_v2(sot_list[-1].copy(), pred_dicts[0]['pred_boxes']) #distance value return
                                    min_dist_idx = np.where(dist_mask[0]==dist_mask.min())[0].item()
                                    dist_mask = dist_mask[0] < args.max_transfer_dist
                                    # import pdb; pdb.set_trace()
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
                                        pred_dicts_sot = temp_
                                        _pred_dicts_ = temp_
                                            
                                        sot_list = []
                                        if len(sot_list) < 10:
                                            sot_list.append(np.array([temp_[0]['pred_boxes'][0][:6]]))
                                        else:
                                            sot_list.append(np.array([temp_[0]['pred_boxes'][0][:6]]))
                                            sot_list.pop(0)
                                        selected_id = _pred_dicts_[0]['pred_id'].item()
                                        ocl_count = 0
                                    else:
                                        ocl_count += 1
                                        pc_viz=True
                                else:
                                    ocl_count += 1
                                    pc_viz=True
                            else:
                                ocl_count += 1
                        except:
                            pc_viz=True
                            
                # ==== sot 3d box to 2d projection and viz==============
                # import pdb; pdb.set_trace()
                try:
                    box3dto2d = transform_3d_to_2d_w_hw(box3ds=temp_[0]['pred_boxes'], H=nearir_img.shape[0] ,W=nearir_img.shape[1], horizontal_FOV=horizontal_FOV, vertical_FOV=vertical_FOV)
                    blue_color = (255,0,0)
                    for idx in range(len(box3dto2d)):
                        box_3dto2d_ = box3dto2d[idx].astype(int)
                        cv2.rectangle(imgees, tuple(box_3dto2d_[:2].tolist()), tuple(box_3dto2d_[2:].tolist()),blue_color)
                except:
                    pass
                # =========================================================
                
    
                # local to global transformation ==================================
                viz_dicts = _pred_dicts_
                rotated_box = _pred_dicts_[0]['pred_boxes'][0].copy()
                pub_dict = _pred_dicts_[0].copy()
                # import pdb; pdb.set_trace()
                pub_dict['pred_velocity'] = np.array([[vx, vy, vz]])
                homogeneous_center = np.array([0., 0., 0., 1.0])
                homogeneous_center[:3] = rotated_box[:3] # os_updowm
                homogeneous_center[:3] += np.array([1.4, 0.0, -0.37]) # os_updown -> base_link
                
                homogeneous_center = (odom2base @ homogeneous_center.T).T # base_link -> odom

                rotated_box[:3] = homogeneous_center[:3]
                rotated_box[:3] += earth2map[:3, 3] # odom -> earth
                try:
                    target_msg = pub_target_info(pub_dict, rotated_box)
                except:
                    import pdb; pdb.set_trace()
                target_info_pub.publish(target_msg)
                
                # dynamic_msgs publish
                dyn_msg = pub_dyn_candidate_info(dyn_obj_dicts[0], dyn_obj_dicts[0]['pred_boxes'])
                dyn_candidate_info_pub.publish(dyn_msg)    
                # ================================================================
        
        try:
            img_pub.publish(self.bridge.cv2_to_imgmsg(imgees))  
        except:
            pass
        
        self.ender.record()
        torch.cuda.synchronize()
        curr_latency = self.starter.elapsed_time(self.ender)
        print('det_time(ms): ', curr_latency)
        # if curr_latency >=90.:
        #     print('det_time(ms): ', curr_latency)
        #     # try:
        #     #     # print("temp_pc.shape : {}".format(temp_pc.shape))
        #     # except:
        #     #     pass
        #     # import pdb; pdb.set_trace()
        
        if pc_viz == False:
            try:
                # if selected_id != None and selected_id != -1 and slc_label != 1:
                #     if ocl_count ==0:
                #         # 임시 박스 h 변경 
                #         # _pred_dicts_[0]['pred_boxes'][0, 5] = 1.0
                #         viz_dicts[0]['pred_boxes'][0, 5] = 1.0
                # _pred_dicts_[0]['pred_boxes'][:, 1:3] *= -1
                # _pred_dicts_[0]['pred_boxes'][:, 2] *= -1
                last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=_pred_dicts_, last_box_num=last_box_num, class_id=_pred_dicts_[0]['pred_id'])
                # last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=viz_dicts, last_box_num=last_box_num, class_id=viz_dicts[0]['pred_id'])
            except:
                last_box_num, _ = ros_vis.ros_print_wo_id(data_dict['points_rviz'][:, 0:4], pred_dicts=_pred_dicts_, last_box_num=last_box_num,)
                # last_box_num, _ = ros_vis.ros_print_wo_id(data_dict['points_rviz'][:, 0:4], pred_dicts=viz_dicts, last_box_num=last_box_num,)
        else:
            ros_vis.ros_print_2(data_dict['points_rviz'][:, 0:4])
        

if __name__ == '__main__':
    args = parse_config()
    model = LD_base()

    checkpoint = torch.load(args.pt, map_location=torch.device('cpu'))  
    model.load_state_dict({k.replace('module.', ''):v for k, v in checkpoint['model_state_dict'].items()})
    model.cuda()
    
    # ================ yolox ================================
    exp_file = args.exp_file+"{}.py".format(args.yolox_model)
    trt_file = args.yolox_pt+args.yolox_model+"/model_trt.pth"
    
    # yolox_args = argparse.Namespace(camid=0, ckpt=None, conf=args.yolox_conf, demo='image',
    #                                 device='cpu', exp_file=exp_file, 
    #                                 experiment_name=None, fp16=False, fuse=False, legacy=False, name=None, nms=args.yolox_nms, 
    #                                 path=None, save_result=True, trt=True, tsize=None)
    
    yolox_args = argparse.Namespace(camid=0, ckpt=None, conf=args.yolox_conf, demo='image',
                                    device='gpu', exp_file=exp_file, 
                                    experiment_name=None, fp16=False, fuse=False, legacy=False, name=None, nms=args.yolox_nms, 
                                    path=None, save_result=True, trt=True, tsize=None)
    
    exp = get_exp(yolox_args.exp_file, yolox_args.name)

    if yolox_args.trt:
        yolox_args.device = "gpu"

    if yolox_args.conf is not None:
        exp.test_conf = yolox_args.conf
    if yolox_args.nms is not None:
        exp.nmsthre = yolox_args.nms
    if yolox_args.tsize is not None:
        exp.test_size = (yolox_args.tsize, yolox_args.tsize)
    model_2d = exp.get_model()

    if yolox_args.device == "gpu":
        model_2d.cuda()
    model_2d.eval()

    if yolox_args.trt:
        assert not yolox_args.fuse, "TensorRT model is not support model fusing!"
        # import pdb; pdb.set_trace()
        assert os.path.exists(
            trt_file
        ), "TensorRT model is not found!\n Run python3 tools/trt.py first!"
        # import pdb; pdb.set_trace()
        model_2d.head.decode_in_inference = False
        decoder = model_2d.head.decode_outputs
    else:
        trt_file = None
        decoder = None

    predictor = Predictor(
        model_2d, exp, COCO_CLASSES, trt_file, decoder,
        yolox_args.device, yolox_args.fp16, yolox_args.legacy,
    )
    #=========================================================
    
    
    
    # =========== for SOT tracking =====================================
    cluster_count=0
    index2label = {1:'Vehicle', 2:'Pedestrian', 3:'Something'}
    label2pub = {'Vehicle':2, 'Pedestrian':1}
    ocl_count = 0
    count = 0
    sot_list = []
    selected_id = None
    odom_roll_list = []
    odom_yaw_list = []
    odom_pitch_list = []
    lidar_z_list = []

    # ================== CenterPoint & poinc cloud scriber node =====================================    
    odom_flag = True
    demo_ros = ros_demo(model, args)
    sub = rospy.Subscriber("/arion/mtt/core/points_flip", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    # sub = rospy.Subscriber("/ouster/points", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    # ==============================================================================================
    
    # =============== mtt selection node =========================================
    slc_track = rospy.Subscriber("/arion/mtt/core/follow_target_id", Int32, selection_callback)
    # tf = rospy.Subscriber("/tf", TFMessage, tf_callback_3)
    # ============================================================================
    
    
    # =============== subscribe tf topic =================================================================
    earth2map_subscriber = rospy.Subscriber("/arion/mtt/core/earth2map", Float64MultiArray, earth2map_callback)
    odom2base_subscriber = rospy.Subscriber("/arion/mtt/core/odom2base", Float64MultiArray, odom2base_callback)
    map2odom_subscriber = rospy.Subscriber("/arion/mtt/core/map2odom", Float64MultiArray, map2odom_callback)
    odom_roll_list_subscriber = rospy.Subscriber("/arion/mtt/core/odom_roll_list", Float64MultiArray, odom_roll_list_callback)
    odom_yaw_list_subscriber = rospy.Subscriber("/arion/mtt/core/odom_yaw_list", Float64MultiArray, odom_yaw_list_callback)
    odom_pitch_list_subscriber = rospy.Subscriber("/arion/mtt/core/odom_yaw_list", Float64MultiArray, odom_pitch_list_callback)
    # ====================================================================================================
    
    # ============for target publish =======================================================================
    target_info_pub = rospy.Publisher('/arion/mtt/core/follow_target_info', FollowTargetInfo, queue_size=10)
    candidate_info_pub = rospy.Publisher('/arion/mtt/core/target_candidate', TargetCandidateList, queue_size=10)
    dyn_candidate_info_pub = rospy.Publisher('/arion/mtt/core/dynamic_object', DynamicObjectList, queue_size=10)
    # =====================================================================================================
    
    
    # ============for image porcess node ====================================
    bridge = CvBridge()
    # global img_pub, nearir_img, range_img, signal_img, ref_img
    F_img = rospy.Subscriber("/arion/mtt/core/reflec_flipped_img", Image, ref_img_callback)
    R_img = rospy.Subscriber("/arion/mtt/core/range_flipped_img", Image, range_img_callback)
    S_img = rospy.Subscriber("/arion/mtt/core/signal_flipped_img", Image, signal_img_callback)
    N_img = rospy.Subscriber("/arion/mtt/core/nearir_flipped_img", Image, nearir_img_callback)
    img_pub = rospy.Publisher("/arion/mtt/core/vis_image_with_box", Image, queue_size=10)
    # =======================================================================
    
    
    print("set up subscriber!")
    rospy.loginfo("set up subscriber!")

    rospy.spin()
    