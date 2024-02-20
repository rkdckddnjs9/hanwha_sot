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
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
# from ultralytics import YOLO

import cv2
from cv_bridge import CvBridge
import math

# for SOT =========================================================================
import importlib
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
# pdb.set_trace()
# ====================================================================================

# for IoU calculation ================================================================
from scipy.spatial import ConvexHull
# ================================================================================


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

    parser.add_argument('--pt', type=str, default="/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/pt/livox_model_2.pt", help='checkpoint to start from')

    args = parser.parse_args()
    return args

def calculate_angles(num_rows, num_cols, h_fov, v_fov):

    h_step = (h_fov[1] - h_fov[0]) / num_cols
    v_step = (v_fov[1] - v_fov[0]) / num_rows

    h_angles = np.radians(np.linspace(h_fov[0], h_fov[1], num_cols))
    v_angles = np.radians(np.linspace(v_fov[0], v_fov[1], num_rows))

    return h_angles, v_angles

def bboxes_to_3d_coordinates(range_image, bboxes, h_angles, v_angles):

    coords = []

    for bbox in bboxes:
        xmin, ymin, xmax, ymax = bbox[:4]
        xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
        box_depth = np.mean(range_image[ymin:ymax, xmin:xmax])

        x_coords = []
        y_coords = []
        z_coords = []

        for i in range(ymin, ymax):
            for j in range(xmin, xmax):
                h_angle = h_angles[j]
                v_angle = v_angles[i]

                x = box_depth * np.cos(v_angle) * np.cos(h_angle)
                y = box_depth * np.cos(v_angle) * np.sin(h_angle)
                z = box_depth * np.sin(v_angle)

                x_coords.append(x)
                y_coords.append(y)
                z_coords.append(z)

        x_mean = np.mean(x_coords)
        y_mean = np.mean(y_coords)
        z_mean = np.mean(z_coords)

        coords.append((x_mean, y_mean, z_mean))

    return np.array(coords)

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
            odom_x = transform.transform.translation.x - init_x
            odom_y = transform.transform.translation.y - init_y
            
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            rotation_matrix = quaternion_to_rotation_matrix(quaternion)
            
def selection_callback(data):
    global selected_id
    selected_id = int(data.data)
    # pdb.set_trace()
    
def heading_to_quaternion(heading_angle): 
    half_angle = heading_angle / 2.0
    return (0, 0, math.sin(half_angle), math.cos(half_angle))

def quaternion_to_yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def signal_img_callback(data):
    global bridge, signal_img
    signal_img = bridge.imgmsg_to_cv2(data, "32FC1")
    # cv_img = ros_numpy.numpify(data)
    normalized_data = cv2.normalize(signal_img, None, 0, 255, cv2.NORM_MINMAX)
    uint8_data = normalized_data.astype(np.uint8)
    signal_img = cv2.flip(uint8_data, 0)
    # result_2d = model_2d(uint8_data)
    # pdb.set_trace()

def ref_img_callback(data):
    global model_2d, bridge, ref_img
    ref_img = bridge.imgmsg_to_cv2(data, "32FC1")
    # cv_img = ros_numpy.numpify(data)
    normalized_data = cv2.normalize(ref_img, None, 0, 255, cv2.NORM_MINMAX)
    uint8_data = normalized_data.astype(np.uint8)
    ref_img = cv2.flip(uint8_data, 0)

def depth_to_3d_coordinates(range_image, h_angles, v_angles):
    num_rows, num_cols = range_image.shape
    x_coords = np.zeros_like(range_image)
    y_coords = np.zeros_like(range_image)
    z_coords = np.zeros_like(range_image)

    for i in range(num_rows):
        for j in range(num_cols):
            depth = range_image[i, j]
            h_angle = h_angles[j]
            v_angle = v_angles[i]

            x_coords[i, j] = depth * np.cos(v_angle) * np.cos(h_angle)
            y_coords[i, j] = depth * np.cos(v_angle) * np.sin(h_angle)
            z_coords[i, j] = depth * np.sin(v_angle)

    return x_coords, y_coords, z_coords

def bbox_3d_coordinates(x_coords, y_coords, z_coords, bbox):
    xmin, ymin, xmax, ymax = bbox
    return x_coords[ymin:ymax, xmin:xmax], y_coords[ymin:ymax, xmin:xmax], z_coords[ymin:ymax, xmin:xmax]

def range_img_callback(data):
    global model_2d, bridge, range_img, range_pred, box_2d_to_3d, img_pub
    range_img = bridge.imgmsg_to_cv2(data, "mono8")
    # cv_img = ros_numpy.numpify(data)
    # normalized_data = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)
    uint8_data = range_img.astype(np.uint8)
    uint8_data = cv2.flip(uint8_data, 0)
    # print(uint8_data.shape)
    # uint8_data = np.swapaxes(cv2.resize(signal_img, (640, 640)), 1, 0)

    # uint8_data = cv2.resize(ref_img, (96, 640))
    # uint8_data = np.swapaxes(uint8_data, 0, 1)
    # pdb.set_trace()
    # uint8_data = ref_img[np.newaxis, ...]
    # img = np.concatenate((uint8_data[np.newaxis,...], uint8_data[np.newaxis,...], uint8_data[np.newaxis,...]), axis=0)
    
    
    # pdb.set_trace()
    
    if signal_img.mean() <= 20:
        box_2d_to_3d = np.array([])
        return box_2d_to_3d
    # result_2d_1 = model_2d(signal_img)
    result_2d_1 = model_2d(uint8_data)
    # print(signal_img.mean())
    # result_2d_1 = model_2d(img)
    print(result_2d_1.pred[0].shape)
    # result_2d_2 = model_2d(uint8_data)
    # result_2d = torch.cat((result_2d_1.pred[0], result_2d_2.pred[0]), dim=0)
    # print(result_2d_1.pred[0].shape, result_2d_2.pred[0])
    if result_2d_1.pred[0].shape[0] > 10:
        pdb.set_trace()
        print("=============fuck : {} ".format(signal_img.mean()))
        # pdb.set_trace()
        result_2d_1.pred[0] = []
    
    # range_pred = result_2d.pred[0]
    # range_pred = result_2d
    range_pred = result_2d_1.pred[0]
    for i, box in enumerate(range_pred):
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3]) 
        # x1, y1, x2, y2 = int((box[0]/640)*1024), int((box[1]/640)*128), int((box[2]/640)*1024), int((box[3]/640)*128) 
        # x1, y1, x2, y2 = int((box[0]/640)*1024), int((box[1]/96)*128), int((box[2]/640)*1024), int((box[3]/96)*128) 
        # range_pred[i][0] = x1
        # range_pred[i][1] = y1
        # range_pred[i][2] = x2
        # range_pred[i][3] = y2
        cv2.rectangle(signal_img, (x1, y1), (x2, y2), (0, 255, 0), 2) #128 1024
    img_pub.publish(bridge.cv2_to_imgmsg(signal_img, "mono8")) 
    # h_fov = (-180, 180)  # 수평 FOV 예시
    h_fov = (-90, 90)  # 수평 FOV 예시
    # h_fov = (270, 90)
    # h_fov = (0, 360)
    v_fov = (-45, 45)  # 수직 FOV 예시
    h_angles, v_angles = calculate_angles(range_img.shape[0], range_img.shape[1], h_fov, v_fov)
    box_2d_to_3d = bboxes_to_3d_coordinates(range_img, range_pred, h_angles, v_angles)
    # if range_pred.shape[0] != 0:
    #     pdb.set_trace()

def calculate_iou(box1, box2):
    overlap_x = max(0, min(box1[0] + box1[3], box2[0] + box2[3]) - max(box1[0], box2[0]))
    overlap_y = max(0, min(box1[1] + box1[4], box2[1] + box2[4]) - max(box1[1], box2[1]))
    overlap_z = max(0, min(box1[2] + box1[5], box2[2] + box2[5]) - max(box1[2], box2[2]))

    # 겹치는 부분의 부피
    overlap_volume = overlap_x * overlap_y * overlap_z

    # 각 박스의 부피
    volume1 = box1[3] * box1[4] * box1[5]
    volume2 = box2[3] * box2[4] * box2[5]

    # IoU 계산
    iou = overlap_volume / float(volume1 + volume2 - overlap_volume)
    return iou

def find_best_match(target_box, pred_boxes):
    best_iou = 0
    best_index = -1

    for i, pred_box in enumerate(pred_boxes):
        iou = calculate_iou(target_box, pred_box)
        if iou > best_iou:
            best_iou = iou
            best_index = i

    return best_index

# =========================== I o U Code =======================================================
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
        # pc = ros_numpy.numpify(msg)
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

        x_cond = (cloud_array['x'] >= 0) & (cloud_array['x'] <= 40)
        y_cond = (cloud_array['y'] >= -40) & (cloud_array['y'] <= 40)
        z_cond = (cloud_array['z'] >= -2) & (cloud_array['z'] <= 4)
        # x_cond = (cloud_array['x'] >= 0) & (cloud_array['x'] <= 40)
        # y_cond = (cloud_array['y'] >= -8) & (cloud_array['y'] <= 8)
        # z_cond = (cloud_array['z'] >= -2) & (cloud_array['z'] <= 4)
    
        filtered_array = cloud_array[x_cond & y_cond & z_cond]
        filtered_array['z'] *= -1
        filtered_array['z'] += -2.0
        
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
        global frame_id, tracker, odom_x, odom_y, rotation_matrix, box_2d_to_3d, selected_id, _pred_dicts_, tracklet_nums, results_BBs, PCs
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
        
        # local box to global box =========================================
        # pred_dicts_[0]['pred_boxes'][:, :3] = pred_dicts_[0]['pred_boxes'][:, :3]@rotation_matrix.T
        # pred_dicts_[0]['pred_boxes'][:, 0] + odom_x
        # pred_dicts_[0]['pred_boxes'][:, 0] + odom_y
        # ====================================================================
        
        # for MOT ===========================================================
        # pred_dicts_[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][:, [5, 4, 3, 0, 1, 2, 6]] #shape : [6, 7] #x, y, z, l, w, h, rot -> h, w, l, x, y, z, rot
        # pdb.set_trace()
        # 탐지 결과를 추적기에 입력
        if pred_dicts_[0]['pred_boxes'].shape[0] > 0:
            # pdb.set_trace()
            # dets_all = {'dets': pred_dicts_[0]['pred_boxes'], 'info': np.concatenate((pred_dicts_[0]['pred_scores'][:, None], pred_dicts_[0]['pred_labels'][:, None]), axis=1)}
            dets_all = {'dets': pred_dicts[0]['pred_boxes'][:, [5, 4, 3, 0, 1, 2, 6]], 'info': np.concatenate((pred_dicts_[0]['pred_scores'][:, None], pred_dicts_[0]['pred_labels'][:, None]), axis=1)}
        else:
            dets_all = {'dets': np.empty([0, 7]), 'info': np.empty([0, 1])}
            
        #  score thresholding ====================================================
        mask_ped = ((dets_all['info'][:, 0] >0.4) * (dets_all['info'][:, 1] ==2.)) # ped
        mask_car = ((dets_all['info'][:, 0] >0.5) * (dets_all['info'][:, 1] ==1.)) # car
        mask = mask_ped + mask_car
        dets_all['dets'] = dets_all['dets'][mask]
        dets_all['info'] = dets_all['info'][mask]
        # ==========================================================================
        
        # distance based filtering ==================================================
        dist =  self.get_distance(dets_all['dets'])
        mask_ped = ((dist <= 30) * (dets_all['info'][:, 1] ==2.)) # ped
        mask_car = ((dist <= 40) * (dets_all['info'][:, 1] ==1.)) # car
        mask = mask_ped + mask_car
        dets_all['dets'] = dets_all['dets'][mask]
        dets_all['info'] = dets_all['info'][mask]
        
        # # distance based top k selection (k=5)
        # dist =  self.get_distance(dets_all['dets'])
        # dist_mask = np.full((dist.shape), False)
        # dist_mask[np.argsort(dist)[:5]] = True
        
        # dets_all['dets'] = dets_all['dets'][dist_mask]
        # dets_all['info'] = dets_all['info'][dist_mask]
        # ==================================================================================
        
        
        # 2d detection result based 3d filtering =======================================
        # if box_2d_to_3d.shape[0] != 0:
        #     dist = self.get_target_distance(box_2d_to_3d, dets_all['dets'])
        #     dets_all['dets'] = dets_all['dets'][dist]
        #     dets_all['info'] = dets_all['info'][dist]
        #     # pdb.set_trace()
        # else:
        #     dets_all['dets'] = []
        #     dets_all['info'] = []
        # =============================================================================
        
        
        # pdb.set_trace()
        
        if selected_id == None:
            # MOT ========================================================================================
            results, affi = tracker.track(dets_all, frame_id, 'hanwha') # h,w,l,x,y,z,theta, ID, other info, confidence
            # pdb.set_trace()
            box_ = results[0][:, [3,4,5,2,1,0,6]]  #h,w,l,x,y,z,theta -> x,y,z,l,w,h,rot
            
            pred_dicts_[0]['pred_boxes'] = box_
            # pred_dicts_[0]['pred_scores'] = np.ones((box_.shape[0],), dtype=np.float32)
            pred_dicts_[0]['pred_scores'] = np.array(results[0][:, 8],  dtype=np.float32)
            
            # pred_dicts_[0]['pred_labels'] = np.ones((box_.shape[0],), dtype=np.int8)
            pred_dicts_[0]['pred_labels'] = np.array(results[0][:, 9], dtype=np.int8)
            pred_dicts_[0]['pred_id'] = np.array(results[0][:, 7], dtype=np.int8)
            _pred_dicts_ = pred_dicts_
            # ============================================================================================
        else:
            tracker = None
            # SOT ==========================================================================================
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
                
                # if len(results_BBs) <= 3:   # FIFO structure
                #     results_BBs.append(slc_inst)
                # else:
                #     results_BBs.append(slc_inst)
                #     results_BBs.pop(0)      
                
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
                # tracker = None
                cur_PC = PointCloud(data_infer['points'][:, 1:4].transpose(1, 0)) #.cuda()
                pre_BB = results_BBs[-1]
                
                # pdb.set_trace()
                # mask_dist = self.get_target_distance(np.concatenate([pre_BB.center[np.newaxis, ...], pre_BB.wlh[np.newaxis, ...][:, [1,0,2]], np.array(quaternion_to_yaw(pre_BB.orientation.q)).reshape(1, 1)], axis=1), pred_dicts_[0]['pred_boxes'])
                pre_box = np.concatenate([pre_BB.center[np.newaxis, ...], pre_BB.wlh[np.newaxis, ...][:, [1,0,2]], np.array(quaternion_to_yaw(pre_BB.orientation.q)).reshape(1, 1)], axis=1)
                corners_3d_target  = get_3d_box((pre_box[0][3], pre_box[0][4], pre_box[0][5]), pre_box[0][-1], (pre_box[0][0], pre_box[0][1], pre_box[0][2]))  # (lwh), rot, (xyz)
                
                iou_list = []
                
                # # pdb.set_trace()
                # cls_mask =  (pred_dicts_[0]['pred_labels'] ==  _pred_dicts_[0]['pred_labels'].item())
                # pred_dicts_[0]['pred_boxes'] = pred_dicts_[0]['pred_boxes'][cls_mask]
                # pred_dicts_[0]['pred_scores'] = pred_dicts_[0]['pred_scores'][cls_mask]
                # pred_dicts_[0]['pred_labels'] = pred_dicts_[0]['pred_labels'][cls_mask]
                
                for i in range(pred_dicts_[0]['pred_boxes'].shape[0]):
                    corners_3d_pred = get_3d_box((pred_dicts_[0]['pred_boxes'][i][3], pred_dicts_[0]['pred_boxes'][i][4], pred_dicts_[0]['pred_boxes'][i][5]), pred_dicts_[0]['pred_boxes'][i][-1], (pred_dicts_[0]['pred_boxes'][i][0], pred_dicts_[0]['pred_boxes'][i][1], pred_dicts_[0]['pred_boxes'][i][2]))  # (lwh), rot, (xyz)
                    (IOU_3d,IOU_2d)=box3d_iou(corners_3d_pred, corners_3d_target)
                    iou_list.append(IOU_3d)
                iou_list = np.array(iou_list)
                if iou_list.max() >= 0.35:
                    best_match_index = np.where(iou_list==iou_list.max())[0][0]
                    box3d = pred_dicts_[0]['pred_boxes'][best_match_index].reshape(-1)
                    score = pred_dicts_[0]['pred_scores'][best_match_index].reshape(-1)
                    label = pred_dicts_[0]['pred_labels'][best_match_index].reshape(-1)
                    slc_inst = BoundingBox(
                                        center=box3d[:3], #xyz
                                        size=box3d[3:6][[1,0,2]], #lwh->wlh
                                        orientation=Quaternion(heading_to_quaternion(box3d[-1])) , #yaw -> quaternion
                                        label=label,
                                        score=score,
                                        )
                    # results_BBs.append(slc_inst)
                else:
                    best_match_index = None
                    slc_inst = pre_BB
                
                # step 3. Get the point cloud
                # target_PC = kitti_utils.cropAndCenterPC(cur_PC, pre_BB, offset=opts['offset_BB'], scale=1.25, limit_area=np.array(opts['area_extents']).reshape(3, 2)) 
                
                # model_PC = kitti_utils.getModel([PCs[0], PCs[tracklet_nums-1]], [results_BBs[0], results_BBs[tracklet_nums-1]], scale=1.25)
                # model_PC = kitti_utils.getModel([PCs[-1], cur_PC], [results_BBs[0], results_BBs[tracklet_nums-1]], scale=1.25)
                # model_PC = kitti_utils.getModel([PCs, cur_PC], [results_BBs[0], slc_inst], scale=1.25)
                
                target_PC = kitti_utils.cropAndCenterPC(cur_PC, pre_BB, offset=opts['offset_BB'], scale=1.25, limit_area=np.array(opts['area_extents']).reshape(3, 2)) 
                model_PC = kitti_utils.getModel([PCs[-1], cur_PC], [results_BBs[0], slc_inst], scale=1.25)
                
                # PCs = cur_PC
                PCs.append(cur_PC)
                # step 3.1 translate to numpy
                target_PC = target_PC.points
                model_PC = np.array(model_PC.points, dtype=np.float32)
                # step 3.2 subsample
                target_PC = kitti_utils.subsamplePC(target_PC, opts['subsample_number'])    # (M  , 3), tensor
                model_PC = kitti_utils.subsamplePC(model_PC, opts['subsample_number']//2)   # (M/2, 3), tensor
                    
                # step 4. Regression
                pred_hm, pred_loc, pred_z_axis = sot_model( model_PC.unsqueeze(0).cuda(), 
                                                        target_PC.unsqueeze(0).cuda())  
                
                # step 5. Get current bounding box
                with torch.no_grad():
                    hm = pred_hm.sigmoid_()
                    xy_img_z_ry = mot_decode(hm,pred_loc,pred_z_axis,K=1)

                xy_img_z_ry_cpu = xy_img_z_ry.squeeze(0).detach().cpu().numpy()
                xy_img_z_ry_cpu[:,:2] = (xy_img_z_ry_cpu[:,:2]+opts['min_img_coord'].numpy())*opts['xy_size'].numpy()
                estimate_box = xy_img_z_ry_cpu[0] #offset value
                # print(estimate_box[:3])
                # estimate_box[:3]*=0.5
                
                box = kitti_utils.getOffsetBBtest(pre_BB, estimate_box[:4], category=opts['db']['category_name'])
                # box = kitti_utils.getOffsetBBtest(slc_inst, estimate_box[:4], category=opts['db']['category_name'])
                
                results_BBs.append(box)
                
                # pdb.set_trace()
                if best_match_index == None:
                    pred_dicts_[0]['pred_boxes'] = np.concatenate([box.center[np.newaxis, ...], box.wlh[np.newaxis, ...][:, [1,0,2]], np.array(quaternion_to_yaw(box.orientation.q)).reshape(1, 1)], axis=1)
                    pred_dicts_[0]['pred_scores'] = _pred_dicts_[0]['pred_scores']
                    pred_dicts_[0]['pred_labels'] = _pred_dicts_[0]['pred_labels']
                    pred_dicts_[0]['pred_id'] = _pred_dicts_[0]['pred_id']
                else:
                    # pdb.set_trace()
                    pred_dicts_[0]['pred_boxes'] = pred_dicts[0]['pred_boxes'][best_match_index].reshape(1, -1)
                    pred_dicts_[0]['pred_scores'] = pred_dicts[0]['pred_scores'][best_match_index].reshape(-1)
                    pred_dicts_[0]['pred_labels'] = pred_dicts[0]['pred_labels'][best_match_index].reshape(-1)
                    pred_dicts_[0]['pred_id'] = _pred_dicts_[0]['pred_id']
                pass
        
        # global box to local box for visualization =====================================================
        # pred_dicts_[0]['pred_boxes'][:, :3] = pred_dicts_[0]['pred_boxes'][:, :3]@np.linalg.inv(rotation_matrix).T
        # pred_dicts_[0]['pred_boxes'][:, 0] - odom_x
        # pred_dicts_[0]['pred_boxes'][:, 0] - odom_y
        # ===========================================================================================
       
        global last_box_num
        # pdb.set_trace() 
        # last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts, last_box_num=last_box_num)
        last_box_num, _ = ros_vis.ros_print(data_dict['points_rviz'][:, 0:4], pred_dicts=pred_dicts_, last_box_num=last_box_num, class_id=pred_dicts_[0]['pred_id'])
        
        self.ender.record()
        torch.cuda.synchronize()
        curr_latency = self.starter.elapsed_time(self.ender)
        # print('det_time(ms): ', curr_latency)

if __name__ == '__main__':
    global count, depth, model_2d, bridge, box_2d_to_3d, img_pub, selected_id, tracklet_nums, results_BBs, PCs
    count = 0
    tracklet_nums = 0
    results_BBs = []
    PCs = []
    # MOT tracking ========================================
    global frame_id, cfg, tracker
    frame_id = 0
    # load config files
    # config_path = './hanwha.yml'
    config_path = "/home/changwon/data_2/hanwha_projects/hanwha_sot/livox_detection/tools/hanwha.yml"
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
    # pdb.set_trace()
    checkpoint = torch.load(args.pt, map_location=torch.device('cpu'))  
    model.load_state_dict({k.replace('module.', ''):v for k, v in checkpoint['model_state_dict'].items()})
    model.cuda()
    
    
    # SOT tracking ==================================================
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
    # pdb.set_trace()
    
    # ===============================================================
    
    
    # bridge = CvBridge()
    # model_2d = torch.hub.load("ultralytics/yolov5", "custom", "./yolov5x.pt")
    # model_2d = torch.hub.load("ultralytics/yolov5", "yolov5s")
    # model_2d.amp=True
    # model_2d = torch.hub.load("ultralytics/yolov5", "custom", path="./yolov5l.engine")

    demo_ros = ros_demo(model, args)
    #sub = rospy.Subscriber(
    #    "/livox/lidar", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    sub = rospy.Subscriber("/ouster/points", PointCloud2, queue_size=10, callback=demo_ros.online_inference)
    tf = rospy.Subscriber("/tf", TFMessage, tf_callback)
    # img = rospy.Subscriber("/ouster/reflec_image", Image, ref_img_callback)
    # img = rospy.Subscriber("/ouster/range_image", Image, range_img_callback)
    # img = rospy.Subscriber("/ouster/signal_image", Image, signal_img_callback)
    # img_pub = rospy.Publisher("/ouster/range_image_with_box", Image, queue_size=10)\
    
    slc_track = rospy.Subscriber("/tracking_id", String, selection_callback)
    print("set up subscriber!")

    rospy.spin()
    
