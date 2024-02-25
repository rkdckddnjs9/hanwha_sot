import importlib
import math
from pyquaternion import Quaternion
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
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PoseStamped, Point
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from sklearn.decomposition import PCA
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree

from sklearn.linear_model import RANSACRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures
from pub_tracker import PubTracker, NUSCENES_TRACKING_NAMES, greedy_assignment

def heading_to_quaternion(heading_angle):
    half_angle = heading_angle / 2.0
    return (0, 0, math.sin(half_angle), math.cos(half_angle))

def quaternion_to_yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

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
    # pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "os_updown"
    # pose_stamped.header.frame_id = "base_link"

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

def farthest_point_sampling_optimized(points, num_samples):
    N, D = points.shape
    farthest_pts = np.zeros(num_samples, dtype=np.int32)
    distances = np.ones(N) * np.inf
    first_index = np.random.randint(len(points))
    farthest_pts[0] = first_index
    current_distances = np.sum((points - points[first_index])**2, axis=1)
    for i in range(1, num_samples):
        farthest_pts[i] = np.argmax(current_distances)
        new_distances = np.sum((points - points[farthest_pts[i]])**2, axis=1)
        current_distances = np.minimum(current_distances, new_distances)
    return points[farthest_pts]

def pca_bounding_box(cluster):
    # Apply PCA to find the orientation of the cluster
    pca = PCA(n_components=3)
    pca.fit(cluster)
    center = pca.mean_
    size = np.max(cluster, axis=0) - np.min(cluster, axis=0)

    return center, size

def cluster_obstacles_2(cloud, eps=0.2, min_samples=10, max_points=250):
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(cloud)
    labels = clustering.labels_
    clusters = []
    for label in np.unique(labels):
        if label == -1:
            # -1 is for noise points
            continue
        cluster = cloud[labels == label]
        clusters.append(cluster)
    return clusters

def calculate_aabb(cluster):
    # Calculate the axis-aligned bounding box for a cluster
    min_point = np.min(cluster, axis=0)
    max_point = np.max(cluster, axis=0)
    
    center = (max_point + min_point) / 2
    size = max_point - min_point
    
    return center, size

def euclidean_clustering(points, tolerance=1.0, min_points=10):
    tree = cKDTree(points)
    clusters = []
    visited_points = np.zeros(points.shape[0], dtype=bool)

    for i in range(points.shape[0]):
        if not visited_points[i]:
            indices = tree.query_ball_point(points[i], r=tolerance)
            if len(indices) >= min_points:
                cluster = points[indices]
                clusters.append(cluster)
                visited_points[indices] = True

    return clusters

def calculate_bounding_boxes(clusters):
    bounding_boxes = []
    for cluster in clusters:
        min_bound = np.min(cluster, axis=0)
        max_bound = np.max(cluster, axis=0)
        center = (min_bound + max_bound) / 2
        size = max_bound - min_bound
        bounding_boxes.append(np.concatenate([center[:3], size, np.array([0])]))
    return np.vstack(bounding_boxes)

def nms_without_scores(boxes, iou_threshold):
    if len(boxes) == 0:
        return []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1) * (y2 - y1)
    order = areas.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0, xx2 - xx1)
        h = np.maximum(0, yy2 - yy1)
        intersection = w * h

        iou = intersection / (areas[i] + areas[order[1:]] - intersection)
        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]

    return keep

def remove_ground_points(points, n_degree=5, threshold=1):

    # Assume that the ground is primarily in the x-y plane and z is the height
    xy = points[:, :2]
    z = points[:, 2]

    # Fit RANSAC regressor to the data
    ransac = make_pipeline(PolynomialFeatures(n_degree), RANSACRegressor(residual_threshold=threshold))
    ransac.fit(xy, z)

    # Predict ground points and find inliers
    inlier_mask = ransac.named_steps['ransacregressor'].inlier_mask_
    
    # Select non-ground points
    non_ground_points = points[~inlier_mask]
    return non_ground_points

def mask_points_out_of_range(pc, pc_range):
    pc_range = np.array(pc_range)
    pc_range[3:6] -= 0.01  #np -> cuda .999999 = 1.0
    mask_x = (pc[:, 0] > pc_range[0]) & (pc[:, 0] < pc_range[3])
    mask_y = (pc[:, 1] > pc_range[1]) & (pc[:, 1] < pc_range[4])
    mask_z = (pc[:, 2] > pc_range[2]) & (pc[:, 2] < pc_range[5])
    mask = mask_x & mask_y & mask_z
    pc = pc[mask]
    return pc

def mask_points_out_of_range_2(pc, pc_range):
    pc_range = np.array(pc_range.copy())
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

def transform_pred(pred):
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

def transform_track(tracks):
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

def transform_track_np(tracks):
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
    annos['pred_boxes']=np.array(annos['pred_boxes'])
    annos['pred_scores']=np.array(annos['pred_scores'])
    annos['pred_labels']=np.array(annos['pred_labels'])
    annos['pred_id']=np.array(annos['pred_id'])
    return [annos]

def get_distance(box):
    # box shape : N, 7
    dist = np.sqrt(box[:,0]**2 + box[:,1]**2+ box[:,2]**2)
    return dist

def get_target_distance(target, box):
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

def get_target_distance_v2(target, box):
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


def update_dynamic_z_filter_base(sot_list, z_variation_threshold=0.1, min_z_filter=-0.3, max_z_filter=0.4):
    z_values = np.array([pos[0][2] for pos in sot_list])

    z_diff = np.diff(z_values)

    significant_changes = np.abs(z_diff) > z_variation_threshold

    if significant_changes.any():
        recent_change = z_diff[-1]
        if recent_change > 0:
            new_z_filter = min(max_z_filter, min_z_filter + recent_change)
        else:
            new_z_filter = max(min_z_filter, max_z_filter + recent_change)
    else:
        new_z_filter = min_z_filter

    return new_z_filter

def calculate_dynamic_cut_range(sot_list, base_cut_range=[0.36, 0.36, 0.4], velocity_scale_factor=0.5):
    positions = np.array([pos[0] for pos in sot_list])
    velocities = np.diff(positions, axis=0)

    if len(velocities) > 0:
        recent_velocity = np.mean(velocities[-3:], axis=0)
        velocity_magnitude = np.linalg.norm(recent_velocity)

        dynamic_range = np.array(base_cut_range) + velocity_scale_factor * velocity_magnitude
    else:
        dynamic_range = np.array(base_cut_range)

    current_pos = positions[-1]
    cut_range = np.concatenate([current_pos - dynamic_range, current_pos + dynamic_range])

    return cut_range

def calculate_point_cloud_bias(box, points):
    point_cloud_center = np.mean(points, axis=0)
    
    bias_direction = point_cloud_center - box
    
    bias_magnitude = np.linalg.norm(bias_direction)
    
    return bias_direction, bias_magnitude
