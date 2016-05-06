"""This module implements some utilities for a
gradient-based coverage algorithm.
"""

import numpy as np
import geometry_msgs.msg as gms
import quad_control.msg as qms



TOLERANCE = 0.01



def skew(ver):
    return np.array([[ver[0], -ver[1]], [ver[1], ver[0]]])
    

def distance_factor(distance):
    return 1.0/1.0+distance**2
     
def distance_factor_derivative_over_distance(distance):
    return -1.0/(1.0+distance**2)**2
    
    
def versor_from_angle(theta):
    return np.array([np.cos(theta), np.sin(theta)])
    
def angle_from_versor(ver):
    return np.arctan2(ver[1], ver[0])
    
    
def pose2D_from_pos_ver(pos, ver):
    angle = angle_from_versor(ver)
    return gms.Pose2D(pos[0], pos[1], angle)
    
def pos_ver_from_pose2D(pose):
    pos = np.array([pose.x, pose.y])
    ver = versor_from_angle(pose.theta)
    return pos, ver
    
def landmark_from_pos_vis(pos, vis):
    return qms.Landmark(pos[0], pos[1], vis)
    
def pos_vis_from_landmark(lmk):
    pos = np.array([lmk.x, lmk.y])
    vis = lmk.visibility
    return pos, vis
    
    
def visibility_function(pos, ver, q):
    dis = np.linalg.norm(pos-q)
    return distance_factor(distance)*(q-pos).dot(ver)
    
def visibility_gradient_pos(pos, ver, q):
    dis = np.linalg.norm(pos-q)
    mat = distance_factor_derivative_over_distance(dis)*np.outer(q-pos,q-pos)\
        - visibility_function(dis)
    return mat.dot(ver)
   
def visibility_gradient_ver(pos, ver, q):
    dis = np.linalg.norm(pos-q)
    return dis*skew(ver)*(q-pos)
    
    
def coverage_function(pos, ver, qs):
    cov = 0.0
    for q in qs:
        cov += visibility_function(pos, ver, q)
    return cov
    
def coverage_gradient_pos(pos, ver, qs):
    grad = np.zeros(2)
    for q in qs:
        grad += visibility_gradient_pos(pos, ver, q)
    return grad
    
def coverage_gradient_ver(pos, ver, qs):
    grad = np.zeros(2)
    for q in qs:
        grad += visibility_gradient_ver(pos, ver, q)
    return grad
    
    
def reassign_landmarks(pose1, pose2, landmarks1, landmarks2):
    to_remove_from_1 = []
    to_add_to_2 = []
    to_remove_from_2 = []
    to_add_to_1 = []
    success = False
    pos1, ver1 = pos_ver_from_pose2D(pose1)
    pos2, ver2 = pos_ver_from_pose2D(pose2)
    for index, landmark in enumerate(landmarks1):
        q, vis = pos_vis_from_landmark(landmark)
        if visibility_function(pos2, ver2, q) > vis + TOLERANCE:
            to_remove_from_1.append(index)
            to_give_to_2.append(landmark)
            success = True
    for index, landmark in enumerate(landmarks2):
        q, vis = pos_vis_from_landmark(landmark)
        if visibility_function(pos1, ver1, q) > vis + TOLERANCE:
            to_remove_from_2.append(index)
            to_give_to_1.append(landmark)
            success = True
    return success, to_remove_from_1, to_add_to_1, to_remove_from_2, to_add_to_2
    
    
    
#"""Testing"""
#pose = gms.Pose2D(1.0, 1.0, 0.3)
#print pose
#pos, ver = pos_ver_from_pose2D(pose)
#print pos, ver
#pose = pose2D_from_pos_ver(pos, ver)
#print pose
#lmk = qms.Landmark(1.0, 1.0, 1.0)
#print lmk
#pos, vis = pos_vis_from_landmark(lmk)
#print pos, vis
