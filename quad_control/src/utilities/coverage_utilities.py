"""This module implements some utilities for a
gradient-based coverage algorithm.
"""

import numpy as np
import random as rdm
import tf.transformations as tfm
import geometry_msgs.msg as gms
import quad_control.msg as qms


TOLERANCE = 0.01


def pose2D_from_pos_ver(pos, ver):
    angle = angle_from_versor(ver)
    return gms.Pose2D(pos[0], pos[1], angle)
    
def pos_ver_from_pose2D(pose):
    pos = np.array([pose.x, pose.y])
    ver = versor_from_angle(pose.theta)
    return pos, ver
    
    
def landmark_from_pos(pos):
    return qms.Landmark(pos[0], pos[1])
    
def pos_from_landmark(lmk):
    return np.array([lmk.x, lmk.y])
    


def versor_gradient(ver):
    return np.array([-ver[1], ver[0]])
    

def distance_factor(distance):
    return 1.0/(1.0+distance**2)
     
def distance_factor_derivative_over_distance(distance):
    return -2.0/(1.0+distance**2)**2
    
    
def versor_from_angle(theta):
    return np.array([np.cos(theta), np.sin(theta)])
    
def angle_from_versor(ver):
    return np.arctan2(ver[1], ver[0])
    
    
def visibility_function(pos, ver, q):
    dis = np.linalg.norm(pos-q)
    return -distance_factor(dis)*(pos-q).dot(ver)
    
def visibility_gradient_pos(pos, ver, q):
    dis = np.linalg.norm(pos-q)
    mat = -distance_factor_derivative_over_distance(dis)*np.outer(pos-q,pos-q) - distance_factor(dis)*np.eye(2)
    return mat.dot(ver)
   
def visibility_gradient_ver(pos, ver, q):
    dis = np.linalg.norm(q-pos)
    return -distance_factor(dis)*versor_gradient(ver).dot(pos-q)
    
    
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
    grad = 0.0
    for q in qs:
        grad += visibility_gradient_ver(pos, ver, q)
    return grad
    
    
def reassign_landmarks(p1, v1, p2, v2, lst1, lst2):
    remove_from_1 = []
    add_to_2 = []
    remove_from_2 = []
    add_to_1 = []
    success = False
    for i, q in enumerate(lst1):
        if visibility_function(p2, v2, q) > visibility_function(p1, v1, q) + TOLERANCE:
            remove_from_1.append(i)
            add_to_2.append(q)
            success = True
    for i, q in enumerate(lst2):
        if visibility_function(p1, v1, q) > visibility_function(p2, v2, q) + TOLERANCE:
            remove_from_2.append(i)
            add_to_1.append(q)
            success = True
    return success, remove_from_1, add_to_1, remove_from_2, add_to_2
    
    
    
NUM_LANDMARKS = 400
SIZE = 5.0
LANDMARKS = []
for index in range(NUM_LANDMARKS):
    x = float(index//20)/20.0*2.0*SIZE-SIZE+SIZE/20.0
    y = float(np.mod(index, 20))/20.0*2.0*SIZE-SIZE+SIZE/20.0
    LANDMARKS.append(np.array([x, y]))
    
    
PLANNERS_NAMES = ['Axel', 'Bo', 'Calle', 'David']
INITIAL_LANDMARKS_LISTS = {'Axel':[], 'Bo':[], 'Calle':[], 'David':[]}
for lmk in LANDMARKS:
    agent = rdm.choice(INITIAL_LANDMARKS_LISTS.keys())
    INITIAL_LANDMARKS_LISTS[agent].append(lmk)

    
    
#"""Testing"""
#pose = gms.Pose2D(1.0, 1.0, 0.3)
#print pose
#pos, ver = pos_ver_from_pose2D(pose)
#print pos, ver
#pose = pose2D_from_pos_ver(pos, ver)
#print pose
#lmk = qms.Landmark(1.0, 1.0)
#print lmk
#q = pos_from_landmark(lmk)
#print q
