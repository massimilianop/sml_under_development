#!/usr/bin/env python
"""This module implements a ROS node that
acts as a planner for a coverage task."""



import utilities.coverage_utilities as cov
import geometry_msgs.msg as gms
import std_msgs.msg as sms
import quad_control.msg as qms
import quad_control.srv as qsv
import threading as thd
import numpy as np
import rospy as rp
import random as rdm



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





__NAME = rp.get_param('name', 'Axel')
__OTHERS_NAMES = rp.get_param('others_names', 'Bo Calle David').split()
__TOLERANCE = 0.1

__trade_lock = thd.Lock()
__pose_lock = thd.Lock()

__landmarks = cov.INITIAL_LANDMARKS_LISTS[__NAME]
__position = np.zeros(2)
__versor = cov.versor_from_angle(0.0)
__possible_trade_partners = list(__OTHERS_NAMES)









def __trade_landmarks_handler(request):
    global __pose_lock, __trade_lock
    global __position, __versor, __landmarks
    __trade_lock.acquire()
    __pose_lock.acquire()
    p1 = __position
    v1 = __versor
    p2 = np.array([request.pose.x, request.pose.y])
    v2 = cov.versor_from_angle(request.pose.theta)
    lst1 = __landmarks
    lst2 = [cov.pos_from_landmark(lmk) for lmk in request.landmarks]
    result = reassign_landmarks(__pose, request.pose, __landmarks, request.landmarks)
    success, remove_from_1, add_to_1, remove_from_2, add_to_2 = result
    if success:
        __landmarks = [__landmarks[index] for index in range(len(__landmarks))-remove_from_1]
        __landmarks += add_to_1
        __possible_trade_partners = list(__OTHERS_NAMES)
    else:
        if request.name in __possible_trade_partners:
            __possible_trade_partners.remove(request.name)
    __pose_lock.release()
    __trade_lock.release()
    return success, remove_from_2, add_to_2
    
    
def __pose_callback(pose):
    global __pose_lock
    global __position, __versor
    __pose_lock.acquire()
    __position = np.array([pose.x, pose.y])
    __versor = cov.versor_from_angle(pose.theta)
    __pose_lock.release()
    
    
    
rp.init_node('coverage_planner')
__RATE = rp.Rate(1e2)
rp.Subscriber('pose_2d', gms.Pose2D, __pose_callback)
rp.Service('trade_service', qsv.TradeLandmarks, __trade_landmarks_handler)
proxies = [rp.ServiceProxy(name + '/trade_service', qsv.TradeLandmarks)
    for name in __OTHERS_NAMES]
__cmd_vel_pub = rp.Publisher('cmd_vel', gms.Pose2D, queue_size=10)
__cov_pub = rp.Publisher('coverage', sms.Float64, queue_size=10)


def __work():
    global __pose_lock, __trade_lock
    global __position, __versor, __landmarks
    global __pub
    __pose_lock.acquire()
    __trade_lock.acquire()
    dot_p = cov.coverage_gradient_pos(__position, __versor, __landmarks)
    dot_theta = cov.coverage_gradient_ver(__position, __versor, __landmarks)
    coverage = cov.coverage_function(__position, __versor, __landmarks)
    __trade_lock.release()
    __pose_lock.release()
    #if np.linalg.norm(dot_p) > 0.3:
    #    dot_p *= 0.3/np.linalg.norm(dot_p)
    #dot_theta = np.clip(dot_theta, -0.1, 0.1)
    cmd_vel = gms.Pose2D(dot_p[0], dot_p[1], dot_theta)
    #msg = gms.Pose2D()
    __cmd_vel_pub.publish(cmd_vel)
    __cov_pub.publish(coverage)


while not rp.is_shutdown():
    __work()
    __RATE.sleep()
