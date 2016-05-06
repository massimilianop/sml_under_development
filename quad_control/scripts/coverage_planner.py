#!/usr/bin/env python
"""This module implements a planner for a coverage task."""



import utilities.coverage as cov
import geometry_msgs.msg as gms
import quad_control.srv as qsv
import rospy as rp
import scipy.optimize as opt
import numpy as np
import threading as thd


__NAME = 'David'
__OTHERS_NAMES = ['Axel', 'Bo', 'Calle']
__XBB = [-5.0, 5.0]
__YBB = [-5.0, 5.0]
__TBB = [-np.pi, np.pi]
__TOLERANCE = 0.1

__waypoint = gms.Pose2D()
__landmarks = []
__pose = gms.Pose2D()
__possible_partners = list(__OTHERS_NAMES)

__pose_lock = thd.Lock()
__trade_lock = thd.Lock()
__mode = 'navigating'


def __get_pose_callback(msg):
    global __pose
    __pose_lock.acquire()
    __pose = msg
    __pose_lock.release()

    
def __trade_landmarks_handler(request):
    global __waypoint, __landmarks, __possible_partners, __OTHERS_NAMES, __lock
    __trade_lock.acquire()
    for lmk in __landmarks:
        lmk = cov.Landmark(lmk)
    for lmk in other_landmarks:
        lmk = cov.Landmark(lmk)
    success, my_new, other_new = cov.reassign_landmarks(
        __waypoint, request.pose, __landmarks, request.landmarks)
    __landmarks = []
    for lmk in my_new:
        __landmarks.append(lmk.to_pose())
    for lmk in other_new:
        lmk = lmk.to_pose()
    if success:
        __update_waypoint()
        __possible_partners = list(__OTHERS_NAMES)
        __mode = 'navigating'
    else:
        __possible_partners.remove(request.name)
    __trade_lock.release()
    return success, __NAME, other_new
    




def __objective_function(wpt):
    global __landmarks
    vis = 0.0
    for lmk in __landmarks:
        vis += cov.Landmark(lmk).compute_visibility(wpt)
    return vis
    
    
def __update_waypoint():
    global __waypoint
    __waypoint = opt.differential_evolution(
        __objective_function,
        [__XBB, __YBB, __TBB])


def __distance(pose1, pose2):
    dp = pose1-pose2
    return np.sqrt(dp[0]**2+dp[1]**2+0.1*dp[2]**2)


def __work():
    #TODO incomplete: only accepts trading requests, does not make them
    global __pose, __waypoint, __TOLERANCE, __mode, __pose_lock
    if __mode == 'navigating':
        __pose_lock.acquire()
        if __distance(__pose, __waypoint) < __TOLERANCE:
            __mode = 'trading'
        __pose_lock.release()
        
        
        

rp.init_node('coverage_'+__NAME)

__pose_sub = rp.Subscriber('pose', gms.Pose2D, __get_pose_callback)
    
__trade_landmarks_service = rp.Service(
    'trade_landmarks_' + __NAME,
    qsv.TradeLandmarks,
    __trade_landmarks_handler)

__trade_landmarks_proxies = [
    rp.ServiceProxy(
        'trade_landmarks_' + __OTHERS_NAMES[index],
        qsv.TradeLandmarks,
        __trade_landmarks_handler)
    for index in range(len(__OTHERS_NAMES))]

__rate = rp.Rate(10)

    
while not rp.is_shutdown():
    __rate.sleep()      
    
    
    
    
    
    
    
    
    
    
    
    
    
