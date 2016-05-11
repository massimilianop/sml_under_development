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





__NAME = rp.get_param('name', 'Axel')
__OTHERS_NAMES = rp.get_param('others_names', 'Bo Calle David').split()
__TOLERANCE = 0.1

__INITIAL_POSE = cov.INITIAL_POSES[__NAME]
__INITIAL_LANDMARKS = cov.INITIAL_LANDMARKS_LISTS[__NAME]
__agent = cov.Agent(
    __INITIAL_POSE['x'],
    __INITIAL_POSE['y'],
    __INITIAL_POSE['theta'],
    __INITIAL_LANDMARKS)

__trade_lock = thd.Lock()
__pose_lock = thd.Lock()

# __landmarks = cov.INITIAL_LANDMARKS_LISTS[__NAME]
# __position = np.zeros(2)
# __versor = cov.versor_from_angle(0.0)

__possible_trade_partners = list(__OTHERS_NAMES)
__token = rp.get_param('token', False)






def __trade_landmarks_handler(request):
    global __pose_lock, __trade_lock
    global __agent
    global __NAME, __OTHERS_NAMES, __possible_trade_partners
    __pose_lock.acquire()
    __pose_lock.release()
    __trade_lock.acquire()
    landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in request.agent.landmarks]
    x = request.agent.pose.x
    y = request.agent.pose.y
    theta = request.agent.pose.theta
    success, you_remove, you_add = __agent.trade(x, y, theta, landmarks)
    if success:
        __possible_trade_partners = list(__OTHERS_NAMES)
    elif request.name in __possible_trade_partners:
        __possible_trade_partners.remove(request.name)
    __trade_lock.release()
    you_add = [lmk.to_pose2d() for lmk in you_add]
    rp.logwarn(request.name + ' requests a trade with ' + __NAME)
    rp.logwarn('Success of the trade: ' + str(success))
    return success, you_remove, you_add


def __receive_token_handler(request):
    global __trade_lock
    global __possible_trade_partners
    global __token
    __trade_lock.acquire()
    if len(__possible_trade_partners) > 0:
        __token = True
        result = True
        rp.logwarn(__NAME + ': token accepted')
    else:
        result = False
        rp.logwarn(__NAME + ': token refused')
    __trade_lock.release()
    return result
    
    
    
def __pose_callback(pose):
    global __pose_lock
    global __agent
    __pose_lock.acquire()
    __agent.set_pose(pose.x, pose.y, pose.theta)
    __pose_lock.release()
    
    
    
rp.init_node('coverage_planner')
__initial_time = rp.get_time()
__RATE = rp.Rate(1e2)
rp.Subscriber('pose_2d', gms.Pose2D, __pose_callback)
rp.Service('trade_landmarks', qsv.TradeLandmarks, __trade_landmarks_handler)
rp.Service('receive_token', qsv.ReceiveToken, __receive_token_handler)
__trade_proxies = {}
__token_proxies = {}
for name in __OTHERS_NAMES:
    rp.wait_for_service('/' + name + '/trade_landmarks')
    __trade_proxies[name] = rp.ServiceProxy('/' + name + '/trade_landmarks', qsv.TradeLandmarks)
    rp.wait_for_service('/' + name + '/receive_token')
    __token_proxies[name] = rp.ServiceProxy('/' + name + '/receive_token', qsv.ReceiveToken)
__cmd_vel_pub = rp.Publisher('cmd_vel', gms.Pose2D, queue_size=10)
__cov_pub = rp.Publisher('coverage', sms.Float64, queue_size=10)


def __work():
    global __pose_lock, __trade_lock
    global __agent
    global __NAME, __OTHERS_NAMES, __possible_trade_partners
    global __token
    global __cmd_vel_pub, __cov_pub
    global __initial_time
    __pose_lock.acquire()
    __trade_lock.acquire()
    dot_p = __agent.position_coverage_gradient()
    dot_theta = __agent.orientation_coverage_gradient()
    coverage = __agent.coverage()
    # pose = gms.Pose2D(__position[0], __position[1], cov.angle_from_versor(__versor))
    __trade_lock.release()
    __pose_lock.release()
    if np.linalg.norm(dot_p) > 10.3:
        dot_p *= 10.3/np.linalg.norm(dot_p)
    dot_theta = np.clip(dot_theta, -10.0, 10.0)
    cmd_vel = gms.Pose2D(dot_p[0], dot_p[1], dot_theta)
    __cmd_vel_pub.publish(cmd_vel)
    __cov_pub.publish(coverage)
    __trade_lock.acquire()
    if np.linalg.norm(dot_p) < __TOLERANCE and np.linalg.norm(dot_theta) < __TOLERANCE and __token:
        if len(__possible_trade_partners)>0:
            partner = rdm.choice(__possible_trade_partners)
            agent = __agent.to_coverage_agent()
            response = __trade_proxies[partner](__NAME, agent)
            if response.success:
                __possible_trade_partners = list(__OTHERS_NAMES)
                indexes_to_remove = response.to_remove
                landmarks_to_add = [cov.Landmark.from_pose2d(lmk) for lmk in response.to_add]
                __agent.update_landmarks(indexes_to_remove, landmarks_to_add)
            else:
                __possible_trade_partners.remove(partner)
        token_accepted = False
        possible_token_receivers = list(__OTHERS_NAMES)
        while not token_accepted and len(possible_token_receivers)>0:
            rp.logwarn(__NAME + ': possible token receivers: ' + str(possible_token_receivers))
            name = rdm.choice(possible_token_receivers)
            possible_token_receivers.remove(name)
            rsp = __token_proxies[name]()
            token_accepted = rsp.accepted
        if token_accepted:
            __token = False
        elif len(__possible_trade_partners) == 0:
            final_time = rp.get_time()
            rp.logwarn('Finish! Elapsed time: ' + str(final_time-__initial_time))
            __token = False
    __trade_lock.release()
    
    



while not rp.is_shutdown():
    __work()
    __RATE.sleep()
