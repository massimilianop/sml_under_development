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





__NAME = rp.get_param('name')
__OTHERS_NAMES = rp.get_param('others_names').split()
__TOLERANCE = 0.1
__MAX_SPEED = 1.0
__GAIN = 0.1

__INITIAL_POSE = cov.INITIAL_POSES[__NAME]
__INITIAL_LANDMARKS = cov.INITIAL_LANDMARKS_LISTS[__NAME]
__agent = cov.Agent(*__INITIAL_POSE, landmarks=__INITIAL_LANDMARKS)

__lock = thd.Lock()

# __landmarks = cov.INITIAL_LANDMARKS_LISTS[__NAME]
# __position = np.zeros(2)
# __versor = cov.versor_from_angle(0.0)

__possible_trade_partners = list(__OTHERS_NAMES)
#__token = rp.get_param('token', False)
__token = rp.get_param('token')

__coverage_done_flag = False
__coverage_done_lock = thd.Lock()




def __trade_landmarks_handler(request):
    global __lock
    global __agent
    global __NAME, __OTHERS_NAMES, __possible_trade_partners
    __lock.acquire()
    others_landmarks = cov.landmarks_from_point_2d_array(request.landmarks)
    ox = request.pose.x
    oy = request.pose.y
    otheta = request.pose.theta
    old_landmarks = __agent.get_landmarks()
    success, indexes_you_remove, landmarks_you_add = __agent.trade(ox, oy, otheta, others_landmarks)
    if success:
        __possible_trade_partners = list(__OTHERS_NAMES)
        __landmarks_pub.publish(__agent.get_landmark_array())
    elif request.name in __possible_trade_partners:
        __possible_trade_partners.remove(request.name)
    __lock.release()
    rp.logwarn(request.name + ' requests a trade with ' + __NAME)
    #rp.logwarn(request)
    rp.logwarn('Success of the trade: ' + str(success))

    array_you_add = cov.point_2d_array_from_landmarks(landmarks_you_add)
    return qsv.TradeLandmarksResponse(success, indexes_you_remove, array_you_add)


def __receive_token_handler(request):
    global __lock
    global __possible_trade_partners
    global __token
    __lock.acquire()
    if len(__possible_trade_partners) > 0:
        __token = True
        result = True
        rp.logwarn(__NAME + ': token accepted')
    else:
        result = False
        rp.logwarn(__NAME + ': token refused')
    __lock.release()
    return result
    
    
def __coverage_done_callback(msg):
    global __coverage_done_flag, __coverage_done_lock
    __coverage_done_lock.acquire()
    __coverage_done_flag = True
    __coverage_done_lock.release()

    
def __pose_callback(pose):
    global __lock
    global __agent
    __lock.acquire()
    __agent.set_pose(pose.x, pose.y, pose.theta)
    __lock.release()
    
    
    
rp.init_node('coverage_planner')
__initial_time = rp.get_time()
__RATE = rp.Rate(1e2)
rp.Subscriber('coverage_pose_2d', gms.Pose2D, __pose_callback)
rp.Service('coverage_trade_landmarks', qsv.TradeLandmarks, __trade_landmarks_handler)
rp.Service('coverage_receive_token', qsv.ReceiveToken, __receive_token_handler)
__trade_proxies = {}
__token_proxies = {}
for name in __OTHERS_NAMES:
    rp.wait_for_service('/' + name + '/coverage_trade_landmarks')
    __trade_proxies[name] = rp.ServiceProxy('/' + name + '/coverage_trade_landmarks', qsv.TradeLandmarks)
    rp.wait_for_service('/' + name + '/coverage_receive_token')
    __token_proxies[name] = rp.ServiceProxy('/' + name + '/coverage_receive_token', qsv.ReceiveToken)
__cmd_vel_pub = rp.Publisher('coverage_cmd_vel', gms.Pose2D, queue_size=10)
__num_landmarks_pub = rp.Publisher('coverage_num_landmarks', sms.Int32, queue_size=10)
__cov_pub = rp.Publisher('coverage_coverage', sms.Float64, queue_size=10)
__landmarks_pub = rp.Publisher('coverage_landmarks', qms.Point2DArray, queue_size=10)
__coverage_done_pub = rp.Publisher('/coverage_done', sms.Empty, queue_size=10)
__coverage_done_sub = rp.Subscriber('/coverage_done', sms.Empty, __coverage_done_callback)



def __work():
    global __lock
    global __agent
    global __NAME, __OTHERS_NAMES, __possible_trade_partners
    global __token
    global __cmd_vel_pub, __cov_pub, __agent_pub
    global __initial_time
    __lock.acquire()
    dot_p = __GAIN*__agent.position_coverage_gradient()
    dot_theta = __GAIN*__agent.orientation_coverage_gradient()
    __cov_pub.publish(__agent.coverage())
    __landmarks_pub.publish(__agent.get_landmark_array())
    #if np.linalg.norm(dot_p) > __MAX_SPEED:
        #dot_p *= __MAX_SPEED/np.linalg.norm(dot_p)
    #dot_theta = np.clip(dot_theta, -__MAX_SPEED, __MAX_SPEED)
    cmd_vel = gms.Pose2D(dot_p[0], dot_p[1], dot_theta)
    __cmd_vel_pub.publish(cmd_vel)
    __num_landmarks_pub.publish(len(__agent.get_landmarks()))
    if np.linalg.norm(dot_p) < __TOLERANCE and np.linalg.norm(dot_theta) < __TOLERANCE and __token:
        if len(__possible_trade_partners)>0:
            partner = rdm.choice(__possible_trade_partners)
            pose = __agent.get_pose_2d()
            landmark_array = __agent.get_landmark_array()
            assert len(landmark_array.data) == len(__agent.get_landmarks())
            response = __trade_proxies[partner](__NAME, pose, landmark_array)
            if response.success:
                __possible_trade_partners = list(__OTHERS_NAMES)
                indexes_to_remove = response.indexes_to_remove
                landmarks_to_add = cov.landmarks_from_point_2d_array(response.landmarks_to_add)
                __agent.update_landmarks(indexes_to_remove, landmarks_to_add)
                #new_landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in response.landmarks.data]
                #__agent.set_landmarks(new_landmarks)
                #assert len(__agent.get_landmarks()) == len(new_landmarks)
                #array = __agent.get_landmark_array()
                #assert len(array.data) == len(new_landmarks)
                #__landmarks_pub.publish(array)
            else:
                __possible_trade_partners.remove(partner)
                # new_landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in response.landmarks.data]
                # __agent.set_landmarks(new_landmarks)
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
            __coverage_done_pub.publish(sms.Empty())
            __token = False
    __lock.release()
    
    


__lock.acquire()
while not rp.is_shutdown() and not __coverage_done_flag:
    __lock.release()
    __work()
    __RATE.sleep()
    __lock.acquire()
__lock.release()
