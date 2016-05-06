"""This module implements a ROS node that
acts as a planner for a coverage task."""



import utilities.coverage_utilities as cov
import geometry_msgs.msg as gms
import quad_control.msg as qms
import quad_control.srv as qsv
import threading as thd
import rospy as rp
import random as rdm



__NAME = 'Axel'
__OTHERS_NAMES = ['Bo', 'Calle', 'David']
__TOLERANCE = 0.1

__trade_lock = thd.Lock()
__pose_lock = ths.Lock()

__landmarks = []
__pose = gms.Pose2D()
__possible_trade_partners = list(__OTHERS_NAMES)


def __trade_landmarks_handler(request):
    global __landmarks
    global __pose
    __trade_lock.acquire()
    __pose_lock.acquire()
    success, to_remove_from_1, to_add_to_1, to_remove_from_2, to_add_to_2 = reassign_landmarks(__pose, request.pose, __landmarks, request.landmarks)
    if success:
        __landmarks = [__landmarks[index] for index not in to_remove_from_1]
        __landmarks += to_add_to_1
        __possible_trade_partners = list(__OTHERS_NAMES)
    else:
        if request.name in __possible_trade_partners:
            __possible_trade_partners.remove(request.name)
    __pose_lock.release()
    __trade_lock.release()
    return success, to_remove_from_2, to_add_to_2
    
    
def __pose_callback(pose):
    __pose_lock.acquire()
    __pose = gms.Pose2D(pose)
    __pose_lock.release()
    
    
def __work():
    #TODO
    pass
    
    
    
rp.init_node('coverage_planner')
rp.Subscriber('pose_subscriber', gms.Pose2D, __pose_callback)
rp.Service('trade_service', qsv.TradeLandmarks, __trade_landmarks_handler)
proxies = [rp.ServiceProxy(name + '/trade_service', qsv.TradeLandmarks) for name in __OTHERS_NAMES]
rp.Publisher
