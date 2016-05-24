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









class CoveragePlanner:

    
    __THRESHOLD = 0.3
    __TOLERANCE = 0.1
    

    def __trade_landmarks_handler(self, msg):
        pass
        
        
    def __receive_token_handler(self):
        self.__lock.acquire()
        if len(self.__possible_trade_partners) > 0:
            self.__token = True
            result = True
            rp.logwarn(self.__NAME + ': token accepted')
        else:
            result = False
            rp.logwarn(self.__NAME + ': token refused')
        self.__lock.release()
        return result
        
        
    def __pose_callback(self, pose):
        self.__lock.acquire()
        self.__agent.set_pose(pose.x, pose.y, pose.theta)
        self.__lock.release()
        
        
    def __coverage_done_callback(self, msg):
        self.__lock.acquire()
        self.__done_flag = True
        self.__lock.release()
        
        
    def __init__(self,
            name,
            others_names,
            token,
            initial_pose,
            initial_landmarks,
            ):
        
        self.__NAME = name
        self.__OTHERS_NAMES = others_names
        
        
        self.__agent = cov.Agent(*initial_pose, landmarks=initial_landmarks)
        #rp.logwarn(len(initial_landmarks))
        #rp.logwarn(len(self.__agent.get_landmarks()))
        
        self.__lock = thd.Lock()
        self.__done_flag = False
        self.__token = token
        
        self.__possible_trade_partners = others_names
        
        rp.Subscriber('coverage_pose_2d', gms.Pose2D, self.__pose_callback)
        rp.Subscriber('/coverage_done', sms.Empty, self.__coverage_done_callback)
        rp.Service('coverage_trade_landmarks', qsv.TradeLandmarks, self.__trade_landmarks_handler)
        rp.Service('coverage_receive_token', qsv.ReceiveToken, self.__receive_token_handler)
        self.__cmd_vel_pub = rp.Publisher('coverage_cmd_vel', gms.Pose2D, queue_size=2)
        self.__landmarks_pub = rp.Publisher('coverage_landmarks', qms.Point2DArray, queue_size=2)
        self.__coverage_pub = rp.Publisher('coverage_coverage', sms.Float64, queue_size=2)
        self.__num_landmarks_pub = rp.Publisher('coverage_num_landmarks', sms.Int32, queue_size=2)
        self.__done_pub = rp.Publisher('/coverage_done', sms.Empty, queue_size=2)
        
        self.__trade_proxies = {}
        self.__token_proxies = {}
        
        for name in self.__OTHERS_NAMES:
            rp.wait_for_service('/' + name + '/coverage_trade_landmarks')
            self.__trade_proxies[name] = rp.ServiceProxy('/' + name + '/coverage_trade_landmarks', qsv.TradeLandmarks)
            rp.wait_for_service('/' + name + '/coverage_receive_token')            
            self.__token_proxies[name] = rp.ServiceProxy('/' + name + '/coverage_receive_token', qsv.ReceiveToken)
            
        rate = rp.Rate(1e2)
        self.__lock.acquire()
        while(not rp.is_shutdown() and not self.__done_flag):
            self.__lock.release()
            self.__work()
            rate.sleep()
            self.__lock.acquire()
        
        
    def __work(self):
        
        self.__lock.acquire()
        dot_p = self.__agent.position_coverage_gradient()
        dot_theta = self.__agent.orientation_coverage_gradient()
        coverage = self.__agent.coverage()
        if np.linalg.norm(dot_p) > self.__THRESHOLD:
            dot_p *= self.__THRESHOLD/np.linalg.norm(dot_p)
        dot_theta = np.clip(dot_theta, -self.__THRESHOLD, self.__THRESHOLD)
        cmd_vel = gms.Pose2D(dot_p[0], dot_p[1], dot_theta)
        self.__cmd_vel_pub.publish(cmd_vel)
        self.__coverage_pub.publish(coverage)
        #rp.logwarn(len(self.__agent.get_landmarks()))
        self.__num_landmarks_pub.publish(len(self.__agent.get_landmarks()))
        #rp.logwarn(self.__agent.get_landmark_array())
        self.__landmarks_pub.publish(self.__agent.get_landmark_array())
        if np.linalg.norm(dot_p) < self.__TOLERANCE and np.linalg.norm(dot_theta) < self.__TOLERANCE:
            self.__done_pub.publish(sms.Empty())
            if False:
            #if len(__possible_trade_partners)>0:
                partner = rdm.choice(__possible_trade_partners)
                pose = self.__agent.get_pose_2d()
                landmark_array = self.__agent.get_landmark_array()
                response = __trade_proxies[partner](self.__NAME, pose, landmark_array)
                if response.success:
                    self.__possible_trade_partners = list(self.__OTHERS_NAMES)
                    i2r = response.indexes_to_remove
                    l2a = cov.landmarks_from_point_2d_array(response.landmarks_to_add)
                    self.__agent.update_landmarks(i2r, l2a)
                    #__landmarks_pub.publish(cov.point_2d_array_from_landmarks(__agent.get_landmarks()))
                    # new_landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in response.landmarks.data]
                    #__agent.set_landmarks(new_landmarks)
                    # assert len(__agent.get_landmarks()) == len(new_landmarks)
                    # array = __agent.get_landmark_array()
                    # assert len(array.data) == len(new_landmarks)
                    #__landmarks_pub.publish(array)
                else:
                    self.__possible_trade_partners.remove(partner)
                    # new_landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in response.landmarks.data]
                    # __agent.set_landmarks(new_landmarks)
            token_accepted = False
            possible_token_receivers = list(self.__OTHERS_NAMES)
            while not token_accepted and len(possible_token_receivers)>0:
                rp.logwarn(self.__NAME + ': possible token receivers: ' + str(possible_token_receivers))
                name = rdm.choice(possible_token_receivers)
                possible_token_receivers.remove(name)
                rsp = self.__token_proxies[name]()
                token_accepted = rsp.accepted
            if token_accepted:
                self.__token = False
            elif len(__possible_trade_partners) == 0:
                final_time = rp.get_time()
                rp.logwarn('Finish! Elapsed time: ' + str(final_time-__INITIAL_TIME))
                self.__done_pub.publish(sms.Empty())
                self.__token = False
        self.__lock.release()
        
        

rp.init_node('coverage_planner')
        
name = rp.get_param('name')
others_names = rp.get_param('others_names').split()
token = rp.get_param('token')
pose = cov.INITIAL_POSES[name]
landmarks = cov.INITIAL_LANDMARKS_LISTS[name]

rp.logwarn(len(landmarks))
coverage_planner = CoveragePlanner(name, others_names, token, pose, landmarks)







#__NAME = rp.get_param('name', 'Axel')
#__OTHERS_NAMES = rp.get_param('others_names', 'Bo Calle David').split()
#__TOLERANCE = 0.1

#__INITIAL_POSE = cov.INITIAL_POSES[__NAME]
#__INITIAL_LANDMARKS = cov.INITIAL_LANDMARKS_LISTS[__NAME]
#__COLOR = cov.AGENTS_COLORS[__NAME]
#__agent = cov.Agent(*__INITIAL_POSE, landmarks=__INITIAL_LANDMARKS, color=__COLOR)
#assert __agent.get_landmarks() == __INITIAL_LANDMARKS

#__lock = thd.Lock()
#__token = rp.get_param('token', False)

## __landmarks = cov.INITIAL_LANDMARKS_LISTS[__NAME]
## __position = np.zeros(2)
## __versor = cov.versor_from_angle(0.0)

#__possible_trade_partners = list(__OTHERS_NAMES)
#__coverage_done_flag = False
#    







#def __trade_landmarks_handler(request):
#    global __lock
#    global __agent
#    global __NAME, __OTHERS_NAMES, __possible_trade_partners
#    __lock.acquire()
#    others_landmarks = cov.landmarks_from_point_2d_array(request.landmarks)
#    ox = request.pose.x
#    oy = request.pose.y
#    otheta = request.pose.theta
#    old_landmarks = __agent.get_landmarks()
#    success, i2r, l2a = __agent.trade(ox, oy, otheta, others_landmarks)
#    assert len(__agent.get_landmarks())-len(old_landmarks) == len(i2r)-len(l2a)
#    if success:
#        __possible_trade_partners = list(__OTHERS_NAMES)
#        #__landmarks_pub.publish(cov.point_2d_array_from_landmarks(__agent.get_landmarks()))
#        #__landmarks_pub.publish(__agent.get_landmark_array())
#    elif request.name in __possible_trade_partners:
#        __possible_trade_partners.remove(request.name)
#    __lock.release()
#    array = cov.point_2d_array_from_landmarks(l2a)
#    rp.logwarn(request.name + ' requests a trade with ' + self.__NAME)
#    #rp.logwarn(request)
#    rp.logwarn('Success of the trade: ' + str(success))
#    return success, i2r, array


#def __receive_token_handler(request):
#    global __lock
#    global __possible_trade_partners
#    global __token
#    __lock.acquire()
#    if len(__possible_trade_partners) > 0:
#        __token = True
#        result = True
#        rp.logwarn(__NAME + ': token accepted')
#    else:
#        result = False
#        rp.logwarn(__NAME + ': token refused')
#    __lock.release()
#    return result
#    
#    
#def __coverage_done_callback(msg):
#    global __coverage_done_flag, __lock
#    __lock.acquire()
#    __coverage_done_flag = True
#    __lock.release()

#    
#def __pose_callback(pose):
#    global __lock
#    global __agent
#    __lock.acquire()
#    __agent.set_pose(pose.x, pose.y, pose.theta)
#    __lock.release()
    
    
    
#rp.init_node('coverage_planner')
#__INITIAL_TIME = rp.get_time()
#__RATE = rp.Rate(1e2)

#rp.Subscriber('pose_2d', gms.Pose2D, __pose_callback)
#rp.Subscriber('/coverage_done', sms.Empty, __coverage_done_callback)
#rp.Service('trade_landmarks', qsv.TradeLandmarks, __trade_landmarks_handler, buff_size=10*65536)
#rp.Service('receive_token', qsv.ReceiveToken, __receive_token_handler, buff_size=10*65536)

#__trade_proxies = {}
#__token_proxies = {}
#for name in __OTHERS_NAMES:
#    rp.wait_for_service('/' + name + '/trade_landmarks')
#    __trade_proxies[name] = rp.ServiceProxy('/' + name + '/trade_landmarks', qsv.TradeLandmarks)
#    rp.wait_for_service('/' + name + '/receive_token')
#    __token_proxies[name] = rp.ServiceProxy('/' + name + '/receive_token', qsv.ReceiveToken)

#__cmd_vel_pub = rp.Publisher('cmd_vel', gms.Pose2D, queue_size=10)
#__num_landmarks_pub = rp.Publisher('num_landmarks', sms.Int32, queue_size=10)
#__cov_pub = rp.Publisher('coverage', sms.Float64, queue_size=10)
#__landmarks_pub = rp.Publisher('landmarks', qms.Point2DArray, queue_size=10)
#__done_pub = rp.Publisher('/coverage_done', sms.Empty, queue_size=10)



#def __work():
#    global __lock
#    global __agent
#    global __NAME, __OTHERS_NAMES, __possible_trade_partners, __trade_proxies
#    global __token
#    global __cmd_vel_pub, __cov_pub, __landmarks_pub, __num_landmarks_pub
#    global __INITIAL_TIME
#    __lock.acquire()
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    dot_p = __agent.position_coverage_gradient()
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    dot_theta = __agent.orientation_coverage_gradient()
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    coverage = __agent.coverage()
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    if np.linalg.norm(dot_p) > 10.0:
#        dot_p *= 10.0/np.linalg.norm(dot_p)
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    dot_theta = np.clip(dot_theta, -10.0, 10.0)
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    cmd_vel = gms.Pose2D(dot_p[0], dot_p[1], dot_theta)
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    __cmd_vel_pub.publish(cmd_vel)
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    __cov_pub.publish(coverage)
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    #__num_landmarks_pub.publish(len(__agent.get_landmarks()))
#    __landmarks_pub.publish(cov.point_2d_array_from_landmarks(__agent.get_landmarks()))
#    assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#    if np.linalg.norm(dot_p) < __TOLERANCE and np.linalg.norm(dot_theta) < __TOLERANCE and __token:
#        assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#        __done_pub.publish(sms.Empty())
#        assert __agent.get_landmarks() == __INITIAL_LANDMARKS
#        if len(__possible_trade_partners)>0 and False:
#            partner = rdm.choice(__possible_trade_partners)
#            pose = gms.Pose2D(*__agent.get_pose())
#            landmark_array = cov.point_2d_array_from_landmarks(__agent.get_landmarks())
#            assert len(landmark_array.data) == len(__agent.get_landmarks())
#            response = __trade_proxies[partner](__NAME, pose, landmark_array)
#            if response.success:
#                __possible_trade_partners = list(__OTHERS_NAMES)
#                i2r = response.indexes_to_remove
#                l2a = cov.landmarks_from_point_2d_array(response.landmarks_to_add)
#                __agent.update_landmarks(i2r, l2a)
#                #__landmarks_pub.publish(cov.point_2d_array_from_landmarks(__agent.get_landmarks()))
#                # new_landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in response.landmarks.data]
#                #__agent.set_landmarks(new_landmarks)
#                # assert len(__agent.get_landmarks()) == len(new_landmarks)
#                # array = __agent.get_landmark_array()
#                # assert len(array.data) == len(new_landmarks)
#                #__landmarks_pub.publish(array)
#            else:
#                __possible_trade_partners.remove(partner)
#                # new_landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in response.landmarks.data]
#                # __agent.set_landmarks(new_landmarks)
#        token_accepted = False
#        possible_token_receivers = list(__OTHERS_NAMES)
#        while not token_accepted and len(possible_token_receivers)>0:
#            rp.logwarn(__NAME + ': possible token receivers: ' + str(possible_token_receivers))
#            name = rdm.choice(possible_token_receivers)
#            possible_token_receivers.remove(name)
#            rsp = __token_proxies[name]()
#            token_accepted = rsp.accepted
#        if token_accepted:
#            __token = False
#        elif len(__possible_trade_partners) == 0:
#            final_time = rp.get_time()
#            rp.logwarn('Finish! Elapsed time: ' + str(final_time-__INITIAL_TIME))
#            __done_pub.publish(sms.Empty())
#            __token = False
#    __lock.release()
#    
#    


#__lock.acquire()
#while not rp.is_shutdown() and not __coverage_done_flag:
#    __lock.release()
#    __work()
#    __RATE.sleep()
#    __lock.acquire()
#rp.logwarn(__NAME + ': Coverage is done!')
#__lock.release()
