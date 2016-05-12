#!/usr/bin/env python
"""This module implements a ROS node that
plots the data relative to the coverage task."""


import matplotlib.pyplot as plt
import geometry_msgs.msg as gms
import quad_control.msg as qms
import std_msgs.msg as sms
import rospy as rp
import utilities.coverage_utilities as cov
import threading as thd
# from utilities import figtodat, images2gif



__AGENTS_NAMES = rp.get_param('agents_names', 'Axel Bo Calle David').split()
__agents = {}
__pose_locks = {}
__landmarks_locks = {}


for name in __AGENTS_NAMES:
    pose = cov.INITIAL_POSES[name]
    landmarks = list(cov.INITIAL_LANDMARKS_LISTS[name])
    color = cov.AGENTS_COLORS[name]
    __agents[name] = cov.Agent(*pose, landmarks=landmarks, color=color)
    __pose_locks[name] = thd.Lock()
    __landmarks_locks[name] = thd.Lock()

assert sum([len(__agents[name].get_landmarks()) for name in __AGENTS_NAMES]) == cov.NUM_LANDMARKS

__plotter_lock = thd.Lock()
__coverage_done_lock = thd.Lock()
__total_landmarks = cov.NUM_LANDMARKS


def __pose_callback(msg, name):
    global __agents, __pose_locks, __plotter_lock
    __pose_locks[name].acquire()
    __plotter_lock.acquire()
    __agents[name].set_pose(msg.x, msg.y, msg.theta)
    # rp.logwarn('Plotter pose callback!!!')
    __plotter_lock.release()
    __pose_locks[name].release()


def __landmarks_callback(msg, name):
    global __agents, __landmarks_locks, __plotter_lock, __total_landmarks
    __landmarks_locks[name].acquire()
    __plotter_lock.acquire()
    __total_landmarks -= len(__agents[name].get_landmarks())
    __agents[name].set_landmarks([cov.Landmark.from_pose2d(lmk) for lmk in msg.data])
    assert len(__agents[name].get_landmarks()) == len(msg.data)
    __total_landmarks += len(__agents[name].get_landmarks())
    # rp.logwarn('Plotter landmarks callback!!!')
    # rp.logwarn(__agents[name].get_landmark_array())
    __plotter_lock.release()
    __landmarks_locks[name].release()


def __num_landmarks_callback(msg, name):
    pass


def __coverage_done_callback(msg):
    global __coverage_done_flag, __coverage_done_lock
    __coverage_done_lock.acquire()
    __coverage_done_flag = True
    rp.logwarn('Plotter coverage done callback!!!')
    __coverage_done_lock.release()




rp.init_node('coverage_plotter')
for name in __AGENTS_NAMES:
    rp.Subscriber('/' + name + '/pose_2d', gms.Pose2D, __pose_callback, callback_args=name)
    rp.Subscriber('/' + name + '/landmarks', qms.LandmarkArray, __landmarks_callback, callback_args=name)
    rp.Subscriber('/' + name + '/num_landmarks', sms.Int32, __num_landmarks_callback, callback_args=name)

__coverage_done_flag = False
__coverage_done_sub = rp.Subscriber('/coverage_done', sms.Empty, __coverage_done_callback)

__total_landmarks_pub = rp.Publisher('/total_landmarks', sms.Int32, queue_size=10)

__RATE = rp.Rate(1)

def __work():
    global __agents, __plotter_lock, __total_landmarks
    plt.figure()
    __plotter_lock.acquire()
    for name in __AGENTS_NAMES:
        __agents[name].draw()
        __total_landmarks_pub.publish(__total_landmarks) 
    __plotter_lock.release()
    rp.logwarn('Plotter work!!!')


__coverage_done_lock.acquire()
while not rp.is_shutdown() and not __coverage_done_flag:
    __coverage_done_lock.release()
    __work()
    __RATE.sleep()
    __coverage_done_lock.acquire()
__coverage_done_lock.release()


plt.show()
