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



__AGENTS_NAMES = cov.AGENTS_NAMES
__agents = {}

__lock = thd.Lock()


for name in __AGENTS_NAMES:
    pose = cov.INITIAL_POSES[name]
    landmarks = list(cov.INITIAL_LANDMARKS_LISTS[name])
    __agents[name] = cov.Agent(*pose, landmarks=landmarks)

assert sum([len(__agents[name].get_landmarks()) for name in __AGENTS_NAMES]) == cov.NUM_LANDMARKS

__total_landmarks = cov.NUM_LANDMARKS


def __pose_callback(msg, name):
    global __agents, __lock
    __lock.acquire()
    __agents[name].set_pose(msg.x, msg.y, msg.theta)
    # rp.logwarn('Plotter pose callback!!!')
    __lock.release()


def __landmarks_callback(msg, name):
    global __agents, __lock, __total_landmarks
    __lock.acquire()
    __total_landmarks -= len(__agents[name].get_landmarks())
    __agents[name].set_landmarks(cov.landmarks_from_point_2d_array(msg))
    #rp.logwarn(__agents[name].get_landmarks())
    assert len(__agents[name].get_landmarks()) == len(msg.data)
    __total_landmarks += len(__agents[name].get_landmarks())
    # rp.logwarn('Plotter landmarks callback!!!')
    # rp.logwarn(__agents[name].get_landmark_array())
    __lock.release()


def __coverage_done_callback(msg):
    global __coverage_done_flag, __coverage_done_lock
    __lock.acquire()
    __coverage_done_flag = True
    rp.logwarn('Plotter coverage done callback!!!')
    __lock.release()




rp.init_node('coverage_plotter')

for name in __AGENTS_NAMES:
    rp.Subscriber('/' + name + '/coverage_pose_2d', gms.Pose2D, __pose_callback, callback_args=name)
    rp.Subscriber('/' + name + '/coverage_landmarks', qms.Point2DArray, __landmarks_callback, callback_args=name)
__coverage_done_flag = False
__coverage_done_sub = rp.Subscriber('/coverage_done', sms.Empty, __coverage_done_callback)

__total_landmarks_pub = rp.Publisher('/coverage_total_landmarks', sms.Int32, queue_size=2)

__RATE = rp.Rate(3e0)

def __work():
    global __lock
    global __total_landmarks
    global __agents
    __lock.acquire()
    plt.figure()
    for name in __AGENTS_NAMES:
        __agents[name].draw(cov.AGENTS_COLORS[name])
        __total_landmarks_pub.publish(__total_landmarks)
    plt.xlim((-cov.DOMAIN_SIZE, cov.DOMAIN_SIZE))
    plt.ylim((-cov.DOMAIN_SIZE, cov.DOMAIN_SIZE))
    __lock.release()
    plt.draw()
    rp.logwarn('Plotter work!!!')


__lock.acquire()
while not rp.is_shutdown() and not __coverage_done_flag:
    __lock.release()
    __work()
    __RATE.sleep()
    __lock.acquire()
__lock.release()


plt.show()
