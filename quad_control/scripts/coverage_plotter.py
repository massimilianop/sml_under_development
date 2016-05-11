#!/usr/bin/env python
"""This module implements a ROS node that
plots the data relative to the coverage task."""


import matplotlib.pyplot as plt
import geometry_msgs.msg as gms
import quad_control.msg as qms
import rospy as rp
import utilities.coverage_utilities as cov
import threading as thd



__AGENTS_NAMES = rp.param('agents_names', 'Axel Bo Calle David').split()
__agents = {}
for name in __AGENTS_NAMES:
	__agents[name] = {
		'position':np.zeros(2),
		'orientation': 0.0,
		'landmarks': []
		'lock': thd.Lock()
		}




def __pose_callback(msg, name):
	global __agents
    position, versor = cov.pos_ver_from_pose2D(msg.pose)
    __agents[name]['lock'].acquire()
    __agents[name]['position'] = position
    __agents[name]['orientation'] = orientation
    __agents[name]['lock'].release()


def __landmarks_callback(msg, name):
	global __agents
	new_landmarks = msg.data
	__agents[name]['lock'].acquire()
    __agents[name]['landmarks'] = [cov.pos_from_landmark(lmk) for lmk in new_landmarks]
    __agents[name]['lock'].release()



for name in __AGENTS_NAMES:
    rp.Subscriber('/' + name + '/pose', gms.Pose2D, __pose_callback, callback_args=name)
    rp.Subscriber('/' + name + '/landmarks', gms.LandmarksList, __landmarks_callback, callback_args=name)



def work():
	global __agents
