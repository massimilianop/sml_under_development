#!/usr/bin/env python
"""This module implements a ROS node that
plots the data relative to the coverage task."""


import matplotlib.pyplot as plt
import geometry_msgs.msg as gms
import quad_control.msg as qms
import rospy as rp
import utilities.coverage_utilities as cov




def __pose_callback(msg, name):
    position, versor = cov.pos_ver_from_pose2D(msg)



for name in cov.PLANNERS_NAMES:
    rp.Subscriber('/' + name + '/pose', gms.Pose2D, __pose_callback, callback_args=name)



