#!/usr/bin/env python
"""This module implements a ROS node
that simulates a planar integrator.
"""



import numpy as np
import rospy as rp
import geometry_msgs.msg as gms
import threading as thd
import utilities.utility_functions as uts
import tf.transformations as tfm
import utilities.coverage_utilities as cov



def quaternion_from_yaw(yaw):
    return tfm.quaternion_from_euler(0.0, 0.0, yaw)


__name = rp.get_param('name', 'Axel')
__pos = cov.INITIAL_POSES[__name][0:2]
__ang = cov.INITIAL_POSES[__name][2]
__cmd_vel_pos = np.zeros(2)
__cmd_vel_ang = 0.0
__cmd_vel_lock = thd.Lock()

def __cmd_vel_callback(msg):
    global __cmd_vel_lock
    global __cmd_vel_pos, __cmd_vel_ang
    __cmd_vel_lock.acquire()
    __cmd_vel_pos = np.array([msg.x, msg.y])
    __cmd_vel_ang = msg.theta
    __cmd_vel_lock.release()
    
    
rp.init_node('planar_integrator')
__RATE = rp.Rate(1e2)
rp.Subscriber('cmd_vel', gms.Pose2D, __cmd_vel_callback)
__pose_2d_pub = rp.Publisher('pose_2d', gms.Pose2D, queue_size=10)
__pose_stamped_pub = rp.Publisher('pose_stamped', gms.PoseStamped, queue_size=10)


def __work():
    global __cmd_vel_lock
    global __cmd_vel_pos, __cmd_vel_ang
    global __pos, __ang
    global __pub
    __cmd_vel_lock.acquire()
    __pos += __cmd_vel_pos*1e-2
    __ang += __cmd_vel_ang*1e-2
    __ang = np.arctan2(np.sin(__ang), np.cos(__ang))
    pose_2d = gms.Pose2D(__pos[0], __pos[1], __ang)
    pose_stamped = gms.PoseStamped(pose=gms.Pose(position=gms.Point(x=__pos[0], y=__pos[1]), orientation = gms.Quaternion(*quaternion_from_yaw( __ang))))
    __cmd_vel_lock.release()
    __pose_2d_pub.publish(pose_2d)
    __pose_stamped_pub.publish(pose_stamped)
    
    
while not rp.is_shutdown():
    __work()
    __RATE.sleep()
