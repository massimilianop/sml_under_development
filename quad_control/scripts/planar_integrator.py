#!/usr/bin/env python
"""This module implements a ROS node
that simulates a planar integrator.
"""



import numpy as np
import rospy as rp
import geometry_msgs.msg as gms
import std_msgs.msg as sms
import threading as thd
import utilities.utility_functions as uts
import tf.transformations as tfm
import utilities.coverage_utilities as cov



def quaternion_from_yaw(yaw):
    return tfm.quaternion_from_euler(0.0, 0.0, yaw)


__NAME = rp.get_param('name', 'Axel')
__pos = cov.INITIAL_POSES[__NAME][0:2]
__ang = cov.INITIAL_POSES[__NAME][2]
__cmd_vel_pos = np.zeros(2)
__cmd_vel_ang = 0.0
__done_flag = False
__lock = thd.Lock()


def __cmd_vel_callback(cmd_vel_msg):
    global __lock
    global __cmd_vel_pos, __cmd_vel_ang
    __lock.acquire()
    __cmd_vel_pos = np.array([cmd_vel_msg.x, cmd_vel_msg.y])
    __cmd_vel_ang = cmd_vel_msg.theta
    __lock.release()
    
    
def __done_callback(empty):
    global __done_flag
    __done_flag = True
    
    
    
rp.init_node('planar_integrator')
__RATE = rp.Rate(1e2)
rp.Subscriber('coverage_cmd_vel', gms.Pose2D, __cmd_vel_callback)
rp.Subscriber('coverage_done', sms.Empty, __done_callback)
__pose_2d_pub = rp.Publisher('coverage_pose_2d', gms.Pose2D, queue_size=1)
__pose_stamped_pub = rp.Publisher('coverage_pose_stamped', gms.PoseStamped, queue_size=2)



def __work():
    global __lock
    global __cmd_vel_pos, __cmd_vel_ang
    global __pos, __ang
    global __pose_2d_pub, __pose_stamped_pub
    __lock.acquire()
    __pos += __cmd_vel_pos*1e-2
    __ang += __cmd_vel_ang*1e-2
    #__ang = np.arctan2(np.sin(__ang), np.cos(__ang))
    pose_2d = gms.Pose2D(__pos[0], __pos[1], __ang)
    pose_stamped = gms.PoseStamped(pose=gms.Pose(position=gms.Point(x=__pos[0], y=__pos[1]), orientation = gms.Quaternion(*quaternion_from_yaw( __ang))))
    __lock.release()
    __pose_2d_pub.publish(pose_2d)
    __pose_stamped_pub.publish(pose_stamped)
    

__lock.acquire()
while not rp.is_shutdown() and not __done_flag:
    __lock.release()
    __work()
    __RATE.sleep()
    __lock.acquire()
__lock.release()
