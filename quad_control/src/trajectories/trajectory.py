"""This module implements the Trajectory abstract class,
that is used for defining a desired trajectory
for a rigid body in the 3D space"""



import numpy as np
from utilities import utility_functions as uts
from utilities import jsonable as js

import rospy

DEFAULT_OFFSET = np.array([
    rospy.get_param("trajectry_offset_x",0),
    rospy.get_param("trajectry_offset_y",0),
    rospy.get_param("trajectry_offset_z",0)]) 

class Trajectory(js.Jsonable):

    @classmethod
    def description(cls):
        #raise NotImplementedError()
        return "<b>Abstract Trajectory</b> with 3D offset in (m) and rotation as [roll,pitch,yaw] in (deg)"


    def __init__(self, offset=DEFAULT_OFFSET, rotation=np.zeros(3)):

        self.__offset = np.array(offset)
        self.__rotation = np.array(rotation)
        self.__rotation_matrix = uts.GetRotFromEulerAnglesDeg(self.__rotation)
        #TODO change the names in the utility_functions module
    
    
    def __str__(self):
        string = self.description()
        string += "\nOffset: " + str(self.__offset)
        string += "\nRotation: " + str(self.__rotation)
        return string
        
    
    def get_offset(self):
        return np.array(self.__offset)
        
        
    def get_rotation(self):
        return np.array(self.__rotation)
        
        
    def set_offset(self, offset):
        self.__offset = np.array(offset)
        
        
    def set_rotation(self):
        self.__rotation = numpy.array(rotation)
        self.__rotation_matrix = uts.GetRotFromEulerAnglesDeg(rotation)
        
        
    def desired_trajectory(self, time):
        raise NotImplementedError()
    
        
    def __add_offset_and_rotation(self, pos, vel, acc, jrk, snp):
        
        off = self.__offset
        rot = self.__rotation_matrix
        
        pos_out = rot.dot(pos) + off
        vel_out = rot.dot(vel)
        acc_out = rot.dot(acc)
        jrk_out = rot.dot(jrk)
        snp_out = rot.dot(snp)
        
        #return pos_out, vel_out, acc_out, jrk_out, snp_out
        return np.concatenate([pos_out, vel_out, acc_out, jrk_out, snp_out])
        
        
    def output(self, time):
        return self.__add_offset_and_rotation(*self.desired_trajectory(time))


# """Test"""
#string = Trajectory.to_string()
#print string
#tr = Trajectory.from_string(string)
#print tr
        
        

