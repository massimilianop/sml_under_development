"""This module implements the Trajectory abstract class,
that is used for defining a desired trajectory
for a rigid body in the 3D space"""


import numpy as np
from utilities import utility_functions as uts
import json

class Trajectory:


    @classmethod
    def description(cls):
        raise NotImplementedError()
        #return "Abstract Trajectory"
    
    
    @classmethod
    def get_parameters_names(cls):
        raise NotImplementedError()
        #return ()


    @classmethod
    def offset_and_rotation_to_string(cls, offset=np.array([0.0, 0.0, 1.0]),
            rotation=np.zeros(3)):
        dic = {'offset':offset, 'rotation':rotation}
        return json.dumps(dic)
        
        
    @classmethod
    def string_to_offset_and_rotation(cls, string):
        dic = json.loads(string)
        offset = dic['offset']
        rotation = dic['rotation']
        return offset, rotation
        

    @classmethod
    def parameters_to_string(cls, parameters):
        raise NotImplementedError()
        
        
    @classmethod
    def string_to_parameters(cls, string):
        raise NotImplementedError()


    def __init__(self, offset=np.zeros(3), rotation=np.zeros(3)):
        
        self.__offset = offset
        self.__rotation = rotation
        self.__rotation_matrix = uts.GetRotFromEulerAnglesDeg(rotation)
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
        
        
        
"""Test"""
#tr = Trajectory()
#print tr
        
        

