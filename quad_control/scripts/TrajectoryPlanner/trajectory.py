"""This module implements the Trajectory abstract class,
that is used for defining a desired trajectory
for a rigid body in the 3D space"""


import numpy as np
import utilities.utility_functions as uts


class Trajectory:


    PARAMETERS_NAMES = ()


    @classmethod
    def description(cls):
        return "Abstract Trajectory"
    
    
    def __init__(self, offset=np.zeros(3), rotation=np.zeros(3), parameters={}):
        
        self.__offset = offset
        self.__rotation = rotation
        self.__rotation_matrix = uts.GetRotFromEulerAnglesDeg(rotation)
        #TODO change the names in the utility_functions module
        
        self.__parameters = {}
        for name, value in parameters.items():
            if name in self.PARAMETERS_NAMES:
                self.__parameters[name] = value
    
    
    def __str__(self):
        string = self.description()
        string += "\nOffset: " + str(self.__offset)
        string += "\nRotation: " + str(self.__rotation)
        for name, value in self.__parameters.items():
            string += "\n" + name + ": " + str(value)
        return string
        
    
    def get_offset(self):
        return np.array(self.__offset)
        
        
    def get_rotation(self):
        return np.array(self.__rotation)
        
      
    def get_parameters(self):
        return dict(self.__parameters)
        
        
    def desired_trajectory(self, time):
        raise NotImplementedError(
            "Please, implement this method in all the Trajectory child classes")
    
        
    def __add_offset_and_rotation(self, pos, vel, acc, jrk, snp):
        
        off = self.__offset
        rot = self.__rotation_matrix
        
	# purposedly changing this: to test merge
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
        
        

