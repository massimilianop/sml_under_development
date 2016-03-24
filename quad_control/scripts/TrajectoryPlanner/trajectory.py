"""This module implements the Trajectory abstract class,
that is used for defining a desired trajectory
for a rigid body in the 3D space"""


import numpy as np



class Trajectory:

    @classmethod
    def description(cls):
        return "Abstract Trajectory"
    
    
    def __init__(self, offset=np.zeros(3), rotation=np.eye(3)):
        
        self.__offset = offset
        self.__rotation = rotation
        
    
    def __str__(self):
        return self.description()
        
    
    def get_offset(self):
        return np.array(self.__offset)
        
        
    def get_rotation(self):
        return np.array(self.__rotation)
        
        
    def desired_trajectory(self, time):
        raise NotImplementedError(
            "Please, implement this method in all the Trajectory child classes")
    
        
    def __add_offset_and_rotation(self, pos, vel, acc, jrk, snp):
        
        off = self.__offset
        rot = self.__rotation
        
	# purposedly changing this: to test merge
        pos_out = rot.dot(pos) + 1.0*off
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
        
        

