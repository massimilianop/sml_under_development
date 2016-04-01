"""This class implements a fixed-point trajectory,
i.e., a trajectory that stays in the same point forever.
"""

import trajectory as tj
import numpy as np
import json

import rospy


class FixedPointTrajectory(tj.Trajectory):

    @classmethod
    def description(cls):
        return "Fixed Point"
        
    @classmethod
    def get_parameters_names(cls):
        return ()
        
    @classmethod
    def parameters_to_string(cls):
        dic = dict()
        return json.dumps(dic)
        
    @classmethod
    def string_to_parameters(cls, string):
        return tuple()
        
    def __init__(self, point=np.array([0.0, 0.0, 0.0]),rotation=np.zeros(3)):
        tj.Trajectory.__init__(self, point, rotation)
        
    def __str__(self):
        string = self.description()
        string += "\nPoint: " + str(self.get_offset())
        return string
        
    def desired_trajectory(self, time):
        pos = np.zeros(3)
        vel = np.zeros(3)
        acc = np.zeros(3)
        jrk = np.zeros(3)
        snp = np.zeros(3)
        return pos, vel, acc, jrk, snp
        
        
        
        
# """Test"""
#string = FixedPointTrajectory.parameters_to_string()
#print string
#parameters = FixedPointTrajectory.string_to_parameters(string)
#print parameters
#tr = FixedPointTrajectory(np.zeros(3), *parameters)
#print tr
