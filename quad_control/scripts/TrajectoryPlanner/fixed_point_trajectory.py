"""This class implements a fixed-point trajectory,
i.e., a trajectory that stays in the same point forever.
"""

import trajectory as tj
import numpy as np

import rospy


class FixedPointTrajectory(tj.Trajectory):

    @classmethod
    def description(cls):
        return "Fixed Point"
        
    def __init__(self, offset):
        tj.Trajectory.__init__(self, offset, np.eye(3))
        
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
# tr = FixedPointTrajectory()
# print tr
