"""This class implements a fixed-point trajectory,
i.e., a trajectory that stays in the same point forever.
"""

from .. import trajectory as tj
import numpy as np
import json

class FixedPointTrajectory(tj.Trajectory):

    @classmethod
    def description(cls):
        return "<b>Stay at rest at speficied point</b>"
        
    def __init__(self, point=np.array([0.0, 0.0, 1.0])):
        tj.Trajectory.__init__(self, offset=point, rotation=np.zeros(3))
        
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
        
        
        
        
# Test
#string = FixedPointTrajectory.to_string()
#print string
#tr = FixedPointTrajectory.from_string(string)
#print tr
