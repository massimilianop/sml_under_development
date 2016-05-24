"""This class implements a fixed-point trajectory,
i.e., a trajectory that stays in the same point forever.
"""

from .. import yaw_trajectory
import numpy as np
import json

class FixedYawTrajectory(yaw_trajectory.YawTrajectory):

    @classmethod
    def description(cls):
        return "<b>Yaw angle to be constant</b>"
        
    def __init__(self, offset=0.0):
        yaw_trajectory.YawTrajectory.__init__(self, offset=offset)
        
    def __str__(self):
        string = self.description()
        string += "\nOffset: " + str(self.get_offset())
        return string
        
    def desired_trajectory(self, time):
        yaw_0dot = 0.0
        yaw_1dot = 0.0
        return yaw_0dot, yaw_1dot