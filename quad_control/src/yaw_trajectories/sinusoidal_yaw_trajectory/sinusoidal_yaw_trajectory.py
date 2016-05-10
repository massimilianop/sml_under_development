"""This class implements a fixed-point trajectory,
i.e., a trajectory that stays in the same point forever.
"""

from .. import yaw_trajectory
import numpy as np
import json
import math


class SinusoidalYawTrajectory(yaw_trajectory.YawTrajectory):


    @classmethod
    def description(cls):
        return "<b>Sinusoidal Yaw Trajectory</b> with <b>amplitude</b> in (deg), and <b>angular_speed</b> in (deg/s)"
        
        
    def __init__(self,
    offset        = np.array([0.0, 0.0, 1.0]), 
    amplitude     = 1.0, 
    angular_speed = 0.1
    ):
        yaw_trajectory.YawTrajectory.__init__(self, offset=offset)
        # convert from degrees to radians
        self.__amplitude     = amplitude*math.pi/180.0
        self.__angular_speed = angular_speed*math.pi/180.0
        
        
    def __str__(self):
        string = tj.Trajectory.__str__(self)
        string += "\nAmplitude: " + str(self.__amplitude)
        string += "\nAngular Speed: " + str(self.__angular_speed)
        return string

        
    def desired_trajectory(self, time):

        a = self.__amplitude
        w = self.__angular_speed
        
        yaw_0dot = a*w**0*np.sin(w*t)
        yaw_1dot = a*w**1*np.cos(w*t)

        return yaw_0dot,yaw_1dot

    
# """Test"""
#string = CircleTrajectory.to_string()
#print string
#tr = CircleTrajectory.from_string(string)
#print tr


