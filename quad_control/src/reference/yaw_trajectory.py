"""This module implements the YawTrajectory abstract class,
that is used for defining the desired yaw angle"""


import numpy as np
from utilities import utility_functions as uts
from utilities import jsonable as js

import math

class YawTrajectory(js.Jsonable):

    @classmethod
    def description(cls):
        #raise NotImplementedError()
        return "<b>Abstract Yaw Trajectory</b> with desired yaw angle in <b>deg</b>"


    def __init__(self, offset=0.0):

        # convert from degrees to radians
        self.__offset = offset*math.pi/180.0
    
    def __str__(self):
        string = self.description()
        string += "\nOffset: " + str(self.__offset)
        return string
        
    
    def get_offset(self):
        return np.array(self.__offset)
        
    def set_offset(self, offset):
        self.__offset = offset

    def desired_trajectory(self, time):
        raise NotImplementedError()

    def __add_offset(self, yaw_0dot, yaw_1dot):

        yaw_0dot = yaw_0dot + self.__offset
        # yaw_1dot = yaw_1dot
        
        return np.array([yaw_0dot, yaw_1dot])
        
        
    def output(self, time):
        return self.__add_offset(*self.desired_trajectory(time))

@js.add_to_database(default=True)
class FixedYawTrajectory(YawTrajectory):
    """This class implements a fixed-point trajectory,
    i.e., a trajectory that stays in the same point forever.
    """

    @classmethod
    def description(cls):
        return "<b>Yaw angle to be constant</b>"
        
    def __init__(self, offset=0.0):
        YawTrajectory.__init__(self, offset=offset)
        
    def __str__(self):
        string = self.description()
        string += "\nOffset: " + str(self.get_offset())
        return string
        
    def desired_trajectory(self, time):
        yaw_0dot = 0.0
        yaw_1dot = 0.0
        return yaw_0dot, yaw_1dot

@js.add_to_database()
class SinusoidalYawTrajectory(YawTrajectory):
    """This class implements a fixed-point trajectory,
    i.e., a trajectory that stays in the same point forever.
    """

    @classmethod
    def description(cls):
        return "<b>Sinusoidal Yaw Trajectory</b> with <b>amplitude</b> in (deg), and <b>angular_speed</b> in (deg/s)"
        
        
    def __init__(self,
    offset        = np.array([0.0, 0.0, 1.0]), 
    amplitude     = 1.0, 
    angular_speed = 0.1
    ):
        YawTrajectory.__init__(self, offset=offset)
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
