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
