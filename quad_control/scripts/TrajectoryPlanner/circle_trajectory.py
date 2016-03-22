"""This class implements a fixed-point trajectory,
i.e., a trajectory that stays in the same point forever.
"""

import trajectory as tj
import numpy as np


class CircleTrajectory(tj.Trajectory):

    @classmethod
    def description(cls):
        return "Circle"
        
    def __init__(self, radius=1.0, speed=0.1, offset=np.array([0.0, 0.0, 1.0]), rotation=np.eye(3)):
        tj.Trajectory.__init__(self, offset, rotation)
        self.__radius = radius
        self.__speed = speed
        
    def __str__(self):
        string = self.description()
        string += "\nCenter: " + str(self.get_offset())
        string += "\nRadius: " + str(self.__radius)
        string += "\nSpeed: " + str(self.__speed)
        return string
        
    def get_radius(self):
        return float(self.__radius)
        
    def get_speed(self):
        return float(self.__speed)
        
    def desired_trajectory(self, time):
        c = np.cos
        s = np.sin
        r = self.__radius
        w = self.__speed
        t = time
        pos = r*w**0*np.array([ c(w*t),-s(w*t),0.0]);
        vel = r*w**1*np.array([-s(w*t),-c(w*t),0.0]);
        acc = r*w**2*np.array([-c(w*t), s(w*t),0.0]);
        jrk = r*w**3*np.array([ s(w*t), c(w*t),0.0]);
        snp = r*w**4*np.array([ c(w*t),-s(w*t),0.0]);
        return pos, vel, acc, jrk, snp
        
        
        
        
"""Test"""
#tr = CircleTrajectory(1.0, 0.1, np.array([0.0, 0.0, 1.0]))
#print tr
