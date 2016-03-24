"""This class implements a fixed-point trajectory,
i.e., a trajectory that stays in the same point forever.
"""

import trajectory as tj
import numpy as np


class CircleTrajectory(tj.Trajectory):

    PARAMETERS_NAMES = ("radius", "speed")

    @classmethod
    def description(cls):
        return "Circle"
        
    def __init__(self, offset=np.array([0.0, 0.0, 1.0]), rotation=np.zeros(3),
            radius=1.0, speed=0.1):
        self.__radius = radius
        self.__speed = speed
        parameters = {"radius": radius, "speed": speed}
        tj.Trajectory.__init__(self, offset, rotation, parameters)
        
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
#tr = CircleTrajectory()
#print tr
