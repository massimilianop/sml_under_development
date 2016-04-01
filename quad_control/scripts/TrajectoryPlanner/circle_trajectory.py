"""This class implements a fixed-point trajectory,
i.e., a trajectory that stays in the same point forever.
"""

import trajectory as tj
import numpy as np
import json


class CircleTrajectory(tj.Trajectory):


    @classmethod
    def description(cls):
        return "Circle"
    
        
    @classmethod
    def parameters_to_string(cls, radius=1.0, speed=0.1):
        dic = {'radius':radius, 'speed':speed}
        return json.dumps(dic)
        
        
    @classmethod
    def string_to_parameters(cls, string):
        dic = json.loads(string)
        radius = dic['radius']
        speed = dic['speed']
        return radius, speed
        
        
    def __init__(self, offset=np.array([0.0, 0.0, 1.0]), rotation=np.zeros(3),
            radius=1.0, speed=0.1):
        tj.Trajectory.__init__(self, offset, rotation)
        self.__radius = radius
        self.__speed = speed
        
        
    def __str__(self):
        string = tj.Trajectory.__str__(self)
        string += "\nRadius: " + str(self.__radius)
        string += "\nSpeed: " + str(self.__speed)
        return string

        
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
#tr = CircleTrajectory(offset=np.array([0.0, 0.0, 1.0]), radius=0.5, speed=0.2)
##print tr

#string = CircleTrajectory.offset_and_rotation_to_string()
#print string
#offset, rotation = CircleTrajectory.string_to_offset_and_rotation(string)
#print offset, rotation
#traj = CircleTrajectory(offset, rotation)
#print traj


