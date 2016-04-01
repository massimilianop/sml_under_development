#!/usr/bin/env python
# this line is just used to define the type of document

import numpy as np


class YawController:

    
    @classmethod
    def description(cls):
        return "Abstract yaw controller"
    
    
    @classmethod
    def parameters_to_string(cls, parameters):
        raise NotImplementedError()
        
        
    @classmethod
    def string_to_parameters(cls, string):
        raise NotImplementedError()


    def __init__(self):
        pass
        
        
    def __str__(self):
        return self.description()
        
        
    def output(self, state, reference):
        # state = euler_angles in RAD 
        # + euler_angles_time_derivative in RAD/SEC
        # state_desired = psi_desired in RAD + 
        # psi_desired_time_derivative in RAD/SEC
    
        #TODO everything else is in degrees,
        # maybe we should have this in degrees as well
        raise NotImplementedError()
