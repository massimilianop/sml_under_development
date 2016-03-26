#!/usr/bin/env python
# this line is just used to define the type of document

import numpy as np


class TrackingController:

    
    @classmethod
    def description(cls):
        return "Abstract Tracking controller"
    
    
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
        
        
    def output(self, delta_t, state, reference):
        # delta_t = time interval from previous call of output
        # state ...
        # reference ...    
        #TODO 
        raise NotImplementedError()
