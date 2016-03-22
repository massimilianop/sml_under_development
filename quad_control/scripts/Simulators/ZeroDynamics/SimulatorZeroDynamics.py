#!/usr/bin/env python
# this is just to define file type

import numpy

class SimulatorWithZeroDynamics():
    
    def __init__(self,parameters = None):
        return

    def update_parameters(self,parameters):
        return

    def output(self,t, y, U):   
        # positon (3) + velocity (3) + rotation matrix (9)    
        return numpy.zeros(3+3+9)

