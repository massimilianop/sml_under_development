#!/usr/bin/env python
# this line is just used to define the type of document

# in case we want to use rospy.logwarn or logerror
import rospy

import numpy

class ControllerNeutral():
    
    # def __init__(self): 

    # def reset_estimate_xy(self):
    #     return

    # def reset_estimate_z(self):
    #     return        

    # def update_parameters(self,parameters):
    #     self.parameters = parameters

    def output(self,time,states,states_d):
        # desired force is null (no force, stay at rest)
        return numpy.zeros(3)     
