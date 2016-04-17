#!/usr/bin/env python
# this line is just used to define the type of document

# in case we want to use rospy.logwarn or logerror
import rospy
import numpy

from controllers import controller

class NeutralController(controller.Controller):
    
    
    @classmethod
    def description(cls):
        return "Neutral Controller"


    def __init__(self):
        pass


    def output(self, delta_t, state, reference):
        # desired force is null (no force, stay at rest)
        return numpy.zeros(3)
        
        
# # Test
# string = NeutralController.to_string()
# print string
# con = NeutralController.from_string(string)
