#!/usr/bin/env python
# this line is just used to define the type of document

# in case we want to use rospy.logwarn or logerror
import rospy

import numpy
import json

from controllers_hierarchical import controller


class NeutralController(controller.Controller):

    
    @classmethod
    def contained_objects(cls):
        return {}
    
    
    @classmethod
    def description(cls):
        return "Neutral Controller"
    
    
    @classmethod
    def parameters_to_string(cls, parameters=None):
        return json.dumps(parameters)
        
        
    @classmethod
    def string_to_parameters(cls, string=None):
        return dict()


    def output(self, delta_t, state, reference):
        # desired force is null (no force, stay at rest)
        return numpy.zeros(3)
        
        
        
