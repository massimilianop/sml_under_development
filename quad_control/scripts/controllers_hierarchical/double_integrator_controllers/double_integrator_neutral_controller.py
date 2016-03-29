#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
import json


import controllers_hierarchical.double_integrator_controllers.double_integrator_controller as dic 



class DoubleIntegratorNeutralController(dic.DoubleIntegratorController):

    
    @classmethod
    def description(cls):
        return "Double-integrator neutral controller"
    
    
    @classmethod
    def parameters_to_string(cls, parameters=None):
        return json.dumps(parameters)
        
        
    @classmethod
    def string_to_parameters(cls, string):
        return json.loads(string)
        

    @classmethod
    def contained_objects(cls):
        return dict()        

    
    def output(self, position, velocity):
        #TODO make the dimension a parameter?
        return numpy.zeros(len(position))
        
        
        
# Test
#con = DoubleIntegratorNeutralController()
#print con
#print con.output(numpy.zeros(3),numpy.zeros(3))
