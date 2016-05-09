#!/usr/bin/env python
# this line is just used to define the type of document

# from .. import controller
from controllers import controller

class VectorThrustController(controller.Controller):

    
    @classmethod
    def description(cls):
        return "Abstract **Vector Thrust Controller**: controller u(.) for system x = u"

    #TODO do these have special parameters?
    def __init__(self):
        pass
        
    def __str__(self):
        string = controller.Controller.__str__(self)
        return string
    
    def output(self, position, velocity): 
        raise NotImplementedError()
