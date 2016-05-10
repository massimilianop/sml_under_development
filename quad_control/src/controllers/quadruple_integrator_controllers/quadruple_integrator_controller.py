#!/usr/bin/env python
# this line is just used to define the type of document

# from .. import controller
from controllers import controller

class QuadrupleIntegratorController(controller.Controller):

    
    @classmethod
    def description(cls):
        return "Abstract **Quadruple Integrator Controller**: controller u(p,v,a,j) for system p^(4) = u"

    #TODO do these have special parameters?
    def __init__(self):
        pass
        
    def __str__(self):
        string = controller.Controller.__str__(self)
        return string
    
    def output(self, position, velocity, acceleration, jerk): 
        raise NotImplementedError()
