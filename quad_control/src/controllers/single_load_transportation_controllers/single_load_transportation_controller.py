#!/usr/bin/env python
# this line is just used to define the type of document

# from .. import controller
from controllers import controller

class SingleLoadTransportationController(controller.Controller):

    @classmethod
    def description(cls):
        return "Abstract **Single Load Transportation Controller**: controller for single aerial vehicle transportation a load attached by cable"

    #TODO do these have special parameters?
    def __init__(self):
        pass
        
    def __str__(self):
        string = controller.Controller.__str__(self)
        return string
    
    def output(self, position, velocity): 
        raise NotImplementedError()
