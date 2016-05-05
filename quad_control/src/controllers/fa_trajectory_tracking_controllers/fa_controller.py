#!/usr/bin/env python
# this line is just used to define the type of document


from utilities import jsonable as js

from controllers import controller


class FAController(controller.Controller):

    
    @classmethod
    def description(cls):
        return "Controller for **fully actuated** quadrotor model"
        
    def __str__(self):
        return self.description()
        
    def get_mass(self):
        return NotImplementedError()
