#!/usr/bin/env python
# this line is just used to define the type of document


class DoubleIntegratorController:

    
    @classmethod
    def description(cls):
        return "Abstract Double Integrator Controller"
    
    
    @classmethod
    def parameters_to_string(cls, parameters):
        raise NotImplementedError()
        
        
    @classmethod
    def string_to_parameters(cls, string):
        raise NotImplementedError()


    @classmethod
    def dictionary(cls, string):
        raise NotImplementedError()


    def __init__(self):
        pass
        
        
    def __str__(self):
        return self.description()
        
        
    def output(self, position, velocity): 
        #TODO 
        raise NotImplementedError()
