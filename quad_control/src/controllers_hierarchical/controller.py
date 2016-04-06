#!/usr/bin/env python
# this line is just used to define the type of document


from utilities import jsonable as js



class Controller(js.Jsonable):

    
    @classmethod
    def description(cls):
        return "Abstract tracking controller"
        
        
    def __str__(self):
        return self.description()
        
        
    def output(self, delta_t, state, reference):
        # delta_t = time interval from previous call of output
        # state ...
        # reference ...    
        #TODO 
        raise NotImplementedError()
