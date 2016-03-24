"""This file implements the parent class for a simulator."""


import numpy as np
import scipy.integrate as spi

class Simulator():


    __GRAVITY = 9.81
    
    PARAMETERS_NAMES = ()
    STATE_SIZE = 1
    CONTROL_SIZE = 1


    @classmethod
    def description(cls):
        return "Abstract Simulator"
        
        
    @classmethod
    def get_gravity(cls):
        return float(cls.__GRAVITY)


    def __init__(self, parameters={},
            initial_time, initial_state, initial_control):
    
        self.__parameters = {}
        
        self.__time = initial_time
        assert len(initial_state) == self.STATE_SIZE
        self.__state = np.array(initial_state)
        assert len(initial_control) == self.CONTROL_SIZE
        self.__control = np.array(initial_control)
        
        self.__solver = spi.ode(lambda state: self.vector_field(
            self.__time, state, self.__control)).set_integrator('dopri5')
    
        for name in parameters.keys():
            if name in self.PARAMETERS_NAMES:
                self.__parameters[name] = parameters[name]
                
        self.__time_record = []
        self.__state_record = [[] for index in range(self.STATE_SIZE)]
        self.__control_record = [[] for index in range(self.CONTROL_SIZE)]
                
                
    def __str__(self):
        string = self.description()
        for name, value in self.__parameters.items():
            string += "\n" + name + ": " + str(value)
        string += "\nTime: " + self.__time
        string += "\nState: " + self.__state
        string += "\nControl: " + self.__control
        return string
        
    
    def get_state(self):
        return np.array(self.__state)
        
        
    def get_parameters(self):
        return dict(self.__parameters)
        
        
    def get_time_record(self):
        return list(self.__time_record)
        
    
    def get_state_record(self):
        return list(self.__state_record)
        
        
    def get_control_record(self):
        return list(self.__control_record)
        
        
    def set_control(self, control):
        assert len(control) == self.CONTROL_SIZE
        self.__control = np.array(control)    
    
        
    def update_parameters(self, parameters={}):
        for name in parameters.keys():
            if name in self.PARAMETERS_NAMES:
                self.__parameters[name] = parameters[name]
                
                
    def vector_field(self, time, state, control):
        raise NotImplementedError("All the children of the class Simulator\
            should implement this method.")
        
        
    def run(self, time_step):
        self.__time_record.append(self.__time)
        for index in range(self.STATE_SIZE):
            self.__state_record[index].append(self.__state[index])
        for index in range(self.CONTROL_SIZE):
            self.__control_record[index].append(self.__control[index])
        self.__solver.set_initial_value(self.__state, self.__time)
        self.__solver.integrate(self.__time + time_step)
        self.__time = self.__solver.t
        self.__state = np.array(self.__solver.y)
        
