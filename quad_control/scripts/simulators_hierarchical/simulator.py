"""This file implements the parent class for a simulator."""


import numpy as np
import scipy.integrate as spi
import json



def acro_mode_command_to_throttle_and_angular_velocity(
        command,
        mass,
        throttle_gain,
        acro_rpp):
        
    throttle_cmd = command[2]
    ang_vel_cmd = np.zeros(3)
    ang_vel_cmd[0] = command[0]
    ang_vel_cmd[1] = command[1]
    ang_vel_cmd[2] = command[3]
    ths = acro_rpp*4500/100*np.pi/180
    throttle = throttle_gain*throttle_cmd/mass
    ang_vel = np.zeros(3)
    ang_vel[0] =  (ang_vel_cmd[0] - 1500)/500*ths
    ang_vel[1] = -(ang_vel_cmd[1] - 1500)/500*ths
    ang_vel[2] = -(ang_vel_cmd[2] - 1500)/500*ths
    
    return throttle, ang_vel
    
    
    
    
def stabilize_mode_command_to_thrust_and_yaw_rate(command):
    #TODO implement this function
    pass



class Simulator(Jsonable):


    @classmethod
    def description(cls):
        return "Abstract Simulator"
        
       
    @classmethod
    def get_state_size(cls):
        raise NotImplementedError()
        
        
    @classmethod
    def get_control_size(cls):
        raise NotImplementedError()


    def __init__(self,
            initial_time=0.0,
            initial_state=None,
            initial_control=None
            ):
        
        if initial_state==None:
            initial_state = np.zeros(self.get_state_size())
            
        if initial_control==None:
            initial_control = np.zeros(self.get_control_size())
            
        self.__time = initial_time
       
        assert len(initial_state) == self.get_state_size()
        self.__state = np.array(initial_state)
        
        assert len(initial_control) == self.get_control_size()
        self.__control = np.array(initial_control)
        
        #TODO This is not very nice stylistically.
        # We should change f into a descriptive name
        # or use a lambda function.
        def f(t,x):
            return self.vector_field(t, x, self.__control)
        self.__solver = spi.ode(f).set_integrator('dopri5')
                
        self.__time_record = []
        self.__state_record = [[] for index in range(self.get_state_size())]
        self.__control_record = [[] for index in range(self.get_control_size())]
                
                
    def __str__(self):
        string = self.description()
        string += "\nTime: " + str(self.__time)
        string += "\nState: " + str(self.__state)
        string += "\nControl: " + str(self.__control)
        return string
        
        
    def get_time(self):
        return float(self.__time)
        
    
    def get_state(self):
        return np.array(self.__state)
        
        
    def get_control(self):
        return self.__control
        
        
    def get_time_record(self):
        return list(self.__time_record)
        
    
    def get_state_record(self):
        return list(self.__state_record)
        
        
    def get_control_record(self):
        return list(self.__control_record)
        
        
    def get_parameters(self):
        raise NotImplementedError()
        
        
    def set_control(self, control):
        assert len(control) == self.get_control_size()
        self.__control = np.array(control) 
    
    
    def set_parameters(self, parameters):
        raise NotImplementedError()
    
        
    def reset(self, initial_time=0.0,
            initial_state=None,
            initial_control=None):
        
        if initial_state==None:
            initial_state = np.zeros(self.get_state_size())
            
        if initial_control==None:
            initial_control = np.zeros(self.get_control_size())
        
        self.__time = initial_time
        
        assert len(initial_state) == self.get_state_size()
        self.__state = np.array(initial_state)
        
        assert len(initial_control) == self.get_control_size()
        self.__control = np.array(initial_control)
        
        self.__solver.set_initial_value(initial_time, initial_state)
                
                
    def vector_field(self, time, state, control):
        raise NotImplementedError()
        
        
    def run(self, time_step):
        self.__time_record.append(self.__time)
        for index in range(self.get_state_size()):
            self.__state_record[index].append(self.__state[index])
        for index in range(self.get_control_size()):
            self.__control_record[index].append(self.__control[index])
        self.__solver.set_initial_value(np.array(self.__state), self.__time)
        self.__solver.integrate(self.__time + time_step)
        self.__time = self.__solver.t
        self.__state = np.array(self.__solver.y)
        
        
        
        
#"""Test"""
#my_sim = Simulator()
#print my_sim
