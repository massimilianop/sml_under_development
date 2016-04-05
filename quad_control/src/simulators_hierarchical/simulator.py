"""This file implements the parent class for a simulator."""


import numpy as np
import scipy.integrate as spi
import json
from utilities import jsonable as js



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
    ths = acro_rpp*4500.0/100.0*np.pi/180.0
    throttle = throttle_gain*throttle_cmd/mass
    ang_vel = np.zeros(3)
    ang_vel[0] =  (ang_vel_cmd[0] - 1500.0)/500.0*ths
    ang_vel[1] = -(ang_vel_cmd[1] - 1500.0)/500.0*ths
    ang_vel[2] = -(ang_vel_cmd[2] - 1500.0)/500.0*ths
    
    return throttle, ang_vel
    
    
    
    
def stabilize_mode_command_to_thrust_and_yaw_rate(command):
    #TODO implement this function
    pass



class Simulator(js.Jsonable):


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
            
        self.time = initial_time
       
        assert len(initial_state) == self.get_state_size()
        self.state = np.array(initial_state)
        
        assert len(initial_control) == self.get_control_size()
        self.control = np.array(initial_control)
        
        #TODO This is not very nice stylistically.
        # We should change f into a descriptive name
        # or use a lambda function.
        def f(t,x):
            return self.vector_field(t, x, self.control)
        self.solver = spi.ode(f).set_integrator('dopri5')
                
        self.time_record = []
        self.state_record = [[] for index in range(self.get_state_size())]
        self.control_record = [[] for index in range(self.get_control_size())]
                
                
    def __str__(self):
        string = self.description()
        string += "\nTime: " + str(self.time)
        string += "\nState: " + str(self.state)
        string += "\nControl: " + str(self.control)
        return string
        
        
    def get_time(self):
        return self.time
        
    
    def get_state(self):
        return np.array(self.state)
        
        
        
    def get_position(self):
        raise NotImplementedError()
        
        
    def get_attitude(self):
        raise NotImplementedError()
        
        
    def set_control(self, command):
        raise NotImplementedError
    
        
    def reset(self, initial_time=0.0,
            initial_state=None,
            initial_control=None):
        
        if initial_state==None:
            initial_state = np.zeros(self.get_state_size())
            
        if initial_control==None:
            initial_control = np.zeros(self.get_control_size())
        
        self.time = initial_time
        
        assert len(initial_state) == self.get_state_size()
        self.state = np.array(initial_state)
        
        assert len(initial_control) == self.get_control_size()
        self.control = np.array(initial_control)
        
        self.solver.set_initial_value(initial_time, initial_state)
                
                
    def vector_field(self, time, state, control):
        raise NotImplementedError()
        
        
    def run(self, time_step):
        self.time_record.append(self.time)
        self.state_record.append(list(self.state))
        self.control_record.append(list(self.control))
        self.solver.set_initial_value(np.array(self.state), self.time)
        self.solver.integrate(self.time + time_step)
        self.time = self.solver.t
        self.state = np.array(self.solver.y)
              
  
        
        
#"""Test"""
#my_sim = Simulator()
#print my_sim
