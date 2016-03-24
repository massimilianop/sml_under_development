"""This module implements the simulator of the quad
with no attitude inner loop.
"""
#TODO describe this better



import numpy as np
import utilities.utility_functions as uts
import simulator as sim


class NoAttitudeInnerLoopSimulator(sim.Simulator):

    
    @classmethod
    def get_state_size(cls):
        return 15
    
    
    @classmethod
    def get_control_size(cls):
        return 4
    
    
    def __init__(self, initial_time=0.0,
            initial_state=None,
            initial_control=[1484, 1500, 1500, 1500],
            mass=1.442, neutral_throttle=1484, acro_rpp=4.5):
        sim.Simulator.__init__(self, initial_time, initial_state,
            initial_control)
        self.__mass = mass
        self.__neutral_throttle = neutral_throttle
        self.__acro_rpp = acro_rpp
        self.__gain_throttle = mass*self.get_gravity()/neutral_throttle
        
        
    def get_parameters(self):
        return self.__mass, self.__neutral_throttle, self.__acro_rpp
        
        
    def set_parameters(self, mass, neutral_throttle, acro_rpp):
        self.__mass = mass
        self.__neutral_throttle = neutral_throttle
        self.__acro_rpp = acro_rpp
        
        
    def reset(self, initial_time=0.0, initial_state=None,
            initial_control=[1484, 1500, 1500, 1500]):
        sim.Simulator.reset(self, initial_time, initial_state, initial_control)
        
        
    def vector_field(self, time, state, control):
        
        position = np.array(state[0:3])
        velocity = np.array(state[3:6])
        #TODO make sure that this is a rotation matrix
        # for example, convert to euler angles and back
        rotation = np.reshape(state[6:15], (3,3))
        versor = rotation.dot(self.get_e3())
        thrust = control[0]
        omega = np.array(control[1:4])
        
        #TODO convert the control in a conversion module, not here
        ths = self.__acro_rpp*4500/100*np.pi/180
        w = np.zeros(3)
        w[0] =  (omega[0] - 1500)/500*ths;
        w[1] = -(omega[1] - 1500)/500*ths;
        w[2] = -(omega[2] - 1500)/500*ths;
        
        dot_p = np.array(velocity)
        dot_v = self.__gain_throttle*thrust/self.__mass*versor\
            - self.get_gravity()*self.get_e3()
        dot_r = rotation.dot(uts.skew(w))
        
        return np.concatenate([dot_p, dot_v, np.reshape(dot_r, 9)])
            
            
            
            
        
#"""Test"""
#my_sim = NoAttitudeInnerLoopSimulator()
#print my_sim
#my_sim.run(0.01)
#print my_sim
