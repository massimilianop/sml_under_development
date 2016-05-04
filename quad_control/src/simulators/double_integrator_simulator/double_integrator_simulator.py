"""This module implements the simulator of the quad
with no attitude inner loop.
"""
#TODO describe this better


import numpy as np

import utilities.utility_functions as uts
from simulators import simulator
import rospy



class DoubleIntegratorSimulator(simulator.Simulator):


    @classmethod
    def description(cls):
        return "Iris+ simulator as 3D double integrator"

    @classmethod
    def get_state_size(cls):
        return 3+3+9
    
    
    @classmethod
    def get_control_size(cls):
        return 4
    
    
    #TODO maybe take the defaults parameters from rospy.getparam
    # instead of hardcoding them
    # (also in the parameters_to_string method)
    def __init__(self, initial_time = 0.0,
            initial_state    = np.concatenate([np.zeros(3+3),np.reshape(np.identity(3),9)]),
            initial_control  = None,
            mass             = 1.442,
            neutral_throttle = 1484,
            acro_rpp         = 4.5
            ):
            
        simulator.Simulator.__init__(self,
            initial_time,
            initial_state,
            initial_control
            )
            
        self.mass = mass
        self.neutral_throttle = neutral_throttle
        self.acro_rpp = acro_rpp
        self.throttle_gain = mass*uts.GRAVITY/neutral_throttle
        
        
    def get_position(self):
        return self.state[0:3]
        
        
    def get_attitude(self):
        return uts.euler_deg_from_rot(np.reshape(self.state, (3,3)))
        
        
    def set_control(self, command):
        current_psi        = 0.0 
        force_3d, yaw_rate = simulator.stabilize_mode_command_to_thrust_and_yaw_rate(
            command,
            current_psi,
            self.MASS,
            self.THROTTLE_NEUTRAL,
            self.MAX_PSI_SPEED_RAD,
            self.MAX_ANGLE_RAD
            )
        self.control[0:3]  = force_3d
        self.control[3]    = yaw_rate
        
        
    # def vector_field(self, time, state, control1, control2, control3, control4):
    def vector_field(self, time, state, control):        
        
        position = np.array(state[0:3])
        velocity = np.array(state[3:6])

        force_3d = control[0:3]
        # force_3d = np.array([control1, control2, control3])
        # rospy.logwarn(force_3d)

        dot_p = np.array(velocity)
        dot_v = force_3d/self.mass - uts.GRAVITY*uts.E3_VERSOR
        
        return np.concatenate([dot_p, dot_v,np.zeros(9)])
