"""This module implements the simulator of the quad
with attitude inner loop.
"""
#TODO describe this better


import numpy as np
import utilities.utility_functions as uts
from .. import simulator
import rospy
import numpy as np


class AttitudeInnerLoopSimulator(simulator.Simulator):


    @classmethod
    def description(cls):
        return "Iris+ simulator with attitude inner loop"

    
    @classmethod
    def get_state_size(cls):
        return 15
    
    
    @classmethod
    def get_control_size(cls):
        return 4
    
    
    #TODO maybe take the defaults parameters from rospy.getparam
    # instead of hardcoding them
    # (also in the parameters_to_string method)
    def __init__(self, initial_time=0.0,
            initial_state=None,
            initial_control=None,
            mass=1.442,
            neutral_throttle=1484,
            acro_rpp=4.5,
            gain_inner_loop = 1.0
            ):
            
        simulator.Simulator.__init__(self, initial_time, initial_state,
            initial_control)
        self.mass = mass
        self.neutral_throttle = neutral_throttle
        self.acro_rpp = acro_rpp
        self.throttle_gain = mass*uts.GRAVITY/neutral_throttle
        self.gain_inner_loop = gain_inner_loop
        
        
    def get_position(self):
        return self.state[0:3]
        
        
    def get_attitude(self):
        return uts.GetEulerAnglesDeg(np.reshape(self.state, (3,3)))
        
        
    def set_control(self, command):
        throttle, ang_vel = simulator.acro_mode_command_to_throttle_and_angular_velocity(
            command, self.mass, self.throttle_gain, self.acro_rpp)
        self.control[0] = throttle
        self.control[1:4] = ang_vel
        
        
    def vector_field(self, time, state, control):
        
        position = np.array(state[0:3])
        velocity = np.array(state[3:6])
        #TODO make sure that this is a rotation matrix
        # for example, convert to euler angles and back
        rotation = np.reshape(state[6:15], (3,3))
        # Not sure about this... check with Pedro
        current_psi = uts.GetEulerAnglesDeg(rotation)[2]
        unit_vector   = rotation.dot(uts.E3_VERSOR)
        throttle = control[0]

        force_3d, yaw_rate = simulator.stabilize_mode_command_to_thrust_and_yaw_rate(
            control,
            current_psi,
            self.MASS,
            self.THROTTLE_NEUTRAL,
            self.MAX_PSI_SPEED_RAD,
            self.MAX_ANGLE_RAD
            )

        throttle = np.dot(force_3d,unit_vector)

        # gain of inner loop for attitude control
        ktt             = self.gain_inner_loop
        unit_vector_des = force_3d/np.linalg.norm(force_3d)
        versor          = rotation.dot(uts.E3_VERSOR)
        omega           = ktt*uts.skew(unit_vector).dot(unit_vector_des)

        dot_p = np.array(velocity)
        dot_v = throttle/self.mass*versor - uts.GRAVITY*uts.E3_VERSOR
        dot_r = rotation.dot(uts.skew(omega))
        
        return np.concatenate([dot_p, dot_v, np.reshape(dot_r, 9)])
            
            
            
            
        
# """Test"""

# string = NoAttitudeInnerLoopSimulator.to_string()
# print string
# sim = NoAttitudeInnerLoopSimulator.from_string(string)
# print sim
# print sim.vector_field(0.0, np.ones(15), np.ones(4))

