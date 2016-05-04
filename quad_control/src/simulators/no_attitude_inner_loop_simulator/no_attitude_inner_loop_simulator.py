"""This module implements the simulator of the quad
with no attitude inner loop.
"""
#TODO describe this better



import numpy as np
import utilities.utility_functions as uts
from simulators import simulator as sm
import rospy


class NoAttitudeInnerLoopSimulator(sm.Simulator):


    @classmethod
    def description(cls):
        return "Iris+ simulator without attitude inner loop"

    
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
            initial_position=np.zeros(3),
            initial_velocity=np.zeros(3),
            initial_rotation=np.zeros(3),
            initial_control=np.zeros(4),
            mass=1.442,
            neutral_throttle=1484,
            acro_rpp=4.5
            ):
        
        pos = initial_position
        vel = initial_velocity
        rot = np.reshape(uts.rot_from_euler_deg(np.array(initial_rotation)), 9)
        initial_state = np.concatenate([pos, vel, rot])
        sm.Simulator.__init__(self, initial_time, initial_state, initial_control)
        self.mass = mass
        self.neutral_throttle = neutral_throttle
        self.acro_rpp = acro_rpp
        self.throttle_gain = mass*uts.GRAVITY/neutral_throttle
        
        
    def get_position(self):
        return self.state[0:3]
        
        
    def get_attitude(self):
        return uts.euler_deg_from_rot(np.reshape(self.state[6:15], (3,3)))
        
        
    def set_control(self, command):
        throttle, ang_vel = sm.acro_mode_command_to_throttle_and_angular_velocity(
            command, self.mass, self.throttle_gain, self.acro_rpp)
        self.control[0] = throttle
        self.control[1:4] = ang_vel
        
        
    def vector_field(self, time, state, control):
        
        
        position = np.array(state[0:3])
        velocity = np.array(state[3:6])
        #TODO make sure that this is a rotation matrix
        # for example, convert to euler angles and back
        rotation = np.reshape(state[6:15], (3,3))

        versor   = rotation.dot(uts.E3_VERSOR)
        throttle = control[0]
        omega    = np.array(control[1:4])
        
        dot_p = np.array(velocity)
        dot_v = throttle/self.mass*versor - uts.GRAVITY*uts.E3_VERSOR
        rospy.logwarn(dot_v)
        dot_r = rotation.dot(uts.skew(omega))
        
        return np.concatenate([dot_p, dot_v, np.reshape(dot_r, 9)])
            
            
            
            
        
"""Test"""

#string = NoAttitudeInnerLoopSimulator.to_string()
#print string
#sim = NoAttitudeInnerLoopSimulator.from_string(string)
#print sim
#print sim.vector_field(0.0, np.ones(15), np.ones(4))

