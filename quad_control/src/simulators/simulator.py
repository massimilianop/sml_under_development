"""This file implements the parent class for a simulator."""


import numpy as np
import numpy
import scipy.integrate as spi
import json
from utilities import jsonable as js
from utilities import utility_functions

import rospy



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
    throttle = throttle_gain*throttle_cmd
    ang_vel = np.zeros(3)
    ang_vel[0] =  (ang_vel_cmd[0] - 1500.0)/500.0*ths
    ang_vel[1] = -(ang_vel_cmd[1] - 1500.0)/500.0*ths
    ang_vel[2] = -(ang_vel_cmd[2] - 1500.0)/500.0*ths
    
    return throttle, ang_vel



def stabilize_mode_command_to_thrust_and_yaw_rate(
        joysticks,
        current_psi,
        mass,
        throttle_neutral,
        max_psi_speed_rad,
        max_angle_rad,
        ):
    """Convert joysticks inputs to 3D force (in newtons) + psi_rate (in rad/sec)"""

    # joysticks[3]: yaw angular speed
    yaw_rate    = -(joysticks[3] - 1500.0)*max_psi_speed_rad/500.0


    # desired roll and pitch. joysticks[0:1]
    roll_des  =  (joysticks[0] - 1500.0)*max_angle_rad/500.0
    # ATTENTTION TO PITCH: WHEN STICK IS FORWARD PITCH, pwm GOES TO 1000, AND PITCH IS POSITIVE 
    pitch_des = -(joysticks[1] - 1500)*max_angle_rad/500.0
    
    # desired euler angles in (rad)
    ee_des = numpy.array([roll_des,pitch_des,current_psi])

    rotation_matrix = utility_functions.GetRotFromEulerAngles(ee_des)
    unit_vector_des = rotation_matrix.dot(utility_functions.E3_VERSOR)

    # -----------------------------------------------------------------------------#
    # STABILIZE MODE:APM COPTER
    # The throttle sent to the motors is automatically adjusted based on the tilt angle
    # of the vehicle (i.e increased as the vehicle tilts over more) to reduce the 
    # compensation the pilot must fo as the vehicles attitude changes
    # -----------------------------------------------------------------------------#
    # throttle = joysticks[2]
    # this increases the actual throtle
    Throttle = joysticks[2]/unit_vector_des.dot(utility_functions.E3_VERSOR)
    Throttle *= mass*utility_functions.GRAVITY/throttle_neutral
    force_3d = Throttle*unit_vector_des

    return force_3d, yaw_rate



class Simulator(js.Jsonable):

    # 

    # mass of vehicles (kg)
    MASS = rospy.get_param("mass_quad_sim",1.442)

    # throttle that cancels weight
    THROTTLE_NEUTRAL = rospy.get_param("Throttle_neutral_sim",1484.0)

    #--------------------------------------------------------------#
    MAX_ANGLE_DEG = rospy.get_param("MAX_ANGLE_DEG",45.0)
    MAX_ANGLE_RAD = MAX_ANGLE_DEG*numpy.pi/180.0

    # The default of 4.5 commands a 200 deg/sec rate of rotation when the yaw stick is held fully left or right.
    MAX_PSI_SPEED_DEG = rospy.get_param("MAX_PSI_SPEED_Deg",200.0)  
    MAX_PSI_SPEED_RAD = MAX_PSI_SPEED_DEG*numpy.pi/180.0


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
            initial_time    = 0.0,
            initial_state   = None,
            initial_control = None
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
