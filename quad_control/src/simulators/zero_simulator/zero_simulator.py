import numpy as np
import numpy
import scipy.integrate as spi
import json
from utilities import utility_functions
import rospy

from .. import simulator as sim



class ZeroSimulator(sim.Simulator):


    @classmethod
    def description(cls):
        return """Quad simulator with zero dynamics.
            The quad stays where it is.
            The state is a 6d array,
            and corresponds to the position and velocity of the quad.
            There is no control.
            """
        
       
    @classmethod
    def get_state_size(cls):
        return 15
        
        
    @classmethod
    def get_control_size(cls):
        return 0

                
    def __init__(self,
        initial_time        = 0.0,
        position            = np.array([0.0, 0.0, 1.0])
        ):
        
        initial_state = np.concatenate([position, np.zeros(12)])
        sim.Simulator.__init__(self, initial_time, initial_state)
                
                
    def __str__(self):
        string = self.description()
        string += "\nTime: " + str(self.time)
        string += "\nPosition: " + str(self.state[0:3])
        return string


    def stabilize_mode_command_to_thrust_and_yaw_rate(self,joysticks,current_psi):
        """Convert joysticks inputs to 3D force (in newtons) + psi_rate (in rad/sec)"""

        # joysticks[3]: yaw angular speed
        yaw_rate    = -(joysticks[3] - 1500.0)*self.MAX_PSI_SPEED_RAD/500.0


        # desired roll and pitch. joysticks[0:1]
        roll_des  =  (joysticks[0] - 1500.0)*self.MAX_ANGLE_RAD/500.0
        # ATTENTTION TO PITCH: WHEN STICK IS FORWARD PITCH, pwm GOES TO 1000, AND PITCH IS POSITIVE 
        pitch_des = -(joysticks[1] - 1500)*self.MAX_ANGLE_RAD/500.0
        
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
        Throttle *= self.MASS*utility_functions.GRAVITY/self.THROTTLE_NEUTRAL
        force_3d = Throttle*unit_vector_des

        return (force_3d,yaw_rate)        
        
        
    def get_time(self):
        return self.time
        
    
    def get_state(self):
        return np.array(self.state)
        
        
    def get_position(self):
        return self.state[0:3]
        
        
    def get_attitude(self):
        return None
    
    
    def set_control(self, command):
        pass
                
                
    def vector_field(self, time, state, control):
        return np.zeros(15)
              
  
        
        
#"""Test"""
#my_sim = Simulator()
#print my_sim
