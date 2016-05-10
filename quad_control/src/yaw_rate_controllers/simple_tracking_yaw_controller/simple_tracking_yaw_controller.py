#!/usr/bin/env python
# this line is just used to define the type of document

import numpy as np
from yaw_rate_controllers import yaw_controller as yc


class SimpleTrackingYawController(yc.YawController):

    @classmethod
    def description(cls):
        return "Simple yaw tracking controller, based on <b>feedback linearization of yaw rate equation</b>"

    def __init__(self, gain = 1.0):
        self.__gain = gain
    
    
    def __str__(self):
        string = yc.YawController.__str__(self)
        string += "\nGain: " + str(self.__gain)
        return string


    def output(self, state, state_desired):
        # state = euler_angles in RAD + euler_angles_time_derivative in RAD/SEC
        # state_desired = psi_desired in RAD + psi_desired_time_derivative in RAD/SEC
        #return self.controller(state,state_desired)

        #--------------------------------------#
        # current phi and theta and psi
        euler_angles = state[0:3]
        phi          = euler_angles[0]
        theta        = euler_angles[1]
        psi          = euler_angles[2]

        euler_angles_time_derivative = state[3:6]
        
        phi_dot   = euler_angles_time_derivative[0]
        theta_dot = euler_angles_time_derivative[1]
        psi_dot   = euler_angles_time_derivative[2]

        #--------------------------------------#
        psi_star     = state_desired[0]
        psi_star_dot = state_desired[1]
        psi_dot      = psi_star_dot - self.__gain*np.sin(psi - psi_star)
        yaw_rate     = 1.0/np.cos(phi)*(np.cos(theta)*psi_dot - np.sin(phi)*theta_dot)
        
        return yaw_rate


# """Test"""
# 
#string = TrackingYawController.parameters_to_string()
#print string
#parameters = TrackingYawController.string_to_parameters(string)
#print parameters
#controller = TrackingYawController(parameters)
#print controller
#output = controller.output(np.zeros(6), np.ones(2))
#print output





