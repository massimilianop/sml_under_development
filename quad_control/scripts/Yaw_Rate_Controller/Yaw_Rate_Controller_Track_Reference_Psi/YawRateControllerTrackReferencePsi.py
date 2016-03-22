#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
import rospy

class YawRateControllerTrackReferencePsi(object):
    
    GAIN_YAW_CONTROL = 1.0

    # """docstring for YawRateControllerNeutral"""
    # def __init__(self, arg):
    #     super(YawRateControllerNeutral, self).__init__()
    #     self.arg = arg

    def output(self,state,state_desired):
        # state = euler_angles in RAD + euler_angles_time_derivative in RAD/SEC
        # state_desired = psi_desired in RAD + psi_desired_time_derivative in RAD/SEC
        return self.controller(state,state_desired)
        
    def controller(self,state,state_desired):

        #--------------------------------------#
        # current phi and theta and psi
        euler_angles = state[0:3]
        phi          = euler_angles[0]
        theta        = euler_angles[1]
        psi          = euler_angles[2]

        euler_angles_time_derivative = state[3:6]
        phi_dot                      = euler_angles_time_derivative[0]
        theta_dot                    = euler_angles_time_derivative[1]
        psi_dot                      = euler_angles_time_derivative[2]

        #--------------------------------------#
        psi_star     = state_desired[0]
        psi_star_dot = state_desired[1]
        psi_dot      = psi_star_dot - self.GAIN_YAW_CONTROL*numpy.sin(psi - psi_star)
        yaw_rate     = 1.0/numpy.cos(phi)*(numpy.cos(theta)*psi_dot - numpy.sin(phi)*theta_dot)
        
        return yaw_rate