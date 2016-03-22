#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
import rospy

class YawRateControllerNeutral(object):
    
    GAIN_YAW_CONTROL = 1.0

    # """docstring for YawRateControllerNeutral"""
    # def __init__(self, arg):
    #     super(YawRateControllerNeutral, self).__init__()
    #     self.arg = arg

    def output(self,state,state_desired):
        # state = euler_angles in RAD + euler_angles_time_derivative in RAD/SEC
        # state_desired = psi_desired in RAD + psi_desired_time_derivative in RAD/SEC
        
        # neutral controller: do nothing
        yaw_rate = 0.0
        return yaw_rate
        