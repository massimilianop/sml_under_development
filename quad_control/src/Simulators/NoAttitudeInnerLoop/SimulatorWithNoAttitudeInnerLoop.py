#!/usr/bin/env python
# this is just to define file type

import numpy

import rospy

# we use pi
import math

# we need some functions
from utility_functions import skew

#--------------------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------------------#


class SimulatorWithNoAttitudeInnerLoop(object):

    # acceleration due to gravity (m/s^2); this parameter is defined in the Launch file
    GRAVITY = rospy.get_param("gravity_sim",9.81)
    
    # mass of vehicles (kg)
    MASS = rospy.get_param("mass_quad_sim",1.442)

    # throttle that cancels weight
    THROTTLE_NEUTRAL = rospy.get_param("Throttle_neutral_sim",1484.0)

    # ACRO mode (angular velocity sensitivity)
    ACRO_RP_P = rospy.get_param("ACRO_RP_P_sim",4.5)    
    
    def __init__(self,parameters = None):
        
        # if parameters == None:
        # stick with the initialized parameter
        if parameters != None:
            # update mass and throttle neutral
            MASS = parameters[0]
            THROTTLE_NEUTRAL = parameters[1]
            #ACRO_RP_P = parameters[2]

    def update_parameters(self,parameters):
        self.parameters = parameters

    def output(self,t, y, U):
        # states = y
        # return sys_dynamics(states,states_d,self.parameters)
        UU = numpy.array([U.U0,U.U1,U.U2,U.U3])        
        return self.sys_dynamics(y,UU)

    def sys_dynamics(self,states , U):

        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])

        # states
        # transported mass: position and velocity
        x = states[0:3]; v = states[3:6];

        # thrust unit vector
        R  = states[6:15]; R  = numpy.reshape(R,(3,3)); n  = R.dot(e3)

        #------------------------------------------------#
        # rearrage to proper order [roll,pitch,throttle,yaw] in sticks in model order is [throttle,roll,pitch,yaw]
        U_new    = numpy.zeros(4)
        U_new[0] = U[2]
        U_new[1] = U[0]
        U_new[2] = U[1]
        U_new[3] = U[3]        
        U        = U_new

        #------------------------------------------------#

        T = U[0];
        w = U[1:4]

        Max = self.ACRO_RP_P*4500/100*math.pi/180;
        w[0] =  (w[0] - 1500)/500*Max;
        w[1] = -(w[1] - 1500)/500*Max;
        w[2] = -(w[2] - 1500)/500*Max;

        pDot = v
        # acceleration of quad
        GAIN_THROTTLE  = self.MASS*self .GRAVITY/self.THROTTLE_NEUTRAL
        vDot = GAIN_THROTTLE*(T*n)/self.MASS - self.GRAVITY*e3;
        
        RDot = R.dot(skew(w))
        RDot = numpy.reshape(RDot,9)

        # collecting derivatives
        derivatives = numpy.concatenate([pDot,vDot,RDot])
          
        return derivatives
