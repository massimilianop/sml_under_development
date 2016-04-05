#!/usr/bin/env python
# this is just to define file type

import numpy

import rospy

# we need some functions
from utility_functions import skew,GetEulerAngles,GetRotFromEulerAngles

# we use pi
import math


class SimulatorWithAttitudeInnerLoop(object):

    # acceleration due to gravity (m/s^2); this parameter is defined in the Launch file
    GRAVITY = rospy.get_param("gravity_sim",9.81)
    
    # mass of vehicles (kg)
    MASS = rospy.get_param("mass_quad_sim",1.442)

    # throttle that cancels weight
    THROTTLE_NEUTRAL = rospy.get_param("Throttle_neutral_sim",1484.0)

    #--------------------------------------------------------------#
    # Simulator 1
    MAX_ANGLE_DEG     = rospy.get_param("MAX_ANGLE_DEG",45.0)
    GAIN_INNER_LOOP   = rospy.get_param("ktt_inner_loop",10.0)

    # The default of 4.5 commands a 200 deg/sec rate of rotation when the yaw stick is held fully left or right.
    MAX_PSI_SPEED_DEG = rospy.get_param("MAX_PSI_SPEED_Deg",200.0)    
    
    def __init__(self,parameters = None):
        
        # if parameters == None:
        # stick with the initialized parameter

        if parameters != None:
            # update mass and throttle neutral
            MASS = parameters[0]
            THROTTLE_NEUTRAL = parameters[1]
            GAIN_INNER_LOOP = parameters[2]

    def update_parameters(self,parameters):
        self.parameters = parameters

    def output(self,t, y, U):
        # states = y
        # return sys_dynamics(states,states_d,self.parameters)
        UU = numpy.array([U.U0,U.U1,U.U2,U.U3])        
        return self.sys_dynamics(y,UU)

    def sys_dynamics(self, states, U):
        
        # U = Full actuation vehicles

        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])

        # states
        # transported mass: position and velocity
        x = states[0:3]; v = states[3:6];

        # thrust unit vector
        R  = states[6:15]; R  = numpy.reshape(R,(3,3)); n  = R.dot(e3)


        # ----------------------------------------------#
        # rearrage to proper order
        # [roll,pitch,throttle,yaw] in sticks
        # in model order is [throttle,roll,pitch,yaw]
        U_new    = numpy.zeros(4)
        U_new[0] = U[2]; U_new[1] = U[0]; U_new[2] = U[1]; U_new[3] = U[3]        
        U        = U_new

        # ----------------------------------------------#
        # This model would require more work
        # U[0:1] : as desired full actuation
        # Td      = U[0:3];
        # Tddot   = .... some derivator
        # Tdnorm  = numpy.linalg.norm(Td);
        # nTd     = Td/Tdnorm;
        # w_feedforward = skew(nTd).dot(TdDot)/numpy.linalg.norm(Td)
        # w = ktt*skew(n).dot(nTd) + wd_feedforward
        # ----------------------------------------------#

        # current  euler angles
        ee  = GetEulerAngles(R)
        # current psi
        psi = ee[2]

        # degrees per second
        MAX_PSI_SPEED_DEG = self.MAX_PSI_SPEED_DEG
        MAX_PSI_SPEED_RAD = MAX_PSI_SPEED_DEG*math.pi/180.0

        MAX_ANGLE_DEG = self.MAX_ANGLE_DEG
        MAX_ANGLE_RAD = MAX_ANGLE_DEG*math.pi/180.0

        # desired roll and pitch. U[1:3] : desired roll and pitch
        roll_des  =  (U[1] - 1500.0)*MAX_ANGLE_RAD/500.0;
        # ATTENTTION TO PITCH: WHEN STICK IS FORWARD PITCH, pwm GOES TO 1000, AND PITCH IS POSITIVE 
        pitch_des = -(U[2] - 1500)*MAX_ANGLE_RAD/500.0;
        # desired euler angles
        ee_des = numpy.array([roll_des,pitch_des,psi])

        # gain of inner loop for attitude control
        ktt = self.GAIN_INNER_LOOP

        nTd     = GetRotFromEulerAngles(ee_des).dot(e3);
        w       = ktt*skew(n).dot(nTd)
        RT      = numpy.transpose(R)
        w       = RT.dot(w)

        # U[3]: yaw angular speed
        w[2]    = -(U[3] - 1500.0)*MAX_PSI_SPEED_RAD/500.0

        # -----------------------------------------------------------------------------#
        # STABILIZE MODE:APM COPTER
        # The throttle sent to the motors is automatically adjusted based on the tilt angle
        # of the vehicle (i.e increased as the vehicle tilts over more) to reduce the 
        # compensation the pilot must fo as the vehicles attitude changes
        # -----------------------------------------------------------------------------#
        # Throttle = U[0]
        # this increases the actual throtle
        Throttle = U[0]/numpy.dot(n,e3)

        # velocity
        pDot = v
        # acceleration of quad
        GAIN_THROTTLE  = self.MASS*self.GRAVITY/self.THROTTLE_NEUTRAL
        vDot = GAIN_THROTTLE*(Throttle*n)/self.MASS - self.GRAVITY*e3;

        RDot = R.dot(skew(w))
        RDot = numpy.reshape(RDot,9)

        # collecting derivatives
        derivatives = numpy.concatenate([pDot,vDot,RDot])
          
        return derivatives
