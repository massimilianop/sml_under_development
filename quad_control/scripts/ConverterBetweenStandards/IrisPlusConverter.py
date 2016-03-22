#!/usr/bin/env python
# this line is just used to define the type of document
 
'''
    Purpose: converts a desired force (in NEWTONS) to IRIS+ standard

    THROTTLE_NEUTRAL is the most critical parameter, that needs to be reseted when an experiment is commenced

'''

import numpy
import rospy
from utility_functions import roll_pitch,bound,Rz,GetRotFromEulerAngles
# we use pi
import math

class IrisPlusConverter(object):

    # mass of IRIS+ (kg)
    MASS    = rospy.get_param("mass_quad_ctr",1.442)

    # acceleration due to gravity (m/s/s)
    GRAVITY = rospy.get_param("gravity_ctr",9.81)  

    # Throttle neutral 
    THROTTLE_NEUTRAL = 1430.0

    # The default of 4.5 commands a 200 deg/sec rate
    # of rotation when the yaw stick is held fully left or right.
    MAX_PSI_SPEED_Deg = 200.0 

    MAX_ANGLE_DEG = 45.0 

    # in RADIANS
    euler_angles = numpy.array([0.0,0.0,0.0])

    # """docstring for IrisPlusConverter"""
    # def __init__(self, arg):
    #     super(IrisPlusConverter, self).__init__()
    #     self.arg = arg

    def reset_parameters(self):
        return 

    def descriptive_message(self):
        # needs to be corrected
        return 'Converter for IRIS+ with parameters' + str(self.THROTTLE_NEUTRAL)

    def set_rotation_matrix(self,euler_angles):
        # euler_angles must come in RADIANS
        self.euler_angles = euler_angles
        return 

    def input_conveter(self,desired_3d_force,yaw_rate_desired):

        #---------------------------------------------------------------------#
        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])
        rotation_matrix      = GetRotFromEulerAngles(self.euler_angles)
        throttle_unit_vector = rotation_matrix.dot(e3) 

        # STABILIZE MODE:APM COPTER
        # The throttle sent to the motors is automatically adjusted based on the tilt angle
        # of the vehicle (i.e increased as the vehicle tilts over more) to reduce the 
        # compensation the pilot must fo as the vehicles attitude changes

        # computing desired Throttle, desired roll angle, pitch angle, and desired yaw rate
        Throttle = numpy.dot(desired_3d_force,throttle_unit_vector)
        # this decreases the throtle, which will be increased
        Throttle = Throttle*numpy.dot(throttle_unit_vector,e3)


        roll_desired,pitch_desired = self.roll_pitch(desired_3d_force)

        #---------------------------------------------------------------------#
        # degrees per second
        MAX_PSI_SPEED_Deg = self.MAX_PSI_SPEED_Deg
        MAX_PSI_SPEED_Rad = MAX_PSI_SPEED_Deg*math.pi/180.0

        MAX_ANGLE_DEG = self.MAX_ANGLE_DEG
        MAX_ANGLE_RAD = MAX_ANGLE_DEG*math.pi/180.0

        #---------------------------------------------------------------------#
        # input for IRIS+ comes in specific order
        # [U[0],U[1],U[2],U[3]] = [roll,pitch,throttle,yaw]
        U = numpy.zeros(4)

        #---------------------------------------------------------------------#
        # angles comand between 1000 and 2000 PWM

        # Roll
        U[0]  =  1500.0 + roll_desired*500.0/MAX_ANGLE_RAD;    
        # Pitch: 
        # ATTENTTION TO PITCH: WHEN STICK IS FORWARD PITCH,
        # pwm GOES TO 1000, AND PITCH IS POSITIVE 
        U[1]  =  1500.0 - pitch_desired*500.0/MAX_ANGLE_RAD;    

        # psi angular speed 
        U[3]  =  1500.0 - yaw_rate_desired*500.0/MAX_PSI_SPEED_Rad;    

        # REMARK: the throtle comes between 1000 and 2000 PWM
        U[2]  = Throttle*self.THROTTLE_NEUTRAL/(self.GRAVITY*self.MASS);

        # need to bound between 1000 and 2000; element-wise operation
        U     = bound(U,2000,1000) 

        return U

    def roll_pitch(self,Full_actuation):

        psi = self.euler_angles[2]

        #--------------------------------------#
        # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des

        # desired roll and pitch angles
        n_des     = Full_actuation/numpy.linalg.norm(Full_actuation)
        n_des_rot = Rz(-psi).dot(n_des)


        sin_phi   = -n_des_rot[1]
        sin_phi   = bound(sin_phi,1,-1)
        phi       = numpy.arcsin(sin_phi)

        sin_theta = n_des_rot[0]/numpy.cos(phi)
        sin_theta = bound(sin_theta,1,-1)
        cos_theta = n_des_rot[2]/numpy.cos(phi)
        cos_theta = bound(cos_theta,1,-1)
        pitch     = numpy.arctan2(sin_theta,cos_theta)

        return (phi,pitch)
