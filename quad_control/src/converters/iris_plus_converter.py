#!/usr/bin/env python
# this line is just used to define the type of document
 
'''
    Purpose: converts a desired force (in NEWTONS) to IRIS+ standard

    __THROTTLE_NEUTRAL is the most critical parameter, that needs to be reseted when an experiment is commenced

'''

import numpy
import rospy
import utilities.utility_functions as uts



class IrisPlusConverter(object):

    # mass of IRIS+ (kg)
    __MASS    = rospy.get_param("mass_quad_ctr",1.442)

    # acceleration due to gravity (m/s/s)
    __GRAVITY = rospy.get_param("gravity_ctr",9.81)

    # Throttle neutral 
    __THROTTLE_NEUTRAL = rospy.get_param("throttle_neutral",1450)

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

    def set_mass(self,mass):
        self.__MASS = mass
        pass

    def get_k_throttle_neutral(self):
        return self.__THROTTLE_NEUTRAL

    def reset_k_trottle_neutral(self,force):
        # if force > m*gravity, decrease neutral value
        # if force < m*gravity, increase neutral value

        self.__THROTTLE_NEUTRAL = self.__THROTTLE_NEUTRAL*force/(self.__MASS*self.__GRAVITY)

        # for safety better bound this value
        self.__THROTTLE_NEUTRAL = numpy.clip(self.__THROTTLE_NEUTRAL,1400,1600)
        rospy.logwarn('new neutral value = ' + str(self.__THROTTLE_NEUTRAL) + ' in [1400,1600]')
        return 

    def set_k_trottle_neutral(self,throttle_neutral):
        # setting throttle value
        self.__THROTTLE_NEUTRAL = throttle_neutral
        # for safety better bound this value
        self.__THROTTLE_NEUTRAL = numpy.clip(self.__THROTTLE_NEUTRAL,1400,1600)
        rospy.logwarn('settting neutral value to = ' + str(self.__THROTTLE_NEUTRAL) + ' in [1400,1600]')
        return 

    def reset_parameters(self):
        return 

    def descriptive_message(self):
        # needs to be corrected
        return 'Converter for IRIS+ with parameters' + str(self.__THROTTLE_NEUTRAL)

    def set_rotation_matrix(self,euler_angles):
        # euler_angles must come in RADIANS
        self.euler_angles = euler_angles
        return 

    def input_conveter(self,desired_3d_force,yaw_rate_desired):

        #---------------------------------------------------------------------#
        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])
        rotation_matrix      = uts.rot_from_euler_rad(self.euler_angles)
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
        MAX_PSI_SPEED_Rad = MAX_PSI_SPEED_Deg*numpy.pi/180.0

        MAX_ANGLE_DEG = self.MAX_ANGLE_DEG
        MAX_ANGLE_RAD = MAX_ANGLE_DEG*numpy.pi/180.0

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
        U[2]  = Throttle*self.__THROTTLE_NEUTRAL/(self.__GRAVITY*self.__MASS);

        # need to bound between 1000 and 2000; element-wise operation
        U     = numpy.clip(U,1000,2000) 

        return U

    def roll_pitch(self,Full_actuation):

        psi = self.euler_angles[2]

        #--------------------------------------#
        # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des
        # desired roll and pitch angles
        n_des     = Full_actuation/numpy.linalg.norm(Full_actuation)
        n_des_rot = uts.rot_z(-psi).dot(n_des)


        sin_phi   = -n_des_rot[1]
        sin_phi   = numpy.clip(sin_phi,-1,1)
        phi       = numpy.arcsin(sin_phi)

        sin_theta = n_des_rot[0]/numpy.cos(phi)
        sin_theta = numpy.clip(sin_theta,-1,1)
        cos_theta = n_des_rot[2]/numpy.cos(phi)
        cos_theta = numpy.clip(cos_theta,-1,1)
        pitch     = numpy.arctan2(sin_theta,cos_theta)

        return (phi,pitch)
