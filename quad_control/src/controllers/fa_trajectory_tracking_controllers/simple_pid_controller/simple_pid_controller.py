#!/usr/bin/env python
# this line is just used to define the type of document
""" This is a dynamic controller, not a static controller """

import numpy

from controllers import controller

from utilities import utility_functions

import rospy

class SimplePIDController(controller.Controller):

    
    @classmethod
    def contained_objects(cls):
        return {}
    
    
    @classmethod
    def description(cls):
        return "PID Controller, with saturation on integral part"


    def __init__(self,              \
        proportional_gain_xy = 1.0, \
        derivative_gain_xy   = 1.0, \
        integral_gain_xy     = 0.0, \
        bound_integral_xy    = 0.0, \
        proportional_gain_z  = 1.0, \
        derivative_gain_z    = 1.0, \
        integral_gain_z      = 0.5, \
        bound_integral_z     = 0.0,
        quad_mass            = rospy.get_param("quadrotor_mass",1.442)
        ):

        self.__proportional_gain_xy = proportional_gain_xy
        self.__derivative_gain_xy   = derivative_gain_xy
        self.__integral_gain_xy     = integral_gain_xy
        self.__bound_integral_xy    = bound_integral_xy
        self.__proportional_gain_z  = proportional_gain_z
        self.__derivative_gain_z    = derivative_gain_z
        self.__integral_gain_z      = integral_gain_z
        self.__bound_integral_z     = bound_integral_z
        self.__quad_mass            = quad_mass

        #TODO get from utilities?
        self.MASS = quad_mass
        self.GRAVITY = 9.81

        self.disturbance_estimate   = numpy.array([0.0,0.0,0.0])
        self.t_old  = 0.0
        
        
        
    def __str__(self):
        #TODO add all the parameters and the state
        string = controller.Controller.__str__(self)
        string += "\nProportional gain xy: " + str(self.__proportional_gain_xy)
        string += "\nDerivative gain xy: " + str(self.__derivative_gain_xy)
        return string


    def output(self, delta_t, state, reference):

        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])        
        
        #--------------------------------------#
        # position and velocity
        x  = state[0:3]
        v  = state[3:6]
        # thrust unit vector and its angular velocity
        # R  = state[6:15]; R  = numpy.reshape(R,(3,3))

        #--------------------------------------#
        # desired quad trajectory
        xd = reference[0:3]
        vd = reference[3:6]
        ad = reference[6:9]
        
        #--------------------------------------#
        # position error and velocity error
        ep = x - xd
        ev = v - vd

        u, V_v = self.input_and_gradient_of_lyapunov(ep,ev)

        Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3 - self.disturbance_estimate)

        # -----------------------------------------------------------------------------#
        # update disturbance estimate

        gains_integral_action    = numpy.array([self.__integral_gain_xy ,self.__integral_gain_xy ,self.__integral_gain_z ])
        max_disturbance_estimate = numpy.array([self.__bound_integral_xy,self.__bound_integral_xy,self.__bound_integral_z])

        # derivatice of disturbance estimate (ELEMENT WISE PRODUCT)
        

        d_est_dot  = gains_integral_action*V_v
        # new disturbance estimate
        t_new = delta_t
        disturbance_estimate = self.disturbance_estimate + d_est_dot*(t_new - self.t_old) 
        # saturate estimate just for safety (element wise bound)
        self.disturbance_estimate = utility_functions.bound(disturbance_estimate,max_disturbance_estimate,-1.0*max_disturbance_estimate)
        # update old time
        self.t_old = t_new
        # -----------------------------------------------------------------------------#

        return Full_actuation


    def input_and_gradient_of_lyapunov(self,ep,ev):

        u    = numpy.array([0.0,0.0,0.0])
        V_v  = numpy.array([0.0,0.0,0.0])

        kp     = self.__proportional_gain_xy
        kv     = self.__derivative_gain_xy
        u[0]   = -kp*ep[0] - kv*ev[0]
        u[1]   = -kp*ep[1] - kv*ev[1]
        V_v[0] = (kp/2*ep[0] + ev[0])
        V_v[1] = (kp/2*ep[1] + ev[1])

        kp     = self.__proportional_gain_z
        kv     = self.__derivative_gain_z
        u[2]   = -kp*ep[2] - kv*ev[2]
        V_v[2] = (kp/2*ep[2] + ev[2])

        return u, V_v


    def reset_estimate_xy(self):
        self.disturbance_estimate[0] = 0.0
        self.disturbance_estimate[1] = 0.0
        return


    def reset_estimate_z(self):
        self.disturbance_estimate[2] = 0.0
        return         

    
# Test
#string = SimpleBoundedIntegralPIDController.to_string()
#print string
#con = SimpleBoundedIntegralPIDController.from_string(string)
#print con
#print con.output(0.1, numpy.zeros(9), numpy.zeros(9))
