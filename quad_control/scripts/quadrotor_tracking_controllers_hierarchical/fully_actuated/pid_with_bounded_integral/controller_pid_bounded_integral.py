#!/usr/bin/env python
# this line is just used to define the type of document
"""This is a dynamic controller, not a static controller"""

# in case we want to use rospy.logwarn or logerror
import rospy

import numpy

import collections


from systems_functions.Double_Integrator_Functions.Double_Integrator_Bounded_Not_Component_wise_No_Inertial_Measurements_needed.DI_Bounded_2 import DI_controller

from ... import controller

from utilities import utility_functions


class ControllerPIDBoundedIntegral(controller.TrackingController):

    
    @classmethod
    def description(cls):
        return "PID Controller, with saturation on integral part"
    
    @classmethod
    def parameters_to_string(cls,   \
        integral_gain_xy     = 0.0, \
        bound_integral_xy    = 0.0, \
        integral_gain_z      = 0.5, \
        bound_integral_z    = 0.0):


        return json.dumps(dic)    
        
    @classmethod
    def string_to_parameters(cls, string):
        dic = json.loads(string)
        proportional_gain_xy = dic['proportional_gain_xy']
        derivative_gain_xy   = dic['derivative_gain_xy']
        integral_gain_xy     = dic['integral_gain_xy']
        bound_integral_xy    = dic['bound_integral_xy']
        proportional_gain_z  = dic['proportional_gain_z']
        derivative_gain_z    = dic['derivative_gain_z']
        integral_gain_z      = dic['integral_gain_z']
        bound_integral_z     = dic['bound_integral_z']
        return proportional_gain_xy, derivative_gain_xy, integral_gain_xy, bound_integral_xy, proportional_gain_z, derivative_gain_z, integral_gain_z , bound_integral_z

    def __init__(self,              \
        proportional_gain_xy = 1.0, \
        derivative_gain_xy   = 1.0, \
        integral_gain_xy     = 0.0, \
        bound_integral_xy    = 0.0, \
        proportional_gain_z  = 1.0, \
        derivative_gain_z    = 1.0, \
        integral_gain_z      = 0.5, \
        bound_integral_z     = 0.0):

        self.__proportional_gain_xy = proportional_gain_xy
        self.__derivative_gain_xy   = derivative_gain_xy
        self.__integral_gain_xy     = integral_gain_xy
        self.__bound_integral_xy    = bound_integral_xy
        self.__proportional_gain_z  = proportional_gain_z
        self.__derivative_gain_z    = derivative_gain_z
        self.__integral_gain_z      = integral_gain_z
        self.__bound_integral_z     = bound_integral_z


        self.MASS    = 1.66779

        self.GRAVITY = 9.81

        self.disturbance_estimate   = numpy.array([0.0,0.0,0.0])

        self.t_old  = 0.0

        pass
        
        
    def __str__(self):
        return self.description()


    def output(self, delta_t, state, reference):

        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])        
        
        #--------------------------------------#
        # position and velocity
        x  = state[0:3]; v  = state[3:6]

        #--------------------------------------#
        # desired quad trajectory
        xd = reference[0:3];
        vd = reference[3:6];
        ad = reference[6:9];
        
        #--------------------------------------#
        # position error and velocity error
        ep = x - xd
        ev = v - vd

        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.DI_Ctrll.output(ep,ev)

        Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3 - self.d_est)


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

        return Full_actuation


    def reset_disturbance_estimate(self):
        self.disturbance_estimate = numpy.array([0.0,0.0,0.0])
        return
            
