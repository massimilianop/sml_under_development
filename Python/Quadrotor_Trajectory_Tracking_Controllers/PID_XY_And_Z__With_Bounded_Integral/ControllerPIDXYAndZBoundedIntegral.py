#!/usr/bin/env python
# this line is just used to define the type of document

# in case we want to use rospy.logwarn or logerror
import rospy

import numpy

import collections

# Relative path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..','..'))
from Double_Integrator_Functions.Double_Integrator_Bounded_and_Component_wise.DI_Bounded_1 import DI_controller


#--------------------------------------------------------------------------#
from numpy import cos as c
from numpy import sin as s
import math

# this function works for arrays
def bound(x,maxmax,minmin):
    return numpy.maximum(minmin,numpy.minimum(maxmax,x))

def Rx(tt):    
    return numpy.array([[1.0,0.0,0.0],[0.0,c(tt),-s(tt)],[0.0,s(tt),c(tt)]])

def Ry(tt):
    return numpy.array([[c(tt),0.0,s(tt)],[0.0,1,0.0],[-s(tt),0.0,c(tt)]])

def Rz(tt):    
    return numpy.array([[c(tt),-s(tt),0.0],[s(tt),c(tt),0.0],[0.0,0.0,1]])

def GetRotFromEulerAngles(ee_rad):
    return Rz(ee_rad[2]).dot(Ry(ee_rad[1]).dot(Rx(ee_rad[0])))

def GetRotFromEulerAnglesDeg(ee_deg):
    return GetRotFromEulerAngles(ee_deg*math.pi/180.0)

#--------------------------------------------------------------------------#
# This is a dynamic controller, not a static controller

class ControllerPIDXYAndZBounded():
    
    MASS    = 1.66779

    GRAVITY = 9.81

    # -----------------------------------------------------------------------------#

    wn      = 2.0
    xi      = numpy.sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 0.5
    kp      = wn**2
    sigma_p = 0.5
    eps     = 0.01

    PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
    par = PAR(kv,sigma_v,kp,sigma_p,eps) 
    # print(par)
    
    DI_Ctrll_xy = DI_controller(par)


    # -----------------------------------------------------------------------------#

    wn      = 2.0
    xi      = numpy.sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 0.5
    kp      = wn**2
    sigma_p = 0.5
    eps     = 0.01

    PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
    par = PAR(kv,sigma_v,kp,sigma_p,eps) 
    # print(par)
    
    DI_Ctrll_z = DI_controller(par)    


    # -----------------------------------------------------------------------------#
    # estimated disturbance
    d_est  = numpy.zeros(3)
    # initial time used for integration of disturbance dynamics
    t_old  = 0.0
    # Maximum of disturbance estimate
    MAX_DISTURBANCE_ESTIMATE = 1.0
    # gains of integral action
    GAINS_INTEGRAL_ACTION = numpy.array([0.1,0.1,1.0])
    # -----------------------------------------------------------------------------#

    def __init__(self,parameters = None):        

        if parameters != None:
            # update mass and throttle neutral
            self.parameters.kp_C1   = parameters[0]
            self.parameters.kv_C1   = parameters[1]
            self.parameters.ki_C1   = parameters[2]

            self.parameters.kp_z_C1 = parameters[3]
            self.parameters.kv_z_C1 = parameters[4]
            self.parameters.ki_z_C1 = parameters[5]

            # time is last parameter 
            self.t_old = parameters[7]   

    def reset_estimate_xy(self):
        self.d_est[0] = 0.0
        self.d_est[1] = 0.0
        return

    def reset_estimate_z(self):
        self.d_est[2] = 0.0
        return        

    def update_parameters(self,parameters):
        self.parameters = parameters

    def output(self,time,states,states_d):
        # convert euler angles from deg to rotation matrix
        ee = states[6:9]
        R  = GetRotFromEulerAnglesDeg(ee)
        R  = numpy.reshape(R,9)
        # collecting states
        states  = numpy.concatenate([states[0:6],R])
        return self.controller(time,states,states_d)     

    # Controller
    def controller(self,t_new,states,states_d):
         
        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])        
        
        #--------------------------------------#
        # position and velocity
        x  = states[0:3]; v  = states[3:6]
        # thrust unit vector and its angular velocity
        # R  = states[6:15]; R  = numpy.reshape(R,(3,3))

        #--------------------------------------#
        # desired quad trajectory
        xd = states_d[0:3];
        vd = states_d[3:6];
        ad = states_d[6:9];
        
        #--------------------------------------#
        # position error and velocity error
        ep = x - xd
        ev = v - vd

        u,V_v = self.input_and_gradient_of_lyapunov(ep,ev)

        Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3 - self.d_est)

        # -----------------------------------------------------------------------------#
        # update disturbance estimate

        # derivatice of disturbance estimate (ELEMENT WISE PRODUCT)
        d_est_dot  = self.GAINS_INTEGRAL_ACTION*V_v
        # new disturbance estimate
        self.d_est = self.d_est + d_est_dot*(t_new - self.t_old) 
        # element-wise division
        ratio      = self.d_est/self.MAX_DISTURBANCE_ESTIMATE
        # saturate estimate just for safety (element wise multiplication)
        self.d_est = self.MAX_DISTURBANCE_ESTIMATE*bound(ratio,1,-1)
        # update old time
        self.t_old = t_new
        # -----------------------------------------------------------------------------#

        return Full_actuation

    def input_and_gradient_of_lyapunov(self,ep,ev):

        u_x,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v_x,V_v_p,V_v_v = self.DI_Ctrll_xy.output(ep[0],ev[0])
        u_y,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v_y,V_v_p,V_v_v = self.DI_Ctrll_xy.output(ep[1],ev[1])
        u_z,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v_z,V_v_p,V_v_v = self.DI_Ctrll_z.output(ep[2],ev[2])

        u   = numpy.array([u_x,u_y,u_z])
        V_v = numpy.array([V_v_x,V_v_y,V_v_z])

        return (u,V_v)