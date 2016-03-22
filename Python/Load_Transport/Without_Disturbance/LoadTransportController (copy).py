#!/usr/bin/python

import os

def cls():
    os.system(['clear','cls'][os.name == 'nt'])

# now, to clear the screen
cls()

# for ploting
import matplotlib.pyplot as plt

# for integrating
from scipy.integrate import ode

import numpy

from numpy import *

#--------------------------------------------------------------------------#
# Some functions

from numpy import cos as c
from numpy import sin as s

from numpy import sqrt  as sqrt
from numpy import zeros as zeros

from VectorThrustController2 import Vector_Thrust_Controller

import collections

def skew(x):
    out = numpy.zeros((3,3))
    out[0,1] = -x[2]
    out[0,2] =  x[1]
    out[1,2] = -x[0]
    out[1,0] =  x[2]
    out[2,0] = -x[1]
    out[2,1] =  x[0]
    return out


class Load_Transport_Controller(object): 

    # quadrotor mass
    m = 1.56779
    # load mass
    M = 0.250
    # cable length
    L = 0.6

    # gravity
    g = 9.81

    # PAR = collections.namedtuple('VT_paramteres',['...','...'])
    # par = PAR(...,...)
    # VT_Ctrll = Vector_Thrust_Controller()
    
    # VT_Ctrll = Vector_Thrust_Controller()
    
    
    # The class "constructor" - It's actually an initializer
    def __init__(self,gravity_function_handle):
        self.VT_Ctrll = Vector_Thrust_Controller(gravity_function_handle)

    def output(self,state,stated):

        x,gravity = self._state_transform(state,stated)
        Thrust,Tau,V,VD,V_dT,V_dTau = self.VT_Ctrll.output(x,gravity)
        print(Thrust)
        print(Tau)
        U = self._input_transform(Thrust,Tau)

        return U


    def _state_transform(self,state,stated):

        # masses and cable length
        m   = self.m;
        M   = self.M;
        L   = self.L;

        # gravity
        g   = self.g;

        e3  = numpy.array([0.0,0.0,1.0])

        # current LOAD position
        pM  = state[0:3]
        # current LOAD velocity
        vM  = state[3:6]
        # current QUAD position
        pm  = state[6:9]
        # current QUAD velocity
        vm  = state[9:12]

        # DESIRED LOAD position
        pd = stated[0:3]
        # DESIRED LOAD velocity
        vd = stated[3:6]

        # transformation of state
        p = pM - pd
        v = vM - vd

        # direction of cable
        n = (pm - pM)/numpy.linalg.norm(pm - pM)
        # angular velocity of cable
        w  = dot(skew(n),(vm - vM)/numpy.linalg.norm(pM - pm))

        x = concatenate([p,v,n,w])
        print x
        self.x = x

        #---------------------------------------------------#
        # DESIRED LOAD acceleration, jerk and snap
        ad = stated[6:9]   
        jd = stated[9:12] 
        sd = stated[12:15] 
        gravity = concatenate([g*e3 + ad,jd,sd])

        return (x,gravity)

    def _input_transform(self,Thrust,Tau):

        # masses and cable length
        m   = self.m
        M   = self.M
        L   = self.L
        
        n = self.x[6:9]
        w = self.x[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + Tau*m*L;

        return U

    def _input_transform2(self,x,Thrust,Tau):

        # masses and cable length
        m   = self.m
        M   = self.M
        L   = self.L
        
        n = x[6:9]
        w = x[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + Tau*m*L;

        return U


    def blabla():
        x_t_1dt = self.VT_Ctrll.predict_x(x_t,t,  delta_t)
        Thrust_t_1dt,Tau_t_1dt,V,VD,V_dT,V_dTau = self.VT_Ctrll.output(x_t_1dt,gravity_function(t + 1.0*delta_t))
        U_t_1dt = self._input_transform2(x_t_1dt,Thrust_t_1dt,Tau_t_1dt)

        x_t_2dt = self.VT_Ctrll.predict_x(x_t,t0,2*delta_t)
        Thrust_t_2dt,Tau_t_2dt,V,VD,V_dT,V_dTau = self.VT_Ctrll.output(x_t_2dt,gravity_function(t + 2.0*delta_t))
        U_t_2dt = self._input_transform2(x_t_2dt,Thrust_t_2dt,Tau_t_2dt)