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

from VectorThrustController import Vector_Thrust_Controller

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
    M = 0.100
    # cable length
    L = 0.6

    # gravity
    g = 9.81

    # PAR = collections.namedtuple('VT_paramteres',['...','...'])
    # par = PAR(...,...)
    # VT_Ctrll = Vector_Thrust_Controller()
    
    VT_Ctrll = Vector_Thrust_Controller()
    
    
    # The class "constructor" - It's actually an initializer
    # def __init__(self):
    #   self.M = 1.1

    def output(self,state,stated):

        x,gravity = self._state_transform(state,stated)
        Thrust,Tau,V,VD,V_dT,V_dTau,derivatives,Thrust_dot,Tau_dot = self.VT_Ctrll.output(x,gravity)
        U = self._input_transform(x,Thrust,Tau)

        return U

    def output2(self,state,stated):

        x,gravity = self._state_transform(state,stated)
        Thrust,Tau,V,VD,V_dT,V_dTau,derivatives,Thrust_dot,Tau_dot = self.VT_Ctrll.output(x,gravity)
        U_t_0dt = self._input_transform(x,Thrust,Tau)

        U, U_1dot = self._input_transform2(x,Thrust,Tau,Thrust_dot,Tau_dot)

        delta_t = 0.001

        x_t_1dt = x + derivatives*delta_t
        x_t_1dt[6:9] = x_t_1dt[6:9]/numpy.linalg.norm(x_t_1dt[6:9])
        Thrust,Tau,V,VD,V_dT,V_dTau,derivatives,Thrust_dot,Tau_dot = self.VT_Ctrll.output(x_t_1dt,gravity)
        U_t_1dt = self._input_transform(x_t_1dt,Thrust,Tau)

        x_t_2dt = x_t_1dt + derivatives*delta_t
        x_t_2dt[6:9] = x_t_2dt[6:9]/numpy.linalg.norm(x_t_2dt[6:9])
        Thrust,Tau,V,VD,V_dT,V_dTau,derivatives,Thrust_dot,Tau_dot = self.VT_Ctrll.output(x_t_2dt,gravity)
        U_t_2dt = self._input_transform(x_t_2dt,Thrust,Tau)        

        U_0dot   = U_t_0dt        
        #U_1dot  = (U_t_1dt - U_t_0dt)/delta_t
        U_2dot  = (U_t_2dt - 2.0*U_t_1dt + U_t_0dt)/(delta_t**2)

        return (U_0dot,U_1dot,U_2dot)


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
        self.x = x

        #---------------------------------------------------#
        # DESIRED LOAD acceleration, jerk and snap
        ad = stated[6:9]   
        jd = stated[9:12] 
        sd = stated[12:15] 
        gravity = concatenate([g*e3 + ad,jd,sd])

        return (x,gravity)

    def _input_transform2(self,state,Thrust,Tau,Thrust_dot,Tau_dot):

        # masses and cable length
        m   = self.m;
        M   = self.M;
        L   = self.L;
        
        n = state[6:9]
        w = state[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + Tau*m*L

        nDot = numpy.dot(skew(w),n)
        wDot = numpy.dot(skew(n),Tau)
        U_dot = nDot*(Thrust*(m+M) - dot(w,w)*m*L)         + \
               n*(Thrust_dot*(m+M) - 2.0*dot(w,wDot)*m*L) + \
               Tau_dot*m*L

        print U_dot
        return U,U_dot

    def _input_transform(self,state,Thrust,Tau):

        # masses and cable length
        m   = self.m;
        M   = self.M;
        L   = self.L;
        
        n = state[6:9]
        w = state[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + Tau*m*L

        return U
