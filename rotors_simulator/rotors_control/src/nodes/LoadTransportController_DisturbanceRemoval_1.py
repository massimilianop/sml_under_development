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
    m = 1.250
    # load mass
    M = 0.200
    # cable length
    L = 0.5

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
        Thrust,Tau,V,VD,V_dT,V_dTau = self.VT_Ctrll.output(x,gravity)
        U = self._input_transform(Thrust,Tau,V_dT,V_dTau)

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
        self.x = x

        #---------------------------------------------------#
        # DESIRED LOAD acceleration, jerk and snap
        ad = stated[6:9]   
        jd = stated[9:12] 
        sd = stated[12:15] 
        gravity = concatenate([g*e3 + ad,jd,sd])

        return (x,gravity)

    def _input_transform(self,Thrust,Tau,V_dT,V_dTau):

        # masses and cable length
        m   = self.m;
        M   = self.M;
        L   = self.L;
        
        n = self.x[6:9]
        w = self.x[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + Tau*m*L;

        k_Motor_est      = self.k_Motor_est
        k_Motor_est_Dot  = k_int*(V_dT*dot(n,U)/(m+M) + dot(V_dTau,U)/(m*L))/k_Motor_est
        k_Motor_est_Dotr = b_est_D(k_Motor_est,k_Motor_est_Dot)

        return U

    def b_est_D(self,b_est,b_est_Dot)

    
        # b_est: disturbance estimate
        # b_est_Dot: unbounded disturbance estimate time derivative

        # some parameters
        b       = self.b
        b_infty = self.b_infty
        delta   = self.delta
        b_eps   = self.b_eps
        n       = self.n

        aux = dot(b_est - b,b_est - b) - b_infty^2
        if aux > 0:
            n1 = aux**(n + 1.0)
        else:
            n1 = 0

        n2 = dot(b_est - b,b_est_Dot) + (dot(b_est - b,b_est_Dot)**2 + delta**2)**(0.5)

        # dynamics of bounded disturbance estimate
        out = b_est_Dot + \
              (-1)*n1*n2*(b_est - b)/(2*(b_eps**2+2*b_eps*b_infty)**(n + 1.0)*b_infty**2)

        return out
