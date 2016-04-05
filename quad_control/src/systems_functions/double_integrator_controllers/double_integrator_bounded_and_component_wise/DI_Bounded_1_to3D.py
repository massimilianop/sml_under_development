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

from DI_Bounded_1 import DI_controller

import collections

class DI_controller_3D(object): 

    PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p'])

    # x and y DI controller
    wn      = 1
    xi      = sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 1.0
    kp      = wn**2
    sigma_p = 1.0

    par_xy = PAR(kv,sigma_v,kp,sigma_p) 
    DI_Ctrll_xy = DI_controller(par_xy)

    # z DI controller
    wn      = 1
    xi      = sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 1.0
    kp      = wn**2
    sigma_p = 1.0

    par_z = PAR(kv,sigma_v,kp,sigma_p) 
    DI_Ctrll_z = DI_controller(par_z)
    
    def output(self,p,v):
        return self._DI_Bounded_NOT_Component(p,v)

    def  _DI_Bounded_NOT_Component(self,p,v):

        u      = zeros(3)
        u_p    = zeros((3,3))
        u_v    = zeros((3,3))
        u_p_p  = zeros((3,3,3))
        u_v_v  = zeros((3,3,3))
        u_p_v  = zeros((3,3,3))

        # V     = zeros(1)
        # VD    = zeros(1)

        V_p   = zeros(3)
        V_v   = zeros(3)
        V_v_p = zeros((3,3))
        V_v_v = zeros((3,3))

        u[0],u_p[0,0],u_v[0,0],u_p_p[0,0,0],u_v_v[0,0,0],u_p_v[0,0,0],V0,VD0,V_p[0],V_v[0],V_v_p[0,0],V_v_v[0,0] =  \
            self.DI_Ctrll_xy.output(p[0],v[0])

        u[1],u_p[1,1],u_v[1,1],u_p_p[1,1,1],u_v_v[1,1,1],u_p_v[1,1,1],V1,VD1,V_p[1],V_v[1],V_v_p[1,1],V_v_v[1,1] =  \
            self.DI_Ctrll_xy.output(p[1],v[1])

        u[2],u_p[2,2],u_v[2,2],u_p_p[2,2,2],u_v_v[2,2,2],u_p_v[2,2,2],V2,VD2,V_p[2],V_v[2],V_v_p[2,2],V_v_v[2,2] =  \
            self.DI_Ctrll_z.output(p[2],v[2])

        V  = V0  + V1  + V2
        VD = VD0 + VD1 + VD2

        return (u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v)
