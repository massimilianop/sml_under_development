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

# from VectorThrustController import Vector_Thrust_Controller

import DI_Bounded_1
import DI_Bounded_3

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
    # VT_Ctrll = Vector_Thrust_Controller()
    
    DI_z  = DI_Bounded_1.DI_controller()
    DI_xy = DI_Bounded_3.DI_controller()

    # The class "constructor" - It's actually an initializer
    # def __init__(self):
    #   self.M = 1.1

    def output(self,state,stated):

        return self._control_law(state,stated)


    def _control_law(self,state,stated):

        # masses and cable length
        m   = self.m
        M   = self.M
        L   = self.L

        # gravity
        g   = self.g

        e3  = numpy.array([0.0,0.0,1.0])


        pM  = state[0:3]   # current position
        vM  = state[3:6]   # current velocity
        pm  = state[6:9]   # current direction of thrust
        vm  = state[9:12]  # current angular velocity

        pd = stated[0:3]    # desired position
        vd = stated[3:6]    # desired velocity
        ad = stated[6:9]    # desired acceleration
        jd = stated[9:12]  # desired jerk
        sd = stated[12:15]  # desired snap
        uud = stated[15:18] # desired snap

        ##
        pd_z = pd[2]
        vd_z = vd[2]
        ad_z = ad[2]
        jd_z = jd[2]

        p_z  = pM[2]
        v_z  = vM[2]

        u_z,u_p_z,u_v_z,u_p_p_z,u_v_v_z,u_p_v_z,V_z,VD_z,V_p_z,V_v_z,V_v_p_z,V_v_v_z = self.DI_z.output(p_z - pd_z, v_z - vd_z)


        n = (pm - pM)/numpy.linalg.norm(pm - pM)
        w  = numpy.dot(skew(n),(vm - vM)/numpy.linalg.norm(pM - pm))

        U_n     = 1.0/numpy.dot(e3,n)*(m + M)*(g + ad_z + u_z) - m*L*numpy.dot(w,w)


        norm_0dot = numpy.linalg.norm(g*e3 + ad)     
        nd_0dot   = (g*e3 + ad)/norm_0dot
        norm_1dot = numpy.dot(nd_0dot,jd)     
        nd_1dot   = jd/norm_0dot - (g*e3 + ad)/norm_0dot**2*norm_1dot
        norm_2dot = numpy.dot(nd_1dot,jd) + numpy.dot(nd_0dot,sd)     
        nd_2dot   = sd/norm_0dot - (g*e3 + ad)/norm_0dot**2*norm_2dot + \
                                   2.0*(g*e3 + ad)/norm_0dot**3*norm_1dot**2
        norm_3dot = numpy.dot(nd_2dot,jd) + 2.0*numpy.dot(nd_1dot,sd) + numpy.dot(nd_0dot,uud)
        nd_3dot   = uud/norm_0dot - (g*e3 + ad)/norm_0dot**2*norm_3dot + \
                                   6.0*(g*e3 + ad)/norm_0dot**3*norm_1dot*norm_2dot - \
                                   6.0*(g*e3 + ad)/norm_0dot**4*norm_1dot**3

             
        pd_quad = pd + nd_0dot*L
        vd_quad = vd + nd_1dot*L
        ad_quad = ad + nd_2dot*L
        jd_quad = jd + nd_3dot*L


        pd_xy = pd_quad[0:2]
        vd_xy = vd_quad[0:2]
        ad_xy = ad_quad[0:2]
        jd_xy = jd_quad[0:2]

        p_quad_xy  = pm[0:2]
        v_quad_xy  = vm[0:2]

        u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v = self.DI_xy.output(p_quad_xy - pd_xy,v_quad_xy - vd_xy)
        u_horizontal = ad_xy + u


        e3       = numpy.array([0.0,0.0,1.0])
        Tension  = M/(m+M)*(U_n + m*L*numpy.dot(w,w))
        uuu_horizontal = numpy.array([u_horizontal[0],u_horizontal[1],0])
        U_n_perp = -1/numpy.dot(e3,n)*numpy.dot(skew(n),numpy.dot(skew(e3),-n*(U_n - Tension) + m*uuu_horizontal))     

        U        = n*U_n + U_n_perp

        u_z_dot          = u_p_z*(v_z - vd_z) + u_v_z*u_z
        u_horizontal_dot = jd_xy + numpy.dot(u_p,v_quad_xy - vd_xy) + numpy.dot(u_v,u)

        w_dot   = numpy.dot(skew(n),U/(m*L)) 
        U_n_dot = -1.0/numpy.dot(e3,n)**2*numpy.dot(e3,numpy.dot(skew(w),n))*(m + M)*(g + ad_z + u_z) + \
                   1.0/numpy.dot(e3,n)*(m + M)*(jd_z + u_z_dot) \
                  -2.0*m*L*numpy.dot(w,w_dot)

        # Tension  = M/(m+M)*(U_n + m*L*(w'*w))
        Tension_dot        = M/(m+M)*(U_n_dot + 2.0*m*L*numpy.dot(w,w_dot))
        uuu_horizontal_dot = numpy.array([u_horizontal_dot[0],u_horizontal_dot[1],0])
        U_n_perp_dot =  1.0/numpy.dot(e3,n)**2*numpy.dot(e3,numpy.dot(skew(w),n))*numpy.dot(skew(n),numpy.dot(skew(e3),-n*(U_n - Tension) + m*uuu_horizontal)) \
                       -1.0/numpy.dot(e3,n)*numpy.dot(skew(numpy.dot(skew(w),n)),numpy.dot(skew(e3),-n*(U_n - Tension) + m*uuu_horizontal))  \
                       -1.0/numpy.dot(e3,n)*numpy.dot(skew(n),numpy.dot(skew(e3),\
                            -numpy.dot(skew(w),n)*(U_n - Tension) \
                            -n*(U_n_dot - Tension_dot) \
                            +m*uuu_horizontal_dot))

        U_dot = numpy.dot(skew(w),n)*U_n + n*U_n_dot + U_n_perp_dot

        # Lyapunov check
        V  = V_z + V
        VD = VD_z + VD

        return (U,U_dot,V,VD)