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

# from ..Double_Integrator_Functions.Double_Integrator_Bounded_Not_Component_wise_No_Inertial_Measurements_needed.DI_Bounded_2 import DI_controller
# from ../Double_Integrator_Functions/Double_Integrator_Bounded_and_Component_wise/DI_Bounded_1 import DI_controller_3D


# Path hack.
# import sys; import os
# sys.path.insert(0, os.path.abspath('..'))
# print(sys.path)

# Relative path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from Double_Integrator_Functions.Double_Integrator_Bounded_and_Component_wise.DI_Bounded_1 import DI_controller


from Quadruple_Integrator_Functions.Quadruple_Integrator_Component_Wise.QI_Integrator import QI_controller


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

def OP(x):
    out = numpy.zeros((3,3))
    I   = numpy.identity(3)
    out = I - outer(x,x)
    return out

class Vector_Thrust_Controller(object): 

    wn      = 2.0
    xi      = sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 0.5
    kp      = wn**2
    sigma_p = 0.5
    eps     = 0.01

    PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
    par = PAR(kv,sigma_v,kp,sigma_p,eps)
    # print(par)
    
    DI_Ctrll = DI_controller(par)
    
    QI_Ctrll = QI_controller()

    # factor  = 1.0
    # ktt     = 20.0
    # ktt2    = factor*kp
    # kw      = 20.0
    # kw2     = factor*kv

    ktt     = 200.0
    ktt2    = 0.5
    kw      = 200.0
    kw2     = 0.5            
    
    # The class "constructor" - It's actually an initializer
    # def __init__(self):
    #   self.M = 1.1

    # The class "constructor" - It's actually an initializer
    def __init__(self,parameters = None):
        if parameters is not None:
            if parameters.parameters_di is not None:
                DI_Ctrll = DI_controller(parameters.parameters_di)


    def output(self,x,gravity):
        return self._VectorThrustController(x,gravity)

    def report(self):
        description   = "Vector Thrust Controller based on double integrator for z component, and quadruple integrator for x and y components \n\n"
        controller_z  = "Controller for z component\n" + self.DI_Ctrll.report()
        controller_xy = "Controller for xy components\n" + self.QI_Ctrll.report()
        return description + controller_z + controller_xy

    def _Vtheta(self,x):

        #     ee = self.ee
        #     ktt = self.ktt
        #     Vtheta0 =  ktt*x*(ee**2 - x**2)**(-1.0/2.0) 
        #     Vtheta1 =  ktt*ee**2*(ee**2 - x**2)^(-3.0/2.0)
        #     Vtheta2 =  ktt*3*x*ee**2*(ee**2 - x**2)**(-5.0/2.0)

        ktt   = self.ktt
        Vtheta0 = ktt*x 
        Vtheta1 = ktt
        Vtheta2 = 0

        return (Vtheta0,Vtheta1,Vtheta2)

    def _VectorThrustController(self,x,gravity):

        e3 = numpy.array([0.0,0.0,1.0])
        g_0t = gravity[0:3]
        g_1t = gravity[3:6]
        g_2t = gravity[6:9]

        # initialize gradient of Lyapunov w.r.t. state
        V_x = numpy.zeros(12)

        # state
        # x = [p;v;n;w]
        p = x[0:3]
        v = x[3:6]
        n = x[6:9]
        n3 = x[8]
        w = x[9:12]


        # control z direction with double integrator

        # z position
        z  = x[2]
        # z velocity
        vz = x[5]

        # self.DI_Ctrll.output(p,v)
        u_z,u_p_z,u_v_z,u_p_p_z,u_v_v_z,u_p_v_z,V_z,VD_z,V_p_z,V_v_z,V_v_p_z,V_v_v_z = self.DI_Ctrll.output(z,vz);
        
        V_x[2] = V_p_z
        V_x[5] = V_v_z

        Thrust_cl = 1.0/n3*(g_0t[2] + u_z)

        # finding quadruple integartor for xy direction

        # auxilar matrix 
        # (extracts first and second component from three dimentional vector)
        PP = numpy.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0]])

        xi1 = x[0:2]
        xi1_grad_x      = numpy.zeros((2,12))
        xi1_grad_x[0,0] = 1.0
        xi1_grad_x[1,1] = 1.0

        xi2 = x[3:5]
        xi2_grad_x = zeros((2,12))
        xi2_grad_x[0,3] = 1
        xi2_grad_x[1,4] = 1 

        xi3 = 1.0/n3*(u_z + g_0t[2])*n[0:2] - g_0t[0:2]
        # xi3 = 1.0/n3*u_z*n[0:2] + 1.0/n3*PP*skew(e3)*skew(n)*g_0t
        xi3_grad_x = numpy.zeros((2,12))
        xi3_grad_x[0:2,2]   =  1.0/n3*n[0:2]*u_p_z
        xi3_grad_x[0:2,5]   =  1.0/n3*n[0:2]*u_v_z
        xi3_grad_x[0:2,6:9] = -1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3),skew(n)))


        xi3_t =  1.0/n3*dot(PP,dot(skew(e3),dot(skew(n),g_1t)))
        xi3_x = -1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3),dot(OP(n),w))) + 1.0/n3*n[0:2]*(u_p_z*vz + u_v_z*u_z)
        xi4   = xi3_t + xi3_x
        xi4_grad_x = numpy.zeros((2,12))
        xi4_grad_x[0:2,2]     = -1.0/n3**2*u_p_z*dot(PP,dot(skew(e3),dot(OP(n),w))) + 1.0/n3*n[0:2]*(u_p_p_z*vz + u_p_v_z*u_z + u_v_z*u_p_z)
        xi4_grad_x[0:2,5]     = -1.0/n3**2*u_p_z*dot(PP,dot(skew(e3),dot(OP(n),w))) + 1.0/n3*n[0:2]*(u_p_v_z*vz + u_p_z + u_v_v_z*u_z + u_v_z*u_v_z)
        xi4_grad_x[0:2,6:9]   = -(1.0/n3*dot(PP,dot(skew(e3),skew(g_1t))) + 1.0/n3**2*dot(PP,dot(skew(e3),dot(skew(n),outer(g_1t,e3))))) + \
                                -1.0/n3**2*dot(PP,dot(skew(e3),skew(n)))*(u_p_z*vz + u_v_z*u_z) + \
                                -1.0/n3**2*(u_p_z*vz + u_v_z*u_z)*dot(PP,dot(skew(e3),skew(n))) + \
                                 2.0/n3**5*dot(e3,dot(skew(w),n))*(u_z + g_0t[2])*dot(PP,dot(skew(e3),skew(n))) + \
                                 1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3),dot(n,w)*numpy.identity(3) + outer(n,w)))
        xi4_grad_x[0:2,9:12] = -1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3),OP(n)))

        # 8 by 12
        xi_grad_x = numpy.concatenate([xi1_grad_x,xi2_grad_x,xi3_grad_x,xi4_grad_x])

        xi3_t_t =  1.0/n3*dot(PP,dot(skew(e3),dot(skew(n),g_2t)))
        xi3_t_x = -dot(1.0/n3*dot(PP,dot(skew(e3),skew(g_1t))) + 1.0/n3**2*dot(PP,dot(skew(e3),dot(skew(n),outer(g_1t,e3)))),dot(skew(w),n))
        xi3_x_t = -1.0/n3**2*g_1t[2]*dot(PP,dot(skew(e3),dot(OP(n),w)))
        xi3_x_x = -1.0/n3**2*dot(PP,dot(skew(e3),dot(OP(n),w)))*(u_p_z*vz + u_v_z*u_z) + \
                   1.0/n3*n[0:2]*((u_p_p_z*vz + u_p_v_z*u_z)*vz + (u_p_v_z*vz + u_v_v_z*u_z)*u_z + u_p_z*u_z + u_v_z*(u_p_z*vz + u_v_z*u_z)) + \
                  -1.0/n3**2*(u_p_z*vz + u_v_z*u_z)*dot(PP,dot(skew(e3),dot(OP(n),w))) + \
                   2/n3**5*dot(e3,dot(skew(w),n))*(u_z + g_0t[2])*dot(PP,dot(skew(e3),dot(OP(n),w))) + \
                   1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3), dot(outer(dot(skew(w),n),n) + outer(n,dot(skew(w),n)) ,w))) 
               
               
        xi5 = xi3_t_t + xi3_t_x + xi3_x_t + xi3_x_x

        # control xy directions with quadruple integrator 

        [u_quadr_int,V_xi,V_of_xi,VD_of_xi] = self.QI_Ctrll.output(xi1,xi2,xi3,xi4)

        # -1.0/n3**2*(u_z + g_0t[2])*PP*skew(e3)*skew(n)*tau
        Tau_cl = n3/(u_z + g_0t[2])*dot(OP(n),numpy.concatenate([u_quadr_int - xi5,[0]]))

        # V_xi  is 8 by 1
        V_x           = V_x + dot(V_xi,xi_grad_x)

        V  = V_z  + V_of_xi
        VD = VD_z + VD_of_xi

        # Thrust_cl = 0.0
        # Tau_cl    = numpy.zeros(3)
        # V         = 0
        # VD        = 0
        # V_x       = numpy.zeros(12)

        return (Thrust_cl,Tau_cl,V,VD,V_x,V_x)


