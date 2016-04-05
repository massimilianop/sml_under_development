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


# Path hack: adding parent folder path to sys.path
import sys; import os
sys.path.insert(0, os.path.abspath('..'))

from Double_Integrator_Functions.Double_Integrator_Bounded_Not_Component_wise_No_Inertial_Measurements_needed.DI_Bounded_2 import DI_controller

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

    wn      = 1.0
    xi      = sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 1.0
    kp      = wn**2
    sigma_p = 1.0
    eps     = 0.01

    PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
    par = PAR(kv,sigma_v,kp,sigma_p,eps) 
    # print(par)
    
    DI_Ctrll = DI_controller(par)
    
    factor  = 1.0
    ktt     = 20.0
    ktt2    = factor*kp
    kw      = 20.0
    kw2     = factor*kv

    # The class "constructor" - It's actually an initializer
    def __init__(self,gravity_function_handle):
        # self.r  = ode(self.sys_dynamics).set_integrator('dopri5')
        self.r  = ode(self.sys_dynamics).set_integrator('dopri5', nsteps = '10')
        self.gravity_function = gravity_function_handle

    def output(self,x,gravity):
        return self._VectorThrustController(x,gravity)

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

    def sys_dynamics(self,t,y):

        # state
        # y = [p;v;n;w]
        p = y[0:3]
        v = y[3:6]
        n = y[6:9]
        w = y[9:12]

        gravity = self.gravity_function(t)

        Thrust,Tau,V,VD,V_dT,V_dTau = self._VectorThrustController(y,gravity)

        p_dot = v
        v_dot = Thrust*n - gravity[0:3]
        n_dot = dot(skew(w),n)
        w_dot = dot(skew(n),Tau)

        derivatives = concatenate([p_dot,v_dot,n_dot,w_dot])

        return derivatives

    def predict_x(self,x,t,dt):

        # solving differential equation
        self.r.set_initial_value(x, t)
        self.r.integrate(self.r.t + dt);
        x_t_dt = self.r.y
        print x_t_dt

        x_t_dt = x + self.sys_dynamics(t,x)*dt
        print x_t_dt

        return x_t_dt

    def _VectorThrustController(self,x,gravity):

        ad  = gravity[0:3]
        jd  = gravity[3:6]
        sd  = gravity[6:9]

        # state
        # x = [p;v;n;w]
        p = x[0:3]
        v = x[3:6]
        n = x[6:9]
        w = x[9:12]


        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.DI_Ctrll.output(p,v)

        Td   = ad + u

        Td_t = jd
        Td_p = u_p
        Td_v = u_v

        nTd      = Td/numpy.linalg.norm(Td)
        # normTd  = numpy.linalg.norm(g*e3 + ad + u)
        normTd   = numpy.linalg.norm(Td)
        normTd_t = dot(nTd,jd)
        normTd_p = dot(u_p.T,nTd)
        normTd_v = dot(u_v.T,nTd)

        # TESTED:  
        # normTdDot = normTd_t + normTd_p*v + normTd_v*(u - OP(n)*Td)
        # block.OutputPort(2).Data = [normTdnormTdDot]

        # TESTED:  
        # uDot = u_p*v + u_v*(u - OP(n)*Td)
        # block.OutputPort(2).Data = [u(1)uDot(1)]

        nTd   = Td/normTd
        nTd_t = dot(OP(nTd),jd/normTd)
        nTd_p = dot(OP(nTd),u_p/normTd)
        nTd_v = dot(OP(nTd),u_v/normTd)

        # TESTED:
        # nTdDot = nTd_t + nTd_p*v + nTd_v*(u - OP(n)*Td) 
        # block.OutputPort(2).Data = [nTd(1)nTdDot(1)]

        # normTd_t_t = nTd'*sd + nTd_t'*jd
        # normTd_p_p     = diag(u_p)*nTd_p + diag(nTd)*diag(u_p_p)
        # normTd_p_v     = diag(u_p)*nTd_v + diag(nTd)*diag(u_p_v)
        # normTd_v_p     = diag(u_v)*nTd_p + diag(nTd)*diag(u_p_v)
        # normTd_v_v     = diag(u_v)*nTd_v + diag(nTd)*diag(u_v_v)

        # nTd_p_p   = -(nTd_p*n' + nTd*nTd_p)*diag(u_p_p)/normTd + OP(nTd)*diag(u_p_p)/normTd - OP(nTd)*diag(u_p)/normTd**2*normTd_p
        # nTd_p_v   = OP(nTd)*diag(u_p)/normTd
        # nTd_v_p   = OP(nTd)*diag(u_v)/normTd
        # nTd_v_v   = OP(nTd)*diag(u_v)/normTd


        xi = 1 - dot(n,nTd)
        Vtt0,Vtt1,Vtt2 = self._Vtheta(xi)
        xi_t = -dot(n,nTd_t)
        xi_p = -dot(n,nTd_p)
        xi_v = -dot(n,nTd_v)
        xi_n = -nTd

        # TESTED: 
        # xiDot = xi_t + xi_p*v + xi_v*(u - OP(n)*Td) + xi_n*skew(w)*n 
        # block.OutputPort(2).Data = [xixiDot]

        # TESTED:  
        # block.OutputPort(2).Data = [Vtt1Vtt2*xiDot]


        aux_w_star   = jd + dot(u_p,v) + dot(u_v,(u - dot(OP(n),Td)))
        aux_w_star_t = sd - dot(u_v,dot(OP(n),Td_t))
        aux_w_star_p = dot(u_p_p.T,v).T + dot(u_v,u_p - dot(OP(n),Td_p)) + dot(u_p_v.T,u - dot(OP(n),Td)).T
        aux_w_star_v = u_p + dot(u_p_v.T,v).T + dot(u_v,u_v - dot(OP(n),Td_v))  + dot(u_v_v.T,u - dot(OP(n),Td)).T
        aux_w_star_n = u_v*dot(n,Td) + dot(u_v,outer(n,Td))

        # TESTED:  
        # aux_w_starDot = aux_w_star_t + aux_w_star_p*v + aux_w_star_v*(u - OP(n)*Td) + aux_w_star_n*skew(w)*n 
        # block.OutputPort(2).Data = [aux_w_star(1)aux_w_starDot(1)]

        w_star   = dot(skew(nTd),aux_w_star/normTd)

        w_star_t = dot(-skew(aux_w_star/normTd),nTd_t)               + \
                   dot(skew(nTd),aux_w_star_t/normTd)                + \
                   dot((-1)*skew(nTd),aux_w_star/normTd**2*normTd_t)
        w_star_p = dot(-skew(aux_w_star/normTd),nTd_p)               + \
                   dot(skew(nTd),aux_w_star_p/normTd)                + \
                   dot((-1)*skew(nTd),outer(aux_w_star,normTd_p)/normTd**2)
        w_star_v = dot(-skew(aux_w_star/normTd),nTd_v)               + \
                   dot(skew(nTd),aux_w_star_v/normTd)                + \
                   dot((-1)*skew(nTd),outer(aux_w_star,normTd_v)/normTd**2)
        w_star_n = dot(skew(nTd),aux_w_star_n/normTd)    
                  
        # TESTED:  
        # w_starDot = w_star_t + w_star_p*v + w_star_v*(u - OP(n)*Td) + w_star_n*skew(w)*n 
        # block.OutputPort(2).Data = [w_star(1)w_starDot(1)]

        # Thrust
        Thrust = dot(Td,n)

        # gains for angular control
        ktt2  = self.ktt2
        # desired angular velocity
        wd = ktt2*dot(skew(n),nTd)      + \
             w_star                     + \
             (-1)*dot(skew(n),V_v)*normTd*1/Vtt1 

        # aux1    = ktt2*skew(n)*nTd
        # aux1Dot = ktt2*skew(n)*nTd_t + ktt2*skew(n)*nTd_p*v +  ktt2*skew(n)*nTd_v*(u - OP(n)*Td) - ktt2*skew(nTd)*skew(w)*n
        # block.OutputPort(2).Data = [aux1(1)aux1Dot(1)] 

        # aux1    = skew(n)*V_v
        # aux1Dot = skew(n)*diag(V_v_p)*v +  skew(n)*diag(V_v_v)*(u - OP(n)*Td) - skew(V_v)*skew(w)*n
        # block.OutputPort(2).Data = [aux1(2)aux1Dot(2)] 

        wd_t = ktt2*dot(skew(n),nTd_t)                  + \
               w_star_t                                 + \
               (-1)*dot(skew(n),V_v)*1/Vtt1*normTd_t    + \
               dot(skew(n),V_v)*normTd*1/Vtt1**2*Vtt2*xi_t
        wd_p = ktt2*dot(skew(n),nTd_p)                              + \
               w_star_p                                             + \
               (-1)*dot(skew(n),normTd*1/Vtt1*V_v_p)                + \
               (-1)*dot(skew(n),outer(V_v,normTd_p)*1/Vtt1)         + \
               dot(skew(n),outer(V_v,xi_p)*normTd*1/Vtt1**2*Vtt2)
        wd_v = ktt2*dot(skew(n),nTd_v)                              + \
               w_star_v                                             + \
               (-1)*dot(skew(n),outer(V_v,normTd_v)*1/Vtt1)         + \
               (-1)*dot(skew(n),normTd*1/Vtt1*V_v_v)                + \
               dot(skew(n),outer(V_v,xi_v)*normTd*1/Vtt1**2*Vtt2)
        wd_n = -ktt2*skew(nTd)                          + \
               w_star_n                                 + \
               skew(V_v)*normTd*1/Vtt1                  + \
               dot(skew(n),outer(V_v,xi_n)*normTd*1/Vtt1**2*Vtt2)
              
        # TESTED:
        wdDot = wd_t + dot(wd_p,v) + dot(wd_v,(u - dot(OP(n),Td))) + dot(wd_n,dot(skew(w),n))
        # block.OutputPort(2).Data = [wd(1)wdDot(1)]
         
        kw   = self.kw
        kw2  = self.kw2
        ew   = dot(skew(n),w - wd)
        Tau  = dot(skew(n),-wdDot - 1/kw*Vtt1*dot(skew(n),nTd) - dot(skew(n),wd)*dot(n,wd)) + kw2*ew

        ## Lyapunov check
        V  = Vpv + Vtt0 + 1/2*kw*dot(ew,ew)
        VD = VpvD - ktt2*Vtt1*numpy.linalg.norm(dot(skew(n),nTd))**2 - kw2*kw*dot(ew,ew)

        V_dT   = dot(V_v - Vtt1*dot(nTd_v.T,n) + kw*dot(wd_v.T,dot(skew(n),ew)),n)
        V_dTau = -kw*dot(OP(n),ew)

        return (Thrust,Tau,V,VD,V_dT,V_dTau)
