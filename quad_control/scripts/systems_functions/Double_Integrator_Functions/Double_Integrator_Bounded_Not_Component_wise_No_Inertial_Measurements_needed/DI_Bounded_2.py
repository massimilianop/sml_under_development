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
from numpy import outer as outer

class DI_controller(object): 

    wn      = 1.0
    xi      = sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 1.0
    kp      = wn**2
    sigma_p = 1.0  
    eps     = 0.01

    # The class "constructor" - It's actually an initializer
    def __init__(self,parameters = None):
        if parameters is not None:
            self.kv      = parameters.kv
            self.sigma_v = parameters.sigma_v
            self.kp      = parameters.kp
            self.sigma_p = parameters.sigma_p
            self.eps     = parameters.eps

    def output(self,p,v):
        return self._DI_Bounded(p,v)

    def report(self):
        description = "Bounded Double Integrator Controller u(p,v) = -sigma(p) - ro(v): simple control law, complicated Lyapunov function\n"
        controller  = "Parameters: sat(x) = k x/sqrt(1 + x**2), with sigma(p) = kp*sat(p/sigma_p) and ro(p) = kv*sat(v/sigma_v)\n"  
        parameters  = "kp = " + str(self.kp) + " and kv = " + str(self.kv) + " and sigma_p = " +  str(self.sigma_p) + "and sigma_v = " +  str(self.sigma_v) + "\n\n"
        return description + controller + parameters


    def _sat(self,x):

        sat     =  1.0/sqrt(1.0 + x**2)
        Dsat    =  -x*(1.0 + x**2)**(-3.0/2.0)
        D2sat   =  (-1.0 + 2.0*x**2)*(1.0 + x**2)**(-5.0/2.0)
        # primitive of sat(x)*x
        sat_Int =  sqrt(1.0 + x**2) - 1.0

        return (sat,Dsat,D2sat,sat_Int)

    # print sat(2.0)

    def  _DI_Bounded(self,p,v):

        # gains
        kp = self.kp
        kv = self.kv

        sigma_p  = self.sigma_p
        sigma_v  = self.sigma_v

        eps = self.eps 

        pp  = numpy.linalg.norm(p)
        vv  = numpy.linalg.norm(v)

        sat_p,Dsat_p,D2sat_p,sat_Int_p = self._sat(pp/sigma_p)
        sat_v,Dsat_v,D2sat_v,sat_Int_v = self._sat(vv/sigma_v)       

        I  = numpy.identity(3) 

        # vector
        h1     = kp*sat_p*p
        if pp >= eps:
            # matrix
            h1_p   = kp*Dsat_p/sigma_p*outer(p,p)/pp + kp*sat_p*I
            # tensor
            h1_p_p = zeros((3,3,3));
            for i in range(3):
                ee = I[:,i];
                h1_p_p[:,:,i] = kp*D2sat_p/sigma_p**2*outer(p,p)/(pp**2)*dot(p,ee)       + \
                                kp*Dsat_p/sigma_p*dot(p,ee)/pp*I                         + \
                                kp*Dsat_p/sigma_p*outer(p,ee)/pp                         + \
                                (-1)*kp*Dsat_p/sigma_p*outer(p,p)/pp**3*dot(p,ee)        + \
                                kp*Dsat_p/sigma_p*outer(ee,p)/pp
        else:
            h1_p          = kp*I
            h1_p_p = zeros((3,3,3))

        # vector
        h2     = kv*sat_v*v
        if vv >= eps: 
            # matrix
            h2_v   = kv*Dsat_v/sigma_v*outer(v,v)/vv + kv*sat_v*I
            # tensor
            h2_v_v = zeros((3,3,3))
            for i in range(3):
                ee = I[:,i]
                h2_v_v[:,:,i] = kv*D2sat_v/sigma_v**2*outer(v,v)/(vv**2)*dot(v,ee)    + \
                                kv*Dsat_v/sigma_v*dot(v,ee)/vv*I                      + \
                                kv*Dsat_v/sigma_v*outer(v,ee)/vv                      + \
                                (-1)*kv*Dsat_v/sigma_v*outer(v,v)/(vv**3)*dot(v,ee)   + \
                                kv*Dsat_v/sigma_v*outer(ee,v)/vv
        else:
            h2_v   = kv*I
            h2_v_v = zeros((3,3,3))


        # vector
        u     = -h1 - h2;

        # matrix
        u_p   = -h1_p;
        # tensor
        u_p_p = -h1_p_p;

        # matrix
        u_v   = - h2_v;
        # tensor
        u_v_v = - h2_v_v;

        u_p_v = zeros((3,3,3));


        beta   = 1.0/(2.0*kp);
        h1_int = kp*sigma_p**2*sat_Int_p;

        # this part is not really necessary
        if pp > eps and vv > eps:
            V  = beta*kv**2*h1_int     + \
                 beta*dot(h1,h2)       + \
                 1.0/2.0*vv**2         + \
                 h1_int                + \
                 beta*1.0/2.0*(kv**2*dot(v,v) - dot(h2,h2))

            VD = (-1)*(\
                       beta*dot(h1,h1)*kv*sat_v                                                        + \
                       dot(v,h2)*beta*kp**2*pp**2*sat_p**2*dot(p/pp,v/vv)**2*(kv*Dsat_v/sigma_v)/(kv*sat_v*vv)   + \
                       dot(v,h2)*(1.0 - beta*kp*(sat_p + Dsat_p/sigma_p*pp*dot(p/pp,v/vv)**2))                   + \
                       beta*dot(v,h2)*kv**2*(1.0 - sat_v*(Dsat_v/sigma_v*vv + sat_v))   \
                      )
        else:
            V  = 0
            VD = 0

        # V_v         = dV/d(v)
        # V_v_p = d/d(p) [dV/d(v)]
        # V_v_v = d/d(v) [dV/d(v)]

        V_p   = beta*kv**2*h1 + beta*dot(h1_p.T,h2) + h1

        V_v   = beta*dot(h2_v.T,h1) + v + beta*(kv**2*v - dot(h2_v.T,h2))

        V_v_p = beta*dot(h2_v.T,h1_p)

        V_v_v = beta*dot(h2_v_v,h1).T                                 + \
                I                                                     + \
                beta*(kv**2*I - dot(h2_v_v,h2).T  - dot(h2_v.T,h2_v))
  

        return (u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v)
