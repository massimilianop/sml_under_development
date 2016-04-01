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

from LoadTransportController import Load_Transport_Controller

import collections


#--------------------------------------------------------------------------#
def b_est_D(t,b_est,b_est_Dot,parameters):


    # b_est: disturbance estimate
    # b_est_Dot: unbounded disturbance estimate time derivative

    # some parameters
    b       = parameters.b
    b_infty = parameters.b_infty
    delta   = parameters.delta
    b_eps   = parameters.b_eps
    n       = parameters.n

    aux = dot(b_est - b,b_est - b) - b_infty**2
    if aux > 0:
        n1 = aux**(n + 1.0)
    else:
        n1 = 0

    n2 = dot(b_est - b,b_est_Dot) + (dot(b_est - b,b_est_Dot)**2 + delta**2)**(0.5)

    # dynamics of bounded disturbance estimate
    out = b_est_Dot + \
          (-1)*n1*n2*(b_est - b)/(2*(b_eps**2+2*b_eps*b_infty)**(n + 1.0)*b_infty**2)

    return out
#--------------------------------------------------------------------------#

class Load_Transport_Controller_Disturbance_Removal(object): 


    # PAR = collections.namedtuple('paramteres',['...','...'])
    # par = PAR(...,...)
    # VT_Ctrll = Vector_Thrust_Controller()
    
    Load_Ctrll = Load_Transport_Controller()

    # estimate of motor coefficient
    k_Motor_est = 0.85
    # gain for disturbance estimate 
    k_int = 0.005

    # parameters for disturbance removal
    PAR = collections.namedtuple('estimator_paramteres',['b','b_infty','delta','b_eps','n'])
    par = PAR(b = k_Motor_est, b_infty = 0.1, delta = 10, b_eps = 0.1, n = 1.0)

    # solving differential equation for disturbance estimate
    r  = ode(b_est_D).set_integrator('dopri5')
    b_est_Dot = numpy.zeros(3)
    r.set_initial_value(k_Motor_est,0.0).set_f_params(b_est_Dot,par)
    
    # dt = 1/FREQUENCY at which controller will be called
    dt = 0.01

    # The class "constructor" - It's actually an initializer
    # def __init__(self):
    #   self.M = 1.1

    def output(self,state,stated):
        # output full actuation and
        # update estimate of motor constant

        x,gravity = self.Load_Ctrll._state_transform(state,stated)
        Thrust,Tau,V,VD,V_dT,V_dTau = self.Load_Ctrll.VT_Ctrll.output(x,gravity)
        U = self.Load_Ctrll._input_transform(Thrust,Tau)
        U_to_send = U/self.k_Motor_est

        #-----------------------------------------------------------#
        # updating estimate for motor constant

        # masses and cable length
        m   = self.Load_Ctrll.m;
        M   = self.Load_Ctrll.M;
        L   = self.Load_Ctrll.L;

        # cable unit vector
        n = x[6:9]
        # cable angular velocity
        w = x[9:12]

        # estimate
        k_Motor_est      = self.k_Motor_est
        # gain for disturbance estimate dynamics
        k_int            = self.k_int
        # estimate update unconstrained
        k_Motor_est_Dot  = k_int*(V_dT*dot(n,U)/(m+M) + dot(V_dTau,U)/(m*L))/k_Motor_est

        self.r.set_f_params(k_Motor_est_Dot,self.par)
        self.r.integrate(self.r.t + self.dt)
        self.k_Motor_est = self.r.y

        #-----------------------------------------------------------#
        
        return U_to_send

