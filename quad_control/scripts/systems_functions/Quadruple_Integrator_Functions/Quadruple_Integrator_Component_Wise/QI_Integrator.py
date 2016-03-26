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

class QI_controller(object):

    state_dimension = 2
    Id = numpy.identity(state_dimension)

    K  = numpy.array([-2.0, -6.0, -7.0, -4.0])
    KK = numpy.kron(K,Id)

    P = numpy.array([[53.0/20.0, 59.0/20.0, 31.0/20.0, 1.0/4.0],
                     [59.0/20.0, 243.0/40.0, 37.0/10.0, 23.0/40.0],
                     [31.0/20.0,37.0/10.0, 15.0/4.0, 3.0/5.0],
                     [1.0/4.0, 23.0/40.0, 3.0/5.0, 11.0/40.0]])
    PP  = numpy.kron(P,Id)

    # A = [0 1 0 0;
    #      0 0 1 0;
    #      0 0 0 1;
    #      K'     ];
    #  
    # P*A + A'*P = - I


    # The class "constructor" - It's actually an initializer
    def __init__(self,parameters = None):
        if parameters is not None:
            self.K = parameters.K
            self.P = parameters.P


    def output(self,x1,x2,x3,x4):
        return self._quadruple_integrator(x1,x2,x3,x4)

    def report(self):
        description = "controller for fourth order integrator: x^(4) = u(x^(0),x^(1),x^(2),x^(3)), where u = K x\n"
        parameters  = "Controller gain: K = " + str(self.K) + " and P is found for PA + A^T P = - I (P is important if gradient of Lyapunov is used)"
        return description + parameters + "\n\n"

    def  _quadruple_integrator(self,x1,x2,x3,x4):

        # state
        x   = numpy.concatenate([x1,x2,x3,x4])
        
        # control input
        u   = dot(self.KK,x)

        # gradient of Lyapunov
        V_x = dot(self.PP,x)

        V  = dot(x,dot(self.PP,x))
        VD = -dot(x,x)

        return (u,V_x,V,VD)



