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


# import sys
# sys.path.insert(0,r'home/pedrootao/Dropbox/KTH_Personal/SML_Python/Python/Load_Transport/Vector_Thrust_Controller')
# sys.path.append('home/perdootao/Dropbox/KTH_Personal/SML_Python/Python/Load_Transport/Vector_Thrust_Controller')
# sys.path.insert(0,r'/Vector_Thrust_Controller')

#export PYTHONPATH=$HOME/Dropbox/KTH_Personal/SML_Python/Python:$PYTHONPATH

# from ..Vector_Thrust_Controller.VectorThrustController import Vector_Thrust_Controller

# Path hack.
# import sys; import os
# sys.path.insert(0, os.path.abspath('../../'))
# print(sys.path)
# from Vector_Thrust_Controller.Vector_Thrust_Controller_Double_Integrator_and_Toque_Backstepping.VectorThrustController import Vector_Thrust_Controller
# from Vector_Thrust_Controller.Vector_Thrust_Controller_Quadruple_Integrator.VectorThrustController import Vector_Thrust_Controller


# Relative path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
# from Vector_Thrust_Controller.Vector_Thrust_Controller_Double_Integrator_and_Toque_Backstepping.VectorThrustController import Vector_Thrust_Controller
from Vector_Thrust_Controller.Vector_Thrust_Controller_Quadruple_Integrator.VectorThrustController import Vector_Thrust_Controller



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

    # # quadrotor mass
    # m = 1.56779
    # # load mass
    # M = 0.100
    # # cable length
    # L = 0.6

    # # gravity
    # g = 9.81

    M      = 0.200
    m      = 1.250
    L      = 0.5
    g      = 9.81    

    # PAR = collections.namedtuple('VT_paramteres',['...','...'])
    # par = PAR(...,...)
    # VT_Ctrll = Vector_Thrust_Controller()

    # ---------------------------------- #
    # Double Integrator Parameters
    wn      = 2.0
    xi      = sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 0.5
    kp      = wn**2
    sigma_p = 0.5
    eps     = 0.01

    PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
    parameters_di = PAR(kv,sigma_v,kp,sigma_p,eps) 
    
    # ---------------------------------- #
    # Parameters backstepping 
    ktt  = 2.0
    ktt2 = sqrt(2)/2.0
    kw   = 2.0*xi*wn
    kw2  = 0.5

    PAR = collections.namedtuple('paramteres_backstepping',['ktt','ktt2','kw','kw2'])
    parameters_backstepping = PAR(ktt,ktt2,kw,kw2)   

    # ---------------------------------- #
    PAR = collections.namedtuple('paramteres',['parameters_di','parameters_backstepping'])
    parameters = PAR(parameters_di,parameters_backstepping) 
    # print(parameters)
    # ---------------------------------- #

    
    VT_Ctrll = Vector_Thrust_Controller(parameters)
    
    
    # The class "constructor" - It's actually an initializer
    # def __init__(self):
    #   self.M = 1.1

    def output(self,state,stated):

        x,gravity = self._state_transform(state,stated)
        Thrust,Tau,V,VD,V_dT,V_dTau = self.VT_Ctrll.output(x,gravity)
        # print(Thrust)
        # print(Tau)
        U = self._input_transform(Thrust,Tau)

        return U

    def report(self):
        description = "Controller for Load Lifting\n"
        parameters  = "quad mass = " + str(self.m) + "(kg), load mas = " + str(self.M) + "(kg), cable length = " + str(self.L) + "(m), gravity = " + str(self.g) + "(m/s/s).\n\n"
        return description + parameters + self.VT_Ctrll.report()


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
        # print x
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
        m   = self.m;
        M   = self.M;
        L   = self.L;
        
        n = self.x[6:9]
        w = self.x[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + Tau*m*L;

        return U
