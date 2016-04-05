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

from LoadTransportController_DisturbanceRemoval_2 import Load_Transport_Controller_Disturbance_Removal



#--------------------------------------------------------------------------#
from numpy import sqrt  as sqrt
from numpy import zeros as zeros

from numpy import cos as c
from numpy import sin as s


def skew(x):
    out = numpy.zeros((3,3))
    out[0,1] = -x[2]
    out[0,2] =  x[1]
    out[1,2] = -x[0]
    out[1,0] =  x[2]
    out[2,0] = -x[1]
    out[2,1] =  x[0]
    return out

#--------------------------------------------------------------------------#

# Desired trajectory for LOAD
def traj_des(t):
#    p = numpy.array([0.6,0.0,0.0]);
#    v = numpy.array([0.0,0.0,0.0]);
#    a = numpy.array([0.0,0.0,0.0]);
#    j = numpy.array([0.0,0.0,0.0]);
#    s = numpy.array([0.0,0.0,0.0]);

    from numpy import cos as c
    from numpy import sin as s

    r = 1.0
    w = 0.5
    
    p = r*w**0*numpy.array([ c(w*t), s(w*t),0.0]);
    v = r*w**1*numpy.array([-s(w*t), c(w*t),0.0]);
    a = r*w**2*numpy.array([-c(w*t),-s(w*t),0.0]);
    j = r*w**3*numpy.array([ s(w*t),-c(w*t),0.0]);
    s = r*w**4*numpy.array([ c(w*t), s(w*t),0.0]);
    
    return concatenate([p,v,a,j,s])

#--------------------------------------------------------------------------#

time = 0.4
# cable length
L = 0.5

# initial LOAD position
pL0 = numpy.array([-1.4,3.4,5.0])
# initial LOAD velocity
vL0 = numpy.array([-4.0,-2.0,3.0])
# initial unit vector
n0 = numpy.array([0.0,s(0.3),c(0.3)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.1,0.2,-0.1])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)

#--------------------------------------------------------------------------#
Load_Ctrll = Load_Transport_Controller_Disturbance_Removal()

out = Load_Ctrll.output(states0,statesd0)
numpy.set_printoptions(precision=15)
print(out)
print(Load_Ctrll.k_Motor_est)


