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

from VectorThrustController import Vector_Thrust_Controller

from numpy import cos as c
from numpy import sin as s

# initial states

# initial position
x0 = numpy.array([-1.4,3.4,5.0])
# initial velocity
v0 = numpy.array([-4.0,-2.0,3.0])
# initial unit vector
n0 = numpy.array([0.0,s(0.3),c(0.3)])
# initial angular velocity
w0 = numpy.array([0.1,0.2,-0.1])


# collecting all initial states
states0  = concatenate([x0,v0,n0,w0])

g0  = numpy.array([-0.2,0.1,9.81])
g1  = numpy.array([0.3,0.2,0.1])
g2  = numpy.array([0.1,0.2,0.3])
gravity = concatenate([g0,g1,g2])

t0 = 0
y0 = states0

Controller = Vector_Thrust_Controller()
out = Controller.output(states0,gravity)
numpy.set_printoptions(precision=15)
for i in range(6):
    print(out[i])
