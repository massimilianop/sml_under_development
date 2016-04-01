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

from VectorThrustController2 import Vector_Thrust_Controller

from numpy import cos as c
from numpy import sin as s

import timeit


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
x_t  = concatenate([x0,v0,n0,w0])

# g0  = numpy.array([-0.2,0.1,9.81])
# g1  = numpy.array([0.3,0.2,0.1])
# g2  = numpy.array([0.1,0.2,0.3])
# gravity = concatenate([g0,g1,g2])

def gravity_function(t):
	g0 = numpy.array([ c(t), s(t),9.0])
	g1 = numpy.array([-s(t), c(t),0.0])
	g2 = numpy.array([-c(t),-s(t),0.0])
	return numpy.concatenate([g0,g1,g2])

gravity = gravity_function(0)

t0 = 0
y0 = x_t

start = timeit.default_timer()
Controller = Vector_Thrust_Controller(gravity_function)
out = Controller.output(x_t,gravity)
stop = timeit.default_timer()
print "time = " + str(stop - start) 

numpy.set_printoptions(precision=15)
for i in range(6):
    print(out[i])

delta_t = 0.001

print '\n'
start = timeit.default_timer()
x_t_dt = Controller.predict_x(x_t,t0,delta_t)
stop = timeit.default_timer()
print "time = " + str(stop - start) 
print '\n'

numpy.set_printoptions(precision=15)
print "x(t) = " + str(x_t)
print "x(t + Dt) = " + str(x_t_dt) 
print x_t_dt[3:6]
print (x_t_dt[0:3] - x_t[0:3])/delta_t



x_t_2dt = Controller.predict_x(x_t,t0,2*delta_t)
numpy.set_printoptions(precision=15)
print out[0]*x_t[6:9] - gravity[0:3]
print (x_t_2dt[0:3] - 2.0*x_t_dt[0:3] + x_t[0:3])/delta_t**2




