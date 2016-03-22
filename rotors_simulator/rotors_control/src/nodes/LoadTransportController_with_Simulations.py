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

from LoadTransportController import Load_Transport_Controller

import collections


#--------------------------------------------------------------------------#
# Some functions

from numpy import cos as c
from numpy import sin as s

def Rx(tt):
    
    return numpy.array([[1.0,0.0,0.0],[0.0,c(tt),-s(tt)],[0.0,s(tt),c(tt)]])

# print Rx(60*3.14/180)

def Ry(tt):
    
    return numpy.array([[c(tt),0.0,s(tt)],[0.0,1,0.0],[-s(tt),0.0,c(tt)]])

# print Ry(60*3.14/180)

def Rz(tt):
    
    return numpy.array([[c(tt),-s(tt),0.0],[s(tt),c(tt),0.0],[0.0,0.0,1]])

# print Rz(60*3.14/180)

def skew(xx):
    
    x = xx[0]
    y = xx[1]
    z = xx[2]
    
    return numpy.array([[0,-z,y],[z,0,-x],[-y,x,0]])

# print skew([1,2,3])

#--------------------------------------------------------------------------#
# orthogonal projection operator
def OP(x):
    
    return -skew(x).dot(skew(x))

#print OP([1,2,3])
#print OP([1,0,0])

#--------------------------------------------------------------------------#
# unit vector
def unit_vec(psi,theta):

    e1  = numpy.array([1.0,0.0,0.0])
    aux = Rz(psi).dot(e1)
    aux = Ry(theta).dot(aux)

    return aux

#print unit_vec(45*3.14/180,0)
#print unit_vec(45*3.14/180,45*3.14/180)
#print unit_vec(0*3.14/180,-90*3.14/180)


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
    w = 2*3.14/10
    
    p = r*w**0*numpy.array([ c(w*t), s(w*t),0.0]);
    v = r*w**1*numpy.array([-s(w*t), c(w*t),0.0]);
    a = r*w**2*numpy.array([-c(w*t),-s(w*t),0.0]);
    j = r*w**3*numpy.array([ s(w*t),-c(w*t),0.0]);
    s = r*w**4*numpy.array([ c(w*t), s(w*t),0.0]);
    
    return concatenate([p,v,a,j,s])

#--------------------------------------------------------------------------#

# system dynamics

def sys_dynamics(states , U , parameters):
    
    # U = Full actuation vehicles
    
    # acceleration due to gravity (m/s^2)
    g  = parameters.g

    # transported mass (kg)
    M  = parameters.M

    # mass of vehicles (kg)
    m = parameters.m

    # cable lengths (m)
    L = parameters.L


    # states

    # transported mass: position and velocity
    xM = states[0:3];
    vM = states[3:6];

    # vehicle: position and velocity
    x  = states[6:9];
    v  = states[9:12];

    n = (x - xM)/numpy.linalg.norm(x - xM);

    T = dot(U,n)*M/(m + M) + dot(vM - v, vM - v)*m*M/(m+M)*1.0/L;

    # third canonical basis vector
    e3 = numpy.array([0.0,0.0,1.0])

    
    # acceleration of vehicle
    vD = (U - T*n)/m - g*e3;
      
    # acceleration of transported mass
    vMD = T*n/M - g*e3;
      
      
    # collecting derivatives
    derivatives = concatenate([vM,vMD,v,vD])
      
    return derivatives

#--------------------------------------------------------------------------#

M      = 0.200
m      = 1.250
L      = 0.5
g      = 9.81
Load_Ctroll = Load_Transport_Controller()
PAR = collections.namedtuple('DI_paramteres',['M','m','L','g','Controller'])
parameters = PAR(M,m,L,g,Load_Ctroll)

#--------------------------------------------------------------------------#
# initial states

# initial load position
xM0 = numpy.array([0.0,0.0,0.0])
# initial load velocity
vM0 = numpy.array([0.0,0.0,0.0])
# initial cable direction
n0  = unit_vec(0*3.14/180,-90*3.14/180)
# cable length
L   = parameters.L
# initial quad position
x0  = xM0 + L*n0;
# initial cable angular velocity
w0  = numpy.array([0,0,0])
# initial quad velocity
v0  = vM0 + L*skew(w0).dot(n0)

# collecting all initial states
states0  = concatenate([xM0,vM0,x0,v0])

# just for testing
#print states0
#U0 = numpy.array([0,0,8])
#print sys_dynamics(states0,U0,parameters)



#--------------------------------------------------------------------------#
# CLOSED LOOP SYSTEM: DYNAMICS + CONTROLLERS

def f(t, y, parameters):

    # desired position and velocity for LOAD
    yd = traj_des(t)

    U = parameters.Controller.output(y, yd)

    return sys_dynamics(y, U, parameters)

#--------------------------------------------------------------------------#
# solving differential equation
r  = ode(f).set_integrator('dopri5')
t0 = 0;
y0 = states0

#U = parameters.Controller.output(y0, traj_des(0))
#print(U)

r.set_initial_value(y0, t0).set_f_params(parameters)

t1 = 12
dt = 0.01
Time = [];
Yvalue = [];

while r.successful() and r.t < t1:
    r.integrate(r.t+dt);

    Time.append(r.t);
    # Yvalue.append(r.y[0])
    # Yvalue.append(r.y[8])

#    # norm of input
#    yd = traj_des(r.t)
#    U = controller(r.y, yd, parameters)
#    Yvalue.append(numpy.linalg.norm(U))


    # position LOAD
    xM    = r.y[0:3]
    # desired position LOAD
    des   = traj_des(r.t)
    xMd   = des[0:3]
    # error position LOAD
    error = numpy.linalg.norm(xM - xMd)
    Yvalue.append(error)

#    # norm of cable
#    xM = r.y[0:3]
#    x  = r.y[6:9]
#    Yvalue.append(numpy.linalg.norm(xM - x))

##    # input
##    U = parameters.Controller.output(r.y, traj_des(r.t))
##    Yvalue.append(U)


plt.plot(Time,Yvalue)
plt.ylabel('Trajectory Tracking error: norm(x - x_des)')
plt.show()


