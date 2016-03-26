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

from DI_Bounded_2 import DI_controller

import collections

# -------------------------------------------------------- #
# -------------------------------------------------------- #

def sys_dynamics(states , U):
    
    # U = Full actuation: acceleration input

    # vehicle: position and velocity
    x  = states[0:3]
    v  = states[3:6]

    # acceleration of vehicle
    vDot = U
      
    # collecting derivatives
    derivatives = concatenate([v,vDot])
      
    return derivatives

# initial states
# initial position
x0 = numpy.array([-1.4,3.4,5.0])
# initial velocity
v0 = numpy.array([0.2,-0.1,0.3])

# collecting all initial states
states0  = concatenate([x0,v0])

# just for testing
#print states0
#U0 = numpy.array([0,0,8])
#print sys_dynamics(states0,U0,parameters)


#--------------------------------------------------------------------------#
# CLOSED LOOP SYSTEM: DYNAMICS + CONTROLLERS

def f(t, y, Controller):

    p = y[0:3]
    v = y[3:6]

    (u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v)  = Controller.output(p,v)

    return sys_dynamics(y, u)   

#--------------------------------------------------------------------------#
# solving differential equation

# setting controller
wn      = 1
xi      = sqrt(2)/2.0
kv      = 2.0*xi*wn
sigma_v = 1.0
kp      = wn**2
sigma_p = 1.0
eps     = 0.01

PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
par = PAR(kv,sigma_v,kp,sigma_p,eps) 
Controller = DI_controller(par)

# setting differetial equation
r = ode(f).set_integrator('dop853')
t0 = 0
y0 = states0
r.set_initial_value(y0, t0).set_f_params(Controller)

t1 = 1
dt = 0.001
Time = []
Pvalue = []
Vvalue = []


# k = 1 or k = 2
dV           = 0
V_integrated = 0
# k = 3
dUvalue        = zeros(3)
u_integrated   = zeros(3)
# k = 4
du_p           = zeros(9)
u_p_integrated = zeros((3,3))
# k = 5
du_v           = zeros(9)
u_v_integrated = zeros((3,3))
# k = 6
dV_v           = zeros(3)
V_v_integrated = zeros(3)


k = 1
# ii = 0 or 1 or 2 or ... or 8
ii = 0

while r.successful() and r.t < t1:

    # position
    p    = r.y[0:3]
    # velocity
    v    = r.y[3:6]    

    (u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v)  = Controller.output(p,v)

    if k == 1 or k == 2:
        dV = numpy.vstack((dV,V - V_integrated))
    if k == 3: 
        dUvalue = numpy.vstack((dUvalue,u - u_integrated))
    if k == 4: 
        du_p = numpy.vstack((du_p,numpy.reshape(u_p - u_p_integrated,9)))    
    if k == 5: 
        du_v = numpy.vstack((du_v,numpy.reshape(u_v - u_v_integrated,9))) 
    if k == 6:
        dV_v = numpy.vstack((dV_v,V_v - V_v_integrated)) 

    #-------------------------------------#

    r.integrate(r.t+dt)

    Time.append(r.t)

    if k == 1:
        V_integrated = V_integrated + VD*dt
    if k == 2:
        VDot         = dot(V_p,v) + dot(V_v,u)
        V_integrated = V_integrated + VDot*dt
    if k == 3:
        uDot         = dot(u_p,v) + dot(u_v,u)
        u_integrated = u_integrated + uDot*dt 
    if k == 4:
        u_pDot         = dot(u_p_p,v) + dot(u_p_v,u)
        u_p_integrated = u_p_integrated + u_pDot*dt 
    if k == 5:
        u_vDot         = dot(u_p_v,v) + dot(u_v_v,u)
        u_v_integrated = u_v_integrated + u_vDot*dt
    if k == 6:
        V_vDot         = dot(V_v_p,v) + dot(V_v_v,u)
        V_v_integrated = V_v_integrated + V_vDot*dt 

    Pvalue.append(p)
    Vvalue.append(v)


if k == 0:
    plt.plot(Time,Pvalue,Time,Vvalue)
    plt.ylabel('Position')
    plt.show()
if k == 1 or k == 2:
    dV = dV[1:,:]
    plt.plot(Time,dV)
    plt.ylabel('Check')
    plt.show()
if k == 3:
    dUvalue = dUvalue[1:,:]
    plt.plot(Time,dUvalue[:,ii])
    plt.ylabel('Check')
    plt.show()
if k == 4:
    du_p = du_p[1:,:]
    plt.plot(Time,du_p[:,ii])
    plt.ylabel('Check')
    plt.show()
if k == 5:
    du_v = du_v[1:,:]
    plt.plot(Time,du_v[:,ii])
    plt.ylabel('Check')
    plt.show()
if k == 6:
    dV_v = dV_v[1:,:]
    plt.plot(Time,dV_v[:,ii])
    plt.ylabel('Check')
    plt.show()

exit()
