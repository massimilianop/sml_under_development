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

wn      = 1
xi      = sqrt(2)/2.0
kv      = 2.0*xi*wn
sigma_v = 1.0
kp      = wn**2
sigma_p = 1.0
eps     = 0.01

PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
par = PAR(kv,sigma_v,kp,sigma_p,eps) 
# print(par)

Controller = DI_controller(par)              
# Controller = DI_controller()

p   = numpy.array([-0.53,-1.23,0.98])
v   = numpy.array([0.69,1.09,-0.79])
out = Controller._DI_Bounded(p,v)
numpy.set_printoptions(precision=15)

for i in range(12):
    # print("{:.15f}".format(out[i]))
    print(out[i])
    print('\n') 
