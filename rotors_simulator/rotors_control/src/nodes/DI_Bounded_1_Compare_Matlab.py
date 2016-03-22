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

from DI_Bounded_1 import DI_controller

import collections

wn      = 0.5
xi      = sqrt(2)/2.0
kv      = 2.0*xi*wn
sigma_v = 1.0
kp      = wn**2
sigma_p = 1.0

PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p'])
par = PAR(kv,sigma_v,kp,sigma_p) 
#print(par)

Controller = DI_controller(par)              
# Controller = DI_controller()

p   = -0.53
v   = 0.69
out = Controller._DI_Bounded_Component(p,v)
for i in range(12):
    print("{:.15f}".format(out[i]))

