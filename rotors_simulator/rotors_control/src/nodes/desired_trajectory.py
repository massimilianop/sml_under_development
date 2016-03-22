#!/usr/bin/env python
# this line is just used to define the type of document

import numpy

from numpy import cos as c
from numpy import sin as s

def output(t):
    return traj_des_circle(t)

# Desired trajectory for LOAD
def traj_des_circle(t):


    r = 0.0
    w = 2*3.14/20.0

    pp = r*w**0*numpy.array([ c(w*t), s(w*t),0.0]);
    vv = r*w**1*numpy.array([-s(w*t), c(w*t),0.0]);
    aa = r*w**2*numpy.array([-c(w*t),-s(w*t),0.0]);
    jj = r*w**3*numpy.array([ s(w*t),-c(w*t),0.0]);
    ss = r*w**4*numpy.array([ c(w*t), s(w*t),0.0]);
    uu = r*w**5*numpy.array([-s(w*t), c(w*t),0.0]);

    pp = pp + numpy.array([0.0,0.0,0.01])


    return numpy.concatenate([pp,vv,aa,jj,ss,uu])


# # Desired trajectory for LOAD
# def traj_des(t):

#     if t <= 0.0:

#         r = 1.0
#         w = 2*3.14/20.0

#         pp = r*w**0*numpy.array([ c(w*t), s(w*t),0.0]);
#         vv = r*w**1*numpy.array([-s(w*t), c(w*t),0.0]);
#         aa = r*w**2*numpy.array([-c(w*t),-s(w*t),0.0]);
#         jj = r*w**3*numpy.array([ s(w*t),-c(w*t),0.0]);
#         ss = r*w**4*numpy.array([ c(w*t), s(w*t),0.0]);

#         pp = pp + numpy.array([0.0,0.0,0.01])

#     else:

#         pp = numpy.array([0.0,0.0,0.01]);
#         vv = numpy.array([0.0,0.0,0.0 ]);
#         aa = numpy.array([0.0,0.0,0.0 ]);
#         jj = numpy.array([0.0,0.0,0.0 ]);
#         ss = numpy.array([0.0,0.0,0.0 ]);

#     return numpy.concatenate([pp,vv,aa,jj,ss])

# # Desired trajectory for LOAD
# def traj_des(t):

#     T = 20.0
    
#     if t <= T/2:

#         Delta = 5.0
#         w     = 2*3.14/T

#         pp = numpy.array([ -0.5*Delta*(w**0)*(c(w*t) - 1.0),0.0,0.0]);
#         vv = numpy.array([  0.5*Delta*(w**1)*s(w*t)     ,0.0,0.0]);
#         aa = numpy.array([  0.5*Delta*(w**2)*c(w*t)     ,0.0,0.0]);
#         jj = numpy.array([ -0.5*Delta*(w**3)*s(w*t)     ,0.0,0.0]);
#         ss = numpy.array([ -0.5*Delta*(w**4)*c(w*t)     ,0.0,0.0]);

#         pp = pp + numpy.array([-Delta,0.0,0.01])
#         pp = pp + numpy.array([0.0,0.0,0.01])

#     else:

#         pp = numpy.array([0.0,0.0,0.0 ]);
#         vv = numpy.array([0.0,0.0,0.0 ]);
#         aa = numpy.array([0.0,0.0,0.0 ]);
#         jj = numpy.array([0.0,0.0,0.0 ]);
#         ss = numpy.array([0.0,0.0,0.0 ]);

#     return numpy.concatenate([pp,vv,aa,jj,ss])


# # Desired trajectory for LOAD
# def traj_des2(t,pp_real,vv_real,aa_real,jj_real,ss_real):


#     pp_final,vv_final,aa_final,jj_final,ss_final
#     if t <= 0.0:

#         r = 1.0
#         w = 2*3.14/10.0

#         pp = r*w**0*numpy.array([ c(w*t), s(w*t),0.0]);
#         vv = r*w**1*numpy.array([-s(w*t), c(w*t),0.0]);
#         aa = r*w**2*numpy.array([-c(w*t),-s(w*t),0.0]);
#         jj = r*w**3*numpy.array([ s(w*t),-c(w*t),0.0]);
#         ss = r*w**4*numpy.array([ c(w*t), s(w*t),0.0]);

#         pp = pp + numpy.array([0.0,0.0,0.01])

#     else:

#         pp = numpy.array([0.0,0.0,0.01]);
#         vv = numpy.array([0.0,0.0,0.0 ]);
#         aa = numpy.array([0.0,0.0,0.0 ]);
#         jj = numpy.array([0.0,0.0,0.0 ]);
#         ss = numpy.array([0.0,0.0,0.0 ]);

#     return numpy.concatenate([pp,vv,aa,jj,ss])