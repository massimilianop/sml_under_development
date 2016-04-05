#!/usr/bin/python

import os

def cls():
    os.system(['clear','cls'][os.name == 'nt'])

# now, to clear the screen
cls()

# for ploting
import matplotlib.pyplot as plt

import numpy

from numpy import *

from LoadTransportController import Load_Transport_Controller

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
def Rx(tt):
    
    return numpy.array([[1.0,0.0,0.0],[0.0,c(tt),-s(tt)],[0.0,s(tt),c(tt)]])

# print Rx(60*3.14/180)

def Ry(tt):
    
    return numpy.array([[c(tt),0.0,s(tt)],[0.0,1,0.0],[-s(tt),0.0,c(tt)]])

# print Ry(60*3.14/180)

def Rz(tt):
    
    return numpy.array([[c(tt),-s(tt),0.0],[s(tt),c(tt),0.0],[0.0,0.0,1]])

#--------------------------------------------------------------------------#

def bound(x,maxmax,minmin):

    return numpy.maximum(minmin,numpy.minimum(maxmax,x))


#--------------------------------------------------------------------------#
# Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des

def euler_desired(U,psi):
    # desired roll and pitch angles
    n_des     = U/numpy.linalg.norm(U)
    n_des_rot = Rz(-psi).dot(n_des)

    sin_phi   = -n_des_rot[1]
    sin_phi   = bound(sin_phi,1,-1)
    phi       = numpy.arcsin(sin_phi)

    sin_theta = n_des_rot[0]/c(phi)
    sin_theta = bound(sin_theta,1,-1)
    cos_theta = n_des_rot[2]/c(phi)
    cos_theta = bound(cos_theta,1,-1)
    pitch     = numpy.arctan2(sin_theta,cos_theta)
    
    return (phi,pitch)

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

    r = 0.0
    w = 0.0
    
    p = r*w**0*numpy.array([ c(w*t), s(w*t),0.0]);
    v = r*w**1*numpy.array([-s(w*t), c(w*t),0.0]);
    a = r*w**2*numpy.array([-c(w*t),-s(w*t),0.0]);
    j = r*w**3*numpy.array([ s(w*t),-c(w*t),0.0]);
    s = r*w**4*numpy.array([ c(w*t), s(w*t),0.0]);
    
    p = p + numpy.array([0.0,0.0,0.01])

    return concatenate([p,v,a,j,s])

#--------------------------------------------------------------------------#

time = 0.0
# cable length
L = 0.6


#--------------------------------------------------------------------------#
print('Goal to the front')

# initial LOAD position
pL0 = numpy.array([-1.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.0,0.0,0.0])
# initial unit vector
n0 = numpy.array([0.0,s(0.0),c(0.0)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)

Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')

#--------------------------------------------------------------------------#
print('Goal to the right')

# initial LOAD position
pL0 = numpy.array([0.0,1.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.0,0.0,0.0])
# initial unit vector
n0 = numpy.array([0.0,s(0.0),c(0.0)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')

#--------------------------------------------------------------------------#
print('On Goal, but moving to the front (0.5)')

# initial LOAD position
pL0 = numpy.array([0.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.5,0.0,0.0])
# initial unit vector
n0 = numpy.array([0.0,s(0.0),c(0.0)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')

#--------------------------------------------------------------------------#
print('On Goal, but moving to the left (0.5)')

# initial LOAD position
pL0 = numpy.array([0.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.0,0.5,0.0])
# initial unit vector
n0 = numpy.array([0.0,s(0.0),c(0.0)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')

#--------------------------------------------------------------------------#
print('On Goal, but moving to the front (0.1)')

# initial LOAD position
pL0 = numpy.array([0.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.1,0.0,0.0])
# initial unit vector
n0 = numpy.array([0.0,s(0.0),c(0.0)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')

#--------------------------------------------------------------------------#
print('On Goal, but moving to the left (0.1)')

# initial LOAD position
pL0 = numpy.array([0.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.0,0.1,0.0])
# initial unit vector
n0 = numpy.array([0.0,s(0.0),c(0.0)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')



#--------------------------------------------------------------------------#
print('On Goal, but tilted to front (1deg)')

# initial LOAD position
pL0 = numpy.array([0.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.1,0.0,0.0])
# initial unit vector
n0 = numpy.array([s(1.0*3.14/180),0.0,c(1.0*3.14/180)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')

#--------------------------------------------------------------------------#
print('On Goal, but tilted to left (1deg)')

# initial LOAD position
pL0 = numpy.array([0.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.0,0.1,0.0])
# initial unit vector
n0 = numpy.array([0.0,s(1.0*3.14/180),c(1.0*3.14/180)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')

#--------------------------------------------------------------------------#
print('On Goal, but tilted to front (20deg)')

# initial LOAD position
pL0 = numpy.array([0.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.1,0.0,0.0])
# initial unit vector
n0 = numpy.array([s(20.0*3.14/180),0.0,c(20.0*3.14/180)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')

#--------------------------------------------------------------------------#
print('On Goal, but tilted to left (20deg)')

# initial LOAD position
pL0 = numpy.array([0.0,0.0,0.0])
# initial LOAD velocity
vL0 = numpy.array([0.0,0.1,0.0])
# initial unit vector
n0 = numpy.array([0.0,s(20.0*3.14/180),c(20.0*3.14/180)])
# initial QUAD position
p0 = pL0 + L*n0
# initial angular velocity
w0 = numpy.array([0.0,0.0,0.0])
# initial quad velocity
v0 = vL0 + L*dot(skew(w0),n0)


# collecting all initial states
states0  = concatenate([pL0,vL0,p0,v0])
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')


#--------------------------------------------------------------------------#
print('BLABLA')

states0 = numpy.array([-2.50742107e+00,   1.60555275e-01,   2.02860167e-01,  4.62115014e-01,   2.04519373e-02,   1.08888745e-03,  -2.50579451e+00,   1.58822321e-01,   8.02929860e-01,  4.61469878e-01,   2.06412752e-02,   6.01303456e-03])


# collecting all initial states
statesd0 = traj_des(time)


Controller = Load_Transport_Controller()
out = Controller.output(states0,statesd0)
print(out)

numpy.set_printoptions(precision=15)
# print(out)
out = euler_desired(out,0.0)
print('roll : ' + str(out[0]*180/3.14))
print('pitch: ' + str(out[1]*180/3.14))
print('\n')