#!/usr/bin/env python
# this line is just used to define the type of document

import numpy


# ----------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------- #
# Load parameters

# cable length meters
cable_length = 0.6 
# load mass kilograms
load_mass = 0.100
# gravity (meters/sec/sec)
gravity = 9.81

# ----------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------- #

# Default vehicle parameters for Asctec Firefly.
kDefaultRotor0Angle  =  0.52359877559
kDefaultRotor1Angle  =  1.57079632679
kDefaultRotor2Angle  =  2.61799387799
kDefaultRotor3Angle  = -2.61799387799
kDefaultRotor4Angle  = -1.57079632679
kDefaultRotor5Angle  = -0.52359877559
kDefaultRotorAngle   = numpy.array([kDefaultRotor0Angle,kDefaultRotor1Angle,kDefaultRotor2Angle,kDefaultRotor3Angle,kDefaultRotor4Angle,kDefaultRotor5Angle])

kDefaultRotorDirection = numpy.array([1.0,-1.0,1.0,-1.0,1.0,-1.0])

# Default vehicle parameters for Asctec Firefly.
kDefaultMass      = 1.56779
kDefaultArmLength = 0.215
kDefaultInertiaXx = 0.0347563
kDefaultInertiaYy = 0.0458929   
kDefaultInertiaZz = 0.0977
kDefaultRotorForceConstant  = 8.54858e-6
kDefaultRotorMomentConstant = 1.6e-2

K = numpy.diag([kDefaultArmLength*kDefaultRotorForceConstant,          \
                kDefaultArmLength*kDefaultRotorForceConstant,          \
                kDefaultRotorMomentConstant*kDefaultRotorForceConstant,\
                kDefaultRotorForceConstant])

K_inv = numpy.diag([1.0/(kDefaultArmLength*kDefaultRotorForceConstant),          \
                    1.0/(kDefaultArmLength*kDefaultRotorForceConstant),          \
                    1.0/(kDefaultRotorMomentConstant*kDefaultRotorForceConstant),\
                    1.0/(kDefaultRotorForceConstant)])

A = numpy.zeros([4,6])
for i in range(6):
    A[0,i] =  numpy.sin(kDefaultRotorAngle[i])
    A[1,i] = -numpy.cos(kDefaultRotorAngle[i])
    A[2,i] = -kDefaultRotorDirection[i]
    A[3,i] =  1.0

A_inv = numpy.dot(A,A.T)
A_inv = numpy.linalg.inv(A_inv)
A_inv = numpy.dot(A.T,A_inv)

I     = numpy.diag([kDefaultInertiaXx,kDefaultInertiaYy,kDefaultInertiaZz,kDefaultMass])
I_inv = numpy.diag([1.0/(kDefaultInertiaXx),1.0/(kDefaultInertiaYy),1.0/(kDefaultInertiaZz),1.0/(kDefaultMass)])
J     = numpy.diag([kDefaultInertiaXx,kDefaultInertiaYy,kDefaultInertiaZz])

matrix_motor_speeds = numpy.dot(A_inv,K_inv)