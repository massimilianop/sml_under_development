#!/usr/bin/env python
# this line is just used to define the type of document

import numpy

def skew(x):
    out = numpy.zeros((3,3))
    out[0,1] = -x[2]
    out[0,2] =  x[1]
    out[1,2] = -x[0]
    out[1,0] =  x[2]
    out[2,0] = -x[1]
    out[2,1] =  x[0]
    return out

def unskew(X):
    out = numpy.zeros(3)
    out[0] = -X[1,2]
    out[1] =  X[0,2]
    out[2] = -X[0,1]
    return out


# testing skew matrix    
# print skew(numpy.array([1,2,3]))

def quaternion_to_rot(quaternion):

    q   = quaternion
    q_v = q[0:3] 
    q_n = q[3]
    qc  = numpy.concatenate([-q_v,[q_n]])

    R  = numpy.dot(q,qc)*numpy.identity(3) + 2*q_n*skew(q_v) + 2*numpy.outer(q_v,q_v)

    return R


def quaternion_from_unit_vector(unit_vector,psi):

    r3 = unit_vector
    r1 = numpy.array([numpy.cos(psi),numpy.sin(psi),0.0])
    r1 = numpy.dot(numpy.identity(3) - numpy.outer(r3,r3),r1)
    r1 = r1/numpy.linalg.norm(r1)
    r2 = numpy.dot(skew(r3),r1)

    RR = column_stack((r1,r2,r3))

    return rot_to_quaternion(RR)


def quaternion_from_unit_vector(unit_vector,psi):

    r3 = unit_vector
    r1 = numpy.array([numpy.cos(psi),numpy.sin(psi),0.0])
    r1 = numpy.dot(numpy.identity(3) - numpy.outer(r3,r3),r1)
    r1 = r1/numpy.linalg.norm(r1)
    r2 = numpy.dot(skew(r3),r1)

    RR = column_stack((r1,r2,r3))

    return rot_to_quaternion(RR)    


def bound(x,maxmax,minmin):

    return numpy.maximum(minmin,numpy.minimum(maxmax,x))


def Rz(tt):
    
    return numpy.array([[numpy.cos(tt),-numpy.sin(tt),0.0],[numpy.sin(tt),numpy.cos(tt),0.0],[0.0,0.0,1]])


def roll_pitch(Full_actuation,psi):

    #--------------------------------------#
    # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des

    # desired roll and pitch angles
    n_des     = Full_actuation/numpy.linalg.norm(Full_actuation)
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
#--------------------------------------------------------------------------#
# For computing velocity from position measurements

class Median_Filter():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.data = numpy.zeros(N)
    
    def update_data(self,new_data):
        N = self.N
        self.data[:-1] = self.data[1:]
        self.data[-1]  = new_data

    def output(self):
        return numpy.median(self.data)

    def up_and_out(self,new_data):
        self.update_data(new_data)
        return self.output()

class Median_Filter_3D():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.Dx =  Median_Filter(N)
        self.Dy =  Median_Filter(N)
        self.Dz =  Median_Filter(N)

    def up_and_out(self,new_data):
        Dx_new = self.Dx.up_and_out(new_data[0])
        Dy_new = self.Dy.up_and_out(new_data[1])
        Dz_new = self.Dz.up_and_out(new_data[2])
        return numpy.array([Dx_new,Dy_new,Dz_new])


class Velocity_Filter():
    def __init__(self,N,old_position,old_time):
        self.median_filter = Median_Filter_3D(N)
        self.old_position = old_position
        self.old_time = old_time

    def out(self,new_position,new_time):
        dt = new_time - self.old_time
        vel_estimate =  (new_position - self.old_position)/dt
        self.old_position = new_position
        self.old_time = new_time
        out = self.median_filter.up_and_out(vel_estimate)

        return out


#--------------------------------------------------------------------------#
#--------------------------------------------------------------------------#

