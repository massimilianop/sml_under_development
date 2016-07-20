#!/usr/bin/env python
# this line is just used to define the type of document

"""This module implements general utitlity functions for the project.
"""

# In conversion functions, use the THIS_from_THAT naming convention,
# so that when you call them, it looks like this = THIS_from_THAT(that).
# If we use the convention THIS_to_THAT,
# the call look like that = THIS_to_THAT(this), which is a little psychic.

import numpy as np

from numpy import cos as c
from numpy import sin as s



GRAVITY = 9.81
E3_VERSOR = np.array([0.0, 0.0, 1.0])


def skew(xx):
    """Skew matrix blabla"""
    x = xx[0]
    y = xx[1]
    z = xx[2]
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def unskew(X):
    out = np.zeros(3)
    out[0] = -X[1,2]
    out[1] =  X[0,2]
    out[2] = -X[0,1]
    return out    

# print skew([1,2,3])

#--------------------------------------------------------------------------#
# orthogonal projection operator
#TODO rename to projection_operator or something like that
# def OP(x):
#     return -skew(x).dot(skew(x))

def ort_proj(x):
    out = np.identity(3) - np.outer(x,x)
    return out

def OP(x):
    out = np.zeros((3,3))
    I   = np.identity(3)
    out = I - np.outer(x,x)
    return out

#print OP([1,2,3])
#print OP([1,0,0])


#TODO rename all the rotations descriptive names,
# not capitalized

def rot_x(tt):
    return np.array([[1.0,0.0,0.0],[0.0,c(tt),-s(tt)],[0.0,s(tt),c(tt)]])

# print Rx(60*3.14/180)

def rot_y(tt):
    return np.array([[c(tt),0.0,s(tt)],[0.0,1,0.0],[-s(tt),0.0,c(tt)]])

# print Ry(60*3.14/180)

def rot_z(tt):
    return np.array([[c(tt),-s(tt),0.0],[s(tt),c(tt),0.0],[0.0,0.0,1]])

# print Rz(60*3.14/180)

#--------------------------------------------------------------------------#
# unit vector
def unit_vector_from_euler_angles(psi, theta):

    e1  = np.array([1.0,0.0,0.0])
    aux = rot_z(psi).dot(e1)
    aux = rot_y(theta).dot(aux)

    return aux

#print unit_vec(45*3.14/180,0)
#print unit_vec(45*3.14/180,45*3.14/180)
#print unit_vec(0*3.14/180,-90*3.14/180)



#--------------------------------------------------------------------------

#TODO decapitalize: get_euler_angles(rot_max)
def euler_rad_from_rot(R):

    #phi   = atan2(R(3,2),R(3,3));
    #theta = asin(-R(3,1));
    #psi   = atan2(R(2,1),R(1,1));

    euler = np.array([0.0,0.0,0.0])

    euler[0] = np.arctan2(np.clip(R[2,1],-1,1),np.clip(R[2,2],-1,1));
    euler[1] = np.arcsin(-np.clip(R[2,0],-1,1));
    euler[2] = np.arctan2(np.clip(R[1,0],-1,1),np.clip(R[0,0],-1,1));    

    return euler


def euler_deg_from_rot(R):
    return euler_rad_from_rot(R)*180.0/np.pi


def rot_from_euler_rad(ee_rad):
    return rot_z(ee_rad[2]).dot(rot_y(ee_rad[1]).dot(rot_x(ee_rad[0])))


def rot_from_euler_deg(ee_deg):
    return rot_from_euler_rad(ee_deg*np.pi/180.0)

# test
#print(euler_deg_from_rot(rot_from_euler_deg(np.array([11,-21,-13]))))

# testing skew matrix    
# print skew(np.array([1,2,3]))

def rot_from_quaternion(quaternion):

    q   = quaternion
    q_v = q[0:3] 
    q_n = q[3]
    qc  = np.concatenate([-q_v,[q_n]])

    R  = np.dot(q,qc)*np.identity(3) + 2*q_n*skew(q_v) + 2*np.outer(q_v,q_v)

    return R

def quaternion_from_rot(rotation_matrix):

    q_n = np.sqrt(1 + np.trace(rotation_matrix))/2.0
    q_v = unskew(rotation_matrix - np.transpose(rotation_matrix))/(4*q_n)

    quaternion = np.concatenate([q_v,[q_n]])

    return quaternion

# test
# print(np.array([0.5*(0.5),0.5*(0.5*np.sqrt(3)),0.5*(0.0),0.5*np.sqrt(3)]))
# print(quaternion_from_rot(rot_from_quaternion(np.array([0.5*(0.5),0.5*(0.5*np.sqrt(3)),0.5*(0.0),0.5*np.sqrt(3)]))))

def quaternion_from_unit_vector_and_yaw_rad(unit_vector, psi):

    r3 = unit_vector
    r1 = np.array([np.cos(psi),np.sin(psi),0.0])
    r1 = np.dot(np.identity(3) - np.outer(r3,r3),r1)
    r1 = r1/np.linalg.norm(r1)
    r2 = np.dot(skew(r3),r1)

    RR = np.column_stack((r1,r2,r3))

    return quaternion_from_rot(RR)  



def roll_and_pitch_from_full_actuation_and_yaw_rad(full_actuation, psi):

    #--------------------------------------#
    # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des

    # desired roll and pitch angles
    norm = np.linalg.norm(full_actuation)
    # TODO: replace 0.1 by 10% of weight
    if norm > 0.1:
        n_des     = full_actuation/norm
        n_des_rot = rot_z(-psi).dot(n_des)
    else:
        n_des     = np.array([0.0,0.0,1.0])
        n_des_rot = rot_z(-psi).dot(n_des)        

    sin_phi   = -n_des_rot[1]
    sin_phi   = np.clip(sin_phi,-1,1)
    phi       = np.arcsin(sin_phi)

    sin_theta = n_des_rot[0]/np.cos(phi)
    sin_theta = np.clip(sin_theta,-1,1)
    cos_theta = n_des_rot[2]/np.cos(phi)
    cos_theta = np.clip(cos_theta,-1,1)
    pitch     = np.arctan2(sin_theta,cos_theta)

    return phi, pitch
    
#--------------------------------------------------------------------------#
#--------------------------------------------------------------------------#
# For computing velocity from position measurements


class MedianFilter:
    # N is order of median filter
    def __init__(self, N,data_initial = 0.0):
        self.N = N
        self.data = data_initial*np.ones(N)
    
    def update_data(self,new_data):
        N = self.N
        self.data[:-1] = self.data[1:]
        self.data[-1]  = new_data

    def output(self):
        return np.median(self.data)

    def up_and_out(self,new_data):
        self.update_data(new_data)
        return self.output()



class MedianFilter3D:
    # N is order of median filter
    def __init__(self, N,data_initial=np.zeros(3)):
        self.N = N
        self.Dx =  MedianFilter(N,data_initial[0])
        self.Dy =  MedianFilter(N,data_initial[1])
        self.Dz =  MedianFilter(N,data_initial[2])

    def up_and_out(self,new_data):
        Dx_new = self.Dx.up_and_out(new_data[0])
        Dy_new = self.Dy.up_and_out(new_data[1])
        Dz_new = self.Dz.up_and_out(new_data[2])
        return np.array([Dx_new,Dy_new,Dz_new])


class VelocityFilter:

    def __init__(self,N,old_position,old_time):
        self.median_filter = MedianFilter3D(N)
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

