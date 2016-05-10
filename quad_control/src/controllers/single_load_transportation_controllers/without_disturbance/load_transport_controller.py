#!/usr/bin/python

# for ploting
import matplotlib.pyplot as plt

import numpy

from numpy import *

import rospy

from controllers.vector_thrust_controllers import vector_thrust_controllers_database


from .. import single_load_transportation_controller

def skew(x):
    out = numpy.zeros((3,3))
    out[0,1] = -x[2]
    out[0,2] =  x[1]
    out[1,2] = -x[0]
    out[1,0] =  x[2]
    out[2,0] = -x[1]
    out[2,1] =  x[0]
    return out


class SingleLoadTransportController(single_load_transportation_controller.SingleLoadTransportationController):   

    inner = {}
    # import dictionary with different vector thrusted controller classes
    inner["vector_thrust_controller"] = vector_thrust_controllers_database.database

    @classmethod
    def description(cls):
        return "Controller for a **single aerial vehicle transporating load** attached by cable"


    def __init__(self,   \
        vector_thrust_controller   = vector_thrust_controllers_database.database["Default"](),
        load_mass    = rospy.get_param("load_mass",0.1),
        quad_mass    = rospy.get_param("quadrotor_mass",1.442),
        cable_length = rospy.get_param("cable_length",0.6),
        ):
    # def __init__(self,   \
    #     load_mass    = rospy.get_param("load_mass",0.1),
    #     quad_mass    = rospy.get_param("quadrotor_mass",1.442),
    #     cable_length = rospy.get_param("cable_length",0.6),
    #     ):

        # self.vector_thrust_controller      = vector_thrust_controllers_database.database["Default"]()
        self.vector_thrust_controller = vector_thrust_controller

        self.quad_mass    = quad_mass
        self.load_mass    = load_mass
        self.cable_length = cable_length
        # TODO import this later from utilities
        self.g            = 9.81


    def output(self,time_instant,state,stated):

        x,gravity = self._state_transform(state,stated)
        Thrust,Tau,V,VD,V_dT,V_dTau = self.vector_thrust_controller.output(x,gravity)
        # print(Thrust)
        # print(Tau)
        U = self._input_transform(Thrust,Tau)

        return U

    def report(self):
        description = "Controller for Load Lifting\n"
        parameters  = "quad mass = " + str(self.quad_mass) + "(kg), load mas = " + str(self.load_mass) + "(kg), cable length = " + str(self.cable_length) + "(m), gravity = " + str(self.g) + "(m/s/s).\n\n"
        return description + parameters + self.VT_Ctrll.report()


    def _state_transform(self,state,stated):

        # masses and cable length
        m   = self.quad_mass;
        M   = self.load_mass;
        L   = self.cable_length;

        # gravity
        g   = self.g;

        e3  = numpy.array([0.0,0.0,1.0])

        # current LOAD position
        pM  = state[0:3]
        # current LOAD velocity
        vM  = state[3:6]
        # current QUAD position
        pm  = state[6:9]
        # current QUAD velocity
        vm  = state[9:12]

        # DESIRED LOAD position
        pd = stated[0:3]
        # DESIRED LOAD velocity
        vd = stated[3:6]

        # transformation of state
        p = pM - pd
        v = vM - vd

        # direction of cable
        n = (pm - pM)/numpy.linalg.norm(pm - pM)
        # angular velocity of cable
        w  = dot(skew(n),(vm - vM)/numpy.linalg.norm(pM - pm))

        x = concatenate([p,v,n,w])
        # print x
        self.x = x

        #---------------------------------------------------#
        # DESIRED LOAD acceleration, jerk and snap
        ad = stated[6:9]   
        jd = stated[9:12] 
        sd = stated[12:15] 
        gravity = concatenate([g*e3 + ad,jd,sd])

        return (x,gravity)

    def _input_transform(self,Thrust,Tau):

        # masses and cable length
        m   = self.quad_mass
        M   = self.load_mass
        L   = self.cable_length
        
        n = self.x[6:9]
        w = self.x[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + Tau*m*L

        return U
