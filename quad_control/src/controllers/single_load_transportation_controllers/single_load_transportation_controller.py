#!/usr/bin/env python
# this line is just used to define the type of document

# for ploting
import matplotlib.pyplot as plt

import numpy

from numpy import *

import rospy

from utilities import jsonable as js

import utilities.utility_functions as uts

class SingleLoadTransportationController(js.Jsonable):

    @classmethod
    def description(cls):
        return "Abstract **Single Load Transportation Controller**: controller for single aerial vehicle transportation a load attached by cable"

    #TODO do these have special parameters?
    def __init__(self):
        pass
    
    def output(self, position, velocity): 
        raise NotImplementedError()




def skew(x):
    out = numpy.zeros((3,3))
    out[0,1] = -x[2]
    out[0,2] =  x[1]
    out[1,2] = -x[0]
    out[1,0] =  x[2]
    out[2,0] = -x[1]
    out[2,1] =  x[0]
    return out


# import double integrator controllers
import controllers.double_integrator_controllers.double_integrator_controller
DIC_DATABASE       = controllers.double_integrator_controllers.double_integrator_controller.database
ONE_D_DIC_DATABASE = controllers.double_integrator_controllers.double_integrator_controller.database_one_d

def sat(x):
    return 0.1*x/numpy.sqrt(x**2 + 0.1**2)

# def sat(x):
#     return x

@js.add_to_database(default=True)
class LoadzUAVxy(SingleLoadTransportationController):   
    '''Decompose control problems in two parts:
    . Control z component of load as double integrator
    . Control x and y of UAV also as double integrator
        . for controlling cable oscillations we produce a desired
        for the xy trajectory of the uav that dampens and steers
        the cable oscillations to zero
    '''

    # double integrator controller for z component of load (position)
    js.Jsonable.add_inner('z_double_integrator_ctr',ONE_D_DIC_DATABASE)    
    # double integrator controller for x and y component of load (position)
    js.Jsonable.add_inner('xy_double_integrator_ctr',DIC_DATABASE)

    @classmethod
    def description(cls):
        return '''
        Controller for a <b>single aerial vehicle transporating load</b> attached by cable. Decompose control problems in two parts:
        <ul>
            <li>Control z component of load as double integrator</li>
            <li>Control x and y of UAV also as double integrator</li>
            <ul>
                <li>for controlling cable oscillations we produce a desired for the xy trajectory of the uav that dampens and steers the cable oscillations to zero</li>
            </ul>
        </ul>
        '''


    def __init__(self, 
        load_mass    = rospy.get_param("load_mass",0.1),
        quad_mass    = rospy.get_param("quadrotor_mass",1.442),
        cable_length = rospy.get_param("cable_length",0.6),
        gains = numpy.array([-1, -6, 0, 1])
        ):

        self.add_inner_defaults()

        self.gains = gains

        self.quad_mass    = quad_mass
        self.load_mass    = load_mass
        self.cable_length = cable_length
        # TODO import this later from utilities
        self.g            = 9.81
        self.gravity = uts.GRAVITY

        self.data = {}

        self.e3 = numpy.array([0.0,0.0,1.0])

    def get_total_weight(self):
        return (self.quad_mass+self.load_mass)*self.g

    def output(self,time_instant,state,stated):

        x,gravity = self._state_transform(state,stated)

        reference = {}
        reference['position'] = stated[2]
        reference['velocity'] = stated[5]
        reference['acceleration'] = stated[8]
        thrust = self.compute_thrust(reference)

        reference = {}
        reference['position'] = stated[0:2]
        reference['velocity'] = stated[3:5]
        uav_xy_reference = self.compute_xy_reference_initial(reference,1.0/35.0)
        torque = self.compute_torque(uav_xy_reference,thrust)

        U = self._input_transform(thrust,torque)

        return U

    def object_descrition(self):
        description = "Controller\n"
        description = "quad mass = " + str(self.quad_mass) + "(kg), load mas = " + str(self.load_mass) + "(kg), cable length = " + str(self.cable_length) + "(m), gravity = " + str(self.g) + "(m/s/s).\n\n"
        return description


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
        self.data['load_position'] = pM
        # current LOAD velocity
        vM  = state[3:6]
        self.data['load_velocity'] = vM
        # current QUAD position
        pm  = state[6:9]
        self.data['uav_position'] = pm
        # current QUAD velocity
        vm  = state[9:12]
        self.data['uav_velocity'] = vm

        # DESIRED LOAD position
        pd = stated[0:3]
        # DESIRED LOAD velocity
        vd = stated[3:6]

        # transformation of state
        p = pM - pd
        v = vM - vd

        # direction of cable
        n = (pm - pM)/numpy.linalg.norm(pm - pM)

        # alpha = 10.0*3.142/180.0
        # if numpy.dot(n,e3)<numpy.cos(alpha):
        #     naux = numpy.dot(uts.ort_proj(e3),n)
        #     n    = numpy.cos(alpha)*e3 + numpy.sin(alpha)*naux/numpy.linalg.norm(naux)

        # angular velocity of cable
        w  = dot(skew(n),(vm - vM)/numpy.linalg.norm(pM - pm))

        self.data['unit_vector'] = n
        self.data['angular_velocity'] = w


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

    def compute_thrust(self,reference):

        # direction of cable
        n = self.data['unit_vector']

        # z position error
        ep = self.data['load_position'][2] - reference['position']
        # z velocity error
        ev = self.data['load_velocity'][2] - reference['velocity']

        # feedforward term: desired acceleration
        ad = reference['acceleration']

        g  = self.gravity

        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.z_double_integrator_ctr.output(ep,ev)

        thrust = 1.0/n[2]*(g + ad + u)

        return thrust

    def compute_torque(self,reference,Thrust):

        # direction of cable
        n = self.data['unit_vector']
        # angular velocity of cable
        w  = self.data['angular_velocity']

        # xy position error
        ep = self.data['uav_position'][0:2] - reference['position']
        # xy velocity error
        ev = self.data['uav_velocity'][0:2] - reference['velocity']

        ad = reference['acceleration']

        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.xy_double_integrator_ctr.output(ep,ev)
        u = ad + u
        u = numpy.concatenate([u,[0]])

        L = self.cable_length

        e3 = self.e3

        torque = -1.0/n[2]*1.0/L
        torque = torque*numpy.dot(skew(n),skew(e3))
        torque = numpy.dot(torque,u - n*(Thrust-L*numpy.dot(w,w)))

        return torque

    def compute_xy_reference_initial(self,reference,Dt):

        self.position_xy_desired = reference['position']
        self.velocity_xy_desired = reference['velocity']
        reference['acceleration'] = numpy.array([0.0,0.0])

        self.compute_xy_reference_initial = self.compute_xy_reference

        return reference

    def compute_xy_reference(self,reference,Dt):

        p_old = self.position_xy_desired
        v_old = self.velocity_xy_desired

        p_star_tilde = reference['position']
        v_star_tilde = reference['velocity']

        # if numpy.linalg.norm(p_old - p_star_tilde) + numpy.linalg.norm(v_old - v_star_tilde) < 0.3:
        if numpy.linalg.norm(p_old - p_star_tilde) < 0.2:
            
            # direction of cable
            n = self.data['unit_vector']
            phi   =  -numpy.arcsin(n[[1]])
            theta =  numpy.arctan(n[[0]]/n[[2]])

            # angular velocity of cable
            w  = self.data['angular_velocity']
            w_phi   = w[0]/numpy.cos(theta) - w[1]*numpy.tan(phi)*numpy.tan(theta)
            w_theta = w[1]/(numpy.cos(theta)**2)

            p  = sat(p_old[0] - p_star_tilde[0])
            v  = sat(v_old[0] - v_star_tilde[0])
            tt = theta
            w  = w_theta
            ux = self.compute_one_d_acceleration(p,v,tt,w)

            px_new = p_star_tilde[0] + p + v*Dt + 0.5*ux*(Dt**2)
            vx_new = v_star_tilde[0] + v + ux*Dt

            p  = -sat(p_old[1] - p_star_tilde[1])
            v  = -sat(v_old[1] - v_star_tilde[1])
            tt = phi
            w  = w_phi
            uy = -self.compute_one_d_acceleration(p,v,tt,w)

            py_new = p_star_tilde[1] - (p + v*Dt + 0.5*(-uy)*(Dt**2))
            vy_new = v_star_tilde[1] - (v + (-uy)*Dt)

            p_new = numpy.array([px_new,py_new])
            v_new = numpy.array([vx_new,vy_new])
            u = numpy.array([ux,uy])

            # p_new = p_star_tilde + p_old + v_old*Dt + 0.5*u*(Dt**2)
            # v_new = v_star_tilde + u*Dt

            reference = {}
            # array is not mutable
            reference['position'] = p_new
            reference['velocity'] = v_new
            reference['acceleration'] = 0.0*numpy.array([ux,uy])

            # reference['position'] = p_new
            # reference['velocity'] = v_new
            # reference['acceleration'] = numpy.array([ux,uy])

            self.position_xy_desired = p_new
            self.velocity_xy_desired = v_new

        else:
            self.position_xy_desired = reference['position']
            self.velocity_xy_desired = reference['velocity']
            reference['acceleration'] = numpy.array([0.0,0.0])

        return reference


    def compute_one_d_acceleration(self,p,v,tt,w):
        '''p = position'''

        g  = self.gravity
        L  = self.cable_length

        xi = numpy.array([p - L*tt,v - L*w,g*tt,g*w])

        #gains = numpy.array([-2.0,-6.0,-7.0,-4.0])
        gains = self.gains
        v     = numpy.dot(gains,xi)

        acceleration = 0.0*xi[2] + L/g*v[0]

        return acceleration[0]

    def _input_transform(self,Thrust,Tau):

        # masses and cable length
        m   = self.quad_mass
        M   = self.load_mass
        L   = self.cable_length
        
        n = self.x[6:9]
        w = self.x[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + numpy.dot(uts.ort_proj(n),Tau*m*L)

        return U

# import controllers for vector thrusted systems
import controllers.vector_thrust_controllers.vector_thrust_controller
VT_CONTROLLERS_DATABASE = controllers.vector_thrust_controllers.vector_thrust_controller.database

@js.add_to_database()
class SingleLoadTransportController(SingleLoadTransportationController):   

    js.Jsonable.add_inner('vector_thrust_controller',VT_CONTROLLERS_DATABASE)

    @classmethod
    def description(cls):
        return "Controller for a **single aerial vehicle transporating load** attached by cable"


    def __init__(self, 
        load_mass    = rospy.get_param("load_mass",0.1),
        quad_mass    = rospy.get_param("quadrotor_mass",1.442),
        cable_length = rospy.get_param("cable_length",0.6),
        ):

        self.add_inner_defaults()

        self.quad_mass    = quad_mass
        self.load_mass    = load_mass
        self.cable_length = cable_length
        # TODO import this later from utilities
        self.g            = 9.81

    def get_total_weight(self):
        return (self.quad_mass+self.load_mass)*self.g

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

        # alpha = 10.0*3.142/180.0
        # if numpy.dot(n,e3)<numpy.cos(alpha):
        #     naux = numpy.dot(uts.ort_proj(e3),n)
        #     n    = numpy.cos(alpha)*e3 + numpy.sin(alpha)*naux/numpy.linalg.norm(naux)

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
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + numpy.dot(uts.ort_proj(n),Tau*m*L)

        return U
