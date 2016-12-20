#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
import numpy as np

from utilities import utility_functions as uts
from utilities import utility_functions

import rospy

from utilities import jsonable as js

import nav_msgs.msg
import geometry_msgs.msg

class Controller(js.Jsonable):

    
    @classmethod
    def description(cls):
        return "Controller for **fully actuated** quadrotor model for collaborative ..."


def position_and_velocity_from_odometry(odometry):
    x = np.array([odometry.pose.pose.position.x,\
                  odometry.pose.pose.position.y,\
                  odometry.pose.pose.position.z])

    # TODO: naming of child_frame_id
    if odometry.child_frame_id == 'firefly/base_link':

        # velocity is in the body reference frame
        v_body = np.array([odometry.twist.twist.linear.x,\
                           odometry.twist.twist.linear.y,\
                           odometry.twist.twist.linear.z])

        quaternion = np.array([odometry.pose.pose.orientation.x,\
                               odometry.pose.pose.orientation.y,\
                               odometry.pose.pose.orientation.z,\
                               odometry.pose.pose.orientation.w])

        # TODO
        rotation_matrix  = uts.rot_from_quaternion(quaternion)

        v = np.dot(rotation_matrix,v_body)

    else:
        # velocity is in the body reference frame
        v = np.array([odometry.twist.twist.linear.x,\
                      odometry.twist.twist.linear.y,\
                      odometry.twist.twist.linear.z])

    return x,v

@js.add_to_database(default=True)
class SimplePIDController(Controller):
    """ This is a dynamic controller, not a static controller """

    @classmethod
    def description(cls):
        return "PID Controller, with saturation on integral part"

    def __init__(self,
    proportional_gain_xy = rospy.get_param("natural_frequency_xy",1.0)**2,
    derivative_gain_xy   = 2*rospy.get_param("damping_xy",0.7)*rospy.get_param("natural_frequency_xy",1.0),
    integral_gain_xy     = rospy.get_param("integral_gain_xy",0.0),
    bound_integral_xy    = rospy.get_param("bound_integral_xy",0.0),
    proportional_gain_z  = rospy.get_param("natural_frequency_z",1.0)**2,
    derivative_gain_z    = 2*rospy.get_param("damping_z",1.0)*rospy.get_param("natural_frequency_z",1.0),
    integral_gain_z      = rospy.get_param("integral_gain_z",0.0),
    bound_integral_z     = rospy.get_param("bound_integral_z",0.0),
    quad_mass            = rospy.get_param("uav_mass",1.442) + rospy.get_param("extra_mass",0.0),
    uav_id               = rospy.get_param("uav_id",1)
    ):

        self.uav_id               = uav_id
        self.proportional_gain_xy = proportional_gain_xy
        self.derivative_gain_xy   = derivative_gain_xy
        self.integral_gain_xy     = integral_gain_xy
        self.bound_integral_xy    = bound_integral_xy
        self.proportional_gain_z  = proportional_gain_z
        self.derivative_gain_z    = derivative_gain_z
        self.integral_gain_z      = integral_gain_z
        self.bound_integral_z     = bound_integral_z
        self.quad_mass            = quad_mass

        #TODO get from utilities?
        self.MASS = quad_mass
        self.GRAVITY = 9.81

        self.disturbance_estimate   = numpy.array([0.0,0.0,0.0])
        self.t_old  = 0.0

    def get_total_weight(self):
        return self.MASS*self.GRAVITY

    def object_description(self):
        string =  "<p>PID Controller, with saturation on integral part</p>"
        string += "<p>force(&#916t, p,pd) = " + str(self.quad_mass) +"*( pd<sup>(2)</sup> + u(p<sup>(0)</sup> - pd<sup>(0)</sup>,p<sup>(1)</sup> - pd<sup>(1)</sup>) + g e<sub>3</sub> - d<sup>est</sup>), where</p>"
        string += "<ul>"
        string += "<li>u<sub>xy</sub>(p,v) = -"+str(self.proportional_gain_xy)+"*p"+"-"+str(self.derivative_gain_xy)+"*v"+"</li>"
        string += "<li>u<sub>z</sub>(p,v) = -"+str(self.proportional_gain_z)+"*p"+"-"+str(self.derivative_gain_z)+"*v"+"</li>"
        string += "<li>d<sub>xy</sub><sup>est(1)</sup> = "+str(self.integral_gain_xy)+"*(kp/2*ep + ev)"+"</li>"
        string += "<li>|d<sub>xy</sub><sup>est(0)</sup>| &#8804 "+str(self.bound_integral_xy)+"</li>"        
        string += "<li>d<sub>z</sub><sup>est(1)</sup> = "+str(self.integral_gain_z)+"*(kp/2*ep + ev)"+"</li>"
        string += "<li>|d<sub>z</sub><sup>est(0)</sup>| &#8804 "+str(self.bound_integral_z)+"</li>"
        string +="</ul>"
        return string
        
    def __str__(self):
        #TODO add all the parameters and the state
        string = Controller.__str__(self)
        string += "\nProportional gain xy: " + str(self.proportional_gain_xy)
        string += "\nDerivative gain xy: " + str(self.derivative_gain_xy)
        return string

    def output(self, 
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry(),
        partner_uav_odometry = nav_msgs.msg.Odometry(),
        bar_odometry = nav_msgs.msg.Odometry(),
        reference = np.zeros(6)):

        reference = nav_msgs.msg.Path()

        posestamped = geometry_msgs.msg.PoseStamped()
        r = posestamped.pose.position
        r.x = 0
        r.y = 0
        r.z = 1

        # 0001?
        q = posestamped.pose.orientation
        q.x = 1
        q.y = 0
        q.z = 0
        q.w = 0

        #print(posestamped)
        r = reference.poses.append(posestamped)
        #print(r)

        reference = np.zeros(9)
        if self.uav_id==1:
            reference[0:3] = np.array([0.0,0.0,1.0])
        else:
            reference[0:3] = np.array([1.0,0.0,1.0])

        x,v = position_and_velocity_from_odometry(uav_odometry)

        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])        
        
        #--------------------------------------#
        # position and velocity
        #x  = state[0:3]
        #v  = state[3:6]
        # thrust unit vector and its angular velocity
        # R  = state[6:15]; R  = numpy.reshape(R,(3,3))

        #--------------------------------------#
        # desired quad trajectory
        xd = reference[0:3]
        vd = reference[3:6]
        ad = reference[6:9]
        
        #--------------------------------------#
        # position error and velocity error
        ep = x - xd
        ev = v - vd

        #u, V_v = self.input_and_gradient_of_lyapunov(ep,ev)
        u = self.ucl_i(ep,ev)

        #Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3)
        Full_actuation = u + self.MASS*ad

        return Full_actuation

    def ucl_i(self,ep,ev):

        # IMPORTANT: All of these parameters need to be confirmed through testing.
        # Also, the masses, cable lenghts and payload dimention should be given as input somewhere!

        # Proportional gains
        kpx     = 1.0
        kpy     = 1.0
        kpz     = 1.0

        # Derivative gains
        kvx     = 1.4
        kvy     = 1.4
        kvz     = 1.4

        #this mass MUST be obtained as input!
        mass_load = 0.1

        # Equilibrium term of the control law
        u_EQ    = numpy.array([0.0,0.0,0.0])
        #LOW priority this law could be made to be more generic
        u_EQ[2] = (self.MASS + 0.5 * mass_load ) * self.GRAVITY

        # PD term of the control law
        u_PD    = numpy.array([0.0,0.0,0.0])
        u_PD[0] = -kpx*ep[0] - kvx*ev[0]
        u_PD[1] = -kpy*ep[1] - kvy*ev[1]
        u_PD[2] = -kpz*ep[2] - kvz*ev[2]

        # Corrective term of the control law
        u_corr  = numpy.array([0.0,0.0,0.0])
        # I will LATER define the additional corrective term

        u = u_EQ + u_PD + u_corr

        return u


    def input_and_gradient_of_lyapunov(self,ep,ev):

        u    = numpy.array([0.0,0.0,0.0])
        V_v  = numpy.array([0.0,0.0,0.0])

        kp     = self.proportional_gain_xy
        kv     = self.derivative_gain_xy
        u[0]   = -kp*ep[0] - kv*ev[0]
        u[1]   = -kp*ep[1] - kv*ev[1]
        V_v[0] = (kp/2*ep[0] + ev[0])
        V_v[1] = (kp/2*ep[1] + ev[1])

        kp     = self.proportional_gain_z
        kv     = self.derivative_gain_z
        u[2]   = -kp*ep[2] - kv*ev[2]
        V_v[2] = (kp/2*ep[2] + ev[2])

        return u, V_v


    def reset_estimate_xy(self):
        self.disturbance_estimate[0] = 0.0
        self.disturbance_estimate[1] = 0.0
        return

    def reset_estimate_z(self):
        self.disturbance_estimate[2] = 0.0
        return         