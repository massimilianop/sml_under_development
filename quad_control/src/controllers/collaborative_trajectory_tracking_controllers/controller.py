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
    if odometry.child_frame_id == 'firefly/base_link' or odometry.child_frame_id == 'firefly_2/base_link': # IS THIS PART I ADDED CORRECT? -- Max

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


def payload_odometry(odometry):
    position = np.array([odometry.pose.pose.position.x,\
                  odometry.pose.pose.position.y,\
                  odometry.pose.pose.position.z])

    quaternion = np.array([odometry.pose.pose.orientation.x,\
                           odometry.pose.pose.orientation.y,\
                           odometry.pose.pose.orientation.z,\
                           odometry.pose.pose.orientation.w])

    linear_vel_body = np.array([odometry.twist.twist.linear.x,\
                  odometry.twist.twist.linear.y,\
                  odometry.twist.twist.linear.z])

    angular_vel_body = np.array([odometry.twist.twist.angular.x,\
                  odometry.twist.twist.angular.y,\
                  odometry.twist.twist.angular.z])

    rotation_matrix  = uts.rot_from_quaternion(quaternion)

    #IS THIS ROTATION RIGHT?
    linear_vel = np.dot(rotation_matrix,linear_vel_body)
    angular_vel = np.dot(rotation_matrix,angular_vel_body)

    return position, rotation_matrix , linear_vel, angular_vel


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
    uav_id               = rospy.get_param("uav_id",1),
    d_i                  = rospy.get_param("d_i"),
    l_i                  = rospy.get_param("cable_length"),
    mL                  = rospy.get_param("payload_mass")
    ):

        self.uav_id               = uav_id
        self.d_i                  = d_i
        self.l_i                  = l_i
        self.mL                   = mL
        self.proportional_gain_xy = proportional_gain_xy
        self.derivative_gain_xy   = derivative_gain_xy
        self.integral_gain_xy     = integral_gain_xy
        self.bound_integral_xy    = bound_integral_xy
        self.proportional_gain_z  = proportional_gain_z
        self.derivative_gain_z    = derivative_gain_z
        self.integral_gain_z      = integral_gain_z
        self.bound_integral_z     = bound_integral_z
        self.quad_mass            = quad_mass
        #self.quad_mass            = 1.53

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

        # These print statements allow us to check that the reference frames are set up correctly
        '''
        print "UAV_%i says: I am" % (self.uav_id),
        print uav_odometry.child_frame_id ,
        print "the other is",
        print partner_uav_odometry.child_frame_id ,
        print "and the bar is",
        print bar_odometry.child_frame_id
        '''

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
            reference[0:3] = np.array([1.0,0.0,1.0])
        else:
            reference[0:3] = np.array([0.0,0.0,1.0])


        # Position and velocity of the UAV
        p_i,v_i = position_and_velocity_from_odometry(uav_odometry)

        # Position and orientation of the payload
        p_bar, r_matrix_bar, v_bar, omega_bar = payload_odometry(bar_odometry)

        # Unit vector expressing the orientation of the payload
        euler_angles_bar = uts.euler_rad_from_rot(r_matrix_bar)
        n_bar = uts.unit_vector_from_euler_angles(euler_angles_bar[2], euler_angles_bar[1])
        #print n_bar

        # Position of the edge of the payload, it coincides with the anchorage point for the respective cable
        d_i = self.d_i
        p_Bi = p_bar + d_i * n_bar
        # print "UAV_%i says: My p_Bi value is " % (self.uav_id),
        # print p_Bi

        # Linear velocity of p_Bi
        v_Bi = v_bar + d_i * np.cross(omega_bar, n_bar)

        # Unit vector expressing the orientation of the cable
        l_i = self.l_i
        n_Ci = (p_i- p_Bi) / l_i
        # print "UAV_%i says: My n_Ci value is " % (self.uav_id),
        # print n_Ci

        # Angular velocity of n_Ci
        v_auxiliary = (v_i - v_Bi) / l_i
        omega_Ci = np.cross(n_Ci, v_auxiliary)

        # Third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])        
        
        #--------------------------------------#
        # position and velocity
        #p  = state[0:3]
        #v  = state[3:6]
        # thrust unit vector and its angular velocity
        # R  = state[6:15]; R  = numpy.reshape(R,(3,3))

        #--------------------------------------#
        # desired quad trajectory
        pd = reference[0:3]
        vd = reference[3:6]
        ad = reference[6:9]
        
        #--------------------------------------#
        # Position error and velocity error of the UAV
        ep = p_i - pd
        ev = v_i - vd

        #print "UAV_%i position error: %.3f %.3f %.3f" % (self.uav_id, float(ep[0]), float(ep[1]), float(ep[2]))

        u = self.ucl_i(ep,ev,p_Bi,v_Bi,p_i,v_i)
        # u = self.ucl_i_Old(ep,ev,d_i,n_Ci,omega_Ci,n_bar,omega_bar)

        Full_actuation = u + self.MASS*ad

        return Full_actuation


    def ucl_i(self,ep,ev,p_Bi,v_Bi,p_i,v_i):

        # Proportional gains
        kp      = 1.0
        kpx     = kp
        kpy     = kp
        kpz     = 3*kp

        # Derivative gains
        kd      = 3.0
        kdx     = kd
        kdy     = kd
        kdz     = kd

        # Gains for the corrective term
        kv      = 0.7

        # Proportional gains for the cab term
        kCp      = 1.0
        kCpx     = kCp
        kCpy     = kCp
        kCpz     = kCp

        # Derivative gains for the cab term
        kCd      = 3.0
        kCdx     = kCd
        kCdy     = kCd
        kCdz     = kCd

        # Equilibrium term of the control law
        u_EQ    = numpy.array([0.0,0.0,0.0])
        u_EQ[2] = (self.MASS + 0.5 * self.mL ) * self.GRAVITY

        # PD term of the control law
        u_PD    = numpy.array([0.0,0.0,0.0])
        u_PD[0] = -kpx*ep[0] - kdx*ev[0]
        u_PD[1] = -kpy*ep[1] - kdy*ev[1]
        u_PD[2] = -kpz*ep[2] - kdz*ev[2]

        # Oscillation term of the control law
        u_osc   = kv * v_Bi
        # u_osc  = numpy.array([0.0,0.0,0.0])

        # Term to keep the cable straight00
        e_Li    = numpy.array([0.0,0.0,self.l_i])
        u_cab   = - kCp * (p_i - p_Bi - e_Li) - kCd * (v_i - v_Bi)

        #u = u_EQ + u_PD + u_osc
        u = u_EQ + u_PD + u_cab

        if abs(ep[0])<0.1 and abs(ep[1])<0.1 :
            u = u_EQ + u_PD + u_osc
            print "switch ON"
        else :
            print "switch OFF"
        

        return u


    # def ucl_i_Old(self,ep,ev,d_i,n_Ci,omega_Ci,n_bar,omega_bar):

    #     # IMPORTANT: All of these parameters need to be confirmed through testing.
    #     # Also, the masses, cable lenghts and payload dimention should be given as input somewhere!

    #     # Proportional gains
    #     kpx     = 1.0
    #     kpy     = 1.0
    #     kpz     = 1.0

    #     # Derivative gains
    #     kvx     = 1.4
    #     kvy     = 1.4
    #     kvz     = 2

    #     # Gains for the corrective term
    #     k_dth   = 1
    #     k_dps   = 2
    #     k_dps2  = 0
    #     k_theta = 1
    #     k_omega = 2.5

    #     inertia_L = 0.4/12                                  #USE ACTUAL VALUE!!!
    #     xtra_term = self.MASS *d_i + (inertia_L / (2*d_i))  #TO DO: check how it influences stability

    #     #this mass MUST be obtained as input!
    #     mass_load = 0.4

    #     # Equilibrium term of the control law
    #     u_EQ    = numpy.array([0.0,0.0,0.0])
    #     #LOW priority this law could be made to be more generic
    #     u_EQ[2] = (self.MASS + 0.5 * mass_load ) * self.GRAVITY

    #     # PD term of the control law
    #     u_PD    = numpy.array([0.0,0.0,0.0])
    #     u_PD[0] = -kpx*ep[0] - kvx*ev[0]
    #     u_PD[1] = -kpy*ep[1] - kvy*ev[1]
    #     u_PD[2] = -kpz*ep[2] - kvz*ev[2]

    #     # Corrective term of the control law
    #     u_corr  = numpy.array([0.0,0.0,0.0])
    #     u_corr[0] = - k_dth * n_Ci[0]                       #this seems to DESTABILIZE!
    #     u_corr[1] = k_dps * omega_Ci[0] - k_dps2 * n_Ci[1]
    #     u_corr[2] = xtra_term * (- k_theta * n_bar[2] + k_omega * omega_bar[1])

    #     u = u_EQ + u_PD + u_corr

    #     return u


    # def input_and_gradient_of_lyapunov(self,ep,ev):

    #     u    = numpy.array([0.0,0.0,0.0])
    #     V_v  = numpy.array([0.0,0.0,0.0])

    #     kp     = self.proportional_gain_xy
    #     kv     = self.derivative_gain_xy
    #     u[0]   = -kp*ep[0] - kv*ev[0]
    #     u[1]   = -kp*ep[1] - kv*ev[1]
    #     V_v[0] = (kp/2*ep[0] + ev[0])
    #     V_v[1] = (kp/2*ep[1] + ev[1])

    #     kp     = self.proportional_gain_z
    #     kv     = self.derivative_gain_z
    #     u[2]   = -kp*ep[2] - kv*ev[2]
    #     V_v[2] = (kp/2*ep[2] + ev[2])

    #     return u, V_v


    def reset_estimate_xy(self):
        self.disturbance_estimate[0] = 0.0
        self.disturbance_estimate[1] = 0.0
        return

    def reset_estimate_z(self):
        self.disturbance_estimate[2] = 0.0
        return         