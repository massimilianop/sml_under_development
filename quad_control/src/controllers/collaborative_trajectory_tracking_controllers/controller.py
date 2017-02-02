#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
import numpy as np

import math

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

    # # TODO: naming of child_frame_id
    # if odometry.child_frame_id == 'firefly/base_link' or odometry.child_frame_id == 'firefly_2/base_link':

    #     # velocity is in the body reference frame
    #     v_body = np.array([odometry.twist.twist.linear.x,\
    #                        odometry.twist.twist.linear.y,\
    #                        odometry.twist.twist.linear.z])

    #     quaternion = np.array([odometry.pose.pose.orientation.x,\
    #                            odometry.pose.pose.orientation.y,\
    #                            odometry.pose.pose.orientation.z,\
    #                            odometry.pose.pose.orientation.w])

    #     # TODO
    #     rotation_matrix  = uts.rot_from_quaternion(quaternion)

    #     v = np.dot(rotation_matrix,v_body)

    # else:
    #     # velocity is in the body reference frame
    #     v = np.array([odometry.twist.twist.linear.x,\
    #                   odometry.twist.twist.linear.y,\
    #                   odometry.twist.twist.linear.z])

    # velocity is in the body reference frame
    v_body = np.array([odometry.twist.twist.linear.x,\
                       odometry.twist.twist.linear.y,\
                       odometry.twist.twist.linear.z])

    quaternion = np.array([odometry.pose.pose.orientation.x,\
                           odometry.pose.pose.orientation.y,\
                           odometry.pose.pose.orientation.z,\
                           odometry.pose.pose.orientation.w])

    rotation_matrix  = uts.rot_from_quaternion(quaternion)

    v = np.dot(rotation_matrix,v_body)

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

    linear_vel = np.dot(rotation_matrix,linear_vel_body)
    angular_vel = np.dot(rotation_matrix,angular_vel_body)

    return position, rotation_matrix , linear_vel, angular_vel


def intermediate_orientation(n,nstar):
    res = np.inner(n,nstar)
    alpha = math.radians(45)
    pi_Eta = math.radians(175)
    if res >= math.cos(alpha) :
        return nstar
    elif res <= math.cos(pi_Eta) :
        return n
    else :
        n_ortho = nstar - n * res
        n_orthonorm = n_ortho / np.linalg.norm(n_ortho)
        return math.cos(alpha) * n + math.sin(alpha) * n_orthonorm


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

        # Discrete integrator variables
        self.errIntegral = 0.0
        self.errIntUpper = 2.0
        self.errIntLower = -2.0

        # Proportional gains
        self.kpx     = 1.0
        self.kpy     = 1.0
        self.kpz     = 3.0 #1.6

        # Derivative gains
        self.kdx     = 2.0
        self.kdy     = 2.0
        self.kdz     = 3.4 #1.8

        # Integral gain
        self.ki      = 0.3

        # Proportional gains for the dampening term
        self.kCpx     = -2 * self.kpx
        self.kCpy     = -2 * self.kpy
        self.kCpz     = -2 * self.kpz

        # Derivative gains for the dampening term
        self.kCdx     = 0
        self.kCdy     = 0
        self.kCdz     = 0

        self.output = self.output_barRef
        # self.output = self.output_uavRef


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

    def output_barRef(self, 
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry(),
        partner_uav_odometry = nav_msgs.msg.Odometry(),
        bar_odometry = nav_msgs.msg.Odometry(),
        uav_reference = nav_msgs.msg.Path(),
        bar_reference = nav_msgs.msg.Path()):

        # Useful values
        e3 = numpy.array([0.0,0.0,1.0])     # Third canonical basis vector
        d_i = self.d_i                      # Distance between anchorage point and center of mass
        l_i = self.l_i                      # Cable length

        # TESTING: These print statements allow us to check that the reference frames are set up correctly
        # print "UAV_%i says: I am" % (self.uav_id),
        # print uav_odometry.child_frame_id ,
        # print "the other is",
        # print partner_uav_odometry.child_frame_id ,
        # print "and the bar is",
        # print bar_odometry.child_frame_id

        # TESTING: This statement prints the current time in seconds
        # print rospy.get_time()

        # ---------------------------- CURRENT ODOMETRY ----------------------------

        # Position and velocity of the UAV
        p_i,v_i = position_and_velocity_from_odometry(uav_odometry)

        # Position and orientation of the payload
        p_bar, r_matrix_bar, v_bar, omega_bar = payload_odometry(bar_odometry)

        # Unit vector expressing the orientation of the payload
        euler_angles_bar = uts.euler_rad_from_rot(r_matrix_bar)
        psi   = euler_angles_bar[2]
        theta = euler_angles_bar[1]
        n_bar = uts.unit_vector_from_euler_angles(psi, theta)

        # Position of the edge of the payload, it coincides with the anchorage point for the respective cable
        p_Bi = p_bar + d_i * n_bar

        # TESTING: 
        # print "UAV_%i says: My p_Bi value is " % (self.uav_id),
        # print p_Bi

        # Linear velocity of p_Bi
        v_Bi = v_bar + d_i * np.cross(omega_bar, n_bar)

        # # Unit vector expressing the orientation of the cable
        # n_Ci = (p_i- p_Bi) / l_i

        # TESTING:
        # print "UAV_%i says: My n_Ci value is " % (self.uav_id),
        # print n_Ci

        # # Angular velocity of n_Ci
        # v_auxiliary = (v_i - v_Bi) / l_i
        # omega_Ci = np.cross(n_Ci, v_auxiliary)


        # -------------------------- CURRENT DESTINATION --------------------------

        # Initializing ref_pose as an empty geometry_msgs/Pose
        #ref_pose = geometry_msgs.msg.Pose()

        # This FOR decides which pose in the path is the current destination
        for pose_stamped in bar_reference.poses :            
            # Confront the current time with the time associated to each pose
            if rospy.get_time() >= pose_stamped.header.stamp.to_sec() :
                ref_pose = pose_stamped.pose

        # TESTING: This print statement allows us to check that the reference pose is the correct one
        # print ref_pose

        r = ref_pose.position
        q = ref_pose.orientation

        p_ref = np.array([r.x,r.y,r.z])

        # TODO : tf instead of uts ?
        q_vector = np.array([q.x,q.y,q.z,q.w])
        R_temp = uts.rot_from_quaternion(q_vector)
        ea_temp = uts.euler_rad_from_rot(R_temp)
        psi_ref   = ea_temp[2]
        theta_ref = ea_temp[1]

        nB_ref_Temp = uts.unit_vector_from_euler_angles(psi_ref, theta_ref)

        nB_ref = intermediate_orientation(n_bar,nB_ref_Temp)

        # ########### Ignore INPUT REFERENCE for TESTING purposes ###########
        p_ref = np.array([0.0,0.0,0.5])                                     #
        #nB_ref = np.array([1.0,0.0,0.0])                                    #
        # ###################################################################

        # TODO : maybe allow different nCi other than e3?
        nCi_ref = e3

        # Computing the reference position for the UAV
        p_i_ref = p_ref + nB_ref * d_i + nCi_ref * l_i

        # Setting the reference velocity
        v_i_ref = np.array([0.0,0.0,0.0])

        # Setting the reference acceleration
        a_i_ref = np.array([0.0,0.0,0.0])


        #--------------------------------------#
        # Position error and velocity error of the UAV
        ep = p_i_ref - p_i
        ev = v_i_ref - v_i

        # TESTING:
        # print ""
        # print "Reference position for bar: ",
        # print p_ref
        # print "Reference position for UAV_%i: " %(self.uav_id),
        # print p_i_ref
        # print "Current position of the bar",
        # print p_bar
        np.set_printoptions(precision=5)
        np.set_printoptions(suppress=True)
        # print "Current position of UAV_%i: " %(self.uav_id),
        # print p_i
        print "BAR_REF: UAV_%i position error: " %(self.uav_id),
        print ep
        print ""

        u = self.ucl_i(ep,ev,p_Bi,v_Bi,p_i,v_i,omega_bar,v_bar)
        # u = self.ucl_i_Old(ep,ev,d_i,n_Ci,omega_Ci,n_bar,omega_bar)

        Full_actuation = u + self.MASS*a_i_ref

        return Full_actuation


    def ucl_i(self,ep,ev,p_Bi,v_Bi,p_i,v_i,omega_bar,v_bar):

        # Equilibrium term of the control law
        u_EQ    = numpy.array([0.0,0.0,0.0])
        u_EQ[2] = (self.MASS + 0.5 * self.mL ) * self.GRAVITY

        # PD term of the control law
        u_PD    = numpy.array([0.0,0.0,0.0])
        u_PD[0] = self.kpx*ep[0] + self.kdx*ev[0]
        u_PD[1] = self.kpy*ep[1] + self.kdy*ev[1]
        u_PD[2] = self.kpz*ep[2] + self.kdz*ev[2]

        # I term of the control law (Forward Euler method)
        delta_time = rospy.get_time() - self.t_old
        self.errIntegral = self.errIntegral  + delta_time * ep[2]

        # Anti wind-up
        if self.errIntegral < self.errIntLower :
            self.errIntegral = self.errIntLower
        elif self.errIntegral > self.errIntUpper :
            self.errIntegral = self.errIntUpper

        u_Iz = self.ki * self.errIntegral
        u_I    = numpy.array([0.0,0.0,u_Iz])

        # Dampening term of the control law
        e_Li    = numpy.array([0.0,0.0,self.l_i])
        
        epB     = p_Bi + e_Li - p_i
        evB     = v_Bi - v_i

        # u_cab   = - kCp * (p_i - p_Bi - e_Li) - kCd * (v_i - v_Bi)
        u_cab    = numpy.array([0.0,0.0,0.0])
        u_cab[0] = self.kCpx * epB[0] + self.kCdx * evB[0]
        u_cab[1] = self.kCpy * epB[1] + self.kCdy * evB[1]
        u_cab[2] = self.kCpz * epB[2] + self.kCdz * evB[2]

        # TESTING other possible options for the dapening term
        # u_cab[0] = kCpx * evB[0]
        # u_cab[1] = kCpy * evB[1]
        # u_cab[2] = kCpz * evB[2]

        #u = u_EQ + u_PD + u_osc
        u = u_EQ + u_PD + u_I + u_cab

        # print "u_cab ",
        # print u_cab
        # print "epB ",
        # print epB

        # if abs(ep[0])<0.1 and abs(ep[1])<0.1 and abs(v_bar[0])<0.05 and abs(v_bar[1])<0.05 and abs(omega_bar[2])<0.05:
        #     u = u_EQ + u_PD + u_I + u_cab
        #     print "Integrator ON"
        # else :
        #     print "integrator OFF"

        # Update time instant (for the discrete intergator)
        self.t_old = rospy.get_time()

        return u


    def reset_estimate_xy(self):
        self.disturbance_estimate[0] = 0.0
        self.disturbance_estimate[1] = 0.0
        return

    def reset_estimate_z(self):
        self.disturbance_estimate[2] = 0.0
        return


    def output_uavRef(self, 
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry(),
        partner_uav_odometry = nav_msgs.msg.Odometry(),
        bar_odometry = nav_msgs.msg.Odometry(),
        uav_reference = nav_msgs.msg.Path(),
        bar_reference = nav_msgs.msg.Path()):

        # ---------------------------- CURRENT ODOMETRY ----------------------------

        # Position and velocity of the UAV
        p_i,v_i = position_and_velocity_from_odometry(uav_odometry)


        # -------------------------- CURRENT DESTINATION --------------------------

        # This FOR decides which pose in the path is the current destination
        for pose_stamped in uav_reference.poses :            
            # Confront the current time with the time associated to each pose
            if rospy.get_time() >= pose_stamped.header.stamp.to_sec() :
                ref_pose = pose_stamped.pose

        r = ref_pose.position

        p_i_ref = np.array([r.x,r.y,r.z])

        v_i_ref = np.array([0.0,0.0,0.0])
        a_i_ref = np.array([0.0,0.0,0.0])

        # Position error and velocity error of the UAV
        ep = p_i_ref - p_i
        ev = v_i_ref - v_i

        # Equilibrium term of the control law
        u_EQ    = numpy.array([0.0,0.0,0.0])
        u_EQ[2] = (self.MASS + 0.5 * self.mL ) * self.GRAVITY

        # PD term of the control law
        u_PD    = numpy.array([0.0,0.0,0.0])
        u_PD[0] = self.kpx*ep[0] + self.kdx*ev[0]
        u_PD[1] = self.kpy*ep[1] + self.kdy*ev[1]
        u_PD[2] = self.kpz*ep[2] + self.kdz*ev[2]

        # I term of the control law (Forward Euler method)
        delta_time = rospy.get_time() - self.t_old
        self.errIntegral = self.errIntegral  + delta_time * ep[2]

        # Anti wind-up
        if self.errIntegral < self.errIntLower :
            self.errIntegral = self.errIntLower
        elif self.errIntegral > self.errIntUpper :
            self.errIntegral = self.errIntUpper

        u_Iz = self.ki * self.errIntegral
        u_I    = numpy.array([0.0,0.0,u_Iz])

        u = u_EQ + u_PD + u_I

        # Update time instant (for the discrete intergator)
        self.t_old = rospy.get_time()

        Full_actuation = u + self.MASS*a_i_ref

        print "UAV_REF: UAV_%i desired position: " %(self.uav_id),
        print p_i_ref
        print "UAV_REF: UAV_%i position error: " %(self.uav_id),
        print ep
        print ""

        return Full_actuation


    def uav2bar(self):
        self.output = self.output_barRef
        return

    def bar2uav(self):
        self.output = self.output_uavRef
        return

    def change_flight_mode(self, mode) :

        uav_reference_modes = ['lift_off', 'landing', 'emergency']
        bar_reference_modes = ['normal']

        if mode in uav_reference_modes :
            self.bar2uav()
        elif mode in bar_reference_modes :
            self.uav2bar()

        print "UAV_%i mode: %s " % (self.uav_id,mode)

        return
