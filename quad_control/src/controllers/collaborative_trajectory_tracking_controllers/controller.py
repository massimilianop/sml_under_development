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
        reference = nav_msgs.msg.Path()):

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

        # Unit vector expressing the orientation of the cable
        n_Ci = (p_i- p_Bi) / l_i

        # TESTING:
        # print "UAV_%i says: My n_Ci value is " % (self.uav_id),
        # print n_Ci

        # Angular velocity of n_Ci
        v_auxiliary = (v_i - v_Bi) / l_i
        omega_Ci = np.cross(n_Ci, v_auxiliary)


        # -------------------------- CURRENT DESTINATION --------------------------

        # Initializing ref_pose as an empty geometry_msgs/Pose
        ref_pose = geometry_msgs.msg.Pose()

        # This FOR decides which pose in the path is the current destination
        for pose_stamped in reference.poses :            
            # Confront the current time with the time associated to each pose
            if rospy.get_time() >= pose_stamped.header.stamp.to_sec() :
                ref_pose = pose_stamped.pose

        # TESTING: This print statement allows us to check that the reference pose is the correct one
        # print ref_pose

        r = ref_pose.position
        q = ref_pose.orientation

        p_ref = np.array([r.x,r.y,r.z])

        # TODO : tf instead of utf ?
        q_vector = np.array([q.x,q.y,q.z,q.w])
        R_temp = uts.rot_from_quaternion(q_vector)
        ea_temp = uts.euler_rad_from_rot(R_temp)

        psi_ref   = ea_temp[2]
        theta_ref = ea_temp[1]

        # # ####################################################################
        # # This IF statement ensures that psi varies in small steps
        # psi_bound = 0.53        # pi/6 rad = 30 deg
        # if abs(psi_ref-psi)>psi_bound :
        #     tmpPsi  = psi + np.sign(psi_ref-psi) * psi_bound
        #     psi_ref = tmpPsi

        # # TEST: check if the angle is correct
        # psiDeg = psi *180.0 / 3.14
        # #print psiDeg
        # # TODO: ADDITIONAL TESTING WITH PRINT STATEMENTS
        # # ####################################################################

        nB_ref = uts.unit_vector_from_euler_angles(psi_ref, theta_ref)

        # TODO : maybe allow different nCi other than e3?
        nCi_ref = e3

        # Computing the reference position for the UAV
        p_i_ref = p_ref + nB_ref * d_i + nCi_ref * l_i

        # Setting the reference velocity
        v_i_ref = np.array([0.0,0.0,0.0])

        # Setting the reference acceleration
        a_i_ref = np.array([0.0,0.0,0.0])


        # TESTING: the reference is set as a constant and overrides the path in the topic /bar_reference_pose_path
        reference = np.zeros(9)
        if self.uav_id==1:
            reference[0:3] = np.array([1.0,0.0,1.0])
        else:
            reference[0:3] = np.array([0.0,0.0,1.0])
        p_i_ref = reference[0:3]
        v_i_ref = reference[3:6]
        a_i_ref = reference[6:9]


        #--------------------------------------#
        # Position error and velocity error of the UAV
        ep = p_i - p_i_ref
        ev = v_i - v_i_ref

        # TESTING:
        # print ""
        # print "Reference position for bar: ",
        # print p_ref
        # print "Reference position for UAV_%i: " %(self.uav_id),
        # print p_i_ref
        # print "Current position of the bar",
        # print p_bar
        # np.set_printoptions(precision=2)
        # np.set_printoptions(suppress=True)
        # print "Current position of UAV_%i: " %(self.uav_id),
        # print p_i
        # #print "UAV_%i position error: %.3f %.3f %.3f" % (self.uav_id, float(ep[0]), float(ep[1]), float(ep[2]))
        # print "UAV_%i position error: " %(self.uav_id),
        # print ep
        # print ""

        u = self.ucl_i(ep,ev,p_Bi,v_Bi,p_i,v_i,omega_bar,v_bar)
        # u = self.ucl_i_Old(ep,ev,d_i,n_Ci,omega_Ci,n_bar,omega_bar)

        Full_actuation = u + self.MASS*a_i_ref

        return Full_actuation


    def ucl_i(self,ep,ev,p_Bi,v_Bi,p_i,v_i,omega_bar,v_bar):

        # Proportional gains
        #kp      = 1.0
        kpx     = 2.5
        kpy     = 2.5
        kpz     = 5.0

        # Derivative gains
        #kd      = 3.0
        kdx     = 4.25
        kdy     = 4.25
        kdz     = 6.0

        # Gains for the corrective term
        kv      = 1.0

        # Proportional gains for the cab term
        #kCp      = 1.0
        kCpx     = 1.5
        kCpy     = 1.5
        kCpz     = 1.5

        # Derivative gains for the cab term
        #kCd      = 3.0
        kCdx     = 4.0
        kCdy     = 4.0
        kCdz     = 4.0

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

        # Term to keep the cable straight
        e_Li    = numpy.array([0.0,0.0,self.l_i])
        
        epB     = p_i - p_Bi - e_Li
        evB     = v_i - v_Bi

        # u_cab   = - kCp * (p_i - p_Bi - e_Li) - kCd * (v_i - v_Bi)
        u_cab    = numpy.array([0.0,0.0,0.0])
        u_cab[0] = - kCpx * epB[0] - kCdx * evB[0]
        u_cab[1] = - kCpy * epB[1] - kCdy * evB[1]
        u_cab[2] = - kCpz * epB[2] - kCdz * evB[2]

        #u = u_EQ + u_PD + u_osc
        u = u_EQ + u_PD + u_cab

        if abs(ep[0])<0.1 and abs(ep[1])<0.1 and abs(v_bar[0])<0.05 and abs(v_bar[1])<0.05 and abs(omega_bar[2])<0.05:
            u = u_EQ + u_PD
            print "STOP dampening"
        else :
            print "dampening ON"

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