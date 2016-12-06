#!/usr/bin/env python

import rospy
import numpy
import numpy as np
from utilities import jsonable as js
import utilities.utility_functions as uts

import mission
import tf.transformations

import nav_msgs
import mav_msgs.msg
import math

def roll_pitch(
    non_zero_vector = numpy.ones(3),
    psi_angle = 0.0):

    #--------------------------------------#
    # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des
    # desired roll and pitch angles
    norm = numpy.linalg.norm(non_zero_vector)
    #if norm > 0.1*self.GRAVITY*self.total_mass:
    if norm > 0.1:
        n_des     = non_zero_vector/norm
        n_des_rot = uts.rot_z(-psi_angle).dot(n_des)
    else:
        n_des     = numpy.array([0.0,0.0,1.0])
        n_des_rot = uts.rot_z(-psi_angle).dot(n_des)


    sin_phi   = -n_des_rot[1]
    sin_phi   = numpy.clip(sin_phi,-1,1)
    phi       = numpy.arcsin(sin_phi)

    sin_theta = n_des_rot[0]/numpy.cos(phi)
    sin_theta = numpy.clip(sin_theta,-1,1)
    cos_theta = n_des_rot[2]/numpy.cos(phi)
    cos_theta = numpy.clip(cos_theta,-1,1)
    pitch     = numpy.arctan2(sin_theta,cos_theta)

    return (phi,pitch)

# Parent class for all types of UAV's
class TypeUAV(js.Jsonable):
    """Class for each type of uav"""

    # mission must be added
    # js.Jsonable.add_inner('mission',MISSIONS_DATABASE)

    @classmethod
    def description(cls):
        string = """Type of uav must be selected. Options are:
        <ul>
          <li>Firefly in gazebo</li>
          <li>IRIS+ in rviz</li>
          <li>IRIS+ in mocap</li>
        </ul>
        Once type is selected, appropriate converter is chosen
        and appropriate publisher is also chosen
        """
        return string 

    # all children to this class must implement the 
    # ALL NEED TO PUBLISH message of type mav_msgs.msg.RollPitchYawrateThrust
    def publish(self):
        raise NotImplementedError()

    def __init__(self):

        if rospy.get_param('playing_back',True):
            self.initialize_save_uav_odometry()
            callback_sub_uav_odometry = self.callback_save_uav_odometry
        else:
            callback_sub_uav_odometry = self.update_uav_odometry

        self.sub_uav_odometry = rospy.Subscriber(
            name       = 'uav_odometry',
            data_class = nav_msgs.msg.Odometry,
            callback   = callback_sub_uav_odometry)

        self.uav_odometry = nav_msgs.msg.Odometry()


        if rospy.get_param('playing_back',True):
            self.initialize_roll_pitch_yawrate_thrust()
            # subscriber
            self.sub_roll_pitch_yawrate_thrust = rospy.Subscriber(
                name = 'uav_roll_pitch_yawrate_thrust',
                data_class = mav_msgs.msg.RollPitchYawrateThrust,
                callback = self.callback_sub_roll_pitch_yawrate_thrust)
            self.publish = self.publish_nothing
        else:
            # publisher
            self.pub_roll_pitch_yawrate_thrust = rospy.Publisher(
                name = 'uav_roll_pitch_yawrate_thrust',
                data_class = mav_msgs.msg.RollPitchYawrateThrust,
                queue_size = 1)

        self.box_center = numpy.array([0.0,0.0,1.0])
        self.box_sides  = numpy.array([2.0,2.0,1.5])
        # go down to z = ... in meters when emergency is triggered
        self.position_z_before_land = 0.3

    def update_uav_odometry(self,msg = nav_msgs.msg.Odometry()):
        self.uav_odometry = msg

    def publish_nothing(self):
        pass

    def test_emergency(self):

        p = np.array(
            [self.uav_odometry.pose.pose.position.x,\
            self.uav_odometry.pose.pose.position.y,\
            self.uav_odometry.pose.pose.position.z])

        return not self.check_inside_limits(position = p)

    def check_inside_limits(self,position):

        if any(numpy.absolute(position - self.box_center) >= self.box_sides):
            return False
        else:
            return True

    def trigger_emergency(self):
        self.mission.trigger_emergency(
            uav_odometry = self.uav_odometry)

    def initialize_save_uav_odometry(self):

        self.times = []

        self.px    = []
        self.py    = []
        self.pz    = []
        self.vx    = []
        self.vy    = []
        self.vz    = []

        self.e1    = []
        self.e2    = []
        self.e3    = []

    def callback_save_uav_odometry(self, msg = nav_msgs.msg.Odometry()):
                
        self.times.append(float(msg.header.stamp.to_sec()))

        self.px.append(float(msg.pose.pose.position.x))
        self.py.append(float(msg.pose.pose.position.y))
        self.pz.append(float(msg.pose.pose.position.z))

        self.vx.append(float(msg.twist.twist.linear.x))
        self.vy.append(float(msg.twist.twist.linear.y))
        self.vz.append(float(msg.twist.twist.linear.z))


        quat = msg.pose.pose.orientation
        quat = numpy.array([quat.x,quat.y,quat.z,quat.w])
        # rotation matrix
        ee = tf.transformations.euler_from_quaternion(quat, axes='sxyz')

        self.e1.append(float(ee[0]*180.0/math.pi))
        self.e2.append(float(ee[1]*180.0/math.pi))
        self.e3.append(float(ee[2]*180.0/math.pi))

    def print_uav_odometry(self):
        
        if self.times:
            # import matplotlib.pyplot as plt
            import matplotlib
            matplotlib.use('Agg')
            from matplotlib import pyplot as plt

            # it is not bad to import it here, since plotting is done aposteriori
            # import operator
            # times = map(operator.sub,self.times,len(self.times)*[self.times[0]])
            times = self.times

            fig2 = plt.figure()
            plt.plot(times, self.px, 'r-', label=r'$x$')
            plt.plot(times, self.py, 'g-', label=r'$y$')
            plt.plot(times, self.pz, 'b-', label=r'$z$')
            plt.title('Positions (m)')
            plt.legend(loc='best')
            plt.grid()

            fig3 = plt.figure()
            plt.plot(times, self.vx, 'r-', label=r'$v_x$')
            plt.plot(times, self.vy, 'g-', label=r'$v_y$')
            plt.plot(times, self.vz, 'b-', label=r'$v_z$')    
            plt.title('Velocities (m/s)')
            plt.legend(loc='best')
            plt.grid()        
            
            fig4 = plt.figure()
            plt.plot(times, self.e1, 'r-', label=r'$\phi$')
            plt.plot(times, self.e2, 'g-', label=r'$\theta$')
            plt.plot(times, self.e3, 'b-', label=r'$\psi$')
            plt.title('Attitudes (deg)')
            plt.legend(loc='best')
            plt.grid()
            # plt.draw()

            # one element tuple
            return fig2,fig3,fig4

        else: 
            print('No uav odometry messages')
            # empty tuple of figures
            return ()


    def to_RollPitchYawrateThrust(self,
        uav_odometry = nav_msgs.msg.Odometry(),
        desired_3d_force = numpy.zeros(3), 
        yaw_rate_desired = 0.0):


        #---------------------------------------------------------------------#
        quat = uav_odometry.pose.pose.orientation
        quat = numpy.array([quat.x,quat.y,quat.z,quat.w])
        # third column of rotation matrix
        self.rotation_matrix = tf.transformations.quaternion_matrix(quat)[0:3,0:3]
        # unit_vector = tf.transformations.quaternion_matrix(quat)[2,0:3]
        #throttle_unit_vector = self.rotation_matrix[2,:]
        psi_angle = tf.transformations.euler_from_quaternion(quat, axes='sxyz')[2]

        #---------------------------------------------------------------------#
        roll_desired,pitch_desired = roll_pitch(
            non_zero_vector = desired_3d_force,
            psi_angle = psi_angle)

        # new message
        message          = mav_msgs.msg.RollPitchYawrateThrust()
        # populate message
        message.header.stamp = rospy.get_rostime()
        message.roll     = roll_desired
        message.pitch    = pitch_desired
        message.yaw_rate = yaw_rate_desired
        message.thrust.x = desired_3d_force[0]
        message.thrust.y = desired_3d_force[1]
        message.thrust.z = desired_3d_force[2]

        return message


    def initialize_roll_pitch_yawrate_thrust(self):

        self.times_input    = []
        self.roll     = []
        self.pitch    = []
        self.yaw_rate = []
        self.thrust   = []

    def callback_sub_roll_pitch_yawrate_thrust(self, 
        msg = mav_msgs.msg.RollPitchYawrateThrust()):
        
        self.times_input.append(float(msg.header.stamp.to_sec()))
        self.roll.append(float(msg.roll*180.0/math.pi))
        self.pitch.append(float(msg.pitch*180.0/math.pi))
        self.yaw_rate.append(float(msg.yaw_rate*180.0/math.pi))
        self.thrust.append(float(msg.thrust.z))

    def print_roll_pitch_yawrate_thrust(self):
        
        if self.times_input:
            # import matplotlib.pyplot as plt
            import matplotlib
            matplotlib.use('Agg')
            from matplotlib import pyplot as plt

            # it is not bad to import it here, since plotting is done aposteriori
            import operator
            # times = map(operator.sub,self.times,len(self.times)*[self.times[0]])
            times = self.times_input

            fig1 = plt.figure()
            plt.plot(times[1:], map(operator.sub,times[1:],times[0:-1]), 'r-', label=r'$\Delta t$')       
            plt.title('Time Interval (s)')
            plt.legend(loc='best')
            plt.grid()

            fig2 = plt.figure()
            plt.plot(times, self.roll, 'r-', label=r'$\phi$')
            plt.plot(times, self.pitch, 'g-', label=r'$\theta$')
            plt.plot(times, self.yaw_rate, 'b-', label=r'$\dot{\psi}$')
            plt.plot(times, self.thrust, 'r--', label=r'Thrust')
            plt.title('Commands')
            plt.legend(loc='best')
            plt.grid()

            # two element tuple (of figures)
            return fig1,fig2

        else: 
            print('No roll_pitch_yawrate_thrust messages')
            # empty tuple (of figures)
            return ()

    def get_all_plots(self):
        return self.print_uav_odometry() + self.print_roll_pitch_yawrate_thrust()

#------------------------------------------------------------#
#------------------------------------------------------------#
# Firefly 

MISSIONS_DATABASE = mission.FireflyGazebo.database


firefly_parameters = {}

attitude_proprotial_gain = 10
attitude_derivative_gain = 2*0.8*np.sqrt(attitude_proprotial_gain)
# attitude_proprotial_gain = 3
# attitude_derivative_gain = 0.52 
firefly_parameters['attitude_proprotial_gain'] = attitude_proprotial_gain
firefly_parameters['attitude_derivative_gain'] = attitude_derivative_gain

attitude_z_derivative_gain = 5.0
firefly_parameters['attitude_z_derivative_gain'] = attitude_z_derivative_gain

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
firefly_parameters['kDefaultMass'] = kDefaultMass
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
firefly_parameters['quad_inertia_matrix'] = J

matrix_motor_speeds = numpy.dot(A_inv,K_inv)
firefly_parameters['matrix_motor_speeds'] = matrix_motor_speeds

# this decorator will also create the database
# add class to database
@js.add_to_database(default=True)
class FireflyGazebo(TypeUAV):
    """This class creates the publisher for a Firefly UAV"""

    # add dictionary of missions available to Firefly
    js.Jsonable.add_inner('mission',MISSIONS_DATABASE)

    @classmethod
    def description(cls):
        string = """Missions with firefly in gazebo"""
        string+= """
        Mission with firefly in gazebo: chosen mission. This mission depends on:
        <ul>
          <li>mission: mission</li>
        </ul>
        Converts 3D force and angular velocity into motor speeds, for a Firefly
        """
        return string

    def __init__(self,thrust_gain = 1.0):

        self.thrust_gain = thrust_gain

        TypeUAV.__init__(self)

        # TODO: correct this
        self.time_instant_t0 = 0.0

        self.add_inner_defaults()

        # median filter for thrust component (used to )
        self.force_z_median = uts.MedianFilter(10)

        # publisher: command firefly motor speeds
        self.pub_motor_speeds = rospy.Publisher(
            name = 'firefly_motor_speed',
            data_class = mav_msgs.msg.Actuators,
            queue_size = 1)

        self.firefly_parameters = firefly_parameters

    # def to_do_before_finishing(self):
    #     if rospy.get_param('playing_back',True):
    #         # subscriber
    #         self.sub_roll_pitch_yawrate_thrust.unregister()
    #         self.print_plots(
    #             tuple_of_figures = self.print_roll_pitch_yawrate_thrust(), 
    #             file_name = "/home/pedrootao/SML_CODE/src/quad_control/experimental_data/uav_input.pdf")
    #     else:
    #         # publisher
    #         self.pub_roll_pitch_yawrate_thrust.unregister()

    def object_description(self):
        string = """
        Parameters:
        <ul>
          <li>thrust_gain: """ + str(self.thrust_gain) + """</li>
        </ul>
        """
        string  = "Converts 3D force and angular velocity into motor speeds, for a Firefly"
        string += "\n\t"
        string += "attitude_proprotial_gain = " + str(self.firefly_parameters['attitude_proprotial_gain'])
        string += "\n\t"
        string += "attitude_derivative_gain = " + str(self.firefly_parameters['attitude_derivative_gain'])
        string += "\n\t"
        string += "attitude_z_derivative_gain = " + str(self.firefly_parameters['attitude_z_derivative_gain'])
        string += "\n"
        return string


    def publish(self):

        time_instant = rospy.get_time() - self.time_instant_t0
        
        # it is the job of the mission to compute the 3d_force and yaw_rate
        desired_3d_force = self.mission.compute_3d_force(
            time_instant = time_instant,
            uav_odometry = self.uav_odometry)

        desired_yaw_rate = self.mission.compute_yaw_rate(
            time_instant = time_instant,
            uav_odometry = self.uav_odometry)
        
        message = self.produce_rotor_s_message(
            uav_odometry = self.uav_odometry,
            force_3d = desired_3d_force, 
            yaw_rate = desired_yaw_rate)

        # publish message
        self.pub_motor_speeds.publish(message)

        message = self.to_RollPitchYawrateThrust(
            uav_odometry = self.uav_odometry,
            desired_3d_force = desired_3d_force, 
            yaw_rate_desired = desired_yaw_rate)
        self.pub_roll_pitch_yawrate_thrust.publish(message)

    @js.add_to_methods_list
    def update_thrust_gain(self):

        median_force = self.force_z_median.output()
        weight       = self.firefly_parameters['kDefaultMass']*uts.GRAVITY

        self.thrust_gain = self.thrust_gain*median_force/weight

        # for safety better bound this value
        self.thrust_gain = np.clip(self.thrust_gain,0.9,1.1)
        msg = 'new neutral value = ' + str(self.thrust_gain) + ' in [0.9,1.1]'
        rospy.logwarn(msg) 
        return msg


    def produce_rotor_s_message(self, 
        uav_odometry = nav_msgs.msg.Odometry(),
        force_3d = numpy.zeros(3), 
        yaw_rate = 0.0):

        # update median filter
        self.force_z_median.update_data(force_3d[2])

        #---------------------------------------------------------------#

        quat = uav_odometry.pose.pose.orientation
        quat = numpy.array([quat.x,quat.y,quat.z,quat.w])
        # third column of rotation matrix
        self.rotation_matrix = tf.transformations.quaternion_matrix(quat)[0:3,0:3]
        # unit_vector = tf.transformations.quaternion_matrix(quat)[2,0:3]
        unit_vector = self.rotation_matrix[2,:]
        
        # angular velocity in body reference frame
        omega_body      = uav_odometry.twist.twist.angular
        self.omega_body = numpy.array([omega_body.x,omega_body.y,omega_body.z])

        #---------------------------------------------------------------#

        U_0dot = force_3d
        U_1dot = np.zeros(3)
        U_2dot = np.zeros(3)

        thrust      = self.thrust_gain*np.dot(force_3d,unit_vector)
        torque_body = self.compute_torque(U_0dot,U_1dot,yaw_rate)


        matrix_motor_speeds = self.firefly_parameters['matrix_motor_speeds']
        n = np.dot(matrix_motor_speeds,np.concatenate([torque_body,[thrust]]))
        # speeds cannot be negative; bound below by 0
        n = np.maximum(n,np.zeros(6)) 
        # forces proportional to speed squared
        n = np.sqrt(n)     

        # creating actuators message
        actuators_message = mav_msgs.msg.Actuators()
        # populate message
        actuators_message.angular_velocities = n

        return actuators_message

    def torque_unit_vector(self,n,w,n_star,w_star,w_star_dot):

        kd = self.firefly_parameters['attitude_derivative_gain']
        kp = self.firefly_parameters['attitude_proprotial_gain']

        torque = \
        np.dot(uts.ort_proj(n),w_star_dot)    - \
        kd*np.dot(uts.ort_proj(n),w - w_star) - \
        kp*np.dot(uts.skew(n_star),n)         + \
        np.dot(uts.skew(n),w_star)*np.dot(n,w_star)

        return torque

    def transform_uav_torque(self,unit_vector_torque,torque_orthogonal):

        R  = self.rotation_matrix
        w  = self.omega_body
        J  = self.firefly_parameters['quad_inertia_matrix']
        e3 = np.array([0.0,0.0,1.0])

        aux = \
        np.dot(uts.ort_proj(e3),np.dot(np.transpose(R),unit_vector_torque)) + \
        e3*torque_orthogonal - \
        np.dot(uts.skew(e3),w)*w[2]

        torque = \
        np.dot(uts.skew(w),np.dot(J,w)) + \
        np.dot(J,aux)

        return torque

    def compute_torque(self,
        desired_acceleration,
        desired_acceleration_dot,
        omega_z_body_desired):

        wz       = self.omega_body[2]
        torque_z = -self.firefly_parameters['attitude_z_derivative_gain']*(wz - omega_z_body_desired)

        R   = self.rotation_matrix
        r3  = R[:,2]
        w3  = np.dot(uts.ort_proj(r3),np.dot(R,self.omega_body))
        r3d = desired_acceleration/np.linalg.norm(desired_acceleration)
        w3d = np.dot(uts.skew(r3d),desired_acceleration_dot/np.linalg.norm(desired_acceleration))
        a3d = np.zeros(3)
        torque_xy = self.torque_unit_vector(r3,w3,r3d,w3d,a3d)

        return self.transform_uav_torque(torque_xy,torque_z)


#------------------------------------------------------------#
#------------------------------------------------------------#
# For Real/Simulated Iris

MISSIONS_DATABASE = mission.Iris.database

import mavros_msgs.msg

# add class to database
@js.add_to_database()
class Iris(TypeUAV):
    """This class creates the publisher for a IRIS UAV
    Purpose: converts a desired force (in NEWTONS) to IRIS+ standard
    throttle_neutral is the most critical parameter, that needs to be reseted when an experiment is commenced
    """

    js.Jsonable.add_inner('mission',MISSIONS_DATABASE)

    @classmethod
    def description(cls):
        string = """Missions with real IRIS"""
        return string

    def __init__(self,
    throttle_neutral = rospy.get_param("throttle_neutral",1450),
    total_mass = rospy.get_param("total_mass",1.442)
    ):

        self.throttle_neutral = throttle_neutral
        self.total_mass       = total_mass

        # TODO: correct this
        self.time_instant_t0 = 0.0

        self.add_inner_defaults()

        TypeUAV.__init__(self)

        # publisher: publish PWM signals for IRIS
        self.pub_rc_override = rospy.Publisher(
            name = 'mavros/rc/override',
            data_class = mavros_msgs.msg.OverrideRCIn,
            queue_size = 1)

        self.force_median = uts.MedianFilter3D(70)

        # Throttle neutral
        self.throttle_neutral = throttle_neutral
        self.roll_neutral  = rospy.get_param("roll_neutral_value",1500)
        self.pitch_neutral = rospy.get_param("pitch_neutral_value",1500)


        self.roll_compensation = rospy.get_param("iris_roll_gain",1.0)
        self.pitch_compensation = rospy.get_param("iris_pitch_gain",1.0)

        # Parameters
        # acceleration due to gravity (m/s/s)
        self.GRAVITY = rospy.get_param("gravity",9.81)

        # The default of 4.5 commands a 200 deg/sec rate
        # of rotation when the yaw stick is held fully left or right.
        self.MAX_PSI_SPEED_Deg = 200.0

        self.MAX_ANGLE_DEG = 45.0

        # if rospy.get_param("complete_neutral_reset",False):
        #     self.reset_k_trottle_neutral = self.complete_reset_k_trottle_neutral
        # else:
        #     self.reset_k_trottle_neutral = self.incomplete_reset_k_trottle_neutral

    def object_description(self):
        string = """
        Mission with real IRIS: chosen mission. This mission depends on:
        <ul>
          <li>mission: """ + self.mission.__class__.__name__ +"""</li>
        </ul>
        Mission parameters:
        <ul>
          <li>throttle_neutral: """ + str(self.throttle_neutral) +"""</li>
          <li>total_mass: """ + str(self.total_mass) +"""</li>
        </ul>
        """
        return string

    def publish(self):

        time_instant = rospy.get_time() - self.time_instant_t0

        # it is the job of the mission to compute the 3d_force and yaw_rate
        desired_3d_force = self.mission.compute_3d_force(
            time_instant = time_instant,
            uav_odometry = self.uav_odometry)

        desired_yaw_rate = self.mission.compute_yaw_rate(
            time_instant = time_instant,
            uav_odometry = self.uav_odometry)
        

        message = self.input_conveter(
            uav_odometry = uav_odometry,
            desired_3d_force = desired_3d_force, 
            yaw_rate_desired = desired_yaw_rate)

        self.pub_rc_override.publish(message)

        message = self.to_RollPitchYawrateThrust(
            uav_odometry = self.uav_odometry,
            desired_3d_force = desired_3d_force, 
            yaw_rate_desired = desired_yaw_rate)
        self.pub_roll_pitch_yawrate_thrust.publish(message)


    def input_conveter(self, 
        uav_odometry = nav_msgs.msg.Odometry(),
        desired_3d_force = numpy.zeros(3), 
        yaw_rate_desired = 0.0):

        self.force_median.update_data(desired_3d_force)

        #---------------------------------------------------------------------#
        quat = uav_odometry.pose.pose.orientation
        quat = numpy.array([quat.x,quat.y,quat.z,quat.w])
        # third column of rotation matrix
        self.rotation_matrix = tf.transformations.quaternion_matrix(quat)[0:3,0:3]
        # unit_vector = tf.transformations.quaternion_matrix(quat)[2,0:3]
        throttle_unit_vector = self.rotation_matrix[2,:]
        psi_angle = tf.transformations.euler_from_quaternion(quat, axes='sxyz')[2]

        #---------------------------------------------------------------------#

        # STABILIZE MODE:APM COPTER
        # The throttle sent to the motors is automatically adjusted based on the tilt angle
        # of the vehicle (i.e increased as the vehicle tilts over more) to reduce the 
        # compensation the pilot must fo as the vehicles attitude changes

        # computing desired Throttle, desired roll angle, pitch angle, and desired yaw rate
        # throttle will be smaller when tilted (compensates for adjustsments done onboard)
        Throttle = numpy.dot(desired_3d_force,throttle_unit_vector)
        # TODO: remark here
        # Throttle = numpy.linalg.norm(desired_3d_force)

        # this decreases the throtle, which will be increased (by internal controller)
        # I commented this out, because throtlled was being clipped at minimum (1000)
        # Throttle = Throttle*numpy.dot(throttle_unit_vector,e3)


        roll_desired,pitch_desired = self.roll_pitch(
            non_zero_vector = desired_3d_force,
            psi_angle = psi_angle)

        #---------------------------------------------------------------------#
        # degrees per second
        MAX_PSI_SPEED_Deg = self.MAX_PSI_SPEED_Deg
        MAX_PSI_SPEED_Rad = MAX_PSI_SPEED_Deg*numpy.pi/180.0

        MAX_ANGLE_DEG = self.MAX_ANGLE_DEG
        MAX_ANGLE_RAD = MAX_ANGLE_DEG*numpy.pi/180.0

        #---------------------------------------------------------------------#
        # input for IRIS+ comes in specific order
        # [U[0],U[1],U[2],U[3]] = [roll,pitch,throttle,yaw]
        U = numpy.zeros(4)

        #---------------------------------------------------------------------#
        # angles comand between 1000 and 2000 PWM

        # Roll
        U[0]  =  self.roll_neutral + self.roll_compensation*roll_desired*500.0/MAX_ANGLE_RAD;    
        # Pitch: 
        # ATTENTTION TO PITCH: WHEN STICK IS FORWARD PITCH,
        # pwm GOES TO 1000, AND PITCH IS POSITIVE 
        U[1]  =  self.pitch_neutral - self.pitch_compensation*pitch_desired*500.0/MAX_ANGLE_RAD;    

        # psi angular speed 
        U[3]  =  1500.0 - yaw_rate_desired*500.0/MAX_PSI_SPEED_Rad;    

        # REMARK: the throtle comes between 1000 and 2000 PWM
        U[2]  = Throttle*self.throttle_neutral/(self.GRAVITY*self.total_mass);

        # need to bound between 1000 and 2000; element-wise operation
        U     = numpy.clip(U,1000,2000) 

        # # Roll
        # U[0]  =  1500.0 + roll_desired*500.0/MAX_ANGLE_RAD;    
        # # Pitch: 
        # # ATTENTTION TO PITCH: WHEN STICK IS FORWARD PITCH,
        # # pwm GOES TO 1000, AND PITCH IS POSITIVE 
        # U[1]  =  1500.0 - pitch_desired*500.0/MAX_ANGLE_RAD;    

        # # psi angular speed 
        # U[3]  =  1500.0 - yaw_rate_desired*500.0/MAX_PSI_SPEED_Rad;    

        # # REMARK: the throtle comes between 1000 and 2000 PWM
        # U[2]  = Throttle*self.throttle_neutral/(self.GRAVITY*self.total_mass);

        # # need to bound between 1000 and 2000; element-wise operation
        # U     = numpy.clip(U,1000,2000) 

        # ORDER OF INPUTS IS VERY IMPORTANT: roll, pitch, throttle, yaw_rate
        # create message of type OverrideRCIn
        rc_cmd          = mavros_msgs.msg.OverrideRCIn()
        other_channels  = numpy.array([1500,1500,1500,1500])
        channels        = numpy.concatenate([U,other_channels])
        channels        = numpy.array(channels,dtype=numpy.uint16)
        rc_cmd.channels = channels

        return rc_cmd

    def roll_pitch(self,
        non_zero_vector = numpy.ones(3),
        psi_angle = 0.0):

        #--------------------------------------#
        # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des
        # desired roll and pitch angles
        norm = numpy.linalg.norm(non_zero_vector)
        if norm > 0.1*self.GRAVITY*self.total_mass:
            n_des     = non_zero_vector/norm
            n_des_rot = uts.rot_z(-psi_angle).dot(n_des)
        else:
            n_des     = numpy.array([0.0,0.0,1.0])
            n_des_rot = uts.rot_z(-psi_angle).dot(n_des)


        sin_phi   = -n_des_rot[1]
        sin_phi   = numpy.clip(sin_phi,-1,1)
        phi       = numpy.arcsin(sin_phi)

        sin_theta = n_des_rot[0]/numpy.cos(phi)
        sin_theta = numpy.clip(sin_theta,-1,1)
        cos_theta = n_des_rot[2]/numpy.cos(phi)
        cos_theta = numpy.clip(cos_theta,-1,1)
        pitch     = numpy.arctan2(sin_theta,cos_theta)

        return (phi,pitch)

    @js.add_to_methods_list
    def incomplete_reset_k_trottle_neutral(self):
        """
        Reset k_trottle_neutral by checking current thrust 
        (median over last thrust values)
        Should only be applied once the uav stabilizes at fixed point"""
        # if force > m*gravity, decrease neutral value
        # if force < m*gravity, increase neutral value

        median_force = self.force_median.output()

        median_force_z = median_force[2]
        rospy.logwarn('median force = '+ str(median_force_z))
        weight = self.total_mass*self.GRAVITY

        self.throttle_neutral = self.throttle_neutral*median_force_z/weight

        # for safety better bound this value
        self.throttle_neutral = numpy.clip(self.throttle_neutral,1350,1600)
        rospy.logwarn('new neutral value = ' + str(self.throttle_neutral) + ' in [1350,1600]')

        return

    @js.add_to_methods_list
    def complete_reset_k_trottle_neutral(self):
        """
        Reset k_trottle_neutral by checking current thrust
        (median over last thrust values)
        Should only be applied once the uav stabilizes at fixed point"""
        # if force > m*gravity, decrease neutral value
        # if force < m*gravity, increase neutral value

        median_force = self.force_median.output()

        median_force_z = median_force[2]
        rospy.logwarn('median force = '+ str(median_force_z))

        self.throttle_neutral = self.throttle_neutral*median_force_z/(self.total_mass*self.GRAVITY)

        # for safety better bound this value
        self.throttle_neutral = numpy.clip(self.throttle_neutral,1350,1600)
        rospy.logwarn('new neutral value = ' + str(self.throttle_neutral) + ' in [1350,1600]')


        roll_desired_median,pitch_desired_median = self.roll_pitch(median_force)

        MAX_PSI_SPEED_Deg = self.MAX_PSI_SPEED_Deg
        MAX_PSI_SPEED_Rad = MAX_PSI_SPEED_Deg*numpy.pi/180.0

        MAX_ANGLE_DEG = self.MAX_ANGLE_DEG
        MAX_ANGLE_RAD = MAX_ANGLE_DEG*numpy.pi/180.0

        self.roll_neutral = self.roll_neutral + self.roll_compensation*roll_desired_median*500.0/MAX_ANGLE_RAD
        self.roll_neutral = numpy.clip(self.roll_neutral,1440,1560)
        rospy.logwarn('new roll neutral value = ' + str(self.roll_neutral) + ' in [1440,1560]')

        self.pitch_neutral =  self.pitch_neutral - self.pitch_compensation*pitch_desired_median*500.0/MAX_ANGLE_RAD
        self.pitch_neutral = numpy.clip(self.pitch_neutral,1440,1560)
        rospy.logwarn('new pitch neutral value = ' + str(self.pitch_neutral) + ' in [1440,1560]')

        return

    @js.add_to_methods_list
    def set_min_throttle(self):
        '''Ask for minimum throttle'''

        #Change the flight mode on the Pixhawk flight controller
        try:
            # it waits for service for 2 seconds
            rospy.wait_for_service('ServiceChangeController',1.0)

            try:
                AskForMinThrotlle = rospy.ServiceProxy('ServiceChangeController',SrvControllerChangeByStr)
                
                answer = AskForMinThrotlle(controller_name = "NeutralController", parameters = "")
                if answer.received:
                    rospy.logwarn('Min Throttle')
                    # return True
                else:
                    rospy.logwarn('Could not provide Service')
                    # return False

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))

        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))

    @js.add_to_methods_list
    def change_to_stabilize(self):
        '''change to stabilize'''
        self.set_flight_mode(MODE = 'STABILIZE')

    @js.add_to_methods_list
    def change_to_land(self):
        '''change to stabilize'''
        self.set_flight_mode(MODE = 'LAND')

    @js.add_to_methods_list
    def change_to_acro(self):
        '''change to stabilize'''
        self.set_flight_mode(MODE = 'ACRO')

    def set_flight_mode(self,MODE):
        '''Set flight mode'''
        
        #Change the flight mode on the Pixhawk flight controller
        try:
            # it waits for service for 2 seconds
            rospy.wait_for_service(service = 'mavros/set_mode',timeout= 2.0)

            try:
                change_param = rospy.ServiceProxy(name = 'mavros/set_mode',service_class = SetMode)
                param=change_param(base_mode = 0,custom_mode=MODE)

                if param.success:
                    rospy.logwarn('Flight mode changed to '+MODE)
                    # return True
                else:
                    rospy.logwarn('Could not change Flight mode')
                    # return False

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))

        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))



    @js.add_to_methods_list
    def arming_iris(self):
        'Arm IRIS'

        # change to minimum throttle first
        try:

            rospy.logwarn('Arming Quad ...')
            rospy.wait_for_service(service = 'mavros/cmd/arming',timeout = 2.0)

            try:
                arming = rospy.ServiceProxy(name = 'mavros/cmd/arming',service_class = CommandBool)
                arming_result=arming(value = True)

                if arming_result.success:
                    rospy.logwarn('Quad is Armed!!!!')
                    # return True
                else:
                    rospy.logwarn('Cannot arm quad')
                    # return False

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))

        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))

    @js.add_to_methods_list
    def unarming_iris(self):
        'Unarm IRIS'

        try:

            rospy.logwarn('Un-Arming Quad ...')
            rospy.wait_for_service(service = 'mavros/cmd/arming',timeout = 2.0)

            try:
                arming = rospy.ServiceProxy(name = 'mavros/cmd/arming',service_class = CommandBool)
                arming_result=arming(value = False)

                if arming_result.success:
                    rospy.logwarn('Quad is Un-Armed!!!!')
                    # return True
                else:
                    rospy.logwarn('Cannot un-arm quad')
                    # return False

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))

        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))


    @js.add_to_methods_list
    def set_k_trottle_neutral(self,throttle_neutral = 1480):
        """Set k_trottle_neutral in [1400 1600]"""

        # setting throttle value
        self.throttle_neutral = throttle_neutral
        # for safety better bound this value
        self.throttle_neutral = numpy.clip(self.throttle_neutral,1350,1600)
        rospy.logwarn('settting neutral value to = ' + str(self.throttle_neutral) + ' in [1350,1600]')
        return 

    @classmethod
    def get_data_size(self):
        return 1+4

    def get_data(self):
        """Get all data relevant to the mission
        from this data, mission should be able to do data post-analysis, 
        like ploting or computing average errors
        """        
        default_array = [rospy.get_time()]
        default_array+= self.rc_output.tolist()
        
        # default_array = np.concatenate([default_array,self.get_complementary_data()])
        return default_array

    # despite not saving anything, we can plot the rc commands
    # based on info saving by mission
    @classmethod
    def plot_from_string(cls, string,starting_point):

        #plots
        # import matplotlib.pyplot as plt
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import pyplot as plt

        times       = []
        
        rc_roll     = []
        rc_pitch    = []
        rc_thrust   = []
        rc_yaw_rate = []

        for line in string.split('\n'):
            # ignore empty lines 
            if line:
                numbers = line.split(' ')

                numbers = numbers[starting_point[0]:]
                
                times.append(float(numbers[0]))
                rc_roll.append(float(numbers[1]))
                rc_pitch.append(float(numbers[2]))
                rc_thrust.append(float(numbers[3]))
                rc_yaw_rate.append(float(numbers[4]))
                
                #yaw_rates.append(float(numbers[13]))

        # it is not bad to import it here, since plotting is done aposteriori
        import operator
        times = map(operator.sub,times,len(times)*[times[0]])

        fig1 = plt.figure()
        plt.plot(times, rc_roll, 'r-', label=r'RC $\phi$')
        plt.plot(times, rc_pitch, 'g-', label=r'RC $\theta$')
        plt.plot(times, rc_thrust, 'b-', label=r'RC Thrust')
        plt.plot(times, rc_yaw_rate, 'r--', label=r'RC $\dot{\psi}$')
        plt.title('RC commands PWM [1000,2000]')
        plt.legend(loc='best')
        plt.grid()

        # one element tuple
        return fig1,

#------------------------------------------------------------#
#------------------------------------------------------------#
# NEO

MISSIONS_DATABASE = mission.NEO.database

# already imported
# import mav_msgs

# add class to database
@js.add_to_database()
class NEO(TypeUAV):
    """This class creates the publisher for a NEO UAV"""

    js.Jsonable.add_inner('mission',MISSIONS_DATABASE)

    @classmethod
    def description(cls):
        string = """Missions with NEO"""
        return string

    def __init__(self,
        throttle_neutral = rospy.get_param("throttle_neutral",0.759892503),
        total_mass = rospy.get_param("uav_mass",2.75)
    ):

        self.throttle_neutral = throttle_neutral
        self.total_mass       = total_mass
        print(self.total_mass)
        # TODO: correct this
        self.time_instant_t0 = 0.0

        self.add_inner_defaults()

        TypeUAV.__init__(self)

        # publisher: publish PWM signals for IRIS
        self.pub_rc_override = rospy.Publisher(
            name = 'neo_input',
            data_class = mav_msgs.msg.RollPitchYawrateThrust,
            queue_size = 1)

        self.force_median = uts.MedianFilter3D(70)

        # acceleration due to gravity (m/s/s)
        self.GRAVITY = rospy.get_param("gravity",9.81)


    def object_description(self):
        string = """
        Mission with real IRIS: chosen mission. This mission depends on:
        <ul>
          <li>mission: """ + self.mission.__class__.__name__ +"""</li>
        </ul>
        Mission parameters:
        <ul>
          <li>throttle_neutral: """ + str(self.throttle_neutral) +"""</li>
          <li>total_mass: """ + str(self.total_mass) +"""</li>
        </ul>
        """
        return string

    def publish(self):


        time_instant = rospy.get_time() - self.time_instant_t0

        # it is the job of the mission to compute the 3d_force and yaw_rate
        desired_3d_force = self.mission.compute_3d_force(
            time_instant = time_instant,
            uav_odometry = self.uav_odometry)

        desired_yaw_rate = self.mission.compute_yaw_rate(
            time_instant = time_instant,
            uav_odometry = self.uav_odometry)

        message = self.input_conveter( 
            uav_odometry = self.uav_odometry,
            desired_3d_force = desired_3d_force, 
            yaw_rate_desired = desired_yaw_rate)

        self.pub_rc_override.publish(message)

        message = self.to_RollPitchYawrateThrust(
            uav_odometry = self.uav_odometry,
            desired_3d_force = desired_3d_force, 
            yaw_rate_desired = desired_yaw_rate)
        self.pub_roll_pitch_yawrate_thrust.publish(message)

    def input_conveter(self,
        uav_odometry = nav_msgs.msg.Odometry(),
        desired_3d_force = numpy.zeros(3), 
        yaw_rate_desired = 0.0):

        self.force_median.update_data(desired_3d_force)

        #---------------------------------------------------------------------#
        quat = uav_odometry.pose.pose.orientation
        quat = numpy.array([quat.x,quat.y,quat.z,quat.w])
        # third column of rotation matrix
        self.rotation_matrix = tf.transformations.quaternion_matrix(quat)[0:3,0:3]
        # unit_vector = tf.transformations.quaternion_matrix(quat)[2,0:3]
        throttle_unit_vector = self.rotation_matrix[:,2]
        psi_angle = tf.transformations.euler_from_quaternion(quat, axes='sxyz')[2]

        #---------------------------------------------------------------------#

        # STABILIZE MODE:APM COPTER
        # The throttle sent to the motors is automatically adjusted based on the tilt angle
        # of the vehicle (i.e increased as the vehicle tilts over more) to reduce the 
        # compensation the pilot must fo as the vehicles attitude changes

        # computing desired Throttle, desired roll angle, pitch angle, and desired yaw rate
        # throttle will be smaller when tilted (compensates for adjustsments done onboard)
        #Throttle = numpy.dot(desired_3d_force,throttle_unit_vector)
        Throttle = numpy.linalg.norm(desired_3d_force)
        Throttle = numpy.clip(Throttle,0.0,1.2*20.5) # STOP DOING HARDCODED STUFF BRO!
        # TODO: remark here
        # Throttle = numpy.linalg.norm(desired_3d_force)

        # this decreases the throtle, which will be increased (by internal controller)
        # I commented this out, because throtlled was being clipped at minimum (1000)
        # Throttle = Throttle*numpy.dot(throttle_unit_vector,e3)


        roll_desired,pitch_desired = self.roll_pitch(
            non_zero_vector = desired_3d_force,
            psi_angle = psi_angle)

        roll_desired  = numpy.clip(roll_desired,-0.25,0.25)
        pitch_desired = numpy.clip(pitch_desired,-0.25,0.25)

        # new message
        message          = mav_msgs.msg.RollPitchYawrateThrust()
        # populate message
        message.roll     = roll_desired
        message.pitch    = pitch_desired
        message.yaw_rate = yaw_rate_desired
        message.thrust.z = Throttle
        message.header.frame_id = "base_link"

        return message

    def roll_pitch(self,
        non_zero_vector = numpy.ones(3),
        psi_angle = 0.0):

        #--------------------------------------#
        # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des
        # desired roll and pitch angles
        norm = numpy.linalg.norm(non_zero_vector)
        if norm > 0.1*self.GRAVITY*self.total_mass:
            n_des     = non_zero_vector/norm
            n_des_rot = uts.rot_z(-psi_angle).dot(n_des)
        else:
            n_des     = numpy.array([0.0,0.0,1.0])
            n_des_rot = uts.rot_z(-psi_angle).dot(n_des)


        sin_phi   = -n_des_rot[1]
        sin_phi   = numpy.clip(sin_phi,-1,1)
        phi       = numpy.arcsin(sin_phi)

        sin_theta = n_des_rot[0]/numpy.cos(phi)
        sin_theta = numpy.clip(sin_theta,-1,1)
        cos_theta = n_des_rot[2]/numpy.cos(phi)
        cos_theta = numpy.clip(cos_theta,-1,1)
        pitch     = numpy.arctan2(sin_theta,cos_theta)

        return (phi,pitch)

    @js.add_to_methods_list
    def incomplete_reset_k_trottle_neutral(self):
        """
        Reset k_trottle_neutral by checking current thrust 
        (median over last thrust values)
        Should only be applied once the uav stabilizes at fixed point"""
        # if force > m*gravity, decrease neutral value
        # if force < m*gravity, increase neutral value

        median_force = self.force_median.output()

        median_force_z = median_force[2]
        rospy.logwarn('median force = '+ str(median_force_z))
        weight = self.total_mass*self.GRAVITY

        self.throttle_neutral = self.throttle_neutral*median_force_z/weight

        # for safety better bound this value
        self.throttle_neutral = numpy.clip(self.throttle_neutral,0.9,1.1)
        rospy.logwarn('new neutral value = ' + str(self.throttle_neutral) + ' in [0.9,1.1]')

        return