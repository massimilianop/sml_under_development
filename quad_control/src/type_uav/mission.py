#!/usr/bin/env python

from utilities import jsonable as js
from utilities import utility_functions

from utilities import utility_functions as uts

import numpy as np
import math
import numpy

# for getting time
import rospy

#node will subscribe to uav_odometry measurements
from nav_msgs.msg import Odometry

#plots
# import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

class OdometryUts():
    """Utilities for Odometry message"""


    @staticmethod
    def get_position(odometry):

        p = np.array([odometry.pose.pose.position.x,\
                      odometry.pose.pose.position.y,\
                      odometry.pose.pose.position.z])
        return p

    @staticmethod
    def get_euler_angles(odometry):

        quaternion = np.array([odometry.pose.pose.orientation.x,\
                               odometry.pose.pose.orientation.y,\
                               odometry.pose.pose.orientation.z,\
                               odometry.pose.pose.orientation.w])    

        rotation_matrix  = uts.rot_from_quaternion(quaternion)
        # ee = np.array([roll,pitch,yaw]) in DEGREES
        ee               = uts.euler_deg_from_rot(rotation_matrix)
        return ee

    @staticmethod
    def get_yaw(odometry):

        quaternion = np.array([odometry.pose.pose.orientation.x,\
                               odometry.pose.pose.orientation.y,\
                               odometry.pose.pose.orientation.z,\
                               odometry.pose.pose.orientation.w])    

        rotation_matrix  = uts.rot_from_quaternion(quaternion)
        # ee = np.array([roll,pitch,yaw]) in DEGREES
        ee               = uts.euler_deg_from_rot(rotation_matrix)
        return ee[2]

    @staticmethod
    def get_velocity(odometry):

        # velocity is in the body reference frame
        v_body = np.array([odometry.twist.twist.linear.x,\
                           odometry.twist.twist.linear.y,\
                           odometry.twist.twist.linear.z])

        quaternion = np.array([odometry.pose.pose.orientation.x,\
                               odometry.pose.pose.orientation.y,\
                               odometry.pose.pose.orientation.z,\
                               odometry.pose.pose.orientation.w])    

        rotation_matrix  = uts.rot_from_quaternion(quaternion)

        v_inertial = np.dot(rotation_matrix,v_body)

        return v_inertial

    @staticmethod
    def get_angular_velocity(odometry):

        # angular velocity is in body reference frame
        omega_body =  np.array([odometry.twist.twist.angular.x,\
                                odometry.twist.twist.angular.y,\
                                odometry.twist.twist.angular.z])

        quaternion = np.array([odometry.pose.pose.orientation.x,\
                               odometry.pose.pose.orientation.y,\
                               odometry.pose.pose.orientation.z,\
                               odometry.pose.pose.orientation.w])    

        rotation_matrix  = uts.rot_from_quaternion(quaternion)

        omega_inertial = np.dot(rotation_matrix,omega_body)

        return omega_inertial

    @staticmethod
    def get_r3(odometry):

        quaternion = np.array([odometry.pose.pose.orientation.x,\
                               odometry.pose.pose.orientation.y,\
                               odometry.pose.pose.orientation.z,\
                               odometry.pose.pose.orientation.w])    

        rotation_matrix  = uts.rot_from_quaternion(quaternion)

        # third column
        r3 = rotation_matrix[:,2]

        return r3

# class OdometryUts():
#     """Utilities for Odometry message"""

#     dictionary['position'] = [odometry.pose.pose.position.x,odometry.pose.pose.position.y,odometry.pose.pose.position.z]
#     dictionary['velocity'] = [odometry.pose.pose.position.x,odometry.pose.pose.position.y,odometry.pose.pose.position.z]

#     return dictionary


# for emergency controller
import controllers.fa_trajectory_tracking_controllers.fa_controller
EMERGENCY_CONTROLLER = controllers.fa_trajectory_tracking_controllers.fa_controller.database['SimplePIDController']

class Mission(js.Jsonable):
    """Any mission returns 3d_force and yaw_rate: this is the standard"""

    # any mission will need to subscribe to something
    # the subscriber provides the "state" of the system 
    # it is the job of the subscriber to subscribe to as many
    # as necessary topics, and construct the state of the system
    # js.Jsonable.add_inner('subscriber',SUBSCRIBERS_DATABASE)

    # time_instant_t0 = 0.0

    def test_emergency(self):
        return not self.check_inside_limits(self.get_position(),self.get_velocity())

    def check_inside_limits(self,position,velocity):

        if any(numpy.absolute(position - self.box_center) >= self.box_sides):
            return False
        else:
            return True

    def trigger_emergency(self):

        self.emergency_time_instant = rospy.get_time()

        self.odometry_at_emergency = self.uav_odometry
        self.compute_3d_force = self.compute_3d_force_emergency
        self.compute_yaw_rate = self.compute_yaw_rate_emergency

    def compute_3d_force_emergency(self,time_instant):

        # getting the uav_odometry is delegated to the susbscriber
        self.uav_odometry = self.get_uav_odometry()  

        position = OdometryUts.get_position(self.uav_odometry)
        velocity = OdometryUts.get_velocity(self.uav_odometry)
        controller_state = numpy.concatenate([position,velocity])
        
        old_position = OdometryUts.get_position(self.odometry_at_emergency)
        if rospy.get_time() - self.emergency_time_instant < 5:
            reference = numpy.concatenate([old_position,numpy.zeros(3+3)])
        else:
            reference = numpy.concatenate([old_position[0:2],[0.3],numpy.zeros(3+3)])

        # compute 3d_force, which is what all controllers provide
        self.desired_3d_force = self.emergency_controller.output(0.0,controller_state, reference)

        return self.desired_3d_force

    def compute_yaw_rate_emergency(self,time_instant):

        # compute yaw_rate, which is what all yaw_controllers provide
        self.desired_yaw_rate = 0.0

        return self.desired_yaw_rate

    """Labels for the columns of a file that stores data from this mission."""
    file_labels = [
        'time',
        'position_x',
        'position_y',
        'position_z',
        'velocity_x',
        'velocity_y',
        'velocity_z',
        'position_x_desired',
        'position_y_desired',
        'position_z_desired',
        'velocity_x_desired',
        'velocity_y_desired',
        'velocity_z_desired',        
        'roll',
        'pitch',
        'yaw',
        'force_x',
        'force_y',
        'force_z'
    ]

    @classmethod
    def get_data_size(self):
        return 19

    def get_data(self):
        """Get all data relevant to the mission
        from this data, mission should be able to do data post-analysis, 
        like ploting or computing average errors
        """        
        default_array = [rospy.get_time()]
        default_array+=self.get_position().tolist()
        default_array+=self.get_velocity().tolist()
        default_array+=self.get_position_desired().tolist()
        default_array+=self.get_velocity_desired().tolist()
        default_array+=self.get_euler_angles().tolist()
        default_array+=self.desired_3d_force.tolist()
        
        # default_array = np.concatenate([default_array,self.get_complementary_data()])
        return default_array


    @classmethod
    def plot_from_string(cls, string,starting_point):
        
        times        = []
        
        positions_x  = []
        positions_y  = []
        positions_z  = []

        velocities_x = []
        velocities_y = []
        velocities_z = []

        positions_x_d  = []
        positions_y_d  = []
        positions_z_d  = []

        velocities_x_d = []
        velocities_y_d = []
        velocities_z_d = []

        rolls        = []
        pitches      = []
        yaws         = []

        forces_x   = []
        forces_y   = []
        forces_z   = []
        

        for line in string.split('\n'):
            # ignore empty lines           
            if line:
                numbers = line.split(' ')

                numbers = numbers[starting_point[0]:]

                times.append(float(numbers[0]))

                positions_x.append(float(numbers[1]))
                positions_y.append(float(numbers[2]))
                positions_z.append(float(numbers[3]))
                
                velocities_x.append(float(numbers[4]))
                velocities_y.append(float(numbers[5]))
                velocities_z.append(float(numbers[6]))

                positions_x_d.append(float(numbers[7]))
                positions_y_d.append(float(numbers[8]))
                positions_z_d.append(float(numbers[9]))
                
                velocities_x_d.append(float(numbers[10]))
                velocities_y_d.append(float(numbers[11]))
                velocities_z_d.append(float(numbers[12]))

                rolls.append(float(numbers[13]))
                pitches.append(float(numbers[14]))
                yaws.append(float(numbers[15]))
                
                forces_x.append(float(numbers[16]))
                forces_y.append(float(numbers[17]))
                forces_z.append(float(numbers[18]))
                
                #yaw_rates.append(float(numbers[13]))

        # it is not bad to import it here, since plotting is done aposteriori
        import operator

        times = map(operator.sub,times,len(times)*[times[0]])

        fig1 = plt.figure()
        plt.plot(times[1:], map(operator.sub,times[1:],times[0:-1]), 'r-', label=r'$\Delta t$')       
        plt.title('Time Interval (s)')
        plt.legend(loc='best')
        plt.grid()
        
        fig2 = plt.figure()
        plt.plot(times, positions_x, 'r-', label=r'$x$')
        plt.plot(times, positions_y, 'g-', label=r'$y$')
        plt.plot(times, positions_z, 'b-', label=r'$z$')
        plt.plot(times, positions_x_d, 'r--', label=r'$x^{\star}$')
        plt.plot(times, positions_y_d, 'g--', label=r'$y^{\star}$')
        plt.plot(times, positions_z_d, 'b--', label=r'$z^{\star}$')        
        plt.title('Positions (m)')
        plt.legend(loc='best')
        plt.grid()

        fig3 = plt.figure()
        plt.plot(times, velocities_x, 'r-', label=r'$v_x$')
        plt.plot(times, velocities_y, 'g-', label=r'$v_y$')
        plt.plot(times, velocities_z, 'b-', label=r'$v_z$')
        plt.plot(times, velocities_x_d, 'r--', label=r'$v_x^{\star}$')
        plt.plot(times, velocities_y_d, 'g--', label=r'$v_y^{\star}$')
        plt.plot(times, velocities_z_d, 'b--', label=r'$v_z^{\star}$')        
        plt.title('Velocities (m/s)')
        plt.legend(loc='best')
        plt.grid()        
        
        fig4 = plt.figure()
        plt.plot(times, rolls, 'r-', label=r'$\phi$')
        plt.plot(times, pitches, 'g-', label=r'$\theta$')
        plt.plot(times, yaws, 'b-', label=r'$\psi$')
        plt.title('Attitudes (deg)')
        plt.legend(loc='best')
        plt.grid()
        # plt.draw()
        
        fig5 = plt.figure()
        plt.plot(times, forces_x, 'r-', label=r'$F_x$')
        plt.plot(times, forces_y, 'g-', label=r'$F_y$')
        plt.plot(times, forces_z, 'b-', label=r'$F_z$')
        plt.title('Forces (N)')
        plt.legend(loc='best')
        plt.grid()
        # plt.draw()

        return fig1,fig2,fig3,fig4,fig5


    def description(cls):
        return "Abstract Mission"
    
    
    def __init__(self):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        # initialize initial time
        self.reset_initial_time()         
        
        # initialize state: ultimately defined by child class
        self.initialize_state()

        # initialize desired_3d_force
        self.desired_3d_force = np.zeros(3)
        self.desired_yaw_rate = 0.0

        self.get_sample = self.subscriber.get_sample

        self.emergency_controller = EMERGENCY_CONTROLLER()

        self.box_center = numpy.array([0.0,0.0,1.0])
        self.box_sides  = numpy.array([2.0,2.0,1.5])
        # go down to z = ... in meters when emergency is triggered
        self.position_z_before_land = 0.3

        return

    def reset_initial_time(self,time_instant = None):
        if time_instant == None:
            # TODO
            # self.time_instant_t0 = rospy.get_time()
            self.time_instant_t0 = 0
        else:
            self.time_instant_t0 = time_instant        

    # any mission needs to compute the desired_3d_force
    # that uav will be required to exert
    def compute_3d_force(self,time_instant):
        raise NotImplementedError()

    # any mission needs to compute the desired_yaw_rate
    def compute_yaw_rate(self,time_instant):
        raise NotImplementedError()        

    def get_position(self):
        """Get position (m)"""
        return OdometryUts.get_position(self.uav_odometry)

    def get_velocity(self):
        """Get position (m) and velocity (m/s) of quadrotor"""
        return OdometryUts.get_velocity(self.uav_odometry)

    def get_euler_angles(self):
        # TODO: CHECK WHETHER IT SHOULD BE RAD OR NOT
        """Get euler angles of the quadrotor (rad)"""
        return OdometryUts.get_euler_angles(self.uav_odometry)

    def get_yaw(self):
        "Get psi angle in rad"
        return OdometryUts.get_yaw(self.uav_odometry)

    # any mission needs to provides the methods bellow
    def get_position_desired(self):
        """Get desired position (m) and velocity (m/s) for the quadrotor"""
        return NotImplementedError()

    def get_velocity_desired(self):
        """Get desired position (m) and velocity (m/s) for the quadrotor"""
        return NotImplementedError()

    def get_euler_angles_desired(self):
        """Get desired position (m) and velocity (m/s) for the quadrotor"""
        return NotImplementedError()

    def get_input(self):
        """Get output in standard form:
        . force_ratio = 100*(1 - force/total_weight)
        . roll (deg) 
        . pitch (deg)
        . yaw_rate(deg/secs)
        """

        # initilize output
        out = np.zeros(4)


        # first output is force ratio
        # norm of force
        force_ratio = np.linalg.norm(self.desired_3d_force)
        total_weight = self.get_total_weight()
        # this should be zero in "normal" equilibrium 
        out[0] = 100*(force_ratio/(total_weight) - 1)

        # second and third outputs are pitch and roll angles
        out[1],out[2] = utility_functions.roll_and_pitch_from_full_actuation_and_yaw_rad(self.desired_3d_force,3.142/180.0*self.get_yaw())
        # angles are in degrees
        # roll
        out[1] *= 180.0/3.14159
        # pitch
        out[2] *= 180.0/3.14159

        # fourth output is yaw_rate in deg/sec
        out[3] = self.desired_yaw_rate*180.0/3.14159

        return out

    def get_complementary_data(self):
        """Get complementary data that is not included 
        in get_complete_data() already
        """
        # by default return empty array
        # return np.array([])
        return NotImplementedError()

    def get_labels_complementary_data(self):
        """Get labels of the complementary data as defined in get_complementary_data()
        """
        # by default return empty list of labels
        # return []
        return NotImplementedError()

#-------------------------------------------------------#
#-------------------------------------------------------#
# Trajectory Tracking

 
 # import list of available trajectories
import reference.trajectory
TRAJECTORIES_DATABASE = reference.trajectory.database

# import controllers dictionary
import controllers.fa_trajectory_tracking_controllers.fa_controller
CONTROLLERS_DATABASE = controllers.fa_trajectory_tracking_controllers.fa_controller.database

# import yaw controllers dictionary
import controllers.yaw_controllers.yaw_controller
YAW_CONTROLLERS_DATABASE = controllers.yaw_controllers.yaw_controller.database

# import list of available yaw trajectories
import reference.yaw_trajectory
YAW_TRAJECTORIES_DATABASE = reference.yaw_trajectory.database

@js.inherit_methods_list_from_parents
class TrajectoryTrackingUAV(Mission):

    js.Jsonable.add_inner('controller',CONTROLLERS_DATABASE)
    js.Jsonable.add_inner('reference',TRAJECTORIES_DATABASE)
    js.Jsonable.add_inner('yaw_controller',YAW_CONTROLLERS_DATABASE)
    js.Jsonable.add_inner('yaw_reference',YAW_TRAJECTORIES_DATABASE)
    #js.Jsonable.add_inner('subscriber',SUBSCRIBERS_DATABASE)

    @classmethod
    def description(cls):
        description = """
        <b>Track a desired trajectory</b>. This mission depends on:
        <ul>
          <li>controller: a trajectory tracking controller</li>
          <li>reference: a reference position trajectory to be tracked</li>
          <li>yaw_controller: a yaw controller</li>
          <li>yaw_reference: a yaw reference</li>
        </ul>
        """
        return description
    
    # NOTE: we may define different init according to TYPE_UAV
    def __init__(self):
        # Copy inner_keys into self variables
        # Subscribe to the necessary topics, if any

        self.add_inner_defaults()

        # initialize time instant, and  state
        Mission.__init__(self)

        # inspect.getargspec(Subscriber.__init__)
        # self.subscriber = self.Subscriber()

        # dy default, desired trajectory is staying still in origin
        self.current_reference = self.reference.output(self.time_instant_t0)

        #Get desired yaw in radians, and its time derivative
        input_for_yaw_controller   = self.yaw_reference.output(self.time_instant_t0)
        self.current_yaw_reference = input_for_yaw_controller[0]
        
    def get_total_weight(self):
        return self.controller.get_total_weight()

    def initialize_state(self):

        # initialize uav_odometry
        # from which we get position, velocity
        # euler angles and angular velocities
        self.uav_odometry = Odometry()    

    def object_description(self):
        description = """
        <b>Track a desired trajectory</b>. This mission depends on:
        <ul>
          <li>controller: a trajectory tracking controller</li>
          <li>reference: a reference position trajectory to be tracked</li>
          <li>yaw_controller: a yaw controller</li>
          <li>yaw_reference: a yaw reference</li>
        </ul>
        """
        return description

    def compute_3d_force(self,time_instant):

        # getting the uav_odometry is delegated to the susbscriber
        self.uav_odometry = self.subscriber.get_uav_odometry()

        # job of the subscriber to provide state to the controller
        # in the standard required by the controller
        # in this mission, the controller state standard is
        # position,velocity,euler_angles
        controller_state = self.subscriber.get_controller_state()
        #print(controller_state)
        
        # compute reference
        # all controllers receive reference of the form
        # position^(0),...,position^(4)
        reference = self.reference.output(time_instant)
        self.current_reference = reference    

        # compute 3d_force, which is what all controllers provide
        self.desired_3d_force = self.controller.output(time_instant,controller_state, reference)

        return self.desired_3d_force   

    def compute_yaw_rate(self,time_instant):

        # job of the subscriber to provide state to the yaw_controller
        # in the standard required by the yaw_controller
        # in this mission, the yaw_controller state standard is
        # position,velocity,euler_angles
        yaw_controller_state = self.subscriber.get_yaw_controller_state()

        #Get desired yaw in radians, and its time derivative
        input_for_yaw_controller   = self.yaw_reference.output(time_instant)
        self.current_yaw_reference = input_for_yaw_controller[0]

        # compute yaw_rate, which is what all yaw_controllers provide
        self.desired_yaw_rate = self.yaw_controller.output(yaw_controller_state,input_for_yaw_controller)
        
        return self.desired_yaw_rate

    def get_position_desired(self):
        return self.current_reference[0:3]
    def get_velocity_desired(self):
        return self.current_reference[3:6]
    def get_euler_angles_desired(self):
        return numpy.array([0.0,0.0,self.current_yaw_reference*180.0/3.142])

    # recall that getting the uav_odometry was delegated to the susbscriber
    def get_uav_odometry(self):
        return self.subscriber.get_uav_odometry()

    @js.add_to_methods_list
    def print_sum(self,a=1,b=1):
        print(a+b)

    def hold_position(self,position=numpy.zeros(3)):
        # dy default, desired trajectory is staying still in origin
        reference      = trajectories_database.database["Default"]()
        self.reference = reference
        self.reference.set_offset(offset=numpy.concatenate([position[0:2],[0.0]]))
        return


#-------------------------------------------------------#
#-------------------------------------------------------#
# Load Lifting

# import list of available trajectories
import reference.trajectory
TRAJECTORIES_DATABASE = reference.trajectory.database

import controllers.single_load_transportation_controllers.single_load_transportation_controller
CONTROLLERS_DATABASE = controllers.single_load_transportation_controllers.single_load_transportation_controller.database

# import yaw controllers dictionary
import controllers.yaw_controllers.yaw_controller
YAW_CONTROLLERS_DATABASE = controllers.yaw_controllers.yaw_controller.database

# import list of available yaw trajectories
import reference.yaw_trajectory
YAW_TRAJECTORIES_DATABASE = reference.yaw_trajectory.database

@js.inherit_methods_list_from_parents
class LoadLiftingUAV(Mission):


    js.Jsonable.add_inner('controller',CONTROLLERS_DATABASE)
    js.Jsonable.add_inner('reference',TRAJECTORIES_DATABASE)
    js.Jsonable.add_inner('yaw_controller',YAW_CONTROLLERS_DATABASE)
    js.Jsonable.add_inner('yaw_reference',YAW_TRAJECTORIES_DATABASE)

    @classmethod
    def description(cls):
        description = """
        <b>Load attached to uav to track a desired trajectory</b>. This mission depends on:
        <ul>
          <li>controller: a trajectory tracking controller for system "load+uav"</li>
          <li>reference: a reference position trajectory to be tracked by load</li>
          <li>yaw_controller: a yaw controller</li>
          <li>yaw_reference: a yaw reference</li>
        </ul>
        """
        return description        
        
    def __init__(self):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        self.add_inner_defaults()

        # initialize time instant, and  state
        Mission.__init__(self)

        # self.subscriber = self.Subscriber()

        # dy default, desired trajectory is staying still in origin
        self.current_reference = self.reference.output(self.time_instant_t0)

        #Get desired yaw in radians, and its time derivative
        input_for_yaw_controller   = self.yaw_reference.output(self.time_instant_t0)
        self.current_yaw_reference = input_for_yaw_controller[0]

    def object_description(self):
        string = """No parameters"""
        return string

    def get_total_weight(self):
        return self.controller.get_total_weight()

    def initialize_state(self):

        # initialize uav_odometry
        # from which we get position, velocity
        # euler angles and angular velocities
        self.uav_odometry = Odometry()

    def __str__(self):
        return self.description()
        # Add the self variables

    def compute_3d_force(self,time_instant):

        # getting the uav_odometry is delegated to the susbscriber
        self.uav_odometry = self.get_uav_odometry()

        # job of the subscriber to provide state to the controller
        # in the standard required by the controller
        # in this mission, the controller state standard is
        # position,velocity,euler_angles
        controller_state = self.subscriber.get_controller_state()
        
        # compute reference
        # all controllers receive reference of the form
        # position^(0),...,position^(4)
        reference = self.reference.output(time_instant)
        self.current_reference = reference    

        # compute 3d_force, which is what all controllers provide
        self.desired_3d_force = self.controller.output(time_instant,controller_state, reference)

        # if numpy.linalg.norm(self.desired_3d_force) > 0.1:
        #     r3                     = OdometryUts.get_r3(self.uav_odometry)
        #     self.desired_3d_force *= numpy.linalg.norm(self.desired_3d_force)/numpy.dot(self.desired_3d_force,r3)

        return self.desired_3d_force   

    def compute_yaw_rate(self,time_instant):

        # job of the subscriber to provide state to the yaw_controller
        # in the standard required by the yaw_controller
        # in this mission, the yaw_controller state standard is
        # position,velocity,euler_angles
        yaw_controller_state = self.subscriber.get_yaw_controller_state()

        #Get desired yaw in radians, and its time derivative
        input_for_yaw_controller   = self.yaw_reference.output(time_instant)
        self.current_yaw_reference = input_for_yaw_controller[0]

        # compute yaw_rate, which is what all yaw_controllers provide
        self.desired_yaw_rate = self.yaw_controller.output(yaw_controller_state,input_for_yaw_controller)
        
        return self.desired_yaw_rate

    def get_position_desired(self):
        # TODO: correct this
        return self.current_reference[0:3]
    def get_velocity_desired(self):
        return self.current_reference[3:6]
    def get_euler_angles_desired(self):
        return numpy.array([0.0,0.0,self.current_yaw_reference*180.0/3.142])

    # recall that getting the uav_odometry was delegated to the susbscriber
    def get_uav_odometry(self):
        return self.subscriber.get_uav_odometry()


@js.inherit_methods_list_from_parents
class LoadLiftingLTLUAV(LoadLiftingUAV):

    def __init__(self):
        LoadLifting.__init__(self)

    # "override" the parents method
    def compute_3d_force(self,time_instant):

        # getting the uav_odometry is delegated to the susbscriber
        self.uav_odometry = self.get_uav_odometry()

        # job of the subscriber to provide state to the controller
        # in the standard required by the controller
        # in this mission, the controller state standard is
        # position,velocity,euler_angles
        controller_state = self.subscriber.get_controller_state()
        

        reference_speed = self.subscriber.get_reference()

        # compute reference
        # all controllers receive reference of the form
        # position^(0),...,position^(4)
        # reference = self.reference.output(time_instant)
        # x,y reference comes from LTL
        reference = numpy.concatenate([controller_state[6:8],[0],reference_speed,[0],numpy.zeros(3*3)])

        # reference = numpy.concatenate([controller_state[6:8],[0],numpy.array([0.0,0.1]),[0],numpy.zeros(3*3)])

        self.current_reference = reference    

        # compute 3d_force, which is what all controllers provide
        self.desired_3d_force = self.controller.output(time_instant,controller_state, reference)

        # if numpy.linalg.norm(self.desired_3d_force) > 0.1:
        #     r3                     = OdometryUts.get_r3(self.uav_odometry)
        #     self.desired_3d_force *= numpy.linalg.norm(self.desired_3d_force)/numpy.dot(self.desired_3d_force,r3)

        return self.desired_3d_force


import subscriber

class FireflyGazebo():
    DATABASE = subscriber.FireflyGazeboSubscriber.database

    @js.add_to_database(default=True)
    class TrajectoryTracking(TrajectoryTrackingUAV):
        def __init__(self):
            self.subscriber = FireflyGazebo.DATABASE['TrajectoryTracking']()
            TrajectoryTrackingUAV.__init__(self)

    @js.add_to_database()
    class LoadLifting(LoadLiftingUAV):
        def __init__(self):
            self.subscriber = FireflyGazebo.DATABASE['LoadLifting']()
            LoadLiftingUAV.__init__(self)

    @js.add_to_database()
    class LoadLiftingLTL(LoadLiftingLTLUAV):
        def __init__(self):
            self.subscriber = FireflyGazebo.DATABASE['LoadLiftingLTL']()
            LoadLiftingLTLUAV.__init__(self)

class IrisRviz():
    DATABASE = subscriber.IrisRvizSubscriber.database

    @js.add_to_database(default=True)
    class TrajectoryTracking(TrajectoryTrackingUAV):
        def __init__(self):
            self.subscriber = IrisRviz.DATABASE['TrajectoryTracking']()
            TrajectoryTrackingUAV.__init__(self)

    @js.add_to_database()
    class LoadLifting(LoadLiftingUAV):
        def __init__(self):
            self.subscriber = IrisRviz.DATABASE['LoadLifting']()
            LoadLiftingUAV.__init__(self)

class Iris():

    DATABASE = subscriber.IrisSubscriber.database

    class IrisSubscriber():

        @js.add_to_methods_list
        def change_to_stabilize(self):
            '''change to stabilize'''
            self.subscriber.set_flight_mode(MODE = 'STABILIZE')

        @js.add_to_methods_list
        def change_to_land(self):
            '''change to stabilize'''
            self.subscriber.set_flight_mode(MODE = 'LAND')

        @js.add_to_methods_list
        def change_to_acro(self):
            '''change to stabilize'''
            self.subscriber.set_flight_mode(MODE = 'ACRO')

        @js.add_to_methods_list
        def arming_iris(self,neutral_controller=True):
            'Arm IRIS (setting minimum throttle is done automatically)'

            if neutral_controller:
                print('Minimum throtlle automatically set')
                # set minimum throttle
                DATABASE = controllers.fa_trajectory_tracking_controllers.fa_controller.database
                # THIS IS PROBLEMATIIC
                # self.controller = DATABASE['NeutalController']()
                self.change_inner_key('controller','NeutalController',"{}")
            else:
                print('Minimum throtlle NOT automatically set')

            # arm iris
            self.subscriber.arming_iris()

        @js.add_to_methods_list
        def unarming_iris(self):
            'Unarm IRIS'
            self.subscriber.unarming_iris()

        @js.add_to_methods_list
        def see_all_mocap_bodies(self):
            self.subscriber.see_all_mocap_bodies()

    @js.inherit_methods_list_from_parents
    @js.add_to_database(default=True)
    class TrajectoryTracking(TrajectoryTrackingUAV,IrisSubscriber):

        def __init__(self,
        body_name = rospy.get_param('IRIS_BODY_NAME','Iris2')
        ):
            self.body_name = body_name
            self.subscriber = Iris.DATABASE['TrajectoryTracking'](body_name = body_name)
            TrajectoryTrackingUAV.__init__(self)

    @js.inherit_methods_list_from_parents
    @js.add_to_database()
    class LoadLifting(LoadLiftingUAV,IrisSubscriber):

        def __init__(self,
        body_name = rospy.get_param('IRIS_BODY_NAME','Iris2'),
        load_name = rospy.get_param('LOAD_BODY_NAME','Load1')
        ):
            self.body_name = body_name
            self.load_name = load_name
            self.subscriber = Iris.DATABASE['LoadLifting'](body_name=body_name,load_name=load_name)
            LoadLiftingUAV.__init__(self)


class NEO():

!!!    DATABASE = subscriber.NEOSubscriber.database

    class NEOSubscriber():
        pass

    @js.inherit_methods_list_from_parents
    @js.add_to_database(default=True)
    class TrajectoryTracking(TrajectoryTrackingUAV,NEOSubscriber):

        def __init__(self,
        body_name = rospy.get_param('IRIS_BODY_NAME','Iris2')
        ):
            self.body_name = body_name
            self.subscriber = Iris.DATABASE['TrajectoryTracking'](body_name = body_name)
            TrajectoryTrackingUAV.__init__(self)
