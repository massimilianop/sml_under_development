#!/usr/bin/env python

from utilities import jsonable as js
from utilities import utility_functions

from utilities import utility_functions as uts

import numpy as np
import math
import numpy

# for getting time
import rospy

import nav_msgs.msg

#node will subscribe to uav_odometry measurements
from nav_msgs.msg import Odometry

#plots
# import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

import tf.transformations

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

    def description(cls):
        return "Abstract Mission"
    
    
    def __init__(self):

        # initialize initial time
        self.reset_initial_time()

        self.emergency_controller = EMERGENCY_CONTROLLER()

        return

    def reset_initial_time(self,time_instant = None):
        if time_instant == None:
            # TODO self.time_instant_t0 = rospy.get_time()
            self.time_instant_t0 = 0
        else:
            self.time_instant_t0 = time_instant

    def trigger_emergency(self,
        uav_odometry):

        self.emergency_time_instant = rospy.get_time()

        self.odometry_at_emergency = uav_odometry
        pos = uav_odometry.pose.pose.position
        self.position_at_emergency = np.array([pos.x, pos.y, pos.z])

        self.compute_3d_force = self.compute_3d_force_emergency
        self.compute_yaw_rate = self.compute_yaw_rate_emergency

    def compute_3d_force_emergency(self,
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry()):
        
        old_position = self.position_at_emergency
        if rospy.get_time() - self.emergency_time_instant < 5:
            reference = numpy.concatenate([old_position,numpy.zeros(3+3)])
        else:
            reference = numpy.concatenate([old_position[0:2],[0.2],numpy.zeros(3+3)])

        # compute 3d_force, which is what all controllers provide
        desired_3d_force = self.emergency_controller.output(
            time_instant = 0.0, 
            uav_odometry = uav_odometry,
            reference = reference)

        return desired_3d_force

    def compute_yaw_rate_emergency(self,
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry()):

        # compute yaw_rate, which is what all yaw_controllers provide
        self.desired_yaw_rate = 0.0

        return self.desired_yaw_rate

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

    def compute_3d_force(self,
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry()):

        # job of the subscriber to provide state to the controller
        # in the standard required by the controller
        # in this mission, the controller state standard is
        # position,velocity,euler_angles
        #controller_state = self.subscriber.get_controller_state()
        #print(controller_state)
        
        # compute reference
        # all controllers receive reference of the form
        # position^(0),...,position^(4)
        reference = self.reference.output(time_instant)
        self.current_reference = reference

        desired_3d_force = self.controller.output(
            time_instant = time_instant, 
            uav_odometry = uav_odometry,
            reference = reference)

        return desired_3d_force

    def compute_yaw_rate(self,
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry()):

        #Get desired yaw in radians, and its time derivative
        input_for_yaw_controller   = self.yaw_reference.output(time_instant)
        self.current_yaw_reference = input_for_yaw_controller[0]

        # compute yaw_rate, which is what all yaw_controllers provide
        self.desired_yaw_rate = self.yaw_controller.output(
            uav_odometry = uav_odometry,
            reference = input_for_yaw_controller)

        return self.desired_yaw_rate

    def get_position_desired(self):
        return self.current_reference[0:3]
    def get_velocity_desired(self):
        return self.current_reference[3:6]
    def get_euler_angles_desired(self):
        return numpy.array([0.0,0.0,self.current_yaw_reference*180.0/3.142])

    # recall that getting the uav_odometry was delegated to the susbscriber
    def get_uav_odometry(self):
        return self.uav_odometry


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

        self.sub_load_odometry = rospy.Subscriber(
            name       = rospy.get_param('load_odometry'),
            data_class = nav_msgs.msg.Odometry,
            callback   = self.update_load_odometry)
        self.load_odometry = nav_msgs.msg.Odometry()

    def update_load_odometry(self,msg = nav_msgs.msg.Odometry()):
        self.load_odometry = msg

    def object_description(self):
        string = """No parameters"""
        return string

    def initialize_state(self):

        # initialize uav_odometry
        # from which we get position, velocity
        # euler angles and angular velocities
        #self.uav_odometry = Odometry()
        pass
    def __str__(self):
        return self.description()
        # Add the self variables

    def compute_3d_force(self,
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry()):

        # job of the subscriber to provide state to the controller
        # in the standard required by the controller
        # in this mission, the controller state standard is
        # position,velocity,euler_angles
        #controller_state = self.subscriber.get_controller_state()
        #print(controller_state)
        
        # compute reference
        # all controllers receive reference of the form
        # position^(0),...,position^(4)
        reference = self.reference.output(time_instant)
        self.current_reference = reference

        # # compute 3d_force, which is what all controllers provide
        # self.desired_3d_force = self.controller.output(time_instant, 
        #     uav_odometry = self.uav_odometry, 
        #     reference)

        desired_3d_force = self.controller.output(time_instant, 
            uav_odometry,
            self.load_odometry,
            reference)

        return desired_3d_force

    def compute_yaw_rate(self,
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry()):

        #Get desired yaw in radians, and its time derivative
        input_for_yaw_controller   = self.yaw_reference.output(time_instant)
        self.current_yaw_reference = input_for_yaw_controller[0]

        # # compute yaw_rate, which is what all yaw_controllers provide
        # self.desired_yaw_rate = self.yaw_controller.output(
        #     uav_odometry = self.uav_odometry,
        #     input_for_yaw_controller)
        
        # compute yaw_rate, which is what all yaw_controllers provide
        self.desired_yaw_rate = self.yaw_controller.output(
            uav_odometry = uav_odometry,
            reference = input_for_yaw_controller)

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
        return self.uav_odometry


#-------------------------------------------------------#
#-------------------------------------------------------#
# Load Lifting

# import list of available trajectories
import reference.trajectory
TRAJECTORIES_DATABASE = reference.trajectory.database

import controllers.collaborative_trajectory_tracking_controllers.controller
CONTROLLERS_DATABASE = controllers.collaborative_trajectory_tracking_controllers.controller.database

# import yaw controllers dictionary
import controllers.yaw_controllers.yaw_controller
YAW_CONTROLLERS_DATABASE = controllers.yaw_controllers.yaw_controller.database

# import list of available yaw trajectories
import reference.yaw_trajectory
YAW_TRAJECTORIES_DATABASE = reference.yaw_trajectory.database

@js.inherit_methods_list_from_parents
class BarLiftingUAV(Mission):

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

        self.sub_bar_odometry = rospy.Subscriber(
            name       = rospy.get_param('bar_odometry'),
            data_class = nav_msgs.msg.Odometry,
            callback   = self.update_bar_odometry)
        self.bar_odometry = nav_msgs.msg.Odometry()

        self.sub_uav_partner_odometry = rospy.Subscriber(
            name       = rospy.get_param('partner_uav_odometry'),
            data_class = nav_msgs.msg.Odometry,
            callback   = self.update_partner_uav_odometry)
        self.partner_uav_odometry = nav_msgs.msg.Odometry()


    def update_bar_odometry(self,msg = nav_msgs.msg.Odometry()):
        self.bar_odometry = msg

    def update_partner_uav_odometry(self,msg = nav_msgs.msg.Odometry()):
        self.partner_uav_odometry = msg

    def object_description(self):
        string = """No parameters"""
        return string

    def initialize_state(self):

        # initialize uav_odometry
        # from which we get position, velocity
        # euler angles and angular velocities
        #self.uav_odometry = Odometry()
        pass

    def __str__(self):
        return self.description()
        # Add the self variables

    def compute_3d_force(self,
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry()):

        # job of the subscriber to provide state to the controller
        # in the standard required by the controller
        # in this mission, the controller state standard is
        # position,velocity,euler_angles
        #controller_state = self.subscriber.get_controller_state()
        #print(controller_state)
        
        # compute reference
        # all controllers receive reference of the form
        # position^(0),...,position^(4)
        reference = self.reference.output(time_instant)
        self.current_reference = reference

        # # compute 3d_force, which is what all controllers provide
        # self.desired_3d_force = self.controller.output(time_instant, 
        #     uav_odometry = self.uav_odometry, 
        #     reference)

        desired_3d_force = self.controller.output(
            time_instant = time_instant, 
            uav_odometry = uav_odometry,
            partner_uav_odometry = self.partner_uav_odometry,
            bar_odometry = self.bar_odometry,
            reference = reference)

        return desired_3d_force

    def compute_yaw_rate(self,
        time_instant = 0.0,
        uav_odometry = nav_msgs.msg.Odometry()):

        #Get desired yaw in radians, and its time derivative
        input_for_yaw_controller   = self.yaw_reference.output(time_instant)
        self.current_yaw_reference = input_for_yaw_controller[0]

        # # compute yaw_rate, which is what all yaw_controllers provide
        # self.desired_yaw_rate = self.yaw_controller.output(
        #     uav_odometry = self.uav_odometry,
        #     input_for_yaw_controller)
        
        # compute yaw_rate, which is what all yaw_controllers provide
        self.desired_yaw_rate = self.yaw_controller.output(
            uav_odometry = uav_odometry,
            reference = input_for_yaw_controller)

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
        return self.uav_odometry


class FireflyGazebo():

    @js.add_to_database()
    class TrajectoryTracking(TrajectoryTrackingUAV):
        def __init__(self):
            TrajectoryTrackingUAV.__init__(self)

    @js.add_to_database()
    class LoadLifting(LoadLiftingUAV):
        def __init__(self):
            LoadLiftingUAV.__init__(self)

    @js.add_to_database(default=True)
    class BarLifting(BarLiftingUAV):
        def __init__(self):
            BarLiftingUAV.__init__(self)

class IrisRviz():

    @js.add_to_database(default=True)
    class TrajectoryTracking(TrajectoryTrackingUAV):
        def __init__(self):
            TrajectoryTrackingUAV.__init__(self)

class Iris():

    @js.inherit_methods_list_from_parents
    @js.add_to_database(default=True)
    class TrajectoryTracking(TrajectoryTrackingUAV):

        def __init__(self):
            TrajectoryTrackingUAV.__init__(self)

    @js.inherit_methods_list_from_parents
    @js.add_to_database()
    class LoadLifting(LoadLiftingUAV):

        def __init__(self):
            LoadLiftingUAV.__init__(self)


class NEO():

    @js.add_to_database(default=True)
    class TrajectoryTracking(TrajectoryTrackingUAV):

        def __init__(self):
            TrajectoryTrackingUAV.__init__(self)
