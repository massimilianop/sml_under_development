#!/usr/bin/env python

# import Misson abstract class
from .. import mission

from converter_between_standards.rotorS_converter import RotorSConverter

# node will publish motor speeds
from mav_msgs.msg import Actuators

#node will subscribe to odometry measurements
from nav_msgs.msg import Odometry

# import list of available trajectories
from trajectories import trajectories_database

# import yaw controllers dictionary
from yaw_rate_controllers import yaw_controllers_database

# import controllers dictionary
from controllers.fa_trajectory_tracking_controllers import fa_trajectory_tracking_controllers_database

import math
import numpy

# for subscribing to topics, and publishing
import rospy


class FireflyLoadLifting(mission.Mission):

    inner = {}

    inner['controller']     = fa_trajectory_tracking_controllers_database.database
    inner['reference']      = trajectories_database.database
    inner['yaw_controller'] = yaw_controllers_database.database


    @classmethod
    def description(cls):
        string = """Firefly and load, from RotorS, to track a desired trajectory
        This mission depends on:
        . a trajectory tracking controller
        . a reference trajectory to be tracked
        . a yaw controller
        """
        
        return string
        
    def __init__(self,
            controller     = fa_trajectory_tracking_controllers_database.database["Default"](),
            reference      = trajectories_database.database["Default"](),
            yaw_controller = yaw_controllers_database.database["Default"]()
            ):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        # initialize time instant, and  state
        mission.Mission.__init__(self)

        # converting our controlller standard into rotors standard
        self.RotorSObject = RotorSConverter()

        # subscriber to odometry of firefly center of mass from RotorS
        self.sub_odometry = rospy.Subscriber(
            "/firefly/ground_truth/odometry",
            Odometry,
            self.get_state_from_rotorS_simulator
            )

        # subscriber: to odometry of load attached to firefly
        self.sub_odometry_load = rospy.Subscriber(
            "/firefly/ground_truth/odometry_load",
            Odometry,
            self.update_load_odometry) 


        # publisher: command firefly motor speeds 
        self.pub_motor_speeds = rospy.Publisher(
            '/firefly/command/motor_speed',
            Actuators,
            queue_size=10
            )

        # dy default, desired trajectory is staying still in origin
        self.TrajGenerator = reference
        self.reference     = self.TrajGenerator.output(self.time_instant_t0)

        # controllers selected by default
        self.ControllerObject = controller

        # controllers selected by default
        self.YawControllerObject = yaw_controller


    def initialize_state(self):
        # state of quad: position, velocity and attitude
        # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = numpy.zeros(3+3+3)

        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        self.sub_odometry.unregister()


    def get_quad_ea_rad(self):
    	euler_rad     = self.state_quad[6:9]*math.pi/180
    	euler_rad_dot = numpy.zeros(3)
    	return numpy.concatenate([euler_rad,euler_rad_dot])


    def get_reference(self,time_instant):
        self.reference = self.TrajGenerator.output(time_instant)
        return self.reference


    def get_state(self):
        return self.state_quad


    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.reference[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]


    def real_publish(self,desired_3d_force_quad,yaw_rate):
		# publish message
		self.pub_motor_speeds.publish(self.RotorSObject.rotor_s_message(desired_3d_force_quad,yaw_rate))


    # callback when ROTORS simulator publishes states
    def get_state_from_rotorS_simulator(self,odometry_rotor_s):        
        # RotorS has attitude inner loop that needs to known attitude of quad
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        # get state from rotorS simulator
        self.state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)


    def update_load_odometry(self,data_odometry):

        self.load_odometry_position = numpy.array([data_odometry.pose.pose.position.x,\
                                                   data_odometry.pose.pose.position.y,\
                                                   data_odometry.pose.pose.position.z])

        # beware that velocity obtained in this way is expressed in body reference frame
        # self.load_odometry_velocity = numpy.array([data_odometry.twist.twist.linear.x,\
        #                                            data_odometry.twist.twist.linear.y,\
        #                                            data_odometry.twist.twist.linear.z])

        current_time  = data_odometry.header.stamp.secs + data_odometry.header.stamp.nsecs/1e9
        self.load_odometry_velocity = self.LoadVelocityEstimator.out(self.load_odometry_position,current_time)