#!/usr/bin/env python

# import Misson abstract class
from ... import mission

from converters.rotorS_converter import RotorSConverter

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

# to estimate load velocity: recall that velocity from rotorS comes w.r.t. the body reference frame
from utilities.utility_functions import VelocityFilter

# for subscribing to topics, and publishing
import rospy

from controllers.single_load_transportation_controllers import single_load_transportation_controllers_database


class FireflyLoadLifting(mission.Mission):

    inner = {}

    inner['controller']     = single_load_transportation_controllers_database.database
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
            controller     = single_load_transportation_controllers_database.database["Default"](),
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
            self.update_firefly_odometry
            )

        # subscriber: to odometry of load attached to firefly
        self.sub_odometry_load = rospy.Subscriber(
            "/firefly/ground_truth/odometry_load",
            Odometry,
            self.update_load_odometry) 


        # publisher: command firefly motor speeds 
        dictionary = {}
        dictionary['name'] = "/firefly/command/motor_speed"
        dictionary['data_class'] = Actuators
        dictionary['queue_size'] = 1
        self.pub_motor_speeds = rospy.Publisher(**dictionary)

        self.load_velocity_estimator = VelocityFilter(3,numpy.zeros(3),0.0)
        # self.quad_velocity_estimator = VelocityFilter(3,numpy.zeros(3),0.0)

        # dy default, desired trajectory is staying still in origin
        self.reference = reference
        self.current_reference     = self.reference.output(self.time_instant_t0)

        # controllers selected by default
        self.controller = controller

        # controllers selected by default
        self.yaw_controller = yaw_controller

    def initialize_state(self):
        # state of quad: position, velocity and attitude
        # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad             = numpy.zeros(3+3+3)
        #self.load_odometry_position = numpy.array([0.0,0.0,-self.cable_length]) 
        self.load_odometry_position = numpy.array([0.0,0.0,-1.0]) 
        self.load_odometry_velocity = numpy.zeros(3)

        
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
        self.current_reference = self.reference.output(time_instant)
        return self.current_reference


    def get_state(self):

        position_load = self.load_odometry_position
        velocity_load = self.load_odometry_velocity

        position_quad = self.state_quad[0:3]
        velocity_quad = self.state_quad[3:6]


        state  = numpy.concatenate([position_load, \
                                    velocity_load, \
                                    position_quad, \
                                    velocity_quad ])

        # state  = numpy.concatenate([position_quad, \
        #                             velocity_quad ])

        return state

    def get_position(self):
        return self.state_quad[0:3]

    def get_velocity(self):
        return self.state_quad[3:6]        

    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.current_reference[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]


    def real_publish(self,desired_3d_force_quad,yaw_rate,rc_output):
		# publish message
		self.pub_motor_speeds.publish(self.RotorSObject.rotor_s_message(desired_3d_force_quad,yaw_rate))

    # callback when ROTORS simulator publishes states
    def update_firefly_odometry(self,odometry_rotor_s):        
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

        current_time                = data_odometry.header.stamp.secs + data_odometry.header.stamp.nsecs/1e9
        self.load_odometry_velocity = self.load_velocity_estimator.out(self.load_odometry_position,current_time)