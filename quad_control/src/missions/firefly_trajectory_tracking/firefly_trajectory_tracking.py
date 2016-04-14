#!/usr/bin/env python

# import Misson abstract class
from .. import mission

from ConverterBetweenStandards.RotorSConverter import RotorSConverter

# node will publish motor speeds
from mav_msgs.msg import Actuators

#node will subscribe to odometry measurements
from nav_msgs.msg import Odometry

# import list of available trajectories
from TrajectoryPlanner import trajectories_dictionary

# import controllers dictionary
from Yaw_Rate_Controller import yaw_controllers_dictionary

# import controllers dictionary
from controllers_hierarchical.fully_actuated_controllers import controllers_dictionary

import math
import numpy

# for subscribing to topics, and publishing
import rospy

class FireflyTrajectoryTracking(mission.Mission):

    @classmethod
    def description(cls):
        return "Firefly, from RotorS, to track a desired trajectory"
    
    
    def __init__(self):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        mission.Mission.__init__(self)
    
        self.inner['controllers_dictionary']     = controllers_dictionary.controllers_dictionary
        self.inner['trajectories_dictionary']    = trajectories_dictionary.trajectories_dictionary
        self.inner['yaw_controllers_dictionary'] = yaw_controllers_dictionary.yaw_controllers_dictionary

        # converting our controlller standard into rotors standard
        self.RotorSObject = RotorSConverter()

        # subscriber to odometry from RotorS
        self.sub_odometry = rospy.Subscriber("/firefly/ground_truth/odometry", Odometry, self.get_state_from_rotorS_simulator)

        # publisher: command firefly motor speeds 
        self.pub_motor_speeds = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=10)      

        # dy default, desired trajectory is staying still in origin
        TrajectoryClass    = self.inner['trajectories_dictionary']['StayAtRest']
        self.TrajGenerator = TrajectoryClass()
        self.reference     = self.TrajGenerator.output(self.time_instant_t0)

        # controllers selected by default
        ControllerClass       = self.inner['controllers_dictionary']['PIDSimpleBoundedIntegralController']
        self.ControllerObject = ControllerClass()

        # controllers selected by default
        YawControllerClass       = self.inner['yaw_controllers_dictionary']['YawRateControllerTrackReferencePsi']
        self.YawControllerObject = YawControllerClass()

        pass  

    def initialize_state(self):
        # state of quad: position, velocity and attitude
        # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = numpy.zeros(3+3+3)
        pass
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        self.sub_odometry.unregister()
        pass

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
        pass