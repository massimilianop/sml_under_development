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

# import list of available yaw trajectories
from yaw_trajectories import yaw_trajectories_database

# import controllers dictionary
from controllers.fa_trajectory_tracking_controllers import fa_trajectory_tracking_controllers_database

import math
import numpy

# for subscribing to topics, and publishing
import rospy

class FireflyTrajectoryTracking(mission.Mission):

    inner = {}

    inner['controller']     = fa_trajectory_tracking_controllers_database.database
    inner['reference']      = trajectories_database.database
    inner['yaw_controller'] = yaw_controllers_database.database
    inner['yaw_reference']  = yaw_trajectories_database.database

    @classmethod
    def description(cls):
        description = """
        <b>Firefly</b>, from RotorS, to track a desired trajectory. This mission depends on:
        <ul>
          <li>controller: a trajectory tracking controller</li>
          <li>reference: a reference position trajectory to be tracked</li>
          <li>yaw_controller: a yaw controller</li>
          <li>yaw_reference: a yaw reference</li>
        </ul>
        """
        return description
    
    def __init__(self,
            controller     = fa_trajectory_tracking_controllers_database.database["Default"](),
            reference      = trajectories_database.database["Default"](),
            yaw_controller = yaw_controllers_database.database["Default"](),
            yaw_reference  = yaw_trajectories_database.database["Default"]()
            ):
        # Copy inner_keys into self variables
        # Subscribe to the necessary topics, if any

        # initialize time instant, and  state
        mission.Mission.__init__(self)

        # converting our controlller standard into rotors standard
        self.RotorSObject = RotorSConverter()

        # subscriber to odometry from RotorS
        dictionary = {}
        dictionary['name'] = "/firefly/ground_truth/odometry"
        dictionary['data_class'] = Odometry
        dictionary['callback'] = self.get_state_from_rotorS_simulator
        self.sub_odometry = rospy.Subscriber(**dictionary)
        

        # publisher: command firefly motor speeds 
        dictionary = {}
        dictionary['name'] = "/firefly/command/motor_speed"
        dictionary['data_class'] = Actuators
        dictionary['queue_size'] = 1
        self.pub_motor_speeds = rospy.Publisher(**dictionary)

        # dy default, desired trajectory is staying still in origin
        self.reference = reference
        self.current_reference = self.reference.output(self.time_instant_t0)

        # controllers selected by default
        self.controller = controller

        # controllers selected by default
        self.yaw_controller = yaw_controller

        # controllers selected by default
        self.yaw_reference = yaw_reference

    def initialize_state(self):
        # state of quad: position, velocity and attitude
        # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad  = numpy.zeros(3+3+3)
        
        self.yaw_desired = 0.0


    def object_description(self):
        description = """
        <b>Firefly</b>, from RotorS, to track a desired trajectory. This mission depends on:
        <ul>
          <li>controller: a trajectory tracking controller</li>
          <li>reference: a reference position trajectory to be tracked</li>
          <li>yaw_controller: a yaw controller</li>
          <li>yaw_reference: a yaw reference</li>
        </ul>
        """
        return description
        
    def __str__(self):
        string  = self.description()
        string += self.controller.description()
        string += self.reference.description()
        string += self.yaw_controller.description()
        string += self.yaw_reference.description()
        return string        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        self.sub_odometry.unregister()


    def get_quad_ea_rad(self):
    	euler_rad     = self.state_quad[6:9]*math.pi/180
    	euler_rad_dot = numpy.zeros(3)
    	return numpy.concatenate([euler_rad,euler_rad_dot])

    # overriding mission method
    def get_desired_yaw_rad(self,time_instant):
        '''Get desired yaw in radians, and its time derivative'''
        self.yaw_desired = self.yaw_reference.output(time_instant)[0]
        return self.yaw_reference.output(time_instant)
    
    def get_ea_desired(self):
        return numpy.array([0.0,0.0,self.yaw_desired*180.0/math.pi]) 

    def get_reference(self,time_instant):
        self.current_reference = self.reference.output(time_instant)
        return self.current_reference


    def get_state(self):
        return self.state_quad


    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.current_reference[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]


    def real_publish(self,desired_3d_force_quad,yaw_rate,rc_output):
        # publish message
        self.pub_motor_speeds.publish(self.RotorSObject.rotor_s_message(desired_3d_force_quad,yaw_rate))
        pass

    # callback when ROTORS simulator publishes states
    def get_state_from_rotorS_simulator(self,odometry_rotor_s):        
        # RotorS has attitude inner loop that needs to known attitude of quad
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        # get state from rotorS simulator
        self.state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)