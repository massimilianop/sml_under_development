#!/usr/bin/env python

# import Misson abstract class
from ... import mission

# import converter from 3d_force and yaw rate into iris rc standard 
from converters.iris_plus_converter import IrisPlusConverter

# publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd, quad_state

# import list of available trajectories
from trajectories import trajectories_database
TRAJECTORIES_DATABASE = trajectories_database.database

# import yaw controllers dictionary
from yaw_rate_controllers import yaw_controllers_database
YAW_CONTROLLERS_DATABASE = yaw_controllers_database.database

# import controllers dictionary
from controllers.fa_trajectory_tracking_controllers import fa_trajectory_tracking_controllers_database
CONTROLLERS_DATABASE = fa_trajectory_tracking_controllers_database.database

import math
import numpy

# for subscribing to topics, and publishing
import rospy

import utilities.jsonable as js


@js.inherit_methods_list_from_parents
class IrisSimulatorTrajectoryTracking(mission.Mission):

    js.Jsonable.add_inner('controller',CONTROLLERS_DATABASE)
    js.Jsonable.add_inner('reference',TRAJECTORIES_DATABASE)
    js.Jsonable.add_inner('yaw_controller',YAW_CONTROLLERS_DATABASE)
    # js.Jsonable.add_inner('yaw_reference',YAW_TRAJECTORIES_DATABASE)

    @classmethod
    def description(cls):
        return "Iris, simulated, to track a desired trajectory"

    
    def __init__(self):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        mission.Mission.__init__(self)     

        self.add_inner_defaults()

        # controller needs to have access to STATE: comes from simulator
        self.SubToSim = rospy.Subscriber("simulator/quad_state", quad_state, self.get_state_from_simulator)

        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('simulator/quad_cmd', quad_cmd, queue_size=10)
        

        self.current_reference     = self.reference.output(self.time_instant_t0)

        self.iris_plus_converter_object_mission = IrisPlusConverter()
        self.iris_plus_converter_object_mission.set_mass(self.controller.MASS)

        self.total_mass = self.controller.MASS
        
        pass


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
        self.current_reference = self.reference.output(time_instant)
        return self.current_reference
        # return numpy.zeros(3*5)

    def get_state(self):
        return self.state_quad

    def get_position(self):
        return self.state_quad[0:3]

    def get_velocity(self):
        return self.state_quad[3:6]

    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.current_reference[0:6]


    def get_complementary_data(self):
        return numpy.array([])

    def get_labels_complementary_data(self):
        return []


    def get_euler_angles(self):
        return self.state_quad[6:9]

    def get_psi(self):
        return self.state_quad[8]

    @js.add_to_methods_list
    def reset_iris_neutral_value(self):
        """
        Reset k_trottle_neutral by checking current thrust 
        (median over last thrust values)
        Should only be applied once the uav stabilizes at fixed point"""

        self.iris_plus_converter_object_mission.reset_k_trottle_neutral()
        return

    @js.add_to_methods_list
    def set_iris_neutral_value(self,k_trottle_neutral=1480):
        """Set k_trottle_neutral in [1400 1600]"""
        self.iris_plus_converter_object_mission.set_k_trottle_neutral(k_trottle_neutral)
        return

    def real_publish(self,desired_3d_force_quad,yaw_rate):

        rc_output = self.rc_command(desired_3d_force_quad,yaw_rate)

        # create a message of the type quad_cmd, that the simulator subscribes to 
        cmd  = quad_cmd()
        
        cmd.cmd_1 = rc_output[0]
        cmd.cmd_2 = rc_output[1]
        cmd.cmd_3 = rc_output[2]
        cmd.cmd_4 = rc_output[3]

        cmd.cmd_5 = 1500.0
        cmd.cmd_6 = 1500.0
        cmd.cmd_7 = 1500.0
        cmd.cmd_8 = 1500.0
        
        self.pub_cmd.publish(cmd)


    # callback when simulator publishes states
    def get_state_from_simulator(self, simulator_message):

        # position
        p = numpy.array([simulator_message.x,simulator_message.y,simulator_message.z])
        # velocity
        v = numpy.array([simulator_message.vx,simulator_message.vy,simulator_message.vz])
        #v = self.VelocityEstimator.out(p,rospy.get_time())
        # attitude: euler angles
        ee = numpy.array([simulator_message.roll,simulator_message.pitch,simulator_message.yaw])
        # collect all components of state
        self.state_quad = numpy.concatenate([p,v,ee])  


    def rc_command(self,desired_3d_force_quad,yaw_rate):
        euler_rad          = self.get_euler_angles()*numpy.pi/180
        self.iris_plus_converter_object_mission.set_rotation_matrix(euler_rad)
        iris_plus_rc_input = self.iris_plus_converter_object_mission.input_conveter(desired_3d_force_quad,yaw_rate)
        rc_output     = iris_plus_rc_input

        return rc_output
