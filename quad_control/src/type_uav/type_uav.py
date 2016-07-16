#!/usr/bin/env python

import rospy
import numpy
from utilities import jsonable as js

import mission

# Parent class for all types of UAV's
class TypeUAV(js.Jsonable):
    """Class for each type of uav"""

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

    # def __init__(self):
    #     self.add_inner_defaults()

    #all children to this class must implement the 
    def publish(self):
        raise NotImplementedError()
    def get_position(self):
        raise NotImplementedError()
    def get_velocity(self):
        raise NotImplementedError()
    def get_position_desired(self):
        raise NotImplementedError()
    def get_velocity_desired(self):
        raise NotImplementedError()
    def get_euler_angles(self):
        raise NotImplementedError()
    def get_euler_angles_desired(self):
        raise NotImplementedError()
    def get_input(self):
        raise NotImplementedError()

#------------------------------------------------------------#
#------------------------------------------------------------#
# For firefly 

from converter_firefly import RotorSConverter

# node will publish motor speeds
from mav_msgs.msg import Actuators

MISSIONS_DATABASE = mission.FireflyGazebo.database

# this decorator will also create the database
# add class to database
@js.add_to_database(default=True)
class FireflyGazebo(TypeUAV):
    """This class creates the publisher for a Firefly UAV"""

    js.Jsonable.add_inner('mission',MISSIONS_DATABASE)

    @classmethod
    def description(cls):
        string = """Missions with firefly in gazebo"""
        return string

    def __init__(self):

        # TODO: correct this
        self.time_instant_t0 = 0.0

        self.add_inner_defaults()

        # converting our controlller standard (3D force + yaw_rate)
        # into rotors standard
        self.rotors_object = RotorSConverter()

        # publisher: command firefly motor speeds
        dictionary = {}
        dictionary['name'] = "/firefly/command/motor_speed"
        dictionary['data_class'] = Actuators
        dictionary['queue_size'] = 1
        self.pub_motor_speeds = rospy.Publisher(**dictionary)

    def object_description(self):
        string = """
        Mission with firefly in gazebo: chosen mission. This mission depends on:
        <ul>
          <li>mission: """ + self.mission.__class__.__name__ +"""</li>
        </ul>
        """
        return string

    @js.add_to_methods_list
    def update_thrust_gain(self):
        self.rotors_object.update_thrust_gain()
        return

    def publish(self):

        time_instant = rospy.get_time() - self.time_instant_t0
        
        # it is the job of the mission to compute the 3d_force and yaw_rate
        desired_3d_force = self.mission.compute_3d_force(time_instant)
        desired_yaw_rate = self.mission.compute_yaw_rate(time_instant)

        # RotorS has attitude inner loop that needs to known attitude of quad
        uav_odometry = self.mission.get_uav_odometry()
        self.rotors_object.rotor_s_attitude_for_control(uav_odometry)
        
        # publish message
        self.pub_motor_speeds.publish(self.rotors_object.rotor_s_message(desired_3d_force,desired_yaw_rate))

    def get_mission(self):
        return self.mission

    # delegate the job of "getting stuff" to mission
    # recall that the job of this class is to provide the publisher
    # for a Firefly in Gazebo
    def get_position(self):
        return self.mission.get_position()
    def get_velocity(self):
        return self.mission.get_velocity()
    def get_position_desired(self):
        return self.mission.get_position_desired()
    def get_velocity_desired(self):
        return self.mission.get_velocity_desired()
    def get_euler_angles(self):
        return self.mission.get_euler_angles()
    def get_euler_angles_desired(self):
        return self.mission.get_euler_angles_desired()
    def get_input(self):
        return self.mission.get_input()


#------------------------------------------------------------#
#------------------------------------------------------------#
# For Iris

MISSIONS_DATABASE = mission.IrisRviz.database

# import converter from 3d_force and yaw rate into iris rc standard 
from converter_iris import IrisPlusConverter

# publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd, quad_state

# add class to database
@js.add_to_database()
class IrisRviz(TypeUAV):
    """This class creates the publisher for a IRIS UAV"""

    js.Jsonable.add_inner('mission',MISSIONS_DATABASE)

    @classmethod
    def description(cls):
        string = """Missions with IRIS in Rviz"""
        return string

    def __init__(self):

        # TODO: correct this
        self.time_instant_t0 = 0.0

        self.add_inner_defaults()

        # converting our controlller standard (3D force + yaw_rate)
        # into IRIS standard
        self.iris_plus_converter_object_mission = IrisPlusConverter()
        self.iris_plus_converter_object_mission.set_mass(self.mission.get_total_weight()/9.81)

        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('simulator/quad_cmd', quad_cmd, queue_size=1)

    def object_description(self):
        string = """
        Mission with IRIS in Rviz: chosen mission. This mission depends on:
        <ul>
          <li>mission: """ + self.mission.__class__.__name__ +"""</li>
        </ul>
        """
        return string

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

    def publish(self):

        time_instant = rospy.get_time() - self.time_instant_t0
        
        # it is the job of the mission to compute the 3d_force and yaw_rate
        desired_3d_force = self.mission.compute_3d_force(time_instant)
        desired_yaw_rate = self.mission.compute_yaw_rate(time_instant)

        # RotorS has attitude inner loop that needs to known attitude of quad
        uav_odometry = self.mission.get_uav_odometry()
        self.iris_plus_converter_object_mission.set_rotation_matrix(uav_odometry)
        iris_plus_rc_input = self.iris_plus_converter_object_mission.input_conveter(desired_3d_force,desired_yaw_rate)
        rc_output     = iris_plus_rc_input

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

    # def get_mission(self):
    #     return self.mission

    # delegate the job of "getting stuff" to mission
    # recall that the job of this class is to provide the publisher
    # for a Firefly in Gazebo
    def get_position(self):
        return self.mission.get_position()
    def get_velocity(self):
        return self.mission.get_velocity()
    def get_position_desired(self):
        return self.mission.get_position_desired()
    def get_velocity_desired(self):
        return self.mission.get_velocity_desired()
    def get_euler_angles(self):
        return self.mission.get_euler_angles()
    def get_euler_angles_desired(self):
        return self.mission.get_euler_angles_desired()
    def get_input(self):
        return self.mission.get_input()


#------------------------------------------------------------#
#------------------------------------------------------------#
# For Real Iris

MISSIONS_DATABASE = mission.Iris.database

# import converter from 3d_force and yaw rate into iris rc standard 
from converter_iris import IrisPlusConverter

from mavros_msgs.msg import OverrideRCIn

# add class to database
@js.add_to_database()
class Iris(TypeUAV):
    """This class creates the publisher for a IRIS UAV"""

    js.Jsonable.add_inner('mission',MISSIONS_DATABASE)

    @classmethod
    def description(cls):
        string = """Missions with real IRIS"""
        return string

    def __init__(self):

        # TODO: correct this
        self.time_instant_t0 = 0.0

        self.add_inner_defaults()

        # converting our controlller standard (3D force + yaw_rate)
        # into IRIS standard
        self.iris_plus_converter_object_mission = IrisPlusConverter()
        self.iris_plus_converter_object_mission.set_mass(self.mission.get_total_weight()/9.81)

        # publisher: command firefly motor speeds    
        self.rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1)

    def object_description(self):
        string = """
        Mission with real IRIS: chosen mission. This mission depends on:
        <ul>
          <li>mission: """ + self.mission.__class__.__name__ +"""</li>
        </ul>
        """
        return string

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

    def publish(self):

        time_instant = rospy.get_time() - self.time_instant_t0
        
        # it is the job of the mission to compute the 3d_force and yaw_rate
        desired_3d_force = self.mission.compute_3d_force(time_instant)
        desired_yaw_rate = self.mission.compute_yaw_rate(time_instant)

        # RotorS has attitude inner loop that needs to known attitude of quad
        uav_odometry = self.mission.get_uav_odometry()
        self.iris_plus_converter_object_mission.set_rotation_matrix(uav_odometry)
        iris_plus_rc_input = self.iris_plus_converter_object_mission.input_conveter(desired_3d_force,desired_yaw_rate)
        rc_output     = iris_plus_rc_input

        # ORDER OF INPUTS IS VERY IMPORTANT: roll, pitch, throttle,yaw_rate
        # create message of type OverrideRCIn
        rc_cmd          = OverrideRCIn()
        other_channels  = numpy.array([1500,1500,1500,1500])
        channels        = numpy.concatenate([rc_output,other_channels])
        channels        = numpy.array(channels,dtype=numpy.uint16)
        rc_cmd.channels = channels
        self.rc_override.publish(rc_cmd)

    # def get_mission(self):
    #     return self.mission

    # delegate the job of "getting stuff" to mission
    # recall that the job of this class is to provide the publisher
    # for a Firefly in Gazebo
    def get_position(self):
        return self.mission.get_position()
    def get_velocity(self):
        return self.mission.get_velocity()
    def get_position_desired(self):
        return self.mission.get_position_desired()
    def get_velocity_desired(self):
        return self.mission.get_velocity_desired()
    def get_euler_angles(self):
        return self.mission.get_euler_angles()
    def get_euler_angles_desired(self):
        return self.mission.get_euler_angles_desired()
    def get_input(self):
        return self.mission.get_input()        