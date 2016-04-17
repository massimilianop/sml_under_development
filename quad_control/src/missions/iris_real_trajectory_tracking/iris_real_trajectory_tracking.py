#!/usr/bin/env python

# import Misson abstract class
from .. import mission

# 
from mavros_msgs.msg import OverrideRCIn
# from mavros_msgs.srv import ParamSet,ParamGet,CommandBool,SetMode

# import converter from 3d_force and yaw rate into iris rc standard 
from converter_between_standards.iris_plus_converter import IrisPlusConverter

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

class IrisRealTrajectoryTracking(mission.Mission):

    inner = {}

    inner['controller']     = fa_trajectory_tracking_controllers_database.database
    inner['reference']      = trajectories_database.database
    inner['yaw_controller'] = yaw_controllers_database.database


    @classmethod
    def description(cls):
        return "Firefly, from RotorS, to track a desired trajectory"
    
    
    def __init__(self,
            body_id = 12,
            controller     = fa_trajectory_tracking_controllers_database.database["Default"](),
            reference      = trajectories_database.database["Default"](),
            yaw_controller = yaw_controllers_database.database["Default"]()
            ):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        mission.Mission.__init__(self)
    
        self.inner['controllers_dictionary']     = controllers_dictionary.controllers_dictionary
        self.inner['trajectories_database']    = trajectories_database.trajectories_database
        self.inner['yaw_controllers_dictionary'] = yaw_controllers_dictionary.yaw_controllers_dictionary

        
        # converting our controlller standard into iris+ standard
        self.IrisPlusConverterObject = IrisPlusConverter()

        # establish connection to qualisys
        self.Qs = mocap_source.Mocap(info=0)

        # publisher: command firefly motor speeds    
        self.rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=100)

        self.body_id = body_id

        # intiialization should be done in another way,
        # but median will take care of minimizing effects
        self.VelocityEstimator = Velocity_Filter(3,numpy.zeros(3),0.0)

        # dy default, desired trajectory is staying still in origin
        TrajectoryClass    = self.inner['trajectories_database']['StayAtRest']
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
        pass

    def get_quad_ea_rad(self):
    	euler_rad     = self.state_quad[6:9]*math.pi/180
    	euler_rad_dot = numpy.zeros(3)
    	return numpy.concatenate([euler_rad,euler_rad_dot])


    def get_reference(cls,time_instant):
        return self.TrajGenerator.output(time_instant)


    def get_state(cls):

        bodies = self.Qs.get_body(self.body_id)

        if bodies != 'off':  

            self.flag_measurements = True

            # position
            x=bodies["x"]; y=bodies["y"]; z=bodies["z"]   
            p = numpy.array([x,y,z])

            # velocity
            #v = numpy.array([data.vx,data.vy,data.vz])
            v = self.VelocityEstimator.out(p,rospy.get_time())

            # attitude: euler angles THESE COME IN DEGREES
            roll = bodies["roll"]; pitch=-bodies["pitch"]; yaw  = bodies["yaw"]
            ee = numpy.array([roll,pitch,yaw])

            # collect all components of state
            self.state_quad = numpy.concatenate([p,v,ee])  
        else:
            # do nothing, keep previous state
            self.flag_measurements = False

        return self.state_quad


    def get_pv():
        return self.state_quad[0:6]


    def get_pv_desired():
        return self.reference[0:6]


    def get_euler_angles():
        return self.state_quad[6:9]


    def real_publish(self,desired_3d_force_quad,yaw_rate):

        self.IrisPlusConverterObject.set_rotation_matrix(euler_rad)
        iris_plus_rc_input = self.IrisPlusConverterObject.input_conveter(desired_3d_force_quad,yaw_rate)

        # ORDER OF INPUTS IS VERY IMPORTANT: roll, pitch, throttle,yaw_rate
        # create message of type OverrideRCIn
        rc_cmd          = OverrideRCIn()
        other_channels  = numpy.array([1500,1500,1500,1500])
        channels        = numpy.concatenate([iris_plus_rc_input,other_channels])
        channels        = numpy.array(channels,dtype=numpy.uint16)
        rc_cmd.channels = channels
        self.rc_override.publish(rc_cmd)     
