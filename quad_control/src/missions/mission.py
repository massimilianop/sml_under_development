#!/usr/bin/env python

from utilities import jsonable as js

from utilities import utility_functions

import numpy

# for getting time
import rospy

# The children import needed dictionaries

# For example
# from controllers_hierarchical import controllers_dictionary as cd
# from TrajectoryPlanner import trajectories_dictionary as td 

# If the children need to subscribe to some topic,
# or to publish to some topic,
# then they import rospy and the needed message types.

# For example
# import rospy as rp
# from mavros.mavros_msgs.msg import OverrideRCIn

# If the mocap is needed, the module mocap must be imported
# import mocap 


class Mission(js.Jsonable):

    inner = {
        # For example: controller, reference, etc.
        'controllers_dictionary'     : {},        
        'yaw_controllers_dictionary' : {},
    }

    # inner = {
    #     # For example: controller, reference, etc.
    #     'controllers_dictionary'     : {},        
    #     'yaw_controllers_dictionary' : {},        
    #     'trajectories_dictionary'    : {},
    #     'yaw_trajectories_dictionary': {},
    # }    

    # time_instant_t0 = 0.0

    @classmethod
    def description(cls):
        return "Abstract Mission"
    
    
    def __init__(self):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        # initialize initial time
        self.reset_initial_time()         
        
        # initialize state: ultimately defined by child class
        self.initialize_state()

        # for reseting neutral value that makes iris+ stay at desired altitude
        self.DesiredZForceMedian = utility_functions.Median_Filter(10)
              
        pass
        
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        pass

    def initialize_state(self):
        raise NotImplementedError()

    def real_publish(self,desired_3d_force_quad,yaw_rate):
        return NotImplementedError()

    def get_quad_ea_rad(self):
        '''Get euler angles (roll, pitch, yaw) in radians, and get their time derivatives'''
        # return numpy.zeros(3+3)
        NotImplementedError()

    def get_desired_yaw_rad(self,time_instant):
        '''Get desired yaw in radians, and its time derivative'''
        return numpy.zeros(1+1)
        return NotImplementedError()

    def get_reference(self,time_instant):
        """
        Get reference that must be accepted by controller
        A reference might not exist
        """
        return None

    def get_state(self):
        """
        Get state that is accepted by controller
        It is the job of a child class to write a state that is accepted by the controllers that it imports 
        """
        return NotImplementedError()

    def get_pv(self):
        """Get position (m) and velocity (m/s) of quadrotor"""
        return NotImplementedError()

    def get_pv_desired(self):
        """Get desired position (m) and velocity (m/s) for the quadrotor"""
        return NotImplementedError()

    def get_rc_output(self):
        """Get rc output"""
        return numpy.zeros(8)    

    def get_euler_angles(self):
        # TODO: CHECK WHETHER IT SHOULD BE RAD OR NOT
        """Get euler angles of the quadrotor (rad)""" 
        return NotImplementedError()

    def change_trajectory(self,key,string):
        """Change reference trajectory"""
        TrajectoryClass   = self.inner['trajectories_dictionary'][key]
        self.TrajGenerator = TrajectoryClass.from_string(string)
        pass

    def change_yaw_trajectory(self,key,string):
        """Change yaw reference trajectory"""
        YawTrajectoryClass   = self.inner['yaw_trajectories_dictionary'][key]
        self.YawTrajGenerator = YawTrajectoryClass.from_string(string)
        pass

    def change_controller(self,key,string):
        """Change controller"""
        ControllerClass      = self.inner['controllers_dictionary'][key]
        self.ControllerObject = ControllerClass.from_string(string)
        pass

    def change_yaw_controller(self,key,string):
        """Change yaw controller"""
        YawControllerClass      = self.inner['yaw_controllers_dictionary'][key]
        self.YawControllerObject = YawControllerClass.from_string(string)
        pass

    def reset_initial_time(self,time_instant = None):
        if time_instant == None:
            self.time_instant_t0 = rospy.get_time()
        else:
            self.time_instant_t0 = time_instant
        pass 
   
    def yaw_rate(self,time_instant):

        state_for_yaw_controller = self.get_quad_ea_rad()
        input_for_yaw_controller = self.get_desired_yaw_rad(time_instant)

        yaw_rate = self.YawControllerObject.output(state_for_yaw_controller,input_for_yaw_controller) 

        return yaw_rate

    def compute_desired_3d_force(self,time_instant):

        # reference
        reference = self.get_reference(time_instant)

        # state
        state = self.get_state()

        # compute input to send to QUAD
        desired_3d_force_quad = self.ControllerObject.output(time_instant,state,reference)

        return desired_3d_force_quad

    def publish(self):

        time_instant = rospy.get_time() - self.time_instant_t0

        desired_3d_force_quad = self.compute_desired_3d_force(time_instant)
            
        self.DesiredZForceMedian.update_data(desired_3d_force_quad[2])

        yaw_rate = self.yaw_rate(time_instant)

        self.real_publish(desired_3d_force_quad,yaw_rate)

        pass