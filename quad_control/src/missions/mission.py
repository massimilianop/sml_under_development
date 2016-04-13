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
    }
    
    time_instant_t0 = 0.0
    
    # for reseting neutral value that makes iris+ stay at desired altitude
    DesiredZForceMedian = utility_functions.Median_Filter(10)

    @classmethod
    def description(cls):
        return "Abstract Mission"
    
    
    def __init__(self, params):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any
        pass
        
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        pass

    @classmethod   
    def real_publish(cls,desired_3d_force_quad,yaw_rate):
        return 

    @classmethod   
    def get_quad_ea_rad(cls):
        '''Get euler angles (roll, pitch, yaw) in radians, and get their time derivatives'''
        return numpy.zeros(3+3)

    @classmethod   
    def get_desired_yaw_rad(cls,time_instant):
        '''Get desired yaw in radians, and its time derivative'''
        return numpy.zeros(1+1)    

    @classmethod
    def get_reference(cls,time_instant):
        return None

    @classmethod
    def get_state(cls):
        return None

    def reset_initial_time(self,time_instant = 0.0):
        if time_instant == None:
            self.time_instant_t0 = rospy.get_time()
        else:
            self.time_instant_t0 = time_instant
        return 
   
    def yaw_rate(self,time_instant):

        # convert to iris + standard
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

