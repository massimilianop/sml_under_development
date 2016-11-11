#!/usr/bin/env python
# this line is just used to define the type of document

import numpy as np

from utilities import jsonable as js

import tf
import nav_msgs.msg

class YawController(js.Jsonable):
    
    @classmethod
    def description(cls):
        return "<b>Abstract yaw tracking controller</b>"

    def __init__(self):
        pass
                
    def __str__(self):
        return self.description()

    def output(self, 
        uav_odometry = nav_msgs.msg.Odometry(), 
        reference = np.zeros(2)):
        # state = euler_angles in RAD 
        # + euler_angles_time_derivative in RAD/SEC
        # reference = psi_desired in RAD + psi_desired_time_derivative in RAD/SEC
    
        #TODO everything else is in degrees,
        # maybe we should have this in degrees as well
        raise NotImplementedError()

@js.add_to_database(default=True)
class SimpleTrackingYawController(YawController):

    @classmethod
    def description(cls):
        return "Simple yaw tracking controller, based on <b>feedback linearization of yaw rate equation</b>"

    def __init__(self, gain = 4.0):
        self.gain = gain
    
    
    def object_description(self):
        string = """
        Controller for yaw motion."""
        string += "<li>&#968<sup>(1)</sup> = &#968<sup>*(1)</sup> - gain*sin(&#968 - &#968<sup>*</sup>)</li>"
        string += "<li> yaw rate = cos(&#966)(cos(&#952)*&#968<sup>(1)</sup> - sin(&#966)*&#952<sup>(1)</sup> ) </li>"
        string += """
        Parameters:
        <ul>
          <li>gain: """ + str(self.gain) +"""</li>
        </ul>
        """
        return string

    def output(self, 
        uav_odometry = nav_msgs.msg.Odometry(), 
        reference = np.zeros(2)):
        # state = euler_angles in RAD + euler_angles_time_derivative in RAD/SEC
        # reference = psi_desired in RAD + psi_desired_time_derivative in RAD/SEC
        #return self.controller(state,reference)

        quaternion = np.array([uav_odometry.pose.pose.orientation.x,\
                               uav_odometry.pose.pose.orientation.y,\
                               uav_odometry.pose.pose.orientation.z,\
                               uav_odometry.pose.pose.orientation.w])    

        euler_angles = tf.transformations.euler_from_quaternion(
            quaternion = quaternion, 
            axes='sxyz')

        # TODO:
        if uav_odometry.child_frame_id == 'base_frame':
            # velocity is in the body reference frame
            w_body = np.array([uav_odometry.twist.twist.angular.x,\
                               uav_odometry.twist.twist.angular.y,\
                               uav_odometry.twist.twist.angular.z])

            quaternion = np.array([uav_odometry.pose.pose.orientation.x,\
                                   uav_odometry.pose.pose.orientation.y,\
                                   uav_odometry.pose.pose.orientation.z,\
                                   uav_odometry.pose.pose.orientation.w])  

            rotation_matrix  = uts.rot_from_quaternion(quaternion)

            euler_angles_time_derivative = np.dot(rotation_matrix,w_body)

        if uav_odometry.child_frame_id == 'world_frame':
            # velocity is in the body reference frame
            euler_angles_time_derivative = np.array([uav_odometry.twist.twist.angular.x,\
                          uav_odometry.twist.twist.angular.y,\
                          uav_odometry.twist.twist.angular.z]) 

        euler_angles_time_derivative = np.zeros(3)

        #--------------------------------------#
        # current phi and theta and psi
        # euler_angles = state[0:3]
        phi          = euler_angles[0]
        theta        = euler_angles[1]
        psi          = euler_angles[2]

        #euler_angles_time_derivative = state[3:6]
        
        phi_dot   = euler_angles_time_derivative[0]
        theta_dot = euler_angles_time_derivative[1]
        psi_dot   = euler_angles_time_derivative[2]

        #--------------------------------------#
        psi_star     = reference[0]
        psi_star_dot = reference[1]
        psi_dot      = psi_star_dot - self.gain*np.sin(psi - psi_star)
        yaw_rate     = 1.0/np.cos(phi)*(np.cos(theta)*psi_dot - np.sin(phi)*theta_dot)
        
        return yaw_rate

# """Test"""
# 
#string = TrackingYawController.parameters_to_string()
#print string
#parameters = TrackingYawController.string_to_parameters(string)
#print parameters
#controller = TrackingYawController(parameters)
#print controller
#output = controller.output(np.zeros(6), np.ones(2))
#print output


@js.add_to_database()
class NeutralYawController(YawController):
    '''This yaw controller does nothing: zero yaw rate'''

    @classmethod
    def description(cls):
        return "<b>Zero yaw rate</b>"

    def object_description(self):
        string = """Zero yaw rate"""
        return string

    def output(self, 
        uav_odometry = nav_msgs.msg.Odometry(), 
        reference = np.zeros(2)):
        return 0.0