#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import numpy as np

# node will publish motor speeds
from mav_msgs.msg import Actuators

import utilities.utility_functions as uts

# import firefly parameters: mass, inertia, ...
import firefly_parameters

from utilities import jsonable as js

class RotorSConverter(js.Jsonable):

	matrix_motor_speeds = firefly_parameters.matrix_motor_speeds

	quad_inertia_matrix = firefly_parameters.J

	attitude_proprotial_gain = 10
	attitude_derivative_gain = 2*0.8*np.sqrt(attitude_proprotial_gain)
	# attitude_proprotial_gain = 3
	# attitude_derivative_gain = 0.52 

	attitude_z_derivative_gain = 5.0

	thrust_gain = 1.0

	# I need to initialize these, because control law depends on these, and they come from subscription
	rotation_matrix = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
	omega_body      = np.array([0.0,0.0,0.0])

	@classmethod
	def description(cls):
		return "Converts 3D force and angular velocity into motor speeds, for a Firefly"

	def __init__(self):
		self.force_z_median = uts.MedianFilter(10)

	def object_description(self):
		string  = "Converts 3D force and angular velocity into motor speeds, for a Firefly"
		string += "\n\t"
		string += "attitude_proprotial_gain = " + self.attitude_proprotial_gain
		string += "\n\t"
		string += "attitude_derivative_gain = " + self.attitude_derivative_gain
		string += "\n\t"
		string += "attitude_z_derivative_gain = " + attitude_z_derivative_gain
		string += "\n"
		return string

	@js.add_to_methods_list
	def update_thrust_gain(self):

		median_force = self.force_z_median.output()

		self.thrust_gain = self.thrust_gain*median_force/(firefly_parameters.kDefaultMass*uts.GRAVITY)

		# for safety better bound this value
		self.thrust_gain = np.clip(self.thrust_gain,0.9,1.1)
		rospy.logwarn('new neutral value = ' + str(self.thrust_gain) + ' in [0.9,1.1]')	    	
		return

	def get_quad_state(self,data_odometry):

		#---------------------------------------------------------------#

		quaternion_quad = np.array([data_odometry.pose.pose.orientation.x,\
		                               data_odometry.pose.pose.orientation.y,\
		                               data_odometry.pose.pose.orientation.z,\
		                               data_odometry.pose.pose.orientation.w])    

		# attitude: euler in DEGREES
		# ee = np.array([roll,pitch,yaw])
		rotation_matrix  = uts.rot_from_quaternion(quaternion_quad)  
		ee               = uts.euler_deg_from_rot(rotation_matrix)    

		# omega_body =  np.array([data_odometry.twist.twist.angular.x,\
		#                            data_odometry.twist.twist.angular.y,\
		#                            data_odometry.twist.twist.angular.z])

		#---------------------------------------------------------------#

		p = np.array([data_odometry.pose.pose.position.x,\
		                 data_odometry.pose.pose.position.y,\
		                 data_odometry.pose.pose.position.z])

		# velocity is in the body reference frame
		v_body = np.array([data_odometry.twist.twist.linear.x,\
		                      data_odometry.twist.twist.linear.y,\
		                      data_odometry.twist.twist.linear.z])


		v_inertial = np.dot(rotation_matrix,v_body)


		# current_time  = data_odometry.header.stamp.secs + data_odometry.header.stamp.nsecs/1e9
		# print current_time
		# print self.QuadVelocityEstimator.out(position_quad,current_time)
		# print velocity_quad

		#---------------------------------------------------------------#

		# collect all components of state
		return np.concatenate([p,v_inertial,ee]) 

	def rotor_s_attitude_for_control(self,data_odometry):

		#---------------------------------------------------------------#
		# rotation matrix
		quaternion_quad = np.array([data_odometry.pose.pose.orientation.x,\
		                               data_odometry.pose.pose.orientation.y,\
		                               data_odometry.pose.pose.orientation.z,\
		                               data_odometry.pose.pose.orientation.w])    

		rotation_matrix = uts.rot_from_quaternion(quaternion_quad)        

		#---------------------------------------------------------------#
		# angular velocity in body reference frame
		omega_body =  np.array([data_odometry.twist.twist.angular.x,\
		                           data_odometry.twist.twist.angular.y,\
		                           data_odometry.twist.twist.angular.z])

		#---------------------------------------------------------------#
		# saving as states
		self.rotation_matrix = rotation_matrix 
		self.omega_body      = omega_body


	def rotor_s_standard_converter(self,U,omega_z_body_desired):

		self.force_z_median.update_data(U[2])

		#---------------------------------------------------------------#
		e3              = np.array([0.0,0.0,1.0])
		rotation_matrix = self.rotation_matrix
		unit_vector     = np.dot(rotation_matrix,e3)
		omega_body      = self.omega_body

		#---------------------------------------------------------------#

		U_0dot = U
		U_1dot = np.zeros(3)
		U_2dot = np.zeros(3)   

		thrust      = self.thrust_gain*np.dot(U,unit_vector)      
		torque_body = self.compute_torque(U_0dot,U_1dot,omega_z_body_desired)


		n = np.dot(self.matrix_motor_speeds,np.concatenate([torque_body,[thrust]]))
		# speeds cannot be negative; bound below by 0
		n = np.maximum(n,np.zeros(6)) 
		# forces proportional to speed squared
		n = np.sqrt(n)     

		return n    

	def rotor_s_message(self,U,omega_z_body_desired):   

		# creating actuators message
		actuators_message = Actuators()
		# populate message
		actuators_message.angular_velocities = self.rotor_s_standard_converter(U,omega_z_body_desired)

		return actuators_message

	def torque_unit_vector(self,n,w,n_star,w_star,w_star_dot):

		torque = \
		np.dot(uts.ort_proj(n),w_star_dot) - \
		self.attitude_derivative_gain*np.dot(uts.ort_proj(n),w - w_star) - \
		self.attitude_proprotial_gain*np.dot(uts.skew(n_star),n)         + \
		np.dot(uts.skew(n),w_star)*np.dot(n,w_star)

		return torque

	def transform_uav_torque(self,unit_vector_torque,torque_orthogonal):

		R  = self.rotation_matrix
		w  = self.omega_body
		J  = self.quad_inertia_matrix
		e3 = np.array([0.0,0.0,1.0])

		aux = \
		np.dot(uts.ort_proj(e3),np.dot(np.transpose(R),unit_vector_torque)) + \
		e3*torque_orthogonal - \
		np.dot(uts.skew(e3),w)*w[2]

		torque = \
		np.dot(uts.skew(w),np.dot(J,w)) + \
		np.dot(J,aux)

		return torque

	def compute_torque(self,
		desired_acceleration,
		desired_acceleration_dot,
		omega_z_body_desired):

		wz       = self.omega_body[2]
		torque_z = -self.attitude_z_derivative_gain*(wz - omega_z_body_desired)

		R   = self.rotation_matrix
		r3  = R[:,2]
		w3  = np.dot(uts.ort_proj(r3),np.dot(R,self.omega_body))
		r3d = desired_acceleration/np.linalg.norm(desired_acceleration)
		w3d = np.dot(uts.skew(r3d),desired_acceleration_dot/np.linalg.norm(desired_acceleration))
		a3d = np.zeros(3)
		torque_xy = self.torque_unit_vector(r3,w3,r3d,w3d,a3d)

		return self.transform_uav_torque(torque_xy,torque_z)