#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import numpy as np

# node will publish motor speeds
from mav_msgs.msg import Actuators

import utilities.utility_functions as uts

# import firefly parameters: mass, inertia, ...
import firefly_parameters

class RotorSConverter(object):

	matrix_motor_speeds = firefly_parameters.matrix_motor_speeds

	quad_inertia_matrix = firefly_parameters.J

	attitude_gain     = 3
	angular_rate_gain = 0.52
	# attitude_gain     = 5.0
	# angular_rate_gain = np.sqrt(2*attitude_gain)   

	attitude_gain_z     = 0.15
	angular_rate_gain_z = 0.18

	# I need to initialize these, because control law depends on these, and they come from subscription
	rotation_matrix = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
	omega_body      = np.array([0.0,0.0,0.0])


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

		#---------------------------------------------------------------#

		e3              = np.array([0.0,0.0,1.0])
		rotation_matrix = self.rotation_matrix
		unit_vector     = np.dot(rotation_matrix,e3)
		omega_body      = self.omega_body

		# print 'angle: ' + str(np.arccos((np.dot(unit_vector,unit_vector_des)))*180.0/3.142)

		#---------------------------------------------------------------#

		U_0dot = U
		U_1dot = np.zeros(3)
		U_2dot = np.zeros(3)   

		# finding unit vector associated to desired force vector
		# note that U_0dot cannot be zero vector
		# unit_vector_des,omega_des,omega_des_dot = self.unit_vector_from_vector(U_0dot,U_1dot,U_2dot)

		# omega_inertial = np.dot(rotation_matrix,omega_body)
		# omega_3        = omega_inertial - unit_vector*np.dot(unit_vector,omega)
		# Tau            = self.torque_unit_vector(unit_vector,omega_3,unit_vector_des,omega_des,omega_des_dot)
		# ANGULAR_VELOCITY_GAIN = 1.0
		# Tau_3          = -ANGULAR_VELOCITY_GAIN*np.dot(omega_body,e3)
		# Tau            = Tau + Tau_3*unit_vector

		thrust      = np.dot(U,unit_vector)      
		torque_body = self.compute_torque(U_0dot,U_1dot,rotation_matrix,omega_body,omega_z_body_desired)


		n = np.dot(self.matrix_motor_speeds,np.concatenate([torque_body,[thrust]]))
		# speeds cannot be negative; bound below by 0
		n = np.maximum(n,np.zeros(6)) 
		# forces proportional to speed squared
		n = np.sqrt(n)     

		return n    

	def rotor_s_message(self,U,omega_z_body_desired):   

		# creating actuators message
		actuators_message = Actuators()
		# this is just for testing
		# actuators_message.angular_velocities = np.array([100,100,100,100,100,100])
		# copy motor speeds into message previously created
		# actuators_message.angular_velocities = n
		# just for debug pruposes
		# actuators_message.angular_velocities = np.array([200,200,200,200,200,200])

		actuators_message.angular_velocities = self.rotor_s_standard_converter(U,omega_z_body_desired)

		return actuators_message	    


	def torque_unit_vector(self,n,w,n_star,w_star,w_star_dot):

		ew     = np.dot(uts.skew(n),w - w_star)
		torque = -uts.skew(n).dot(w_star_dot +\
		    self.attitude_gain*uts.skew(n).dot(n_star) +\
		    uts.skew(n).dot(w_star)*n.dot(w_star)) +\
		    self.angular_rate_gain*ew                

		return torque 


	def unit_vector_from_vector(self,U_0dot,U_1dot,U_2dot):

		U_0dot_norm = U_0dot/np.linalg.norm(U_0dot)
		U_1dot_norm = U_1dot/np.linalg.norm(U_0dot)
		U_2dot_norm = U_2dot/np.linalg.norm(U_0dot)

		unit_vector_des = U_0dot_norm
		omega_des       = np.dot(uts.skew(unit_vector_des),U_1dot_norm)
		omega_des_dot   = np.dot(uts.skew(unit_vector_des),U_2dot_norm - 2.0*U_1dot_norm*np.dot(U_1dot_norm,U_0dot_norm))

		return (unit_vector_des,omega_des,omega_des_dot)        

	# def compute_torque(self,desired_acceleration,desired_acceleration_dot,rotation_matrix,angular_velocity_body,omega_z_body_desired):

	# 	r3  = desired_acceleration
	# 	r3  = r3/np.linalg.norm(r3)
	# 	psi = np.arctan2(np.clip(rotation_matrix[1,0],1,-1),np.clip(rotation_matrix[0,0],1,-1))
	# 	#psi = -20.0*3.142/180.0
	# 	r1  = np.array([np.cos(psi),np.sin(psi),0.0])
	# 	r1  = np.dot(np.identity(3) - np.outer(r3,r3),r1)
	# 	r1  = r1/np.linalg.norm(r1)
	# 	r2  = np.dot(uts.skew(r3),r1)        

	# 	R_desired = np.column_stack((r1,r2,r3))

	# 	R_error = np.dot(np.transpose(R_desired),rotation_matrix) 

	# 	# angular_rate_des   = np.zeros(3)
	# 	# angular_rate_error = angular_velocity_body - np.dot(np.transpose(rotation_matrix), np.dot(R_desired, angular_rate_des))
	# 	angular_rate_des   = np.dot(uts.skew(r3),desired_acceleration_dot/np.linalg.norm(desired_acceleration)) + r3*omega_z_body_desired

	# 	angular_rate_error = angular_velocity_body - (np.dot(np.transpose(rotation_matrix), angular_rate_des)) 


	# 	angular_acceleration = -self.attitude_gain*uts.unskew(1.0/2.0*(R_error - np.transpose(R_error))) \
	# 	                       -np.array([self.angular_rate_gain,self.angular_rate_gain,1.0])*angular_rate_error +\
	# 	                       np.dot(uts.skew(angular_velocity_body),np.dot(self.quad_inertia_matrix,angular_velocity_body))

	# 	return angular_acceleration


	def compute_torque(self,desired_acceleration,desired_acceleration_dot,rotation_matrix,angular_velocity_body,omega_z_body_desired):

		r3  = desired_acceleration
		r3  = r3/np.linalg.norm(r3)
		psi = np.arctan2(np.clip(rotation_matrix[1,0],1,-1),np.clip(rotation_matrix[0,0],1,-1))
		#psi = -20.0*3.142/180.0
		r1  = np.array([np.cos(psi),np.sin(psi),0.0])
		r1  = np.dot(np.identity(3) - np.outer(r3,r3),r1)
		r1  = r1/np.linalg.norm(r1)
		r2  = np.dot(uts.skew(r3),r1)        

		R_desired = np.column_stack((r1,r2,r3))

		R_error = np.dot(np.transpose(R_desired),rotation_matrix) 

		# angular_rate_des   = np.zeros(3)
		# angular_rate_error = angular_velocity_body - np.dot(np.transpose(rotation_matrix), np.dot(R_desired, angular_rate_des))
		angular_rate_des   = np.dot(uts.skew(r3),desired_acceleration_dot/np.linalg.norm(desired_acceleration)) + r3*omega_z_body_desired

		angular_rate_error = angular_velocity_body - (np.dot(np.transpose(rotation_matrix), angular_rate_des)) 


		angular_acceleration = -np.dot(np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,0.0]]),self.attitude_gain*uts.unskew(1.0/2.0*(R_error - np.transpose(R_error)))) \
		                       -np.array([self.angular_rate_gain,self.angular_rate_gain,1.0])*angular_rate_error +\
		                       np.dot(uts.skew(angular_velocity_body),np.dot(self.quad_inertia_matrix,angular_velocity_body))

		return angular_acceleration