#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import numpy

# node will publish motor speeds
from mav_msgs.msg import Actuators

from utility_functions import skew,unskew,quaternion_to_rot,Velocity_Filter,quaternion_from_unit_vector,bound,GetEulerAnglesDeg

# import firefly parameters: mass, inertia, ...
import firefly_parameters

class RotorSConverter(object):

	matrix_motor_speeds = firefly_parameters.matrix_motor_speeds

	quad_inertia_matrix = firefly_parameters.J

	attitude_gain     = 3
	angular_rate_gain = 0.52
	# attitude_gain     = 5.0
	# angular_rate_gain = numpy.sqrt(2*attitude_gain)   

	attitude_gain_z     = 0.15
	angular_rate_gain_z = 0.18

	# I need to initialize these, because control law depends on these, and they come from subscription
	rotation_matrix = numpy.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
	omega_body      = numpy.array([0.0,0.0,0.0])


	def get_quad_state(self,data_odometry):

		#---------------------------------------------------------------#

		quaternion_quad = numpy.array([data_odometry.pose.pose.orientation.x,\
		                               data_odometry.pose.pose.orientation.y,\
		                               data_odometry.pose.pose.orientation.z,\
		                               data_odometry.pose.pose.orientation.w])    

		# attitude: euler in DEGREES
		# ee = numpy.array([roll,pitch,yaw])
		rotation_matrix  = quaternion_to_rot(quaternion_quad)  
		ee               = GetEulerAnglesDeg(rotation_matrix)    

		# omega_body =  numpy.array([data_odometry.twist.twist.angular.x,\
		#                            data_odometry.twist.twist.angular.y,\
		#                            data_odometry.twist.twist.angular.z])


		#---------------------------------------------------------------#

		p = numpy.array([data_odometry.pose.pose.position.x,\
		                 data_odometry.pose.pose.position.y,\
		                 data_odometry.pose.pose.position.z])

		# velocity is in the body reference frame
		v_body = numpy.array([data_odometry.twist.twist.linear.x,\
		                      data_odometry.twist.twist.linear.y,\
		                      data_odometry.twist.twist.linear.z])


		v_inertial = numpy.dot(rotation_matrix,v_body)


		# current_time  = data_odometry.header.stamp.secs + data_odometry.header.stamp.nsecs/1e9
		# print current_time
		# print self.QuadVelocityEstimator.out(position_quad,current_time)
		# print velocity_quad

		#---------------------------------------------------------------#

		# collect all components of state
		return numpy.concatenate([p,v_inertial,ee]) 

	def rotor_s_attitude_for_control(self,data_odometry):

		#---------------------------------------------------------------#
		# rotation matrix
		quaternion_quad = numpy.array([data_odometry.pose.pose.orientation.x,\
		                               data_odometry.pose.pose.orientation.y,\
		                               data_odometry.pose.pose.orientation.z,\
		                               data_odometry.pose.pose.orientation.w])    

		rotation_matrix = quaternion_to_rot(quaternion_quad)        

		#---------------------------------------------------------------#
		# angular velocity in body reference frame
		omega_body =  numpy.array([data_odometry.twist.twist.angular.x,\
		                           data_odometry.twist.twist.angular.y,\
		                           data_odometry.twist.twist.angular.z])

		#---------------------------------------------------------------#
		# saving as states
		self.rotation_matrix = rotation_matrix 
		self.omega_body      = omega_body


	def rotor_s_standard_converter(self,U,omega_z_body_desired):

		#---------------------------------------------------------------#

		e3              = numpy.array([0.0,0.0,1.0])
		rotation_matrix = self.rotation_matrix
		unit_vector     = numpy.dot(rotation_matrix,e3)
		omega_body      = self.omega_body

		# print 'angle: ' + str(numpy.arccos((numpy.dot(unit_vector,unit_vector_des)))*180.0/3.142)

		#---------------------------------------------------------------#

		U_0dot = U
		U_1dot = numpy.zeros(3)
		U_2dot = numpy.zeros(3)   

		# finding unit vector associated to desired force vector
		# note that U_0dot cannot be zero vector
		# unit_vector_des,omega_des,omega_des_dot = self.unit_vector_from_vector(U_0dot,U_1dot,U_2dot)

		# omega_inertial = numpy.dot(rotation_matrix,omega_body)
		# omega_3        = omega_inertial - unit_vector*numpy.dot(unit_vector,omega)
		# Tau            = self.torque_unit_vector(unit_vector,omega_3,unit_vector_des,omega_des,omega_des_dot)
		# ANGULAR_VELOCITY_GAIN = 1.0
		# Tau_3          = -ANGULAR_VELOCITY_GAIN*numpy.dot(omega_body,e3)
		# Tau            = Tau + Tau_3*unit_vector

		thrust      = numpy.dot(U,unit_vector)      
		torque_body = self.compute_torque(U_0dot,U_1dot,rotation_matrix,omega_body,omega_z_body_desired)


		n = numpy.dot(self.matrix_motor_speeds,numpy.concatenate([torque_body,[thrust]]))
		# speeds cannot be negative; bound below by 0
		n = numpy.maximum(n,numpy.zeros(6)) 
		# forces proportional to speed squared
		n = numpy.sqrt(n)     

		return n    

	def rotor_s_message(self,U,PsiStar):   

		# creating actuators message
		actuators_message = Actuators()
		# this is just for testing
		# actuators_message.angular_velocities = numpy.array([100,100,100,100,100,100])
		# copy motor speeds into message previously created
		# actuators_message.angular_velocities = n
		# just for debug pruposes
		# actuators_message.angular_velocities = numpy.array([200,200,200,200,200,200])

		actuators_message.angular_velocities = self.rotor_s_standard_converter(U,PsiStar)

		return actuators_message	    


	def torque_unit_vector(self,n,w,n_star,w_star,w_star_dot):

		ew     = numpy.dot(skew(n),w - w_star)
		torque = numpy.dot(skew(n),-w_star_dot - self.attitude_gain*numpy.dot(skew(n),n_star) - numpy.dot(skew(n),w_star)*numpy.dot(n,w_star)) +\
		         self.angular_rate_gain*ew                

		return torque 


	def unit_vector_from_vector(self,U_0dot,U_1dot,U_2dot):

		U_0dot_norm = U_0dot/numpy.linalg.norm(U_0dot)
		U_1dot_norm = U_1dot/numpy.linalg.norm(U_0dot)
		U_2dot_norm = U_2dot/numpy.linalg.norm(U_0dot)

		unit_vector_des = U_0dot_norm
		omega_des       = numpy.dot(skew(unit_vector_des),U_1dot_norm)
		omega_des_dot   = numpy.dot(skew(unit_vector_des),U_2dot_norm - 2.0*U_1dot_norm*numpy.dot(U_1dot_norm,U_0dot_norm))

		return (unit_vector_des,omega_des,omega_des_dot)        

	def compute_torque(self,desired_acceleration,desired_acceleration_dot,rotation_matrix,angular_velocity_body,omega_z_body_desired):

		r3  = desired_acceleration
		r3  = r3/numpy.linalg.norm(r3)
		psi = numpy.arctan2(bound(rotation_matrix[1,0],1,-1),bound(rotation_matrix[0,0],1,-1))
		r1  = numpy.array([numpy.cos(psi),numpy.sin(psi),0.0])
		r1  = numpy.dot(numpy.identity(3) - numpy.outer(r3,r3),r1)
		r1  = r1/numpy.linalg.norm(r1)
		r2  = numpy.dot(skew(r3),r1)        

		R_desired = numpy.column_stack((r1,r2,r3))

		R_error = numpy.dot(numpy.transpose(R_desired),rotation_matrix) 

		# angular_rate_des   = numpy.zeros(3)
		# angular_rate_error = angular_velocity_body - numpy.dot(numpy.transpose(rotation_matrix), numpy.dot(R_desired, angular_rate_des))
		angular_rate_des   = numpy.dot(skew(r3),desired_acceleration_dot/numpy.linalg.norm(desired_acceleration))
		angular_rate_error = angular_velocity_body - (numpy.dot(numpy.transpose(rotation_matrix), angular_rate_des) + numpy.array([0.0,0.0,1.0])*omega_z_body_desired) 

		angular_acceleration = -self.attitude_gain*unskew(1.0/2.0*(R_error - numpy.transpose(R_error))) \
		                       -self.angular_rate_gain*angular_rate_error +\
		                       numpy.dot(skew(angular_velocity_body),numpy.dot(self.quad_inertia_matrix,angular_velocity_body))

		return angular_acceleration