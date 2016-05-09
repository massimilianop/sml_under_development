#!/usr/bin/env python
# this line is just used to define the type of document

"""
Database of the controller for a vector thrusted system
"""

import rospy

database = {}

import vector_thrust_controller_quadruple_integrator.vector_thrust_controller_quadruple_integrator

database["VThrustQuadrupleController"] = vector_thrust_controller_quadruple_integrator.vector_thrust_controller_quadruple_integrator.VectorThrustController

import vector_thrust_controller_double_integrator_and_toque_backstepping.vector_thrust_controller
database["VThrustBacksteppingController"] = vector_thrust_controller_double_integrator_and_toque_backstepping.vector_thrust_controller.BacksteppingVectorThrustController

database["Default"] = database[rospy.get_param("VThrustControllerDefault","VThrustQuadrupleController")]