#!/usr/bin/env python
# this line is just used to define the type of document

"""
Database of the controller for a vector thrusted system
"""

import rospy

database = {}

import without_disturbance.load_transport_controller

database["SingleLoadTransportController"] = without_disturbance.load_transport_controller.SingleLoadTransportController

database["Default"] = database[rospy.get_param("SingleLoadTransportationControllerDefault","SingleLoadTransportController")]