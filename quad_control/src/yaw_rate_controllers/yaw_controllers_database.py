#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

database = {}

from neutral_yaw_controller.neutral_yaw_controller import NeutralYawController
database["NeutralYawController"] = NeutralYawController

from simple_tracking_yaw_controller.simple_tracking_yaw_controller import SimpleTrackingYawController
database["SimpleTrackingYawController"] = SimpleTrackingYawController


database["Default"] = database[rospy.get_param("YawControllerDefault","SimpleTrackingYawController")]