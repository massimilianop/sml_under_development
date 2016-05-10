#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

database = {}

from fixed_yaw_trajectory.fixed_yaw_trajectory import FixedYawTrajectory
database['FixedYaw'] = FixedYawTrajectory

from sinusoidal_yaw_trajectory.sinusoidal_yaw_trajectory import SinusoidalYawTrajectory
database['SinusoidalYaw'] = SinusoidalYawTrajectory

database["Default"] = database[rospy.get_param("YawTrajectoryDefault","FixedYaw")]