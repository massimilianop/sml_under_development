#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

database = {}

from fixed_point_trajectory.fixed_point_trajectory import FixedPointTrajectory
database['StayAtRest'] = FixedPointTrajectory

from circle_trajectory.circle_trajectory import CircleTrajectory
database['DescribeCircle'] = CircleTrajectory

database["Default"] = database[rospy.get_param("TrajectoryDefault","StayAtRest")]