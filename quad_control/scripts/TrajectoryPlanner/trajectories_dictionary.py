#!/usr/bin/env python
# this line is just used to define the type of document

trajectories_dictionary = {}

from fixed_point_trajectory import FixedPointTrajectory
trajectories_dictionary['StayAtRest'] = FixedPointTrajectory

from circle_trajectory import CircleTrajectory
trajectories_dictionary['DescribeCircle'] = CircleTrajectory
