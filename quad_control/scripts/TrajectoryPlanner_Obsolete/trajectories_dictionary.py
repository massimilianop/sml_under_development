#!/usr/bin/env python
# this line is just used to define the type of document

trajectories_dictionary = {}

from Stay_At_Rest.StayAtRest import StayAtRest
trajectories_dictionary['StayAtRest'] = StayAtRest

from Describe_Circle.DescribeCircle import DescribeCircle
trajectories_dictionary['DescribeCircle'] = DescribeCircle
