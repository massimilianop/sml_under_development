
#!/usr/bin/env python
"""
Database of the missions.
"""

import rospy

database = {}

import firefly_trajectory_tracking.firefly_trajectory_tracking
database["FireflyTrajectoryTracking"] = firefly_trajectory_tracking.firefly_trajectory_tracking.FireflyTrajectoryTracking

import firefly_load_lifting.firefly_load_lifting
database["FireflyLoadLifting"] = firefly_load_lifting.firefly_load_lifting.FireflyLoadLifting

database["Default"] = database[rospy.get_param("MissionDefault","FireflyTrajectoryTracking")]
