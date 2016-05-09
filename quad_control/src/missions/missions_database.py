
#!/usr/bin/env python
"""
Database of the missions.
"""

import rospy

database = {}

import firefly_trajectory_tracking.firefly_trajectory_tracking
database["FireflyTrajectoryTracking"] = firefly_trajectory_tracking.firefly_trajectory_tracking.FireflyTrajectoryTracking

import iris_real_trajectory_tracking.iris_real_trajectory_tracking
database["IrisRealTrajectoryTracking"] = iris_real_trajectory_tracking.iris_real_trajectory_tracking.IrisRealTrajectoryTracking

import iris_simulator_trajectory_tracking.iris_simulator_trajectory_tracking
database["IrisSimulatorTrajectoryTracking"] = iris_simulator_trajectory_tracking.iris_simulator_trajectory_tracking.IrisSimulatorTrajectoryTracking

import firefly_load_lifting.firefly_load_lifting
database["FireflyLoadLifting"] = firefly_load_lifting.firefly_load_lifting.FireflyLoadLifting

database["Default"] = database[rospy.get_param("MissionDefault","IrisSimulatorTrajectoryTracking")]
