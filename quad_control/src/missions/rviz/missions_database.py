
#!/usr/bin/env python
"""
Database of the missions.
"""

import rospy

database = {}

import iris_simulator_trajectory_tracking.iris_simulator_trajectory_tracking
database["IrisSimulatorTrajectoryTracking"] = iris_simulator_trajectory_tracking.iris_simulator_trajectory_tracking.IrisSimulatorTrajectoryTracking

database["Default"] = database[rospy.get_param("MissionDefaultIrisSimulator","IrisSimulatorTrajectoryTracking")]