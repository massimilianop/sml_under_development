
#!/usr/bin/env python
"""
Database of the missions.
"""

import rospy

database = {}

import iris_real_trajectory_tracking.iris_real_trajectory_tracking
database["IrisRealTrajectoryTracking"] = iris_real_trajectory_tracking.iris_real_trajectory_tracking.IrisRealTrajectoryTracking

database["Default"] = database[rospy.get_param("MissionDefault","IrisRealTrajectoryTracking")]
