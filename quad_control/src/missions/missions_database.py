
#!/usr/bin/env python
"""
Database of the missions.
"""

import rospy

# by default, gazebo is used
arg = rospy.get_param("mission_type","gazebo")

if arg == "gazebo":
	import gazebo.missions_database
	database = gazebo.missions_database.database

if arg == "mocap":
	import mocap.missions_database
	database = mocap.missions_database.database

if arg == "rviz":
	import rviz.missions_database
	database = rviz.missions_database.database



database2 = {}

import gazebo.firefly_gazebo_mission
database2["gazebo"] = gazebo.firefly_gazebo_mission.FireflyGazeboMission
database2["Default"] = gazebo.firefly_gazebo_mission.FireflyGazeboMission

# import mocap.missions_database
# database2["mocap"] = mocap.missions_database.database

import rviz.iris_rviz_mission
database2["rviz"] = rviz.iris_rviz_mission.IrisRvizMission