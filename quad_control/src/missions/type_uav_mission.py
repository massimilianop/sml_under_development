#!/usr/bin/env python

from utilities import jsonable as js

import missions_database

class MissionGeneral(js.Jsonable):
    """General Mission, where type of uav must be selected"""

    @classmethod
    def description(cls):
        string = """General Mission, where type of uav must be selected. Options are:
        <ul>
          <li>gazebo: ...</li>
          <li>rviz: ...</li>
          <li>mocap: ...</li>
        </ul>
        """
        return string 

    inner = {"type_uav_mission":missions_database.database2}

    def __init__(self, type_uav_mission=missions_database.database2["Default"]()):
        self.type_uav_mission = type_uav_mission

    def get_mission(self):
        return self.type_uav_mission.get_mission()

