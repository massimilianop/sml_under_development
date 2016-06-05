from .. import mission

import missions_database

class IrisRvizMission(mission.Mission):
    """docstring for FireflyGazeboMission"""

    inner = {"mission_object":missions_database.database}

    @classmethod
    def description(cls):
        string = """Missions with Iris simulated and presented in rviz"""
        return string    	

    def __init__(self, mission_object=missions_database.database["Default"]()):
        self.mission_object = mission_object

    def get_mission(self):
    	return self.mission_object