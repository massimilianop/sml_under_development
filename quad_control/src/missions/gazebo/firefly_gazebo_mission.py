from .. import mission

import missions_database
MISSIONS_DATABASE = missions_database.database

import utilities.jsonable as js

class FireflyGazeboMission(mission.Mission):
    """docstring for FireflyGazeboMission"""
    
    js.Jsonable.add_inner('mission_object',MISSIONS_DATABASE)

    @classmethod
    def description(cls):
        string = """Missions with firefly in gazebo"""
        return string

    def object_description(self):
        string = """
        Mission with firefly in gazebo: chosen mission. This mission depends on:
        <ul>
          <li>mission_object: """ + self.mission_object.__class__.__name__ +"""</li>
        </ul>
        """
        return string            

    def __init__(self):
        self.add_inner_defaults()

    def get_mission(self):
    	return self.mission_object