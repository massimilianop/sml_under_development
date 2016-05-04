"""This file implements a coverage mission."""

from missions import mission as ms

import utilities.coverage as cov

from controllers.fa_trajectory_tracking_controllers import fa_trajectory_tracking_controllers_database as fattcs
from yaw_rate_controllers import yaw_controllers_database as ycs

import rospy as rp

class CoverageMission(ms.Mission):
    
    
    inner = {}
    inner['controller'] = fattcs.database
    inner['yaw_controller'] = ycs.database
    
    
    def __init__(self,
        controller = fattcs.database['Default'],
        yaw_controller = ycs.database['Default']
        other_quads_names = ['Iris2', 'Iris3', 'Iris4']
        landmarks = {}
        ):
        
        ms.Mission.__init__(self)
        
