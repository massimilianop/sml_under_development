#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

from quad_control.srv import ChangeFlightMode


# Clients for the ChangeFlightMode service

def change_flight_mode_client(mode=str):
    change_uav_1_mode_client(mode)
    change_uav_2_mode_client(mode)

def change_uav_1_mode_client(mode=str):
    rospy.wait_for_service('/firefly/change_flight_mode')
    try :
        change_uav_1_mode_to = rospy.ServiceProxy('/firefly/change_flight_mode',ChangeFlightMode)
        response_1 = change_uav_1_mode_to(mode)
        return
    except rospy.ServiceException:
        pass

def change_uav_2_mode_client(mode=str):
    rospy.wait_for_service('/firefly_2/change_flight_mode')
    try :
        change_uav_2_mode_to = rospy.ServiceProxy('/firefly_2/change_flight_mode',ChangeFlightMode)
        response_2 = change_uav_2_mode_to(mode)
        return
    except rospy.ServiceException:
        pass