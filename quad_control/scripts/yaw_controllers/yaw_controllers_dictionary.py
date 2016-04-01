#!/usr/bin/env python
# this line is just used to define the type of document

import yaw_controller as yc
import tracking_yaw_controller as tyc

yaw_controllers_dictionary = {}

#from Yaw_Rate_Controller_Neutral.YawRateControllerNeutral import YawRateControllerNeutral
yaw_controllers_dictionary['YawController'] = yc.YawController

#from Yaw_Rate_Controller_Track_Reference_Psi.YawRateControllerTrackReferencePsi import YawRateControllerTrackReferencePsi
yaw_controllers_dictionary['TrackingYawController'] = tyc.TrackingYawController


#print yaw_controllers_dictionary
