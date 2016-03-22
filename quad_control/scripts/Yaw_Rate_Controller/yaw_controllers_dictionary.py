#!/usr/bin/env python
# this line is just used to define the type of document

yaw_controllers_dictionary = {}

from Yaw_Rate_Controller_Neutral.YawRateControllerNeutral import YawRateControllerNeutral
yaw_controllers_dictionary['YawRateControllerNeutral'] = YawRateControllerNeutral

from Yaw_Rate_Controller_Track_Reference_Psi.YawRateControllerTrackReferencePsi import YawRateControllerTrackReferencePsi
yaw_controllers_dictionary['YawRateControllerTrackReferencePsi'] = YawRateControllerTrackReferencePsi
