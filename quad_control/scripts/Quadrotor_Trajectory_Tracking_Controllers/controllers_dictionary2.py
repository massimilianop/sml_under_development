#!/usr/bin/env python
# this line is just used to define the type of document

controllers_dictionary2 = {}
from Fully_Actuated.Controller_Neutral.ControllerNeutral import ControllerNeutral
controllers_dictionary2['NeutralController'] = ControllerNeutral

from Fully_Actuated.PID_With_Bounded_Integral.ControllerPIDBoundedIntegral import ControllerPIDBoundedIntegral
controllers_dictionary2['PIDBoundedIntegralController'] = ControllerPIDBoundedIntegral

from Fully_Actuated.PID_Simple_With_Bounded_Integral.ControllerPIDSimpleBoundedIntegral import ControllerPIDSimpleBoundedIntegral
controllers_dictionary2['PIDSimpleBoundedIntegralController'] = ControllerPIDSimpleBoundedIntegral

from Fully_Actuated.PID_XY_And_Z__With_Bounded_Integral.ControllerPIDXYAndZBoundedIntegral import ControllerPIDXYAndZBounded
controllers_dictionary2['PIDXYAndZBoundedController'] = ControllerPIDXYAndZBounded


controllers_with_state = [1,2]