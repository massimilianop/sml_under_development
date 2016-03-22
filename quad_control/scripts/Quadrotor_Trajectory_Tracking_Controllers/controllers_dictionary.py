#!/usr/bin/env python
# this line is just used to define the type of document

controllers_dictionary = {}
from Fully_Actuated.Controller_Neutral.ControllerNeutral import ControllerNeutral
controllers_dictionary['NeutralController'] = ControllerNeutral

from Fully_Actuated.PID_With_Bounded_Integral.ControllerPIDBoundedIntegral import ControllerPIDBoundedIntegral
controllers_dictionary['PIDBoundedIntegralController'] = ControllerPIDBoundedIntegral

from Fully_Actuated.PID_Simple_With_Bounded_Integral.ControllerPIDSimpleBoundedIntegral import ControllerPIDSimpleBoundedIntegral
controllers_dictionary['PIDSimpleBoundedIntegralController'] = ControllerPIDSimpleBoundedIntegral

from Fully_Actuated.PID_XY_And_Z__With_Bounded_Integral.ControllerPIDXYAndZBoundedIntegral import ControllerPIDXYAndZBounded
controllers_dictionary['PIDXYAndZBoundedController'] = ControllerPIDXYAndZBounded


controllers_with_state = [1,2]