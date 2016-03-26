#!/usr/bin/env python
# this line is just used to define the type of document

controllers_dictionary = {}


from fully_actuated.controller_neutral.controller_neutral import ControllerNeutral
controllers_dictionary['NeutralController'] = ControllerNeutral

from fully_actuated.pid_simple_with_bounded_integral.controller_pid_simple_bounded_integral import ControllerPIDSimpleBoundedIntegral
controllers_dictionary['PIDSimpleBoundedIntegralController'] = ControllerPIDSimpleBoundedIntegral

from fully_actuated.pid_with_bounded_integral.controller_pid_bounded_integral import ControllerPIDBoundedIntegral
controllers_dictionary['PIDBoundedIntegralController'] = ControllerPIDBoundedIntegral

# from fully_actuated.PID_XY_And_Z__With_Bounded_Integral.ControllerPIDXYAndZBoundedIntegral import ControllerPIDXYAndZBounded
# controllers_dictionary['PIDXYAndZBoundedController'] = ControllerPIDXYAndZBounded
