#!/usr/bin/env python
# this line is just used to define the type of document

from double_integrator_controller import DoubleIntregratorController as DIC

from double_integrator_bounded_and_component_wise_controllers.dibcw_controller import DIBCWController as DIBCWC

double_integrator_controllers_dictionaries = {}

from fully_actuated.controller_neutral.controller_neutral import ControllerNeutral
controllers_dictionary['NeutralController'] = ControllerNeutral



