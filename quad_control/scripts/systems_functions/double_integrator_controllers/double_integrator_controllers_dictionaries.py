#!/usr/bin/env python
# this line is just used to define the type of document

double_integrator_controllers_dictionaries = {}
double_integrator_controllers_dictionaries_numbered = {}


from double_integrator_bounded_not_component_wise_no_inertial_measurements_needed.di_controller_2 import DIController
double_integrator_controllers_dictionaries['DefaultDIController'] = DIController
double_integrator_controllers_dictionaries_numbered['DefaultDIController'] = 1

from double_integrator_bounded_not_component_wise_no_inertial_measurements_needed.di_controller_2 import DIController
double_integrator_controllers_dictionaries['DefaultDIController2'] = DIController
double_integrator_controllers_dictionaries_numbered['DefaultDIController2'] = 2

