
#TODO some nice description

simulators_dictionary = {}

import attitude_inner_loop_simulator.attitude_inner_loop_simulator as ails
simulators_dictionary["attitude_inner_loop"] = ails.AttitudeInnerLoopSimulator

import no_attitude_inner_loop_simulator.no_attitude_inner_loop_simulator as nails
simulators_dictionary["no_attitude_inner_loop"] = nails.NoAttitudeInnerLoopSimulator


import double_integrator_simulator.double_integrator_simulator
simulators_dictionary["DoubleIntegratorSimulator"] = double_integrator_simulator.double_integrator_simulator.DoubleIntegratorSimulator
simulators_dictionary["Default"]                   = double_integrator_simulator.double_integrator_simulator.DoubleIntegratorSimulator
