
#TODO some nice description

simulators_dictionary = {}

import simulators_hierarchical.no_attitude_inner_loop_simulator as nails
simulators_dictionary["Default"] = nails.NoAttitudeInnerLoopSimulator
simulators_dictionary["no_attitude_inner_loop"] = nails.NoAttitudeInnerLoopSimulator

import simulators_hierarchical.attitude_inner_loop_simulator as ails
simulators_dictionary["attitude_inner_loop"] = ails.AttitudeInnerLoopSimulator
