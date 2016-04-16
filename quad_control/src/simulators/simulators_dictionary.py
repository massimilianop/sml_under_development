
#TODO some nice description

simulators_dictionary = {}


import attitude_inner_loop_simulator.attitude_inner_loop_simulator as ails
simulators_dictionary["AttitudeInnerLoopSimulator"] = ails.AttitudeInnerLoopSimulator


import no_attitude_inner_loop_simulator.no_attitude_inner_loop_simulator as nails
simulators_dictionary["NoAttitudeInnerLoopSimulator"] = nails.NoAttitudeInnerLoopSimulator


import double_integrator_simulator.double_integrator_simulator
simulators_dictionary["DoubleIntegratorSimulator"] = double_integrator_simulator.double_integrator_simulator.DoubleIntegratorSimulator


from zero_simulator.zero_simulator import ZeroSimulator
simulators_dictionary["ZeroSimulator"] = ZeroSimulator


simulators_dictionary["Default"] = double_integrator_simulator.double_integrator_simulator.DoubleIntegratorSimulator
