database_dic = {}

# no need for this double integrator, since it does nothing, and it should not be the default di controller
# import double_integrator_neutral_controller
# di_ctrl_dictionary["DoubleIntegratorNeutralController"] =  double_integrator_neutral_controller.DoubleIntegratorNeutralController

import double_integrator_controller
database_dic["DefaultDIC"] = double_integrator_controller.DoubleIntegratorController

import component_wise_3d_dic.component_wise_3d_dic
database_dic["ComponentWise3DDIC"] = component_wise_3d_dic.component_wise_3d_dic.ComponentWise3DDIC

import not_component_wise_3d_dic.not_component_wise_3d_dic
database_dic["NOTComponentWise3DDIC"] = not_component_wise_3d_dic.not_component_wise_3d_dic.NotComponentWise3DDIC

import n_dimensional_bounded_dic.n_dimensional_bounded_dic
database_dic["BoundedNotComponentWiseDIC"] = n_dimensional_bounded_dic.n_dimensional_bounded_dic.NDimensionalBoundedDIC

import neutral_dic.neutral_dic
database_dic["NeutralDIC"] = neutral_dic.neutral_dic.NeutralDIC


