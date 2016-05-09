
import rospy

database = {}

# no need for this double integrator, since it does nothing, and it should not be the default di controller
import neutral_dic.neutral_dic
database["NeutralDIC"] = neutral_dic.neutral_dic.NeutralDIC

import component_wise_3d_dic.component_wise_3d_dic
database["ComponentWise3DDIC"] = component_wise_3d_dic.component_wise_3d_dic.ComponentWise3DDIC

import not_component_wise_3d_dic.not_component_wise_3d_dic
database["NOTComponentWise3DDIC"] = not_component_wise_3d_dic.not_component_wise_3d_dic.NotComponentWise3DDIC

import n_dimensional_bounded_dic.n_dimensional_bounded_dic
database["BoundedNotComponentWiseDIC"] = n_dimensional_bounded_dic.n_dimensional_bounded_dic.NDimensionalBoundedDIC

import one_dimensional_bounded_dic.one_dimensional_bounded_dic
database["OneDBoundedDIC"] = one_dimensional_bounded_dic.one_dimensional_bounded_dic.OneDimensionalBoundedDIC


database["Default"] = database[rospy.get_param("DIControllerDefault","ComponentWise3DDIC")]





