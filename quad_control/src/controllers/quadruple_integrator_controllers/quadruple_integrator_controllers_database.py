
import rospy

database = {}

import quadruple_integrator_component_wise.quadruple_integrator_component_wise
database["LinearQuadrupleIntController"] = quadruple_integrator_component_wise.quadruple_integrator_component_wise.LinearQuadrupleIntegratorController

database["Default"] = database[rospy.get_param("QuadrupleIntegratorController","LinearQuadrupleIntController")]





