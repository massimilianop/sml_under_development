#TODO add an abstract fully-actuated controller to import

import rospy

database = {}

from neutral_controller.neutral_controller import NeutralController
database["NeutralController"] = NeutralController

from simple_pid_controller.simple_pid_controller import SimplePIDController
database["SimplePIDController"] = SimplePIDController

from abstract_pid_controller.abstract_pid_controller import ThreeDPIDController
database["AbstractPIDController"] = ThreeDPIDController


database["Default"] = database[rospy.get_param("ControllerDefault","SimplePIDController")]