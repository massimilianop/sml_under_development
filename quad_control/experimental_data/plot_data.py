#!/usr/bin/env python

# import rospkg
# # get an instance of RosPack with the default search paths
# rospack = rospkg.RosPack()
# # get the file path for rospy_tutorials
# import sys
# sys.path.insert(0, rospack.get_path('quad_control'))
# # no need to get quad_control path, since it is package; import controllers dictionary
# from src.utilities import jsonable

# from home.pedrootao.SML_CODE.src.quad_control.src.utilities import jsonable

import missions

# import missions_database

# data_file = ""

# # read mode by default
# read_file = open(data_file)

# read_file = read_file.split(Jsonable.UNIQUE_STRING)

# while len(read_file) >= 1:
# 	description = read_file.pop(0)

# 	name, parametric_description = jsonable.Jsonable.inverse_parametric_description(description)

# 	data = read_file.pop(0)

# 	missions_database[name].plot_from_string(data)