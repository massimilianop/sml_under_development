#!/usr/bin/env python
# this line is just used to define the type of document

from .. import yaw_controller as yc


class NeutralYawController(yc.YawController):

	@classmethod
	def description(cls):
		return "<b>Zero yaw rate</b>"

	# def __init__(self):
	#     pass

	def __str__(self):
		string = yc.YawController.__str__(self)
		return string

	def output(self, state, state_desired):
		return 0.0





