#!/usr/bin/env python
# this is just to define file type

import numpy

import rospy

import json

# import services defined in quad_control
from quad_control.srv import SrvChangeJsonableObjectByStr,SrvChangeJsonableObjectByStrResponse

import simulators.simulators_dictionary
SIMULATORS_DATABASE = simulators.simulators_dictionary.simulators_dictionary

class SimulatorNode():

    def __init__(self):

        # frequency of node (Hz)
        self.frequency = 100

        # frequency of publish = frequency/publish_frequency_ratio
        self.publish_frequency_ratio = int(3)
        # initialize counter
        self.publish_counter = 1

        self.simulator_name  = 'Default'
        # Default Simulator Class
        SimulatorClass = SIMULATORS_DATABASE['Default']        
        # construct a default object
        self.simulator = SimulatorClass()

        # stop simulator when emergency is True
        self.emergency_triggered = False
        # emergency is set to True when position outside of box
        self.box_center = numpy.array([0.0,0.0,1.0])
        self.box_sides  = numpy.array([2.0,2.0,1.5])        

    # callback used for changing simulator
    def __handle_simulator_change_service(self,req):
        SimulatorClass = simdic[req.jsonable_name]
        
        self.sim = SimulatorClass.from_string(req.string_parameters)

        # return message: resquest has been fulfilled
        return SrvCreateJsonableObjectByStrResponse(received = True)


    # callback for when changing mission
    def _handle_service_change_simulator(self,req):

        # dictionary = '{"sequence_inner_key":"","key":"","input_string":""}'
        dictionary = json.loads(req.dictionary)

        sequence_of_inners = dictionary["sequence_inner_key"]

        if sequence_of_inners:
            self.simulator.change_inner_of_inner(**dictionary)
        else:
            # change mission, if sequence is empty
            
            # mission name
            self.simulator_name = dictionary["key"]

            # chosen class taken from dictionary
            SimulatorClass = SIMULATORS_DATABASE[dictionary["key"]]
            
            self.simulator.unregister()
            self.simulator = SimulatorClass.from_string(dictionary["input_string"])            

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvChangeJsonableObjectByStrResponse(received = True)   


    # callback for when changing mission
    def _handle_service_call_simulator_method(self,req):

        # dictionary = '{"sequence_inner_key":"","func_name":"","input_to_func":""}'
        dictionary = json.loads(req.dictionary)

        self.simulator.call_method_inner_of_inner(**dictionary)

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvChangeJsonableObjectByStrResponse(received = True)

    def publish(self):

        if self.publish_counter <= self.publish_frequency_ratio:
            # if we dont publish, we increase counter
            self.publish_counter += 1
        else:
            # if we publish, we reset
            self.publish_counter = 1
            self.simulator.publish()

    def check_inside_limits(self,position):
        if any(numpy.absolute(position - self.box_center) >= self.box_sides):
            return False
        else:
            return True

    def stop_simulator(self):

        self.simulator_name  = 'Default'
        # Default Simulator Class
        SimulatorClass = SIMULATORS_DATABASE['Default']        
        # construct a default object
        self.simulator = SimulatorClass()

    def simulate_quad(self):

        # simulator node 
        rospy.init_node('simulator', anonymous=True)
   
        #-----------------------------------------------------------------------#
        # TO SAVE FLAG
        self.SaveFlag = False

        #-----------------------------------------------------------------------#
        # Service is created, so that user can change simulator on GUI
        rospy.Service('ServiceChangeSimulator', SrvChangeJsonableObjectByStr, self._handle_service_change_simulator)
        rospy.Service('ServiceChangeSimulatorCallMethod', SrvChangeJsonableObjectByStr, self._handle_service_call_simulator_method)
        
        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():

            self.simulator.run(1.0/self.frequency)

            position = self.simulator.get_position()
            if (not self.check_inside_limits(position)) and (not self.emergency_triggered):
                self.emergency_triggered  = True
                self.stop_simulator()

            self.publish()

            # let node sleep
            rate.sleep()

if __name__ == '__main__':
    simulator_node = SimulatorNode()
    simulator_node.simulate_quad()