#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

# import services defined in quad_control
from quad_control.srv import *

services_database = {}
services_database['ServiceChangeMission']       = SrvCreateJsonableObjectByStr
services_database['ServiceChangeReference']     = SrvCreateJsonableObjectByStr
services_database['ServiceChangeYawReference']  = SrvCreateJsonableObjectByStr
services_database['ServiceChangeYawController'] = SrvCreateJsonableObjectByStr
services_database['ServiceChangeController']    = SrvCreateJsonableObjectByStr

services_database['IrisPlusResetNeutral']    = IrisPlusResetNeutral
services_database['IrisPlusSetNeutral']      = IrisPlusSetNeutral
 

import json

# TODO: reply may have other fields
def request_service(namespace,service_name_str,service_inputs_dic):

    ServiceClass = services_database[service_name_str]
    try: 
        # time out of one second for waiting for service
        rospy.wait_for_service(namespace+service_name_str,1.0)
        try:
            # request service
            request = rospy.ServiceProxy(namespace+service_name_str, ServiceClass)
            reply   = request(**service_inputs_dic)

            if reply.received == True:
                rospy.logwarn('Service '+service_name_str+' provided')
        except rospy.ServiceException as exc:
            rospy.logwarn('Service '+service_name_str+' did not process request: ' + str(exc))
            pass
    except rospy.ServiceException as exc:
        rospy.logwarn('Service '+service_name_str+' did not process request: ' + str(exc))
        pass

class ServiceTimeSequencer():

    def __init__(self):

        # frequency of node (Hz)
        self.frequency = 1.0

    def _handle_service_sequence(self,data):
        print('aaaaaaa')
        self.service_sequence = json.loads(data.service_sequence)
        self.initial_instant  = rospy.get_time()

        self.handle_service_sequence()

        return ServiceSequenceResponse(received = True)

    def handle_service_sequence(self):

        # if service_sequence is non-empty
        if self.service_sequence:
            
            # remove first service in list of services
            service = self.service_sequence.pop(0)

            self.next_trigger_instant = service['trigger_instant']
            self.service_name         = service['service_name']
            self.inputs_service       = service['inputs_service']
            
        else:
            self.next_trigger_instant = 3600

    def update_and_request(self):

        print(self.service_name)
        request_service(self.namespace,self.service_name,self.inputs_service)

        self.handle_service_sequence()


    def loop(self):

        # node will be named quad_control (see rqt_graph)
        rospy.init_node('service_time_sequencer', anonymous=True)

        # service for selecting desired trajectory
        TrajDes_service = rospy.Service('ServiceSequencer', ServiceSequence, self._handle_service_sequence)

        rate = rospy.Rate(self.frequency)

        self.namespace = ''

        self.initial_instant      = rospy.get_time()
        self.next_trigger_instant = 3600

        while not rospy.is_shutdown():

            current_time = rospy.get_time() - self.initial_instant

            if current_time >= self.next_trigger_instant:
                self.update_and_request()

            # go to sleep
            rate.sleep()


if __name__ == '__main__':
    service_sequencer = ServiceTimeSequencer()
    try:
        service_sequencer.loop()
    except rospy.ROSInterruptException:
        pass

