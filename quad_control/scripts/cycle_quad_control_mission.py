#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

# controller node publishes message of this type so that GUI can plot stuff
from quad_control.msg import quad_state_and_cmd

# import services defined in quad_control
# SaveData is for saving data in txt file
from quad_control.srv import *

# to work with directories relative to ROS packages
from rospkg import RosPack

import numpy

# import missions.missions_database
# MISSIONS_DATABASE = missions.missions_database.database2

import type_uav.type_uav
MISSIONS_DATABASE = type_uav.type_uav.database

import json

class QuadController():

    def __init__(self):

        # frequency of controlling action!!
        self.frequency = 35.0

        # initialize counter for publishing to GUI
        # we only publish when self.PublishToGUI =1
        self.PublishToGUI = 1
        # Frequency of publishing to GUI (Hz)
        frequency_PubToGui = 10
        # we reset counter when self.PublishToGUI >= PublishToGUIBound
        self.PublishToGUIBound = int(self.frequency/frequency_PubToGui)

        # for saving data
        # determine ROS workspace directory
        rp = RosPack()
        # determine ROS workspace directory where data is saved
        package_path = rp.get_path('quad_control')
        self.package_save_path = package_path+'/experimental_data/data/'

    # callback for when "saving data" is requested
    def _handle_save_data(self,req):
        
        if req.flag_save == True:
            # if GUI request data to be saved create file
            
            # namespace, e.g. /Iris1/
            namespace = rospy.get_namespace()
            if not (namespace == ""):
                # remove / symbol to namespace: e.g, we get namespace= Iris1
                namespace = namespace.replace("/", "")

            # string for time: used for generating files
            # time_stamp = str(int(rospy.get_time() - self.TimeSaveData))
            time_stamp = str(int(rospy.get_time()))

            # file name provided by user in request 
            file_name         = req.file_name
            # note that "_" is necessary because time_stamp is a number (and file name cannot start with numbers)
            self.file_handle  = file(self.package_save_path+'_'+time_stamp+'_'+file_name+namespace+'.txt', 'w')

            self._add_header_mission(flag=True)

            # if GUI request data to be saved, set flag to true
            self.SaveDataFlag = True

        else:
            # if GUI request data NOT to be saved, set falg to False
            self.SaveDataFlag = False

        # return message to Gui, to let it know resquest has been fulfilled
        return SaveDataResponse(True)

    # callback for when changing mission
    def _handle_service_change_mission(self,req):

        # dictionary = '{"sequence_inner_key":"","key":"","input_string":""}'
        dictionary = json.loads(req.dictionary)

        sequence_of_inners = dictionary["sequence_inner_key"]

        if sequence_of_inners:
            self.mission.change_inner_of_inner(**dictionary)
        else:
            # change mission, if sequence is empty
            
            # mission name
            self.mission_name = dictionary["key"]

            # chosen class taken from dictionary
            MissionClass = MISSIONS_DATABASE[dictionary["key"]]
        
            self.mission = MissionClass.from_string(dictionary["input_string"])            


        self._add_header_mission()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvChangeJsonableObjectByStrResponse(received = True)   


    # callback for when changing mission
    def _handle_service_call_mission_method(self,req):

        # dictionary = '{"sequence_inner_key":"","func_name":"","input_to_func":""}'
        dictionary = json.loads(req.dictionary)

        self.mission.call_method_inner_of_inner(**dictionary)

        self._add_header_mission()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvChangeJsonableObjectByStrResponse(received = True)     


    def _add_header_mission(self,flag=False):
        """Print string to file that may be used to reconstruct mission object.
        Only print to file, if saving_mode is ON.
        It might be the case that the mode is turned ON after the header, and for that use flag=True, but self.file_handle must be created before
        """
        
        if self.SaveDataFlag == True or flag:

            # parametric description is a method of jsonable
            string = self.mission.parametric_description(self.mission_name)

            # if data is being saved, append mission header
            numpy.savetxt(self.file_handle, [string], fmt="%s")

    def control_compute(self):

        # node will be named quad_control (see rqt_graph)
        rospy.init_node('quad_control', anonymous=True)

        # message published by quad_control to GUI 
        self.pub = rospy.Publisher('quad_state_and_cmd', quad_state_and_cmd, queue_size=10)

        #-----------------------------------------------------------------------#
        # TO SAVE DATA FLAG
        # by default, NO data is saved
        self.SaveDataFlag = False
        # we will use this just for convenience, when generating the names of the files
        # where the data will be saved
        self.TimeSaveData = rospy.get_time()
        # Service is created, so that data is saved when GUI requests
        Save_data_service = rospy.Service('SaveDataFromGui', SaveData, self._handle_save_data)

        #-----------------------------------------------------------------------#
        rospy.Service('ServiceChangeMission', SrvChangeJsonableObjectByStr, self._handle_service_change_mission)
        rospy.Service('ServiceChangeMissionCallMethod', SrvChangeJsonableObjectByStr, self._handle_service_call_mission_method)
        
        #-----------------------------------------------------------------------#
        self.mission_name  = 'Default'
        # Default Mission Class
        MissionClass = MISSIONS_DATABASE['Default']        
        # construct a default object
        self.mission = MissionClass()

        # self.mission = missions.type_uav_mission.MissionGeneral().get_mission()

        rate = rospy.Rate(self.frequency)

        self.emergency_flag = False

        while not rospy.is_shutdown():

            # this may come after the publish, but beware that it affects the delta_t
            if self.SaveDataFlag == True:
                # if we want to save data
                numpy.savetxt(self.file_handle, [numpy.array(self.mission.get_complete_data())], delimiter=' ')
                # see if we save time by saving like this
                # numpy.savetxt(self.file_handle, [numpy.array(self.mission.get_complete_data())], delimiter=' ', fmt='%.3f')
                # see if we save time by saving like this
                # numpy.savetxt(self.file_handle, [numpy.array([rospy.get_time()])], delimiter=' ', fmt='%.3f')

            self.mission.publish()

            if self.mission.test_emergency():
                if not self.emergency_flag:
                    self.emergency_flag = True
                    self.mission.trigger_emergency()
                    print('EMERGENGY TRIGGERED\n')
                    print('EMERGENGY TRIGGERED\n')
                    print('EMERGENGY TRIGGERED\n')
                    print('EMERGENGY TRIGGERED\n')

            # publish to GUI (it also contains publish state of Control to GUI)
            self.PublishToGui()
            
            # go to sleep
            rate.sleep()

if __name__ == '__main__':
    AQuadController = QuadController()
    try:
        AQuadController.control_compute()
    except rospy.ROSInterruptException:
        pass
