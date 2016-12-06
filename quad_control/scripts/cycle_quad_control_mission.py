#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

# import services defined in quad_control
# SaveData is for saving data in txt file
from quad_control.srv import *

# to work with directories relative to ROS packages
from rospkg import RosPack

import numpy

import type_uav.type_uav
MISSIONS_DATABASE = type_uav.type_uav.database

import json

class QuadController():

    def __init__(self):

        # frequency of controlling action!!
        self.frequency = 35.0

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
            if rospy.get_param('playing_back',True):
                self.message += "\n\n" + 'At ' + str(rospy.get_time() - self.message_time) + ' seconds'
                dictionary = json.loads(req.dictionary)
                sequence_of_inners = dictionary["sequence_inner_key"]      

                jsonable = self.mission.get_inner(sequence_of_inners)
                msg = jsonable.object_combined_description()
                self.message += "\n\n" + msg

        else:
            # change mission, if sequence is empty

            # mission name
            self.mission_name = dictionary["key"]

            # chosen class taken from dictionary
            MissionClass = MISSIONS_DATABASE[dictionary["key"]]
        
            self.mission.to_do_before_finishing() 
            self.mission = MissionClass.from_string(dictionary["input_string"])            

            if rospy.get_param('playing_back',True):
                self.message += "\n\n" + 'At ' + str(rospy.get_time() - self.message_time) + ' seconds'
                self.message += "\n\n" + self.mission.object_combined_description()


        self._add_header_mission()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvChangeJsonableObjectByStrResponse(received = True)   


    # callback for when changing mission
    def _handle_service_call_mission_method(self,req):

        # dictionary = '{"sequence_inner_key":"","func_name":"","input_to_func":""}'
        dictionary = json.loads(req.dictionary)

        msg = self.mission.call_method_inner_of_inner(**dictionary)
        
        self._add_header_mission()

        if rospy.get_param('playing_back',True):
            self.message += "\n\n" + 'At ' + str(rospy.get_time() - self.message_time) + ' seconds'
            self.message += "\n\n" + str(msg)

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

        if rospy.get_param('playing_back',True):
            self.message_time = rospy.get_time()
            self.message  = 'At 0 seconds:'
            self.message += "\n\n" + self.mission.object_combined_description()

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
                    print(4*'EMERGENGY TRIGGERED\n')
            
            # go to sleep
            rate.sleep()

if __name__ == '__main__':
    AQuadController = QuadController()
    try:
        AQuadController.control_compute()
    except rospy.ROSInterruptException:
        pass

    if rospy.get_param('playing_back',True):
        import pdfkit
        pdfkit.from_string(AQuadController.message, "/home/pedrootao/SML_CODE/src/quad_control/experimental_data/data.pdf")
        AQuadController.mission.print_all_plots("/home/pedrootao/SML_CODE/src/quad_control/experimental_data/data.pdf")
        #AQuadController.mission.to_do_before_finishing()

