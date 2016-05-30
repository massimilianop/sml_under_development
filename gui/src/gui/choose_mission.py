import os, rospy

import QtGui, PyQt4.QtCore

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal

# import services defined in quad_control; service being used: SrvCreateJsonableObjectByStr
from quad_control.srv import SrvCreateJsonableObjectByStr,IrisPlusResetNeutral,IrisPlusSetNeutral,ServiceSequence,SrvChangeJsonableObjectByStr

import argparse


import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for rospy_tutorials
import sys
sys.path.insert(0, rospack.get_path('quad_control'))

# no need to get quad_control path, since it is package; import missions dictionary
from src.missions import missions_database

from src.utilities import jsonable

from choose_jsonable import ChooseJsonablePlugin

import json


# read mode by default

class ChooseMissionPlugin(Plugin):

    def __init__(self, context,namespace = None):

        # "global variables"
        self.dic_sequence_services = {}
        self.dic_sequence_services['last_trigger_time']            = 0.0
        self.dic_sequence_services['checked_sequence_of_missions'] = True
        self.dic_sequence_services['list_sequence_services']       = []

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())
        self.context   = context

        super(ChooseMissionPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChooseMissionPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
               
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'choose_mission.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ChooseMissionUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # ---------------------------------------------- #
        # ---------------------------------------------- #

        self._widget.ResetIrisNeutralValue.clicked.connect(self.reset_iris_neutral_value)
        self._widget.SetIrisNeutralValue.clicked.connect(self.set_iris_neutral_value)


        self._widget.SetSequenceMissions.clicked.connect(self.send_list_of_services)
        # ---------------------------------------------- #
        
        # button to request service for setting new controller, with new parameters
        self._widget.SetControllerButton.clicked.connect(self.__get_new_controller_parameters)

        # if item in list is selected, print corresponding message
        self._widget.ListControllersWidget.itemDoubleClicked.connect(self.__controller_item_clicked)
        self._widget.ListControllersWidget.itemClicked.connect(self.__print_controller_item_clicked)

        self.__reset_controllers_widget()

        # button for resetting list of available controllers
        self._widget.ResetControllersList.clicked.connect(self.__reset_controllers_widget)


        # self.choose_reference  = ChooseJsonablePlugin(context,\
        #     self.namespace,\
        #     name_tab = "Reference",
        #     dictionary_of_options = {},\
        #     service_name = 'ServiceChangeReference',\
        #     ServiceClass = SrvCreateJsonableObjectByStr)
        # self.choose_reference.dic_sequence_services = self.dic_sequence_services


        # self.choose_yaw_reference  = ChooseJsonablePlugin(context,\
        #     self.namespace,\
        #     name_tab = "YawReference",
        #     dictionary_of_options = {},\
        #     service_name = 'ServiceChangeYawReference',\
        #     ServiceClass = SrvCreateJsonableObjectByStr)        
        # self.choose_yaw_reference.dic_sequence_services = self.dic_sequence_services


        # self.choose_yaw_controller  = ChooseJsonablePlugin(context,\
        #     self.namespace,\
        #     name_tab = "YawController",
        #     dictionary_of_options = {},\
        #     service_name = 'ServiceChangeYawController',\
        #     ServiceClass = SrvCreateJsonableObjectByStr)
        # self.choose_yaw_controller.dic_sequence_services = self.dic_sequence_services


        # self.choose_controller  = ChooseJsonablePlugin(context,\
        #     self.namespace,\
        #     name_tab = "Controller",
        #     dictionary_of_options = {},\
        #     service_name = 'ServiceChangeController',\
        #     ServiceClass = SrvCreateJsonableObjectByStr)
        # self.choose_controller.dic_sequence_services = self.dic_sequence_services


        self._widget.tabWidget.currentChanged.connect(self.update_last_trigger_instant)


        # determine ROS workspace directory where data is saved
        package_path = rospkg.RosPack().get_path('quad_control')
        self.data_file_path = package_path+'/src/missions/sequence_missions.txt' 

        self._widget.save_new_sequence.clicked.connect(self.save_new_sequence)
        self._widget.new_sequence.clicked.connect(self.new_sequence)

        self._widget.available_mission_sequences.itemDoubleClicked.connect(self.request_sequence_mission)
        self._widget.available_mission_sequences.itemClicked.connect(self.print_sequence_mission_description)

        self.reset_sequence_missions()

    def new_sequence(self):

        if self._widget.new_sequence.text() == "New":

            self._widget.available_mission_sequences.clear()

            self._widget.mission_sequence_description.setText('{"name":"","description":""}')

            self._widget.new_sequence.setText("Reset")

            return 

        if self._widget.new_sequence.text() == "Reset":

            self.reset_sequence_missions()

            self._widget.new_sequence.setText("New")

            return

    def reset_sequence_missions(self):

        self.print_available_sequence_of_missions()

        self._widget.mission_sequence_description.setText('')

        return

    def request_sequence_mission(self):

        sequence_selected = self._widget.available_mission_sequences.currentItem().text()

        data_file = open(self.data_file_path, 'r+') 
        list_sequence_services = data_file.read()
        list_sequence_services = json.loads(list_sequence_services)
        data_file.close()

        for item in list_sequence_services:
            # item is a dictionary
            item = json.loads(item)
            if item["name"] == sequence_selected:
                sequence_description = item["sequence_of_missions"]
                sequence_description = json.dumps(sequence_description)
                self.send_list_of_services_2(sequence_description)
        
        self.reset_sequence_missions()

    def print_sequence_mission_description(self):

        sequence_selected = self._widget.available_mission_sequences.currentItem().text()

        data_file = open(self.data_file_path, 'r+') 
        list_sequence_services = data_file.read()
        list_sequence_services = json.loads(list_sequence_services)
        data_file.close()

        self._widget.mission_sequence_description.clear()

        for item in list_sequence_services:
            # item is a dictionary
            item = json.loads(item)
            if item["name"] == sequence_selected:
                sequence_description = item["description"]
                sequence_description = json.dumps(sequence_description)
                self._widget.mission_sequence_description.setText(sequence_description)

    def save_new_sequence(self):

        if self._widget.new_sequence.text() == "Reset":

            dictionary_string  = self._widget.mission_sequence_description.toPlainText()

            dictionary         = json.loads(dictionary_string)        
            # dictionary  = {"name":"","description":""}

            dictionary["sequence_of_missions"] = self.dic_sequence_services['list_sequence_services']

            dictionary_string = json.dumps(dictionary)

            print(dictionary_string)

            # read mode by default
            data_file = open(self.data_file_path, 'r+') 
            list_sequence_services = data_file.read()
            list_sequence_services = json.loads(list_sequence_services)

            list_sequence_services.append(dictionary_string)
            list_sequence_services = json.dumps(list_sequence_services)

            data_file.close()
            data_file = open(self.data_file_path, 'w+')
            data_file.write(list_sequence_services)
            data_file.close()

            self.reset_sequence_missions()
            
            return

        if self._widget.new_sequence.text() == "New":
            return


    def print_available_sequence_of_missions(self):

        data_file = open(self.data_file_path, 'r+') 
        list_sequence_services = data_file.read()
        list_sequence_services = json.loads(list_sequence_services)
        data_file.close()

        self._widget.mission_sequence_description.clear()

        self._widget.available_mission_sequences.clear()
        for sequence in list_sequence_services:
            sequence = json.loads(sequence)
            self._widget.available_mission_sequences.addItem(sequence['name'])

        return

    def send_list_of_services_2(self,service_sequence_str):
        # # debug
        # for service in self.dic_sequence_services['list_sequence_services']:
        #     print(service)
        #     print('\n\n')

        # request service
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'ServiceSequencer',1.0)
            
            try:
                request = rospy.ServiceProxy(self.namespace+'ServiceSequencer', ServiceSequence)

                reply = request(service_sequence = service_sequence_str)

                if reply.received == True:
                    # if controller receives message
                    print('Success')

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
            
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))


    def send_list_of_services(self):
        # # debug
        # for service in self.dic_sequence_services['list_sequence_services']:
        #     print(service)
        #     print('\n\n')

        # request service
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'ServiceSequencer',1.0)
            
            try:
                request = rospy.ServiceProxy(self.namespace+'ServiceSequencer', ServiceSequence)

                service_sequence = json.dumps(self.dic_sequence_services['list_sequence_services'])
                reply = request(service_sequence = service_sequence)

                if reply.received == True:
                    # if controller receives message
                    print('Success')

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
            
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))

    def update_last_trigger_instant(self):

        index = self._widget.tabWidget.currentIndex()

        tab_name = self._widget.tabWidget.tabText(index)
        
        time_instant = self.dic_sequence_services['last_trigger_time']


        if tab_name == "Mission":
            self._widget.TriggerInstant.setValue(time_instant)

        if tab_name == "Reset":
            self._widget.TriggerInstantNeutral.setValue(time_instant)
        
        for inner_key in self.mission_inner_keys:
            if tab_name == inner_key:
                getattr(self,inner_key)._widget.TriggerInstant.setValue(time_instant)

        # if tab_name == "Controller":
        #     self.choose_controller._widget.TriggerInstant.setValue(time_instant)

        # if tab_name == "Reference":
        #     self.choose_reference._widget.TriggerInstant.setValue(time_instant)

        # if tab_name == "YawController":
        #     self.choose_yaw_controller._widget.TriggerInstant.setValue(time_instant)

        # if tab_name == "YawReference":
        #     self.choose_yaw_reference._widget.TriggerInstant.setValue(time_instant)

    def __print_controller_item_clicked(self):

        if self.__head_class_set == False:
            
            key_class_name_selected = self._widget.ListControllersWidget.currentItem().text()
            ClassSelected           = missions_database.database[key_class_name_selected]

            # message to be printed
            string = ClassSelected.description()

            # print message on GUI
            self._widget.ItemClickedMessage.setText(string)
        else:
            key_class_name_selected = self._widget.ListControllersWidget.currentItem().text()
            ClassSelected           = self.dic_to_print[key_class_name_selected]

            # message to be printed
            string = ClassSelected.description() 

            # print message on GUI
            self._widget.ItemClickedMessage.setText(string)

        pass            

    def __reset_controllers_widget(self):
        """ Clear widget with missions list and print it again """

        # clear items from widget
        self._widget.ListControllersWidget.clear()

        # create list of available controller classes based on **imported dictionary**
        for key in missions_database.database.keys():
            # print all elements in dictionary
            self._widget.ListControllersWidget.addItem(key)

        self.__head_class_set       = False
        self.dic_to_print           = {}
        self.__head_class_completed = False

        self.mission_inner_keys     = []

        self.__remove_tabs()

    def __remove_tabs(self):

        for index in range(self._widget.tabWidget.count()-3):
            # every time I remove a tab, I get a new tabWidget
            # with one less tab; hence, I remove tab with index 1  
            # until I get tabWidget, with just one tab
            # tab_name = self._widget.tabWidget.tabText(3)
            self._widget.tabWidget.removeTab(3)
            # inner_key = tab_name (see __add_tab())
            # delattr(self,tab_name)

    def __add_tab(self):

        for inner_key in self.__HeadClass.inner.keys():

            if not hasattr(self,inner_key):
                dictionary = {}
                dictionary["context"]  = self.context
                dictionary["name_tab"] = inner_key
                dictionary["dictionary_of_options"] = {}
                dictionary["service_name"] = "ServiceMissionChangeInner"
                dictionary["ServiceClass"] = SrvChangeJsonableObjectByStr

                setattr(self,inner_key,ChooseJsonablePlugin(**dictionary))
                setattr(getattr(self,inner_key),"dic_sequence_services",self.dic_sequence_services)

            getattr(self,inner_key).change_dictionary_of_options(self.__HeadClass.inner[inner_key])
            self._widget.tabWidget.addTab(getattr(self,inner_key)._widget,inner_key)
            self.mission_inner_keys.append(inner_key)

            # if 'controller' in self.__HeadClass.inner.keys():
            #     self.choose_controller.change_dictionary_of_options(self.__HeadClass.inner['controller'])
            #     self._widget.tabWidget.addTab(self.choose_controller._widget,'Controller')

            # if 'reference' in self.__HeadClass.inner.keys():  
            #     self.choose_reference.change_dictionary_of_options(self.__HeadClass.inner['reference'])
            #     self._widget.tabWidget.addTab(self.choose_reference._widget,'Reference')

            # if 'yaw_controller' in self.__HeadClass.inner.keys():
            #     self.choose_yaw_controller.change_dictionary_of_options(self.__HeadClass.inner['yaw_controller'])
            #     self._widget.tabWidget.addTab(self.choose_yaw_controller._widget,'YawController')

            # if 'yaw_reference' in self.__HeadClass.inner.keys():
            #     self.choose_yaw_reference.change_dictionary_of_options(self.__HeadClass.inner['yaw_reference'])
            #     self._widget.tabWidget.addTab(self.choose_yaw_reference._widget,'YawReference')            


    def __controller_item_clicked(self):

        if self.__head_class_set == False:

            self.__head_class_set = True
            self.__head_class_key = self._widget.ListControllersWidget.currentItem().text()
            # notice this is a class, not an object
            self.__HeadClass      = missions_database.database[self.__head_class_key]

            head_class_input_dic = {}
            for key in self.__HeadClass.inner.keys():
                head_class_input_dic[key] = []
            self.__head_class_input_dic = head_class_input_dic

            if jsonable.check_completeness(self.__head_class_input_dic):
                self.__print_controller_message()
            else:
                self.__list_keys = self.__head_class_input_dic.keys()            
                self.print_new_list()

        else:   
            
            chosen_class = self._widget.ListControllersWidget.currentItem().text()
            ChosenClass  = self.dic_to_print[chosen_class]
            # chosen_class_list_of_keys = ChosenClass.inner.keys()

            nested_dictionary = {}
            for key in ChosenClass.inner.keys():
                nested_dictionary[key] = []

            list_of_keys_appended = self.__list_keys[0]

            input_dictionary  = self.__head_class_input_dic
            list_keys         = list_of_keys_appended.split(':')
            
            #rospy.logwarn(self.__head_class_input_dic)
            jsonable.update_input_dictionary(input_dictionary,list_keys,chosen_class,nested_dictionary)
            self.__head_class_input_dic = input_dictionary

            if jsonable.check_completeness(self.__head_class_input_dic):
                self.__print_controller_message()
            else:
                list_of_keys_appended = self.__list_keys[0]
                self.__list_keys      = self.__list_keys[1:]
                
                for chosen_class_key in ChosenClass.inner.keys():
                    self.__list_keys.insert(0,list_of_keys_appended+':'+chosen_class_key)
                
                self.print_new_list()

    def print_new_list(self):

        input_dictionary  = self.__head_class_input_dic
        list_of_keys      = self.__list_keys[0]
        list_of_keys      = list_of_keys.split(':')
        self.dic_to_print = self.__HeadClass.get_dic_recursive(input_dictionary,list_of_keys)

        #rospy.logwarn(self.dic_to_print)

        # clear items from widget
        self._widget.ListControllersWidget.clear()
        for key in self.dic_to_print.keys():
            # add item
            self._widget.ListControllersWidget.addItem(key)

    def __print_controller_message(self):
        """Print message with parameters associated to chosen controller class"""

        self.__head_class_completed = True

        # rospy.logwarn(self.__head_class_input_dic)

        string = self.__HeadClass.to_string(self.__head_class_input_dic)
        # print message on GUI
        self._widget.ControllerMessageInput.setPlainText(string)

        # clear items from widget
        self._widget.ListControllersWidget.clear()

        # print message on GUI
        #self._widget.ItemClickedMessage.setText('Selected Mission: '+self.__HeadClass.description())
        self._widget.ItemClickedMessage.setText('<p>Selected Mission:</p>'+self.__HeadClass.combined_description(self.__head_class_input_dic))

        return 

    def __get_new_controller_parameters(self):
        """Request service for new controller with parameters chosen by user"""


        if self.__head_class_completed == True:
        
            if self.dic_sequence_services['checked_sequence_of_missions'] == True:

                # get string that user modified with new parameters
                string              = self._widget.ControllerMessageInput.toPlainText()
                # get new parameters from string
                parameters          = string


                new_service = {}

                trigger_instant = self._widget.TriggerInstant.value()
                new_service['trigger_instant'] = trigger_instant

                # we are changing the last_trigger_time for all objects that share dictionary
                self.dic_sequence_services['last_trigger_time'] = trigger_instant

                new_service['service_name']    = 'ServiceChangeMission'
                new_service['inputs_service']  = {'jsonable_name':self.__head_class_key, 'string_parameters': parameters}
                self.dic_sequence_services['list_sequence_services'].append(new_service)

                self.__add_tab()


            else:

                # get string that user modified with new parameters
                string              = self._widget.ControllerMessageInput.toPlainText()
                # get new parameters from string
                parameters          = string

                # rospy.logwarn(parameters)

                # request service
                try: 
                    # time out of one second for waiting for service
                    rospy.wait_for_service("/"+self.namespace+'ServiceChangeMission',2.0)
                    
                    try:
                        RequestingMission = rospy.ServiceProxy("/"+self.namespace+'ServiceChangeMission', SrvCreateJsonableObjectByStr)
                        reply = RequestingMission(jsonable_name = self.__head_class_key, string_parameters = parameters)

                        if reply.received == True:
                            # if controller receives message
                            self._widget.Success.setChecked(True) 
                            self._widget.Failure.setChecked(False) 


                    except rospy.ServiceException as exc:
                        rospy.logwarn("Service did not process request: " + str(exc))
                        rospy.logwarn('Proxy for service that sets mission FAILED')
                        self._widget.Success.setChecked(False) 
                        self._widget.Failure.setChecked(True) 
                        # print "Service call failed: %s"%e
                    
                except rospy.ServiceException as exc:
                    rospy.logwarn("Service did not process request: " + str(exc))
                    rospy.logwarn('Timeout for service that sets mission')
                    self._widget.Success.setChecked(False) 
                    self._widget.Failure.setChecked(True) 
                    # print "Service not available ..."        
                    pass
        else:
            # print message on GUI
            self._widget.ItemClickedMessage.setText('<b>Service cannot be completed: finish choosing</b>')    

        pass


    def reset_iris_neutral_value(self):

        if self.dic_sequence_services['checked_sequence_of_missions'] == True:

                new_service = {}
                trigger_instant = self._widget.TriggerInstantNeutral.value()
                new_service['trigger_instant'] = trigger_instant
                self.dic_sequence_services['last_trigger_time'] = trigger_instant

                new_service['service_name']    = 'IrisPlusResetNeutral'
                new_service['inputs_service']  = {}
                self.dic_sequence_services['list_sequence_services'].append(new_service)

        else:
            try: 
                # time out of one second for waiting for service
                rospy.wait_for_service(self.namespace+'IrisPlusResetNeutral',1.0)
                try:
                    # request reseting neutral value for iris+ (implicit, with keywords)
                    ResetingNeutral = rospy.ServiceProxy(self.namespace+'IrisPlusResetNeutral', IrisPlusResetNeutral)
                    reply = ResetingNeutral()
                    if reply.received == True:
                        self._widget.ThrottleNeutralValue.setValue(reply.k_trottle_neutral)
                        rospy.logwarn('Service for reseting neutral value succeded')
                except rospy.ServiceException as exc:
                    rospy.logwarn("Service did not process request: " + str(exc))
                    rospy.logwarn('Service for reseting neutral value failed')
                    pass
            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
                rospy.logwarn('Service for reseting neutral value failed')      
                pass


    def set_iris_neutral_value(self):
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'IrisPlusSetNeutral',1.0)
            try:
                # request reseting neutral value for iris+ (implicit, with keywords)
                ResetingNeutral = rospy.ServiceProxy(self.namespace+'IrisPlusSetNeutral', IrisPlusSetNeutral)
                reply = ResetingNeutral(k_trottle_neutral = self._widget.ThrottleNeutralValue.value())
                if reply.received == True:
                    rospy.logwarn('Service for seting neutral value succeded')
            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
                rospy.logwarn('Service for seting neutral value failed')
                pass
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))
            rospy.logwarn('Service for seting neutral value failed')      
            pass            

    def _parse_args(self, argv):

        parser = argparse.ArgumentParser(prog='saver', add_help=False)

        # args = parser.parse_args(argv)

        if argv:
            namespace = argv[0]
            return namespace            
        else:
            # is argv is empty return empty string
            return ""
    
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
