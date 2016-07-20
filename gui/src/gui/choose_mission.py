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

# # no need to get quad_control path, since it is package; import missions dictionary
# from src.missions import missions_database
# # from src.missions import type_uav_mission
# DICTIONARY_OF_OPTIONS = missions_database.database2
# # DICTIONARY_OF_OPTIONS = {"Mission":type_uav_mission.MissionGeneral}

# no need to get quad_control path, since it is package; import missions dictionary
from src.type_uav import type_uav
# from src.missions import type_uav_mission
DICTIONARY_OF_OPTIONS = type_uav.database
# DICTIONARY_OF_OPTIONS = {"Mission":type_uav_mission.MissionGeneral}



from src.utilities import jsonable

from choose_jsonable import ChooseJsonablePlugin

import json

EXAMPLE_DICTIONARY = {}
EXAMPLE_DICTIONARY["name_main_tab"] = "type_uav"
EXAMPLE_DICTIONARY["strServiceChangeName"] = "ServiceChangeMission"
EXAMPLE_DICTIONARY["DICTIONARY_OF_OPTIONS"] = DICTIONARY_OF_OPTIONS
EXAMPLE_DICTIONARY["name_service_sequence_provider"] = 'ServiceSequencer'

class ChooseMissionPlugin(Plugin):

    def __init__(self, context,namespace = None, dictionary_options = EXAMPLE_DICTIONARY):

        for (key,item) in dictionary_options.items():
            setattr(self,key,item)

        # "global variables" in dictionary
        self.dic_sequence_services = {}
        self.dic_sequence_services['last_trigger_time']            = 0.0
        self.dic_sequence_services['list_sequence_services']       = []
        self.dic_sequence_services['set_jsonable_extra_callback']  = self.update_detailed_description

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
        # TODO: I commented this out
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns
        
               
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
        
        # commment this, otherwise form is included when gui is opened
        # Add widget to the user interface
        #context.add_widget(self._widget)

        # ---------------------------------------------- #
        # ---------------------------------------------- #

        self._widget.SetSequenceMissions.clicked.connect(self.send_list_of_services)
        self._widget.pushButtonClearSequenceMissions.clicked.connect(self.clear_list_of_services)
        # ---------------------------------------------- #
        
        # determine ROS workspace directory where data is saved
        package_path = rospkg.RosPack().get_path('quad_control')
        self.data_file_path = package_path+'/src/type_uav/sequence_missions.txt' 

        self._widget.save_new_sequence.clicked.connect(self.save_new_sequence)
        self._widget.new_sequence.clicked.connect(self.new_sequence)


        # if item in list is clicked twice selected, print corresponding message
        self._widget.available_mission_sequences.itemDoubleClicked.connect(self.select_chosen_sequence)

        # if item in list is clicked once, print corresponding message
        self._widget.available_mission_sequences.itemClicked.connect(self.print_sequence_mission_description)


        self.reset_sequence_missions()


        name_tab   = self.name_main_tab
        dictionary = {}
        dictionary["context"]  = self.context
        dictionary["name_tab"] = name_tab
        # dictionary["dictionary_of_options"] = missions_database.database
        dictionary["dictionary_of_options"] = self.DICTIONARY_OF_OPTIONS
        # dictionary["service_name"]  = "ServiceChangeMission"
        dictionary["service_name"]  = self.strServiceChangeName
        dictionary["ServiceClass"]  = SrvChangeJsonableObjectByStr
        dictionary["sequence_tabs"] = []

        setattr(self,name_tab,ChooseJsonablePlugin(**dictionary))
        setattr(getattr(self,name_tab),"dic_sequence_services",self.dic_sequence_services)

        #getattr(self,name_tab).change_dictionary_of_options(self.__HeadClass.inner[inner_key])
        self._widget.tabWidget.addTab(getattr(self,name_tab)._widget,name_tab)
        #self.mission_inner_keys.append(inner_key)

    def select_chosen_sequence(self):

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
                item_selected        = item
                sequence_description = item_selected["description"]
                sequence_description = json.dumps(sequence_description)
                self._widget.mission_sequence_description.setText(sequence_description)


        self.dic_sequence_services['list_sequence_services'] = item_selected["sequence_of_missions"]
        last_service = item_selected["sequence_of_missions"][-1]
        self.dic_sequence_services['last_trigger_time']      = last_service["trigger_instant"]

        # TODO: this does not update trigger if tabs inside main tab are open
        # update last trigger_instant in main_tab
        getattr(self,self.name_main_tab)._widget.TriggerInstant.setValue(last_service["trigger_instant"])

        self.update_detailed_description()        

    def update_detailed_description(self):

        # print detailed description
        self._widget.textEdit_detailed_description.clear()
        detailed_description = ""
        for item in self.dic_sequence_services['list_sequence_services']:
            detailed_description += str(item['trigger_instant'])+": "
            dictionary = item['inputs_service']['dictionary']
            dictionary = json.loads(dictionary)
            if 'key' in dictionary.keys():
                detailed_description += dictionary['key'] +'\n'
            if 'func_name' in dictionary.keys():
                detailed_description += dictionary['func_name'] +'\n'

        self._widget.textEdit_detailed_description.setText(detailed_description)


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

            # TODO: see if we want to print this
            # print(dictionary_string)

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
            rospy.wait_for_service(self.namespace+self.name_service_sequence_provider,1.0)
            
            try:
                request = rospy.ServiceProxy(self.namespace+self.name_service_sequence_provider, ServiceSequence)

                reply = request(service_sequence = service_sequence_str)

                if reply.received == True:
                    # if controller receives message
                    # print('Success')
                    pass

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
            
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))


    def clear_list_of_services(self):
        self.dic_sequence_services['list_sequence_services'] = []
        self.dic_sequence_services['last_trigger_time'] = 0

        self._widget.textEdit_detailed_description.clear()

    def send_list_of_services(self):
        # # debug
        # for service in self.dic_sequence_services['list_sequence_services']:
        #     print(service)
        #     print('\n\n')

        # request service
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+self.name_service_sequence_provider,1.0)
            
            try:
                request = rospy.ServiceProxy(self.namespace+self.name_service_sequence_provider, ServiceSequence)

                service_sequence = json.dumps(self.dic_sequence_services['list_sequence_services'])
                reply = request(service_sequence = service_sequence)

                if reply.received == True:
                    # if controller receives message
                    # print('Success')
                    pass

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
            
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))

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
