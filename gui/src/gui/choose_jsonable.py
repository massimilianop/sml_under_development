import os, rospy

import QtGui, PyQt4.QtCore

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal

# import services defined in quad_control
from quad_control.srv import *

import argparse


import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for rospy_tutorials
import sys
sys.path.insert(0, rospack.get_path('quad_control'))
# no need to get quad_control path, since it is package; import controllers dictionary
from src.utilities import jsonable

class ChooseJsonablePlugin(Plugin):

    def __init__(self, context,namespace = None, name_tab = "", dictionary_of_options = {}, service_name = "", ServiceClass = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())

        # DO NOT COPY DICTIONARY
        self.dictionary_of_options = dictionary_of_options

        self.name_tab     = name_tab
        self.service_name = service_name
        self.ServiceClass = ServiceClass

        super(ChooseJsonablePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChooseJsonablePlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'choose_jsonable.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Choose'+self.name_tab+'Ui')
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

        # create list of available trajectory classes based on dictionary 
        self.__print_list()

        # if item in list is clicked twice selected, print corresponding message
        self._widget.ListJsonableWidget.itemDoubleClicked.connect(self.__jsonable_item_clicked)

        # if item in list is clicked once, print corresponding message
        self._widget.ListJsonableWidget.itemClicked.connect(self.__print_jsonable_description)
        
        # button to request service for setting new jsonable object
        self._widget.SetJsonableButton.clicked.connect(self.__service_jsonable_object)


        # button for resetting list of available jsonable objects
        self._widget.ResetListJsonable.clicked.connect(self.__reset_jsonable_widget)

        self.__reset_jsonable_widget()

    # PUBLIC FUNCTION:
    def change_dictionary_of_options(self,dictionary_of_options):
        self.dictionary_of_options = dictionary_of_options
        self.__reset_jsonable_widget()

    def __reset_jsonable_widget(self):
        """ Clear widget and print it again """
        
        # clear items from widget
        self._widget.ListJsonableWidget.clear()

        # create list of available controller classes based on **imported dictionary**
        for key in self.dictionary_of_options.keys():
            # print all elements in dictionary
            self._widget.ListJsonableWidget.addItem(key)

        self.__head_class_set       = False
        self.__head_class_completed = False
        self.dic_to_print           = {}

    def __print_list(self):
        # clearwindow
        self._widget.ListJsonableWidget.clear()
        # create list of available classes based on dictionary 
        for key in self.dictionary_of_options.keys():
            self._widget.ListJsonableWidget.addItem(key)

    def __print_jsonable_description(self):

        if self.__head_class_set == False:
            
            key_class_name_selected = self._widget.ListJsonableWidget.currentItem().text()
            ClassSelected           = self.dictionary_of_options[key_class_name_selected]

            # message to be printed
            string = ClassSelected.description()

            # print message on GUI
            self._widget.JsonableDescription.setPlainText(string)
        else:
            key_class_name_selected = self._widget.ListJsonableWidget.currentItem().text()
            ClassSelected           = self.dic_to_print[key_class_name_selected]

            # message to be printed
            string = ClassSelected.description() 

            # print message on GUI
            self._widget.JsonableDescription.setPlainText(string)

        pass 
        
    def __jsonable_item_clicked(self):

        if self.__head_class_set == False:

            self.__head_class_set = True
            self.__head_class_key = self._widget.ListJsonableWidget.currentItem().text()
            self.__HeadClass      = self.dictionary_of_options[self.__head_class_key]
            
            head_class_input_dic = {}
            for key in self.__HeadClass.inner.keys():
                head_class_input_dic[key] = []
            self.__head_class_input_dic = head_class_input_dic

            if jsonable.check_completeness(self.__head_class_input_dic):
                self.__print_jsonable_message()
            else:
                self.__list_keys = self.__head_class_input_dic.keys()
                self.print_new_list()

        else:
            chosen_class = self._widget.ListJsonableWidget.currentItem().text()
            ChosenClass  = self.dic_to_print[chosen_class]
            # chosen_class_list_of_keys = ChosenClass.inner.keys()

            nested_dictionary = {}
            for key in ChosenClass.inner.keys():
                nested_dictionary[key] = []

            list_of_keys_appended = self.__list_keys[0]

            input_dictionary  = self.__head_class_input_dic
            list_keys         = list_of_keys_appended.split(':')
            
            rospy.logwarn(self.__head_class_input_dic)
            jsonable.update_input_dictionary(input_dictionary,list_keys,chosen_class,nested_dictionary)
            self.__head_class_input_dic = input_dictionary

            if jsonable.check_completeness(self.__head_class_input_dic):
                self.__print_jsonable_message()
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

        # clear items from widget
        self._widget.ListJsonableWidget.clear()
        for key in self.dic_to_print.keys():
            # add item
            self._widget.ListJsonableWidget.addItem(key)

    def __print_jsonable_message(self):
        """Print message with parameters associated to chosen controller class"""

        # Service for creating jsonable object is now possible
        self.__head_class_completed = True

        string = self.__HeadClass.to_string(self.__head_class_input_dic)
        # print message on GUI
        self._widget.JsonableMessageInput.setPlainText(string)

        # self.__reset_jsonable_widget()

        # clear items from widget
        self._widget.ListJsonableWidget.clear()

        # print message on GUI
        self._widget.JsonableDescription.setPlainText('Selected ' + self.name_tab + ' : ' + self.__HeadClass.description())        

        return 

    def __service_jsonable_object(self):
        """Request service for new jsonable object with parameters chosen by user"""

        if self.__head_class_completed == True:

            if self.dic_sequence_services['checked_sequence_of_missions'] == True:

                # get string that user modified with new parameters
                string              = self._widget.JsonableMessageInput.toPlainText()
                # get new parameters from string
                parameters          = string

                new_service = {}

                trigger_instant = self._widget.TriggerInstant.value()
                new_service['trigger_instant'] = trigger_instant

                # we are changing the last_trigger_time for all objects that share dictionary 
                self.dic_sequence_services['last_trigger_time'] = trigger_instant

                new_service['service_name']    = self.service_name
                new_service['inputs_service']  = {'jsonable_name':self.__head_class_key, 'string_parameters': parameters}
                self.dic_sequence_services['list_sequence_services'].append(new_service)

            else:
                # get string that user modified with new parameters
                string              = self._widget.JsonableMessageInput.toPlainText()
                # get new parameters from string
                parameters          = string

                # request service
                try: 
                    # time out of one second for waiting for service
                    rospy.wait_for_service("/"+self.namespace+self.service_name,2.0)
                    
                    try:
                        Requesting = rospy.ServiceProxy("/"+self.namespace+self.service_name, self.ServiceClass)

                        reply = Requesting(jsonable_name = self.__head_class_key, string_parameters = parameters)

                        if reply.received == True:
                            # if controller receives message
                            self._widget.Success.setChecked(True) 
                            self._widget.Failure.setChecked(False) 


                    except rospy.ServiceException as exc:
                        rospy.logwarn("Service did not process request: " + str(exc))
                        rospy.logwarn('Proxy for service that sets controller FAILED')
                        self._widget.Success.setChecked(False) 
                        self._widget.Failure.setChecked(True) 
                        # print "Service call failed: %s"%e
                    
                except rospy.ServiceException as exc:
                    rospy.logwarn("Service did not process request: " + str(exc))
                    rospy.logwarn('Timeout for service that sets controller')
                    self._widget.Success.setChecked(False) 
                    self._widget.Failure.setChecked(True) 
                    # print "Service not available ..."        
                    pass
        else:
            # print message on GUI
            self._widget.JsonableDescription.setText('<b>Service cannot be completed: finish choosing </b>')                         

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
