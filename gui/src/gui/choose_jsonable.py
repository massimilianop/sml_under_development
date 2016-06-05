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

import json

import inspect

class ChooseJsonablePlugin(Plugin):

    def __init__(self, context,namespace = None, name_tab = "", dictionary_of_options = {}, service_name = "", ServiceClass = None, sequence_tabs = []):

        self.context = context

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())

        # DO NOT COPY DICTIONARY
        self.dictionary_of_options = dictionary_of_options

        self.name_tab      = name_tab
        self.sequence_tabs = sequence_tabs
        
        self.service_name  = service_name
        self.ServiceClass  = ServiceClass

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
        self._widget.SetJsonableButton.setEnabled(False)


        # button for resetting list of available jsonable objects
        self._widget.ResetListJsonable.clicked.connect(self.__reset_jsonable_widget)

        self.__reset_jsonable_widget()

        # description window to display html formatted text
        self._widget.JsonableDescription.setAcceptRichText(True)

        self._widget.tabWidget.setTabText(0,self.name_tab)

        self._widget.tabWidget.currentChanged.connect(self.update_last_trigger_instant)

        
        self._widget.radioButtonMethods.clicked.connect(self.print_available_methods) 
        self._widget.radioButtonMethods.setEnabled(False)   

    def print_available_methods(self):

        if self._widget.radioButtonMethods.isChecked():

            # get string that user modified with new parameters
            string              = self._widget.JsonableMessageInput.toPlainText()
            
            # Class selected by user
            ClassSelected = self.dictionary_of_options[self.__head_class_key]
            # construct object as specified by user
            object_selected = ClassSelected.from_string(string)
            
            # clear items from widget
            self._widget.ListJsonableWidget.clear()
            if hasattr(object_selected,"methods_list"):
                for function_name in object_selected.methods_list:
                    # add item
                    self._widget.ListJsonableWidget.addItem(function_name)
            else:
                print("IMPLEMENT")
        else:
            self.__print_jsonable_message()


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

        self.jsonable_inner_keys     = []

        self.__remove_tabs()
        self._widget.radioButtonMethods.setEnabled(False)
        self._widget.radioButtonMethods.setChecked(False)        
        self._widget.SetJsonableButton.setEnabled(False)

    def __remove_tabs(self):

        number_of_tabs_to_preserve = 1

        for index in range(self._widget.tabWidget.count()-number_of_tabs_to_preserve):
            # every time I remove a tab, I get a new tabWidget
            # with one less tab; hence, I remove tab with index 1
            # until I get tabWidget, with just one tab
            # tab_name = self._widget.tabWidget.tabText(3)
            self._widget.tabWidget.removeTab(number_of_tabs_to_preserve)
            # inner_key = tab_name (see __add_tab())
            # delattr(self,tab_name)        

    def __add_tab(self):

        for inner_key in self.__HeadClass.inner.keys():

            if not hasattr(self,inner_key):
                dictionary = {}
                dictionary["context"]  = self.context
                dictionary["name_tab"] = inner_key
                dictionary["dictionary_of_options"] = {}
                dictionary["service_name"]  = "ServiceChangeMission"
                dictionary["ServiceClass"]  = SrvChangeJsonableObjectByStr
                dictionary["sequence_tabs"] = self.sequence_tabs + [inner_key]

                setattr(self,inner_key,ChooseJsonablePlugin(**dictionary))
                setattr(getattr(self,inner_key),"dic_sequence_services",self.dic_sequence_services)

            getattr(self,inner_key).change_dictionary_of_options(self.__HeadClass.inner[inner_key])
            self._widget.tabWidget.addTab(getattr(self,inner_key)._widget,inner_key)
            self.jsonable_inner_keys.append(inner_key)

    def __print_list(self):
        # clearwindow
        self._widget.ListJsonableWidget.clear()
        # create list of available classes based on dictionary 
        for key in self.dictionary_of_options.keys():
            self._widget.ListJsonableWidget.addItem(key)

    def __print_jsonable_description(self):

        if not self._widget.radioButtonMethods.isChecked():

            if self.__head_class_set == False:
                
                key_class_name_selected = self._widget.ListJsonableWidget.currentItem().text()
                ClassSelected           = self.dictionary_of_options[key_class_name_selected]

                # message to be printed
                string = ClassSelected.description()

                # print message on GUI
                self._widget.JsonableDescription.setText(string)
            else:
                key_class_name_selected = self._widget.ListJsonableWidget.currentItem().text()
                ClassSelected           = self.dic_to_print[key_class_name_selected]

                # message to be printed
                string = ClassSelected.description() 

                # print message on GUI
                self._widget.JsonableDescription.setText(string)

            return

        else:
            func_name = self._widget.ListJsonableWidget.currentItem().text()
            self.func_name_selected = func_name

            # function description taken from SelectedClass function
            func_description = getattr(self.__HeadClass,func_name).__doc__
            self._widget.JsonableDescription.setText(func_description)

            func_args = inspect.getargspec(getattr(self.__HeadClass,func_name))
            if func_args.defaults:
                input_dictionary = dict(zip(func_args.args[-len(func_args.defaults):],func_args.defaults))
                input_dictionary = json.dumps(input_dictionary)
                print(input_dictionary)
            else:
                input_dictionary = json.dumps(dict())

            self._widget.JsonableMessageInput.setPlainText(input_dictionary)



    def update_last_trigger_instant(self):

        index = self._widget.tabWidget.currentIndex()
        print("Tab index:"+str(index)+"\n")

        tab_name = self._widget.tabWidget.tabText(index)
        
        time_instant = self.dic_sequence_services['last_trigger_time']
        
        if tab_name == self.name_tab:
            self._widget.TriggerInstant.setValue(time_instant)
            return

        if tab_name in self.jsonable_inner_keys:
            getattr(self,tab_name)._widget.TriggerInstant.setValue(time_instant)
            getattr(self,tab_name).update_last_trigger_instant()
        
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
        #self._widget.JsonableDescription.setText('Selected ' + self.name_tab + ' : ' + self.__HeadClass.description()) 
        # TODO    
        self._widget.JsonableDescription.setText('<p>Selected Mission:</p>'+self.__HeadClass.combined_description(self.__head_class_input_dic))       

        # make it possible to Set chosen class
        self._widget.SetJsonableButton.setEnabled(True)
        #cannot make methods available yet since, class hasnt been chosen yet    
        #self._widget.radioButtonMethods.setEnabled(True) 

        return         

    def __service_jsonable_object(self):
        """Request service for new jsonable object with parameters chosen by user"""

        if self.__head_class_completed == True:

            if not self._widget.radioButtonMethods.isChecked():

                # get string that user modified with new parameters
                string              = self._widget.JsonableMessageInput.toPlainText()
                # get new parameters from string
                parameters          = string

                # new_service = {'trigger_instant':...,"service_name":...,"","input_service":...}
                new_service = {}

                trigger_instant = self._widget.TriggerInstant.value()
                new_service['trigger_instant'] = trigger_instant

                # we are changing the last_trigger_time for all objects that share dictionary
                self.dic_sequence_services['last_trigger_time'] = trigger_instant

                new_service['service_name']    = self.service_name
                # new_service['inputs_service']  = {'jsonable_name':self.__head_class_key, 'string_parameters': parameters}
                # input_service = {"inner_key":self.name_tab,"key":self.__head_class_key,"input_string":parameters}
                input_service = {"sequence_inner_key":self.sequence_tabs,"key":self.__head_class_key,"input_string":parameters}
                input_service = json.dumps(input_service) 
                print(input_service)
                new_service['inputs_service']  = {'dictionary':input_service}

                self.dic_sequence_services['list_sequence_services'].append(new_service)

                self.__add_tab()
                self._widget.radioButtonMethods.setEnabled(True) 
            else:

                # get string that user modified with new parameters
                func_string_input = self._widget.JsonableMessageInput.toPlainText()
                func_input        = json.loads(func_string_input)

                # new_service = {'trigger_instant':...,"service_name":...,"","input_service":...}
                new_service = {}

                trigger_instant = self._widget.TriggerInstant.value()
                new_service['trigger_instant'] = trigger_instant

                # we are changing the last_trigger_time for all objects that share dictionary
                self.dic_sequence_services['last_trigger_time'] = trigger_instant

                new_service['service_name']    = self.service_name+"CallMethod"
                # new_service['inputs_service']  = {'jsonable_name':self.__head_class_key, 'string_parameters': parameters}
                # dictionary = '{"sequence_inner_key":"","func_name":"","input_string":""}'
                input_service = {"sequence_inner_key":self.sequence_tabs,"func_name":self.func_name_selected,"input_to_func":func_input}
                input_service = json.dumps(input_service) 
                print(input_service)
                new_service['inputs_service']  = {'dictionary':input_service}

                self.dic_sequence_services['list_sequence_services'].append(new_service)                    

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
