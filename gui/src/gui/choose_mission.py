import os, rospy

import QtGui, PyQt4.QtCore

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal

# import services defined in quad_control; service being used: SrvCreateJsonableObjectByStr
from quad_control.srv import SrvCreateJsonableObjectByStr

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

class ChooseMissionPlugin(Plugin):

    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


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

        # ---------------------------------------------- #
        
        # button to request service for setting new controller, with new parameters
        self._widget.SetControllerButton.clicked.connect(self.__get_new_controller_parameters)

        # if item in list is selected, print corresponding message
        self._widget.ListControllersWidget.itemDoubleClicked.connect(self.__controller_item_clicked)
        self._widget.ListControllersWidget.itemClicked.connect(self.__print_controller_item_clicked)

        self.__reset_controllers_widget()

        # button for resetting list of available controllers
        self._widget.ResetControllersList.clicked.connect(self.__reset_controllers_widget)

    def __print_controller_item_clicked(self):

        if self.__head_class_set == False:
            
            key_class_name_selected = self._widget.ListControllersWidget.currentItem().text()
            ClassSelected           = missions_database.database[key_class_name_selected]

            # message to be printed
            string = ClassSelected.description() 

            # print message on GUI
            self._widget.ItemClickedMessage.setPlainText(string)
        else:
            key_class_name_selected = self._widget.ListControllersWidget.currentItem().text()
            ClassSelected           = self.dic_to_print[key_class_name_selected]

            # message to be printed
            string = ClassSelected.description() 

            # print message on GUI
            self._widget.ItemClickedMessage.setPlainText(string)

        pass            


    def __reset_controllers_widget(self):
        """ Clear widget with controllers list and print it again """
        
        # clear items from widget
        self._widget.ListControllersWidget.clear()

        # create list of available controller classes based on **imported dictionary**
        for key in missions_database.database.keys():
            # print all elements in dictionary
            self._widget.ListControllersWidget.addItem(key)

        self.__head_class_set = False
        
    def __controller_item_clicked(self):

        if self.__head_class_set == False:

            self.__head_class_set = True
            self.__head_class_key = self._widget.ListControllersWidget.currentItem().text()
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

        rospy.logwarn(self.__head_class_input_dic)

        string = self.__HeadClass.to_string(self.__head_class_input_dic)
        # print message on GUI
        self._widget.ControllerMessageInput.setPlainText(string)

        self.__reset_controllers_widget()

        return 

    def __get_new_controller_parameters(self):
        """Request service for new controller with parameters chosen by user"""

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

        return


    def reset_iris_neutral_value(self):
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
            except rospy.ServiceException:
                rospy.logwarn('Service for reseting neutral value failed')
                pass
        except:
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
            except rospy.ServiceException:
                rospy.logwarn('Service for seting neutral value failed')
                pass
        except:
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
