import os, rospy

import QtGui, PyQt4.QtCore

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal

# import services defined in quad_control; service being used: SrvControllerChangeByStr
from quad_control.srv import SrvControllerChangeByStr

import argparse


import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for rospy_tutorials
import sys
sys.path.insert(0, rospack.get_path('quad_control'))
# no need to get quad_control path, since it is package; import controllers dictionary
from src.controllers_hierarchical.fully_actuated_controllers import controllers_dictionary

class ChooseControllerPlugin(Plugin):

    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(ChooseControllerPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChooseControllerPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ChooseController.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ChooseControllerUi')
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
        self._widget.ListControllersWidget.itemClicked.connect(self.__controller_item_clicked)

        self.__reset_controllers_widget()

        # button for resetting list of available controllers
        self._widget.ResetControllersList.clicked.connect(self.__reset_controllers_widget)

    def __reset_controllers_widget(self):
        """ Clear widget with controllers list and print it again """
        
        # clear items from widget
        self._widget.ListControllersWidget.clear()

        # create list of available controller classes based on **imported dictionary** 
        count = 0
        for key in controllers_dictionary.controllers_dictionary.keys():
            # print all elements in dictionary
            self._widget.ListControllersWidget.insertItem(count,key)
            count +=1

        # default selected class
        self.__selected_class = controllers_dictionary.controllers_dictionary['NeutralController']

        # selected class may depend on other (parent) classes
        # initialize dictionary which will be input to construct class object
        self.__input_dictionary_for_selected_controller = {} 


    #TODO: this needs a correction: it can return dictionary
    def __get_recursive_class(self,dictionary,list_of_names):
        """ From given dictionary and list of names get dictionary **or** selected class"""

        # this may yield a dictionary with classes for values, or may yield a class
        dictionary_selected_class = dictionary[list_of_names[0]]

        # this try/except is necessary since dictionary_selected_class may not be dictionary
        try:
            if any(dictionary_selected_class) == True:
                # go deeper in the dictionary and get next dictionary or class
                return self.__get_recursive_class(dictionary_selected_class,list_of_names[1:])

        except Exception, e:
            #
            selected_class = dictionary_selected_class
            return selected_class         


    def __controller_item_clicked(self):
        """ what to do when user clicks on an option """
        
        # get string that user selected
        string = self._widget.ListControllersWidget.currentItem().text()

        # string is composed of string separated by :
        # split string in portions
        list_of_names = string.split(':')


        if len(list_of_names) == 1:
            # is list is composed of one string 
            
            selected_class             = controllers_dictionary.controllers_dictionary[list_of_names[0]]

            # save selected class and its names, to be used in printing message to user
            self.__selected_class_name = list_of_names[0]
            self.__selected_class      = selected_class

            if any(selected_class.contained_objects()) == False:
                self.__print_controller_message()
            else:
                # if selected_class depends on other classes, and needs more user information

                # initialize dictionary
                # this dictionary contains **abstracts** of classes
                # TODO: maybe have default controllers in constructors of classes
                self.__input_dictionary_for_selected_controller = selected_class.contained_objects()

                # clear items from widget
                self._widget.ListControllersWidget.clear()

                self._widget.ListControllersWidget.addItem(string)
                for key,item in selected_class.contained_objects().items():
                    # each item is itself a dictionary
                    for key_inner,item_inner in item.items():
                        self._widget.ListControllersWidget.addItem(string+':'+key+':'+key_inner)
        else:
            # if list is composed of more than one string

            dictionary     = self.__selected_class.contained_objects()
            selected_class = self.__get_recursive_class(dictionary,list_of_names[1:])

            # restrict class based on user input
            self.__input_dictionary_for_selected_controller[list_of_names[1]] = selected_class

            print_message_flag = True
            for key,selected_class in self.__input_dictionary_for_selected_controller.items():

                # if selected_class depends on other classes, and needs more user information
                if any(selected_class.contained_objects()) == True:

                    # one class unspecificed is enough to set flag to false
                    print_message_flag = False

                    # clear items from widget
                    self._widget.ListControllersWidget.clear()

                    self._widget.ListControllersWidget.addItem(list_of_names[0])
                    for key,item in selected_class.contained_objects().items():
                        # each item is itself a dictionary
                        for key_inner,item_inner in item.items():
                            self._widget.ListControllersWidget.addItem(string+':'+key+':'+key_inner)

            if print_message_flag == True:
                self.__print_controller_message()

        return

    def __recursion_dictionary_for_selected_class(self,selected_class,selected_controller_class_name):
        
        # initialize empty dictionary
        parameters_dictionary = {}
        # initialize counter
        count = 0
        for key,item in selected_class.contained_objects().items():
            
            selected_class_inner        = item[selected_controller_class_name[1:][count]]
            parameters_dictionary_inner = self.__recursion_dictionary_for_selected_class(selected_class_inner,selected_controller_class_name[1:][count])
            
            parameters_dictionary[key]  = selected_class_inner(**parameters_dictionary_inner)
            count += 1

        return parameters_dictionary


    def __print_controller_message(self):
        """Print message with parameters associated to chosen controller class"""

        parameters_dictionary = self.__input_dictionary_for_selected_controller

        string                = self.__selected_class.parameters_to_string(**parameters_dictionary)
        
        # print message on GUI
        self._widget.ControllerMessageInput.setPlainText(string)

     
        # selected_controller_class_name = self.__chain_selected_controllers_names
        # # get class from dictionary of classes
        # selected_class        = controllers_dictionary.controllers_dictionary[selected_controller_class_name[0]]
        
        # parameters_dictionary = self.__recursion_dictionary_for_selected_class(selected_class,selected_controller_class_name[1:])

        # string                = selected_class.parameters_to_string(**parameters_dictionary)
        
        # # print message on GUI
        # self._widget.ControllerMessageInput.setPlainText(string)

        return 

    def __get_new_controller_parameters(self):
        """Request service for new controller with parameters chosen by user"""

        # get string that user modified with new parameters
        string              = self._widget.ControllerMessageInput.toPlainText()
        # get new parameters from string
        parameters          = string

        # request service
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service("/"+self.namespace+'ServiceChangeController',2.0)
            
            try:
                # RequestingController = rospy.ServiceProxy("/"+self.namespace+'ServiceChangeController', SrvControllerChange)
                RequestingController = rospy.ServiceProxy("/"+self.namespace+'ServiceChangeController', SrvControllerChangeByStr)

                reply = RequestingController(controller_name = self.__selected_class_name, parameters = parameters)

                if reply.received == True:
                    # if controller receives message
                    self._widget.Success.setChecked(True) 
                    self._widget.Failure.setChecked(False) 


            except rospy.ServiceException, e:
                rospy.logwarn('Proxy for service that sets controller FAILED')
                self._widget.Success.setChecked(False) 
                self._widget.Failure.setChecked(True) 
                # print "Service call failed: %s"%e
            
        except:
            rospy.logwarn('Timeout for service that sets controller')
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



    def RESET_xy_PID(self):
        # IMPORTANT
        flag_PID_Controller = 1
        self.ReceiveControllerState(flag_PID_Controller,[1])
            
    def RESET_z_PID(self):
        # IMPORTANT
        flag_PID_Controller = 1
        self.ReceiveControllerState(flag_PID_Controller,[2])


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

    

