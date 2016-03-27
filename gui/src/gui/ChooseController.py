import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal
import PyQt4.QtCore
import pyqtgraph as pg

# necessary to have gui as a client, asking controller to save data
# from python_qt_binding.QtCore import QTimer, Slot
# from python_qt_binding.QtCore import pyqtSlot


import numpy

# import analysis
# import utils
import subprocess



# import services defined in quad_control
# SERVICE BEING USED: Controller_srv
from quad_control.srv import *

# import message of the type controller_state
# because this gui will be able to display the state of the controller
from quad_control.msg import Controller_State


import argparse





# to work with directories relative to ROS packages
from rospkg import RosPack
# determine ROS workspace directory
rp = RosPack()
# determine ROS workspace directory where data is saved
package_path = rp.get_path('quad_control')
# import sys
import sys
sys.path.insert(0, package_path)
# import trajectories dictionaries
from scripts.quadrotor_tracking_controllers_hierarchical import controllers_dictionary


from scripts.systems_functions.double_integrator_controllers import double_integrator_controllers_dictionaries


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
        # ---------------------------------------------- #

        # create list of available controller classes based on dictionary 
        count = 0
        for key in controllers_dictionary.controllers_dictionary.keys():
            self._widget.ListControllersWidget.insertItem(count,key)
            count +=1

        # if item in list is selected, print corresponding message
        self._widget.ListControllersWidget.itemClicked.connect(self.__controller_item_clicked)
        
        # button to request service for setting new trajectory, with new parameters
        self._widget.SetControllerButton.clicked.connect(self.__get_new_controller_parameters)

        self.__chain_controller = []

    def __controller_item_clicked(self):
        # get selected class name on list of classes
        selected_class_name = self._widget.ListControllersWidget.currentItem().text()

        # get class from dictionary of classes
        selected_class      = controllers_dictionary.controllers_dictionary[selected_class_name]
        
        self.__chain_controller.append(selected_class_name)

        if selected_class.parent_class == False:
            self.__print_controller_message()
        else:
            # update list of controllers
            count = 0
            for key in controllers_dictionary.controllers_dictionary.keys(): 
                if key == selected_class_name:
                    for key_child in selected_class.children.keys():
                        self._widget.ListControllersWidget.insertItem(count,key_child)
                        count +=1
                else:
                    self._widget.ListControllersWidget.insertItem(count,key)
                    count +=1

        return

    def __print_controller_message(self):
        """Print message with parameters associated to chosen controller class"""
     
        selected_controller_class_name = self.__chain_controller
        # get class from dictionary of classes
        selected_class      = controllers_dictionary.controllers_dictionary[selected_controller_class_name[0]]
        # get message for chosen class
        if selected_class.parent_class == False:
            string              = selected_class.parameters_to_string()
        else:
            string              = selected_class.parameters_to_string(child_class_name = selected_controller_class_name[1:])
        # print message on GUI
        self._widget.ControllerMessageInput.setPlainText(string)

        return 

    def __get_new_controller_parameters(self):
        """Request service for new controller with parameters chosen by user"""

        # get selected class name on list of classes
        selected_class_name = self._widget.ListControllersWidget.currentItem().text()
        # get class from dictionary of classes
        selected_class      = controllers_dictionary.controllers_dictionary[selected_class_name]
        # get string that user modified with new parameters
        string              = self._widget.ControllerMessageInput.toPlainText()
        # get new parameters from string
        parameters          = selected_class.string_to_parameters(string)

        rospy.logwarn(parameters)

        self.__chain_controller = []

        # request service
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service("/"+self.namespace+'ServiceChangeController',1.0)
            
            try:
                RequestingController = rospy.ServiceProxy("/"+self.namespace+'ServiceChangeController', SrvControllerChange)

                reply = RequestingController(selected_class_name,parameters)

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

    

