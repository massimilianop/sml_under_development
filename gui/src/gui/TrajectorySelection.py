import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal
import PyQt4.QtCore
import pyqtgraph as pg


# import services defined in quad_control
# from quad_control.srv import *
# SERVICE BEING USED: SrvTrajectoryDesired
from quad_control.srv import SrvTrajectoryDesired

# used to get namespace, when passed as argument to GUI
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
from scripts.TrajectoryPlanner import trajectories_dictionary


class TrajectorySelectionPlugin(Plugin):


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(TrajectorySelectionPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TrajectorySelectionPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'TrajectorySelection.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('TrajectorySelectionUi')
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
        count = 0
        for key in trajectories_dictionary.trajectories_dictionary.keys():
            self._widget.ListTrajectoriesWidget.insertItem(count,key)
            count += 1 

        # if item in list is selected, print corresponding message
        self._widget.ListTrajectoriesWidget.itemClicked.connect(self.__print_trajectory_message)
        
        # button to request service for setting new trajectory, with new parameters
        self._widget.SetTrajectory.clicked.connect(self.__get_new_trajectory_parameters)


    def __print_trajectory_message(self):
        """Print message with parameters associated to chosen trajectory class"""
        
        # get selected class name on list of classes
        selected_class_name = self._widget.ListTrajectoriesWidget.currentItem().text()
        # get class from dictionary of classes
        selected_class      = trajectories_dictionary.trajectories_dictionary[selected_class_name]
        # get message for chosen class
        string              = selected_class.to_string()
        # print message on GUI
        self._widget.TrajectoryMessageInput.setPlainText(string)

        # get message associated to offset and rotation
        #string_offset_and_rotation = selected_class.offset_and_rotation_to_string()
        # print message on GUI
        #self._widget.MessageOffsetAndRotation.setPlainText(string_offset_and_rotation)

        return 

    def __get_new_trajectory_parameters(self):
        """Request service for new trajectory with parameters chosen by user"""

        # get selected class name on list of classes
        selected_class_name = self._widget.ListTrajectoriesWidget.currentItem().text()
        # get class from dictionary of classes
        selected_class      = trajectories_dictionary.trajectories_dictionary[selected_class_name]
        # get string that user modified with new parameters
        string              = self._widget.TrajectoryMessageInput.toPlainText()
        # get new parameters from string
        #parameters          = selected_class.string_to_parameters(string)

        # get string that user modified with new offset and rotation
        #string_offset_and_rotation = self._widget.MessageOffsetAndRotation.toPlainText()
        # get offset and rotation from string
        #offset, rotation           = selected_class.string_to_offset_and_rotation(string_offset_and_rotation)

        # request service
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service("/"+self.namespace+'ServiceTrajectoryDesired',1.0)
            
            try:
                SettingTrajectory = rospy.ServiceProxy("/"+self.namespace+'ServiceTrajectoryDesired', SrvTrajectoryDesired)

                #reply = SettingTrajectory(selected_class_name,offset,rotation,parameters)
                reply = SettingTrajectory(selected_class_name, string)
                    
                if reply.received == True:
                    # if controller receives message
                    self._widget.Success.setChecked(True) 
                    self._widget.Failure.setChecked(False) 


            except rospy.ServiceException, e:
                rospy.logwarn('Proxy for service that sets desired trajectory FAILED')
                self._widget.Success.setChecked(False) 
                self._widget.Failure.setChecked(True) 
                # print "Service call failed: %s"%e   
            
        except:
            rospy.logwarn('Timeout for service that sets desired trajectory')
            self._widget.Success.setChecked(False) 
            self._widget.Failure.setChecked(True) 
            # print "Service not available ..."        
            pass                 

        return

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

    

