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
from src.simulators import simulators_dictionary
DICTIONARY_OF_OPTIONS = simulators_dictionary.simulators_dictionary


from choose_jsonable import ChooseJsonablePlugin

class ChooseSimulatorPlugin(Plugin):


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(ChooseSimulatorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChooseSimulatorPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'choose_simulator.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ChooseSimulatorUi')
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

        # "global variables" in dictionary
        self.dic_sequence_services = {}
        self.dic_sequence_services['last_trigger_time']            = 0.0
        self.dic_sequence_services['list_sequence_services']       = []


        self.name_main_tab = "simulator"
        name_tab   = self.name_main_tab
        dictionary = {}
        dictionary["context"]  = context
        dictionary["name_tab"] = name_tab
        # dictionary["dictionary_of_options"] = missions_database.database
        dictionary["dictionary_of_options"] = DICTIONARY_OF_OPTIONS
        dictionary["service_name"]  = "ServiceChangeSimulator"
        dictionary["ServiceClass"]  = SrvChangeJsonableObjectByStr
        dictionary["sequence_tabs"] = []

        setattr(self,name_tab,ChooseJsonablePlugin(**dictionary))
        setattr(getattr(self,name_tab),"dic_sequence_services",self.dic_sequence_services)

        #getattr(self,name_tab).change_dictionary_of_options(self.__HeadClass.inner[inner_key])
        self._widget.tabWidgetSimulator.addTab(getattr(self,name_tab)._widget,name_tab)


        self._widget.start_radio_button.toggled.connect(self.__service_request_simulator)
        self._widget.stop_radio_button.toggled.connect(self.__service_request_simulator)
        self._widget.reset_radio_button.toggled.connect(self.__service_request_simulator)


    #@Slot(bool)
    def __service_request_simulator(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'StartSimulator',1.0)
            
            try:
                AskForStart = rospy.ServiceProxy(self.namespace+'StartSimulator', StartSim)

                # if button is pressed save data
                if self._widget.start_radio_button.isChecked():
                    # request controller to save data
                    reply = AskForStart(True)
                    if reply.Started == True:
                        # if controller receives message, we know it
                        # print('Started')
                        self._widget.SimulatorSuccess.setChecked(True) 
                        self._widget.SimulatorFailure.setChecked(False) 
                else:
                    # request simulator to freeze and restart
                    reply = AskForStart(False)
                    if  reply.Started == True:
                        # if controller receives message, we know it
                        # print('Stopped')
                        self._widget.SimulatorSuccess.setChecked(True) 
                        self._widget.SimulatorFailure.setChecked(False) 

            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.SimulatorSuccess.setChecked(False) 
                self._widget.SimulatorFailure.setChecked(True) 
            
        except: 
            # print "Service not available ..."        
            self._widget.SimulatorSuccess.setChecked(False) 
            self._widget.SimulatorFailure.setChecked(True)
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