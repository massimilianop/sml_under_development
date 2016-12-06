import os
import rospy

import QtGui
# from PyQt4 import QtGui

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
#from controller.msg import Permission
from std_srvs.srv import Empty
from PyQt4.QtCore import QObject, pyqtSignal
#from mavros.msg import OverrideRCIn
#from mavros.msg import BatteryStatus

#import analysis
#import utils
import subprocess

from positionPlot import positionPlotPlugin

from choose_mission import ChooseMissionPlugin

#from choose_system import ChooseSystemPlugin

#from choose_simulator import ChooseSimulatorPlugin

#from choose_gazebo import ChooseGazeboPlugin

#from choose_mocap import ChooseMocapPlugin

import argparse


import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for rospy_tutorials
import sys
sys.path.insert(0, rospack.get_path('quad_control'))

#from src.simulators import simulators_dictionary

class tabbedGUIPlugin(Plugin):

    def __init__(self, context,namespace = None, system_type = 'gazebo'):

        # it is either "" or the input given at creation of plugin
        self.namespace, system_type = self._parse_args(context.argv())
        # warn message for letting user know namespace for the gui
        rospy.logwarn("Gui within namespace: " + self.namespace)


        super(tabbedGUIPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('tabbedGUIPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'tabbedGUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('tabbedGUIUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        # DO NOT COMMENT THIS
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # # Adding all the tabs

        self.ChooseMission   = ChooseMissionPlugin(context,self.namespace)
        self.positionPlot    = positionPlotPlugin(context,self.namespace)
        
        # # self.ChooseSystem    = ChooseSystemPlugin(context,self.namespace)
        # if system_type == 'gazebo':
        #     self.ChooseSystem    = ChooseGazeboPlugin(context,self.namespace)
        # if system_type == 'rviz':
        #     # self.ChooseSystem    = ChooseSimulatorPlugin(context,self.namespace)            
            
            # EXAMPLE_DICTIONARY = {}
            # EXAMPLE_DICTIONARY["name_main_tab"] = "simulator"
            # EXAMPLE_DICTIONARY["strServiceChangeName"] = "ServiceChangeSimulator"
            # EXAMPLE_DICTIONARY["DICTIONARY_OF_OPTIONS"] = simulators_dictionary.simulators_dictionary
            # EXAMPLE_DICTIONARY["name_service_sequence_provider"] = 'simulator/ServiceSequencer'

            # self.ChooseSystem  = ChooseMissionPlugin(context,self.namespace,dictionary_options = EXAMPLE_DICTIONARY)
        
        # if system_type == 'mocap':
        #     self.ChooseSystem    = ChooseMocapPlugin(context,self.namespace)


        self._widget.tabWidget.addTab(self.ChooseMission._widget  ,'Select Mission')
        # if system_type == 'gazebo' or system_type == 'rviz':
        #self._widget.tabWidget.addTab(self.ChooseSystem._widget   ,'Select System')
        self._widget.tabWidget.addTab(self.positionPlot._widget   ,'Check Data')

        self._widget.tabWidget.show()

    def _parse_args(self, argv):

        parser = argparse.ArgumentParser(prog='saver', add_help=False)

        # args = parser.parse_args(argv)

        if argv:
            namespace   = argv[0]
            system_type = argv[1]
            return namespace, system_type
        else:
            # is argv is empty return empty string
            return "","gazebo"

        
    # def execute(self,cmd):
    #     #subprocess.Popen(["bash","-c","cd "+self.pwd+"/src/kampala/gui/scripts; echo "+cmd+" > pipefile" + self.name]) 


    # def shutdown_plugin(self):
    #     # TODO unregister all publishers here
    #     pass

    # def save_settings(self, plugin_settings, instance_settings):
    #     # TODO save intrinsic configuration, usually using:
    #     # instance_settings.set_value(k, v)
    #     instance_settings.set_value("irisindex", self._widget.IrisInputBox.currentIndex())

    # def restore_settings(self, plugin_settings, instance_settings):
    #     # TODO restore intrinsic configuration, usually using:
    #     # v = instance_settings.value(k)
    #     index = instance_settings.value("irisindex",0)
    #     self._widget.IrisInputBox.setCurrentIndex(int(index))
        
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    
