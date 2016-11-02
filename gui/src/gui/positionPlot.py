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

import subprocess

from quad_control.msg import quad_state_and_cmd

# import services defined in quad_control
from quad_control.srv import *
from quad_control.srv import PlotService

import argparse

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for rospy_tutorials
import sys
sys.path.insert(0, rospack.get_path('quad_control'))

class positionPlotPlugin(Plugin):

    combined_data = pyqtSignal(quad_state_and_cmd)

    # ---------------------------------------------- #
    # ---------------------------------------------- #
    # Necessary constants

    # size of vectors: INTEGER
    Size_Vector = 100
    # Period of complete Time Window: SECONDS
    Period_Window = 10.0
    # Period for data saving: it depends of size of vectors and the time window period
    Period_Data_Saving = Period_Window/Size_Vector
    # Period for plotting: be wise, when plotting to many things
    Period_Plot = 1.0
    # Need to know frequency of messages we are subscribing to 
    Frequency_Subscription = 10


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())

        super(positionPlotPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('positionPlotPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'positionPlot.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('positionPlotUi')
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

        # window for positions
        plotwidget = pg.PlotWidget()
        plotwidget.getPlotItem().addLegend()
        plotwidget.setYRange(-2.5,2.5)      


        # window for velocities
        velplotwidget = pg.PlotWidget()
        velplotwidget.getPlotItem().addLegend()
        velplotwidget.setYRange(-2.5,2.5)   

        # window for velocities
        Anglesplotwidget = pg.PlotWidget()
        Anglesplotwidget.getPlotItem().addLegend()
        Anglesplotwidget.setYRange(-30,30)        

        # window for channels
        channelplotwidget = pg.PlotWidget()
        channelplotwidget.getPlotItem().addLegend()
        channelplotwidget.setYRange(-10,10)


        # ---------------------------------------------- #
        layout        = QtGui.QGridLayout()
        vellayout     = QtGui.QGridLayout()
        Angleslayout  = QtGui.QGridLayout()
        channellayout = QtGui.QGridLayout()

        # increase or reset counter plot data saving
        layout.addWidget(plotwidget)
        vellayout.addWidget(velplotwidget)
        Angleslayout.addWidget(Anglesplotwidget)
        channellayout.addWidget(channelplotwidget)

        # ---------------------------------------------- #
        # labels for window with positions
        plotwidget.getPlotItem().setLabel('left','position','m')
        plotwidget.getPlotItem().setLabel('bottom','time','s')

        # labels for window with velocities
        velplotwidget.getPlotItem().setLabel('left','speed','m/s')
        velplotwidget.getPlotItem().setLabel('bottom','time','s')

        # labels for window with angles
        Anglesplotwidget.getPlotItem().setLabel('left','Angles','Degrees')
        Anglesplotwidget.getPlotItem().setLabel('bottom','time','s')

        # labels for window with channels
        channelplotwidget.getPlotItem().setLabel('left','cmd','PWM(?)')
        channelplotwidget.getPlotItem().setLabel('bottom','time','s')


        # complete putting labels
        self._widget.frame.setLayout(layout)
        self._widget.frame_2.setLayout(vellayout)
        self._widget.frame_3.setLayout(Angleslayout)
        self._widget.frame_4.setLayout(channellayout)


        # ---------------------------------------------- #
        # ---------------------------------------------- #          

        # time vector
        # self.timevector = [0]*self.Size_Vector
        self.timevector = [i*1.0/self.Frequency_Subscription for i in range(self.Size_Vector)]

        
        #Setting variables for each coordinate and channel

        # ---------------------------------------------- #  
        # positions x,y,z
        self.Xplotvector = [0]*self.Size_Vector
        self.Xcurve = plotwidget.getPlotItem().plot(self.timevector,self.Xplotvector, name='x')
        self.Xcurve.setPen(pg.mkPen('r'))
        
        self.Yplotvector = [0]*self.Size_Vector
        self.Ycurve = plotwidget.getPlotItem().plot(self.timevector,self.Yplotvector, name='y')
        self.Ycurve.setPen(pg.mkPen('g'))

        self.Zplotvector = [0]*self.Size_Vector
        self.Zcurve = plotwidget.getPlotItem().plot(self.timevector,self.Zplotvector, name='z')
        self.Zcurve.setPen(pg.mkPen('b'))


        # ---------------------------------------------- #  
        # desired positions x,y,z
        self.Xdplotvector = [0]*self.Size_Vector
        self.Xdcurve = plotwidget.getPlotItem().plot(self.timevector,self.Xdplotvector, name='x')
        self.Xdcurve.setPen(pg.mkPen('r', style=PyQt4.QtCore.Qt.DashLine))
        
        self.Ydplotvector = [0]*self.Size_Vector
        self.Ydcurve = plotwidget.getPlotItem().plot(self.timevector,self.Ydplotvector, name='y')
        self.Ydcurve.setPen(pg.mkPen('g', style=PyQt4.QtCore.Qt.DashLine))

        self.Zdplotvector = [0]*self.Size_Vector
        self.Zdcurve = plotwidget.getPlotItem().plot(self.timevector,self.Zdplotvector, name='z')
        self.Zdcurve.setPen(pg.mkPen('b', style=PyQt4.QtCore.Qt.DashLine))        


        # ---------------------------------------------- #  
        # velocities x,y,z
        self.Xvelplotvector = [0]*self.Size_Vector
        self.Xvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Xvelplotvector, name='v<sub>x</sub>')
        self.Xvelcurve.setPen(pg.mkPen('r'))
        
        self.Yvelplotvector = [0]*self.Size_Vector
        self.Yvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Yvelplotvector, name='v<sub>y</sub>')
        self.Yvelcurve.setPen(pg.mkPen('g'))

        self.Zvelplotvector = [0]*self.Size_Vector
        self.Zvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Zvelplotvector, name='v<sub>z</sub>')
        self.Zvelcurve.setPen(pg.mkPen('b'))

        # ---------------------------------------------- #  
        # desired velocities x,y,z
        self.Xdvelplotvector = [0]*self.Size_Vector
        self.Xdvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Xdvelplotvector, name='v<sub>x</sub>')
        self.Xdvelcurve.setPen(pg.mkPen('r', style=PyQt4.QtCore.Qt.DashLine))
        
        self.Ydvelplotvector = [0]*self.Size_Vector
        self.Ydvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Ydvelplotvector, name='v<sub>y</sub>')
        self.Ydvelcurve.setPen(pg.mkPen('g', style=PyQt4.QtCore.Qt.DashLine))

        self.Zdvelplotvector = [0]*self.Size_Vector
        self.Zdvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Zdvelplotvector, name='v<sub>z</sub>')
        self.Zdvelcurve.setPen(pg.mkPen('b', style=PyQt4.QtCore.Qt.DashLine))

        # ---------------------------------------------- #  
        #  IT ACCEPTS HTML CODE FOR WRITTING ANGLES
        # Euler angles
        self.Roll_plotvector = [0]*self.Size_Vector
        self.Roll_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Roll_plotvector, name='&#966;')
        self.Roll_curve.setPen(pg.mkPen('r'))
        
        self.Pitch_plotvector = [0]*self.Size_Vector
        self.Pitch_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Pitch_plotvector, name='&#952;')
        self.Pitch_curve.setPen(pg.mkPen('g'))

        self.Yaw_plotvector = [0]*self.Size_Vector
        self.Yaw_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Yaw_plotvector, name='&#968;')
        self.Yaw_curve.setPen(pg.mkPen('b'))


        # ---------------------------------------------- #  
        # desired Euler angles
        self.Rolld_plotvector = [0]*self.Size_Vector
        self.Rolld_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Rolld_plotvector, name='&#966;<sub>d</sub>')
        self.Rolld_curve.setPen(pg.mkPen('r', style=PyQt4.QtCore.Qt.DashLine))
        
        self.Pitchd_plotvector = [0]*self.Size_Vector
        self.Pitchd_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Pitchd_plotvector, name='&#952;<sub>d</sub>')
        self.Pitchd_curve.setPen(pg.mkPen('g', style=PyQt4.QtCore.Qt.DashLine))

        self.Yawd_plotvector = [0]*self.Size_Vector
        self.Yawd_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Yawd_plotvector, name='&#968;<sub>d</sub>')
        self.Yawd_curve.setPen(pg.mkPen('b', style=PyQt4.QtCore.Qt.DashLine))    

        # ---------------------------------------------- #  
        # channels
        self.Ch1plotvector = [0]*self.Size_Vector
        self.Ch1curve = channelplotwidget.getPlotItem().plot(self.timevector,self.Ch1plotvector, name='WR')
        self.Ch1curve.setPen(pg.mkPen('r'))

        self.Ch2plotvector = [0]*self.Size_Vector
        self.Ch2curve = channelplotwidget.getPlotItem().plot(self.timevector,self.Ch2plotvector, name='&#966;')
        self.Ch2curve.setPen(pg.mkPen('g'))

        self.Ch3plotvector = [0]*self.Size_Vector
        self.Ch3curve = channelplotwidget.getPlotItem().plot(self.timevector,self.Ch3plotvector, name='&#952;')
        self.Ch3curve.setPen(pg.mkPen('b'))

        self.Ch4plotvector = [0]*self.Size_Vector
        self.Ch4curve = channelplotwidget.getPlotItem().plot(self.timevector,self.Ch4plotvector, name='&#968; rate')
        self.Ch4curve.setPen(pg.mkPen('c'))



        # ---------------------------------------------- #
        # THIS IS A NECESSARY MEASURE BECAUSE THE UPDATING OF DATA AND
        # THE SUBSCRIPTION RUN ON DIFFERENT THREADS
        self.combined_data.connect(self.update_data)

        
        # ---------------------------------------------- #
        # counter for data saving
        self.counter = 1
        self.counterBound = numpy.int(self.Period_Data_Saving*self.Frequency_Subscription)

        # ---------------------------------------------- #
        # counter for plotting
        self.counterPlot = 1
        self.counterBoundPlot = numpy.int(self.Period_Plot*self.Frequency_Subscription)

        # ---------------------------------------------- #
        # initial time: this will be used to offset to time to 0 
        # instead of plotting with "real" time
        self.time0 = rospy.get_time()

        # ---------------------------------------------- #
        # BUTTON TO SUBSCRIBE AND UNSUBSCRIBE
        self._widget.ButtonSubscribe.stateChanged.connect(self.SetSubscription)
       
        # ---------------------------------------------- #
        # BUTTON TO ASK FOR SAVING AND STOPPING SAVING AS WELL
        self._widget.ButtonRequestSave.stateChanged.connect(self.SaveDataClient)


        self.path_txt_files = rospack.get_path('quad_control')+"/experimental_data/data/"

        self.refresh_lists()
        self._widget.refresh_button.clicked.connect(self.refresh_lists)

        # if item in list is clicked twice, 
        self._widget.list_text_files.itemDoubleClicked.connect(self.__service_print_plots)

        # if item in list is clicked twice, open pdf
        self._widget.list_pdf_files.itemDoubleClicked.connect(self.__open_plots)

    def print_txt_files(self):

        self._widget.list_text_files.clear()
        for file in os.listdir(self.path_txt_files):
            if file.endswith(".txt"):
                self._widget.list_text_files.addItem(file) 

    def print_pdf_files(self):

        self._widget.list_pdf_files.clear()
        for file in os.listdir(self.path_txt_files):
            if file.endswith(".pdf"):
                self._widget.list_pdf_files.addItem(file) 

    def refresh_lists(self):
        self.print_txt_files()
        self.print_pdf_files()
        pass

    def __open_plots(self):
        pdf_file_selected = self._widget.list_pdf_files.currentItem().text()
        command = 'see '+ self.path_txt_files+pdf_file_selected + ' &'
        # subprocess.call(command, shell=True)
        subprocess.call(command, shell=False)


    def __service_print_plots(self):
        """Request service for new jsonable object with parameters chosen by user"""

        text_file_selected = self._widget.list_text_files.currentItem().text()

        # request service
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service("PlotService",1.0)
            
            try:
                Requesting = rospy.ServiceProxy("PlotService", PlotService)

                reply = Requesting(file_path = self.path_txt_files+text_file_selected)

                if reply.success == True:
                    print('plot service provided')

            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
                rospy.logwarn('Proxy for service that sets controller FAILED')
                print('plot service NOT provided')
            
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))
            rospy.logwarn('Timeout for service that sets controller')
            print('plot service NOT provided')

        pass 

    def update_data(self,data):

        if self.counter == 1:
            self.timevector[:-1] = self.timevector[1:]
            self.timevector[-1]  = data.time - self.time0

        
        self.VariableOnUpdate(self.Xplotvector,data.x,self.Xcurve,self._widget.Xcheck)
        self.VariableOnUpdate(self.Yplotvector,data.y,self.Ycurve,self._widget.Ycheck)
        self.VariableOnUpdate(self.Zplotvector,data.z,self.Zcurve,self._widget.Zcheck)
        
        self.VariableOnUpdate(self.Xdplotvector,data.xd,self.Xdcurve,self._widget.Xdcheck)
        self.VariableOnUpdate(self.Ydplotvector,data.yd,self.Ydcurve,self._widget.Ydcheck)
        self.VariableOnUpdate(self.Zdplotvector,data.zd,self.Zdcurve,self._widget.Zdcheck)
    
        self.VariableOnUpdate(self.Xvelplotvector,data.vx,self.Xvelcurve,self._widget.Xvelcheck)
        self.VariableOnUpdate(self.Yvelplotvector,data.vy,self.Yvelcurve,self._widget.Yvelcheck)
        self.VariableOnUpdate(self.Zvelplotvector,data.vz,self.Zvelcurve,self._widget.Zvelcheck)       

        self.VariableOnUpdate(self.Xdvelplotvector,data.vxd,self.Xdvelcurve,self._widget.Xdvelcheck)
        self.VariableOnUpdate(self.Ydvelplotvector,data.vyd,self.Ydvelcurve,self._widget.Ydvelcheck)
        self.VariableOnUpdate(self.Zdvelplotvector,data.vzd,self.Zdvelcurve,self._widget.Zdvelcheck)
    
        self.VariableOnUpdate(self.Roll_plotvector,data.roll,self.Roll_curve,self._widget.Roll_check)
        self.VariableOnUpdate(self.Pitch_plotvector,data.pitch,self.Pitch_curve,self._widget.Pitch_check)
        self.VariableOnUpdate(self.Yaw_plotvector,data.yaw,self.Yaw_curve,self._widget.Yaw_check)

        self.VariableOnUpdate(self.Rolld_plotvector,data.roll_d,self.Rolld_curve,self._widget.Rolld_check)
        self.VariableOnUpdate(self.Pitchd_plotvector,data.pitch_d,self.Pitchd_curve,self._widget.Pitchd_check)
        self.VariableOnUpdate(self.Yawd_plotvector,data.yaw_d,self.Yawd_curve,self._widget.Yawd_check)

        self.VariableOnUpdate(self.Ch1plotvector,data.cmd_1,self.Ch1curve,self._widget.Ch1check)
        self.VariableOnUpdate(self.Ch2plotvector,data.cmd_2,self.Ch2curve,self._widget.Ch2check)
        self.VariableOnUpdate(self.Ch3plotvector,data.cmd_3,self.Ch3curve,self._widget.Ch3check)            
        self.VariableOnUpdate(self.Ch4plotvector,data.cmd_4,self.Ch4curve,self._widget.Ch4check)  
        
        return

    def callback(self,data):

        self.combined_data.emit(data)

        # increase or reset counter plot data saving
        if self.counterPlot <= self.counterBoundPlot:
            self.counterPlot = self.counterPlot + 1
        else:
            self.counterPlot = 1

        # increase or reset counter plot data saving
        if self.counter <= self.counterBound:
            self.counter = self.counter + 1
        else:
            self.counter = 1


    def VariableOnUpdate(self,VariablePlotVector,data,PlotHandle,WidgetChecked):

        # updata data vector for "x" component
        if self.counter == 1:
            VariablePlotVector[:-1] = VariablePlotVector[1:]
            VariablePlotVector[-1]  = data

        # plot only after certain time interval, according to counter
        if self.counterPlot == 1:

            # plot if box for "x" component is ticked
            if WidgetChecked.isChecked():
           
                # uncomment when interested in cheching time between two data points
                # print(self.timevector[-1] - self.timevector[-2])                
                PlotHandle.setData(self.timevector,VariablePlotVector)

            else:
                # clear plot 
                PlotHandle.setData([],[])


    def SetSubscription(self):

        if self._widget.ButtonSubscribe.isChecked():
            self.sub = rospy.Subscriber(self.namespace+'quad_state_and_cmd', quad_state_and_cmd, self.callback)
        else:
            # unsubscribe to topic
            self.sub.unregister()

            # clear all plots 
            self.Xcurve.setData([],[])
            self.Ycurve.setData([],[])
            self.Zcurve.setData([],[])
            
            self.Xdcurve.setData([],[])
            self.Ydcurve.setData([],[])
            self.Zdcurve.setData([],[])
            
            self.Xvelcurve.setData([],[])
            self.Yvelcurve.setData([],[])
            self.Zvelcurve.setData([],[])            
            
            self.Xdvelcurve.setData([],[])
            self.Ydvelcurve.setData([],[])
            self.Zdvelcurve.setData([],[])

            self.Roll_curve.setData([],[])
            self.Pitch_curve.setData([],[])
            self.Yaw_curve.setData([],[])
            
            self.Rolld_curve.setData([],[])
            self.Pitchd_curve.setData([],[])
            self.Yawd_curve.setData([],[])            

            self.Ch1curve.setData([],[])
            self.Ch2curve.setData([],[])
            self.Ch3curve.setData([],[])            
            self.Ch4curve.setData([],[])   

    #@Slot(bool)
    def SaveDataClient(self):
        
        #rospy.logwarn('testing')
        file_name = self._widget.input_file_name.toPlainText()
        
        if not file_name:
        # if file_name == "":
            #file_name = 'temporary_file'+str(rospy.get_time())
            file_name = 'untitled_file'

        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'SaveDataFromGui',1.0)
            
            try:
                AskForSavingOrNot = rospy.ServiceProxy(self.namespace+'SaveDataFromGui', SaveData)

                # if button is pressed save data
                if self._widget.ButtonRequestSave.isChecked():
                    # request controller to save data (implicit, with keywords)
                    reply = AskForSavingOrNot(flag_save = True, file_name = file_name)
                    if reply.Saving == True:
                        # if controller receives message, we know it
                        print('Saving')
                else:
                    # request controller to STOP saving data
                    reply = AskForSavingOrNot(flag_save = False)
                    if  reply.Saving == True:
                        # if controller receives message, we know it
                        print('Stopped Saving')

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
            
        except: 
            print "Service not available ..."        
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