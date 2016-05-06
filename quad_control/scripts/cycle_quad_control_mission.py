#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

# controller node publishes message of this type so that GUI can plot stuff
from quad_control.msg import quad_state_and_cmd

# node will publish controller state if requested
from quad_control.msg import Controller_State

# import services defined in quad_control
# Four SERVICES ARE BEING USED: SaveData, ServiceTrajectoryDesired, Mocap_Id, StartSim
# SaveData is for saving data in txt file
# TrajDes is for selecting trajectory
# Mocap_Id for body detection from QUALISYS
# StartSim stop simulator
from quad_control.srv import *

# to work with directories relative to ROS packages
from rospkg import RosPack



import numpy
from numpy import *

import missions.missions_database

class QuadController():

    def __init__(self):

        # frequency of controlling action!!
        self.frequency = 35.0

        # initialize counter for publishing to GUI
        # we only publish when self.PublishToGUI =1
        self.PublishToGUI = 1
        # Frequency of publishing to GUI (Hz)
        frequency_PubToGui = 10
        # we reset counter when self.PublishToGUI >= PublishToGUIBound
        self.PublishToGUIBound = int(self.frequency/frequency_PubToGui)

        # for saving data
        # determine ROS workspace directory
        rp = RosPack()
        # determine ROS workspace directory where data is saved
        package_path = rp.get_path('quad_control')
        self.package_save_path = package_path+'/experimental_data/data/'

    # callback for when "saving data" is requested
    def _handle_save_data(self,req):
        
        if req.flag_save == True:
            # if GUI request data to be saved create file
            
            # namespace, e.g. /Iris1/
            namespace = rospy.get_namespace()
            if not (namespace == ""):
                # remove / symbol to namespace: e.g, we get namespace= Iris1
                namespace = namespace.replace("/", "")

            # string for time: used for generating files
            # time_stamp = str(int(rospy.get_time() - self.TimeSaveData))
            time_stamp = str(int(rospy.get_time()))

            # file name provided by user in request 
            file_name    = req.file_name
            self.file_handle  = file(self.package_save_path+'_'+time_stamp+'_'+file_name+namespace+'.txt', 'w')

            # if GUI request data to be saved, set flag to true
            self.SaveDataFlag = True

            self._add_header_mission()

        else:
            # if GUI request data NOT to be saved, set falg to False
            self.SaveDataFlag = False

        # return message to Gui, to let it know resquest has been fulfilled
        return SaveDataResponse(True)

    # callback for when "reseting iris plus neutral value" is requested
    def _handle_iris_plus_reset_neutral(self,req):
        # return message to GUI, to let it know resquest has been fulfilled
        # IrisPlusResetNeutral: service type
        median_force = self.mission_object.DesiredZForceMedian.output()
        rospy.logwarn('median force = '+ str(median_force))
        self.mission_object.iris_plus_converter_object_mission.reset_k_trottle_neutral(median_force)

        # new neutral value
        k_trottle_neutral = self.mission_object.iris_plus_converter_object_mission.get_k_throttle_neutral()
        return IrisPlusResetNeutralResponse(received = True, k_trottle_neutral = k_trottle_neutral)

    # callback for when "seting iris plus neutral value" is requested
    def _handle_iris_plus_set_neutral(self,req):
        # return message to GUI, to let it know resquest has been fulfilled
        # IrisPlusSetNeutral: service type
        self.mission_object.iris_plus_converter_object_mission.set_k_trottle_neutral(req.k_trottle_neutral)
        return IrisPlusSetNeutralResponse(True)


    # callback for publishing state of controller (or stop publishing)
    def handle_Controller_State_Srv(self,req):

        # if controller belongs to list of controllers that have a state
        if req.flag_controller in controllers_with_state:
            # if GUI request for state of controller, parameters = 0
            if int(req.parameters[0]) == 0:

                    # if publishging already, stop publish
                    # if not publishing, start publishing
                    self.flagPublish_ctr_st = not self.flagPublish_ctr_st

            # if GUI request for reseting state of controller, parameters = 1
            elif int(req.parameters[0]) == 1:
                self.ControllerObject.reset_estimate_xy()
            elif int(req.parameters[0]) == 2:
                self.ControllerObject.reset_estimate_z()

        # return message to Gui, to let it know resquest has been fulfilled
        return Controller_SrvResponse(True)


    # callback for when changing controller is requested
    def _handle_service_change_controller(self,req):
        self.mission_object.change_controller(req.jsonable_name,req.string_parameters)
        
        self._add_header_mission()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvCreateJsonableObjectByStrResponse(received = True)

    # callback for when changing controller is requested
    def _handle_service_change_reference(self,req):

        self.mission_object.change_reference(req.jsonable_name,req.string_parameters)
        
        self._add_header_mission()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvCreateJsonableObjectByStrResponse(received = True)

    # callback for when changing controller is requested
    def _handle_service_change_yaw_controller(self,req):
        self.mission_object.change_yaw_controller(req.jsonable_name,req.string_parameters)
        
        self._add_header_mission()
        
        # return message to Gui, to let it know resquest has been fulfilled
        return SrvCreateJsonableObjectByStrResponse(received = True)

    # callback for when changing controller is requested
    def _handle_service_change_yaw_reference(self,req):
        self.mission_object.change_yaw_reference(req.jsonable_name,req.string_parameters)

        self._add_header_mission()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvCreateJsonableObjectByStrResponse(received = True)      

    # callback for when changing mission is requested
    def _handle_service_change_mission(self,req):

        self.mission_name = req.jsonable_name

        # class_name = req.jsonable_name
        # chosen class taken from dictionary
        MissionClass = missions.missions_database.database[req.jsonable_name]
        
        self.mission_object = MissionClass.from_string(req.string_parameters)
        #rospy.logwarn(self.mission_object.__class__.__name__)

        self._add_header_mission()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvCreateJsonableObjectByStrResponse(received = True)        


    # callback for when changing desired trajectory is requested
    def _handle_service_trajectory_des(self, req):
 
        # trajectory_class_name = req.trajectory
        # update class for TrajectoryGenerator
        self.mission_object.change_trajectory(req.trajectory,req.parameters)

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvTrajectoryDesiredResponse(True)

    def _add_header_mission(self):

        string = self.mission_object.parametric_description(self.mission_name)
        
        if self.SaveDataFlag == True:
            rospy.logwarn(string)
            # if data is being saved, append mission header
            numpy.savetxt(self.file_handle, [string], fmt="%s")

    #callback for turning ON/OFF Mocap and turning OFF/ON the subscription to the simulator
    def handle_Mocap(self,req):
        pass
        # # if mocap is turned on
        # if self.flagMOCAP_On == True:

        #     # request to turn OFF Mocap, and turn on subscription to Simulator messages
        #     if req.On == False:

        #         # in case Qs is not defined yet
        #         try: 
        #             # close mocap connection
        #             # self.Qs._stop_measurement()
        #             del self.Qs

        #             self.flagMOCAP_On = False

        #             # set flag to OFF
        #             self.flagMOCAP = False

        #             # subscribe again to simultor messages
        #             self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state_from_simulator) 

        #             # service has been provided
        #             return Mocap_IdResponse(True,True)
        #         except:
        #             # service was NOT provided
        #             return Mocap_IdResponse(True,False) 

        #     # if we are requested to change the body Id
        #     if req.change_id == True:

        #         # see list of available bodies
        #         bodies = self.Qs.get_updated_bodies()

        #         # check if body_id available
        #         body_indice = -1

        #         # Get the corresponding id of the body
        #         if isinstance(bodies,list):
        #             for i in range(0,len(bodies)):
        #                 rospy.logwarn(bodies[i]['id'])
        #                 if(bodies[i]['id']==req.id):
        #                     body_indice=i

        #         # save body id
        #         self.body_id = req.id                        


        #         if body_indice == -1:

        #             # body does not exist
        #             self.flagMOCAP = False

        #             # body does not exist, but service was provided
        #             return Mocap_IdResponse(False,True)
        #         else:
        #             # body exists
                    
        #             # set flag to on
        #             self.flagMOCAP = True

        #             # body EXISTS, and service was provided
        #             return Mocap_IdResponse(True,True)

        # else:
        #     # if Mocap is turned off, and we are requested to turn it on
        #     if req.On == True:
        #         # establish connection to qualisys
        #         self.Qs = mocap_source.Mocap(info=0)

        #         # stop subscription to data from simulator
        #         # unsubscribe to topic
        #         self.SubToSim.unregister()

        #         self.flagMOCAP_On = True

        #         # service was provided
        #         return Mocap_IdResponse(False,True)

    # return list of bodies detected by mocap or numbers 1 to 99 if not available
    def handle_available_bodies(self, dummy):
        pass
        # if self.flagMOCAP_On:
        #     try: # sometimes mocap causes unpredictable errors
        #         bodies = self.Qs.find_available_bodies(False)
        #         if len(bodies) > 0:
        #             return {"bodies": bodies[0]}
        #     except:
        #         pass

        # return {"bodies": range(0,100)}


    def PublishToGui(self):

        # WE ONLY PUBLIS TO TO GUI AT A CERTAIN FREQEUNCY
        # WHICH IS NOT NECESSARILY THE FREQUENCY OF THE NODE
        if self.PublishToGUI <= self.PublishToGUIBound:
            # if we dont publish, we increase counter
            self.PublishToGUI = self.PublishToGUI + 1
        else:
            # if we publish, we increase counter
            self.PublishToGUI = 1

            # create a message of type quad_state_and_cmd
            st_cmd = quad_state_and_cmd()

            # get current time
            st_cmd.time  = rospy.get_time()

            position_velocity = self.mission_object.get_pv()

            # state of quad comes from QUALISYS, or other sensor
            st_cmd.x     = position_velocity[0]; st_cmd.y     = position_velocity[1]; st_cmd.z     = position_velocity[2]
            st_cmd.vx    = position_velocity[3]; st_cmd.vy    = position_velocity[4]; st_cmd.vz    = position_velocity[5]

            euler_angle  = self.mission_object.get_euler_angles()

            st_cmd.roll  = euler_angle[0]; st_cmd.pitch = euler_angle[1]; st_cmd.yaw   = euler_angle[2]
            
            position_velocity_desired = self.mission_object.get_pv_desired()

            st_cmd.xd    = position_velocity_desired[0]; st_cmd.yd    = position_velocity_desired[1]; st_cmd.zd    = position_velocity_desired[2]
            st_cmd.vxd   = position_velocity_desired[3]; st_cmd.vyd   = position_velocity_desired[4]; st_cmd.vzd   = position_velocity_desired[5]

            rc_input_to_quad = self.mission_object.rc_output
            st_cmd.cmd_1 = rc_input_to_quad[0]; st_cmd.cmd_2 = rc_input_to_quad[1]; st_cmd.cmd_3 = rc_input_to_quad[2]; st_cmd.cmd_4 = rc_input_to_quad[3]

            st_cmd.cmd_5 = 1500.0; st_cmd.cmd_6 = 1500.0; st_cmd.cmd_7 = 1500.0; st_cmd.cmd_8 = 1500.0

            self.pub.publish(st_cmd)     

            # controller state is supposed to be published
            if self.flagPublish_ctr_st:
                # publish controller state
                msg       = Controller_State()
                msg.time  = rospy.get_time()
                msg.d_est = self.ControllerObject.d_est
                self.pub_ctr_st.publish(msg) 

    def control_compute(self):

        # node will be named quad_control (see rqt_graph)
        rospy.init_node('quad_control', anonymous=True)

        # message published by quad_control to GUI 
        self.pub = rospy.Publisher('quad_state_and_cmd', quad_state_and_cmd, queue_size=10)

        # for publishing state of the controller
        self.pub_ctr_st = rospy.Publisher('ctr_state', Controller_State, queue_size=10)
        # initialize flag for publishing controller state at false
        self.flagPublish_ctr_st = False

        #-----------------------------------------------------------------------#
        # TO SAVE DATA FLAG
        # by default, NO data is saved
        self.SaveDataFlag = False
        # we will use this just for convenience, when generating the names of the files
        # where the data will be saved
        self.TimeSaveData = rospy.get_time()
        # Service is created, so that data is saved when GUI requests
        Save_data_service = rospy.Service('SaveDataFromGui', SaveData, self._handle_save_data)


        #-----------------------------------------------------------------------#
        # service for selecting desired trajectory
        # by default, staying still in origin is desired trajectory
        TrajDes_service = rospy.Service('ServiceTrajectoryDesired', SrvTrajectoryDesired, self._handle_service_trajectory_des)


        #-----------------------------------------------------------------------#
        # Service is created, so that Mocap is turned ON or OFF whenever we want
        Save_MOCAP_service = rospy.Service('Mocap_Set_Id', Mocap_Id, self.handle_Mocap)

        # Service for providing list of available mocap bodies to GUI
        mocap_available_bodies = rospy.Service('MocapBodies', MocapBodies, self.handle_available_bodies)

        #-----------------------------------------------------------------------#
        # Services are created, so that user can change controller, reference, yaw_controller and yaw reference on GUI
        rospy.Service('ServiceChangeController'   , SrvCreateJsonableObjectByStr, self._handle_service_change_controller)
        rospy.Service('ServiceChangeReference'    , SrvCreateJsonableObjectByStr, self._handle_service_change_reference)
        rospy.Service('ServiceChangeYawController', SrvCreateJsonableObjectByStr, self._handle_service_change_yaw_controller)
        rospy.Service('ServiceChangeYawReference' , SrvCreateJsonableObjectByStr, self._handle_service_change_yaw_reference)

        rospy.Service('ServiceChangeMission', SrvCreateJsonableObjectByStr, self._handle_service_change_mission)

        #-----------------------------------------------------------------------#
        # Service: change neutral value that guarantees that a quad remains at a desired altitude
        rospy.Service('IrisPlusResetNeutral', IrisPlusResetNeutral, self._handle_iris_plus_reset_neutral)
        rospy.Service('IrisPlusSetNeutral', IrisPlusSetNeutral, self._handle_iris_plus_set_neutral)
        #-----------------------------------------------------------------------#

        self.mission_name  = 'Default'
        # Default Mission Class
        MissionClass = missions.missions_database.database['Default']        
        # construct a default object
        self.mission_object = MissionClass()

        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():

            self.mission_object.publish()

            # publish to GUI (it also contains publish state of Control to GUI)
            self.PublishToGui()

            if self.SaveDataFlag == True:
                # if we want to save data
                numpy.savetxt(self.file_handle, [self.mission_object.get_complete_data()], delimiter=' ')
            
            # go to sleep
            rate.sleep()


if __name__ == '__main__':
    AQuadController = QuadController()
    try:
        AQuadController.control_compute()
    except rospy.ROSInterruptException:
        pass
 
