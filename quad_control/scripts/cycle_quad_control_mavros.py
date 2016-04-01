#!/usr/bin/env python
# this line is just used to define the type of document

import mavros

from mavros_msgs.msg import OverrideRCIn
# from mavros_msgs.srv import ParamSet,ParamGet,CommandBool,SetMode

# node will publish motor speeds
from mav_msgs.msg import Actuators

#node will subscribe to odometry measurements
from nav_msgs.msg import Odometry

import rospy

# controller node publishes message of this type so that GUI can plot stuff
from quad_control.msg import quad_state_and_cmd

# node will need to get state from qualysis, other sensor, or simulator
# it subscribes to a message of this tupe
from quad_control.msg import quad_state

# node will publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd

# node will publish controller state if requested
from quad_control.msg import Controller_State

# import services defined in quad_control
# Four SERVICES ARE BEING USED: SaveData, ServiceTrajectoryDesired, Mocap_Id, StartSim
# SaveData is for saving data in txt file
# TrajDes is for selecting trajectory
# Mocap_Id for body detection from QUALISYS
# StartSim stop simulator
from quad_control.srv import *

# when Mocap is used this is necessary
import mocap_source

# to work with directories relative to ROS packages
from rospkg import RosPack

import numpy
from numpy import *

from utility_functions import GetRotFromEulerAnglesDeg,Velocity_Filter,Median_Filter

# import list of available trajectories
from TrajectoryPlanner import trajectories_dictionary
from Yaw_Rate_Controller import yaw_controllers_dictionary

# from quadrotor_tracking_controllers_hierarchical import controllers_dictionary
from controllers_hierarchical.fully_actuated_controllers import controllers_dictionary

from ConverterBetweenStandards.RotorSConverter import RotorSConverter
from ConverterBetweenStandards.IrisPlusConverter import IrisPlusConverter

import math

class quad_controller():

    def __init__(self):

        # frequency of controlling action!!
        self.frequency = 35.0

        # state of quad: position, velocity and attitude 
        # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = numpy.zeros(3+3+3)

        # dy default, desired trajectory is staying still in origin
        TrajectoryClass    = trajectories_dictionary.trajectories_dictionary['StayAtRest']
        self.TrajGenerator = TrajectoryClass()

        # initialize counter for publishing to GUI
        # we only publish when self.PublishToGUI =1
        self.PublishToGUI = 1
        # Frequency of publishing to GUI (Hz)
        frequency_PubToGui = 10
        # we reset counter when self.PublishToGUI >= PublishToGUIBound
        self.PublishToGUIBound = int(self.frequency/frequency_PubToGui)

        # intiialization should be done in another way,
        # but median will take care of minimizing effects
        self.VelocityEstimator = Velocity_Filter(3,numpy.zeros(3),0.0)

        # controllers selected by default
        ControllerClass = controllers_dictionary.controllers_dictionary['PIDSimpleBoundedIntegralController']
        self.ControllerObject = ControllerClass()

        YawControllerClass = yaw_controllers_dictionary.yaw_controllers_dictionary['YawRateControllerTrackReferencePsi']
        self.YawControllerObject = YawControllerClass()

        # for reseting neutral value that makes iris+ stay at desired altitude
        self.DesiredZForceMedian = Median_Filter(10)


        # for saving data
        # determine ROS workspace directory
        rp = RosPack()
        # determine ROS workspace directory where data is saved
        package_path = rp.get_path('quad_control')
        self.package_save_path = package_path+'/experimental_data/data/'


    def GET_STATE_(self):

        # if simulator is on, state is updated by subscription
        # if mocap is on, state comes from mocap
        if self.flagMOCAP == True:

            self.flag_measurements = 1

            bodies = self.Qs.get_body(self.body_id)

            if bodies != 'off':  

                self.flag_measurements = 1

                # position
                x=bodies["x"]; y=bodies["y"]; z=bodies["z"]   
                p = numpy.array([x,y,z])

                # velocity
                #v = numpy.array([data.vx,data.vy,data.vz])
                v = self.VelocityEstimator.out(p,rospy.get_time())

                # attitude: euler angles THESE COME IN DEGREES
                roll = bodies["roll"]; pitch=-bodies["pitch"]; yaw  = bodies["yaw"]
                ee = numpy.array([roll,pitch,yaw])

                # collect all components of state
                self.state_quad = numpy.concatenate([p,v,ee])  
            else:
                # do nothing, keep previous state
                self.flag_measurements = 2
        else:
            # simulator on
            self.flag_measurements = 0

    # callback when simulator publishes states
    def get_state_from_simulator(self, data):
        # position
        p = numpy.array([data.x,data.y,data.z])
        # velocity
        v = numpy.array([data.vx,data.vy,data.vz])
        #v = self.VelocityEstimator.out(p,rospy.get_time())
        # attitude: euler angles
        ee = numpy.array([data.roll,data.pitch,data.yaw])
        # collect all components of state
        self.state_quad = numpy.concatenate([p,v,ee])  

    # callback when ROTORS simulator publishes states
    def get_state_from_rotorS_simulator(self,odometry_rotor_s):

        # RotorS has attitude inner loop that needs to known attitude of quad 
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        # get state from rotorS simulator
        self.state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)        

    # for obtaining current desired state
    def traj_des(self):
        # time for trajectory generation
        time_TrajDes = rospy.get_time() - self.time_TrajDes_t0
        return self.TrajGenerator.output(time_TrajDes)


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

        else:
            # if GUI request data NOT to be saved, set falg to False
            self.SaveDataFlag = False

        # return message to Gui, to let it know resquest has been fulfilled
        return SaveDataResponse(True)

    # callback for when "reseting iris plus neutral value" is requested
    def _handle_iris_plus_reset_neutral(self,req):
        # return message to GUI, to let it know resquest has been fulfilled
        # IrisPlusResetNeutral: service type
        median_force = self.DesiredZForceMedian.output()
        rospy.logwarn('median force = '+ str(median_force))
        self.IrisPlusConverterObject.reset_k_trottle_neutral(median_force)

        # new neutral value
        k_trottle_neutral = self.IrisPlusConverterObject.get_k_throttle_neutral()
        return IrisPlusResetNeutralResponse(received = True, k_trottle_neutral = k_trottle_neutral)

    # callback for when "seting iris plus neutral value" is requested
    def _handle_iris_plus_set_neutral(self,req):
        # return message to GUI, to let it know resquest has been fulfilled
        # IrisPlusSetNeutral: service type
        self.IrisPlusConverterObject.set_k_trottle_neutral(req.k_trottle_neutral)
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

        # controller_class_name = req.controller_name
        # chosen class taken from dictionary
        ControllerClass = controllers_dictionary.controllers_dictionary[req.controller_name]

        parameters_dictionary = ControllerClass.string_to_parameters(req.parameters)
        
        self.ControllerObject = ControllerClass(**parameters_dictionary)

        #rospy.logwarn(self.ControllerObject.__class__.__name__)
        rospy.logwarn('444444444444')

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvControllerChangeByStrResponse(received = True)


    # callback for when changing desired trajectory is requested
    def _handle_service_trajectory_des(self, req):
 
        # trajectory_class_name = req.trajectory
        # update class for TrajectoryGenerator
        TrajectoryClass = trajectories_dictionary.trajectories_dictionary[req.trajectory]
        
        string = req.parameters
        
        offset   = numpy.array(req.offset)
        rotation = numpy.array(req.rotation)

        self.TrajGenerator = TrajectoryClass.from_string(string)

        # we need to update initial time for trajectory generation
        self.time_TrajDes_t0 = rospy.get_time()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvTrajectoryDesiredResponse(True)

    # # function to stop simulator
    # def stop_simulator(self):
    #     try: 
    #         rospy.wait_for_service('StartSimulator',1.0)

    #         try:
    #             AskForStart = rospy.ServiceProxy('StartSimulator', StartSim)

    #             reply = AskForStart(False)
    #             if reply.Started == True:
    #                 return True
    #             else:
    #                 return False
    #         except:
    #             return False

    #     except:
    #         return False

    #callback for turning ON/OFF Mocap and turning OFF/ON the subscription to the simulator
    def handle_Mocap(self,req):



        # if mocap is turned on
        if self.flagMOCAP_On == True:

            # request to turn OFF Mocap, and turn on subscription to Simulator messages
            if req.On == False:

                # in case Qs is not defined yet
                try: 
                    # close mocap connection
                    # self.Qs._stop_measurement()
                    del self.Qs

                    self.flagMOCAP_On = False

                    # set flag to OFF
                    self.flagMOCAP = False

                    # subscribe again to simultor messages
                    self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state_from_simulator) 

                    # service has been provided
                    return Mocap_IdResponse(True,True)
                except:
                    # service was NOT provided
                    return Mocap_IdResponse(True,False) 

            # if we are requested to change the body Id
            if req.change_id == True:

                # see list of available bodies
                bodies = self.Qs.get_updated_bodies()

                # check if body_id available
                body_indice = -1

                # Get the corresponding id of the body
                if isinstance(bodies,list):
                    for i in range(0,len(bodies)):
                        rospy.logwarn(bodies[i]['id'])
                        if(bodies[i]['id']==req.id):
                            body_indice=i

                # save body id
                self.body_id = req.id                        


                if body_indice == -1:

                    # body does not exist
                    self.flagMOCAP = False

                    # body does not exist, but service was provided
                    return Mocap_IdResponse(False,True)
                else:
                    # body exists
                    
                    # set flag to on
                    self.flagMOCAP = True

                    # body EXISTS, and service was provided
                    return Mocap_IdResponse(True,True)


                # # stop simulator
                # stop = self.stop_simulator()

                # # if simulator stopped
                # if stop == True:

                #     if body_indice == -1:

                #         # body does not exist
                #         self.flagMOCAP = False

                #         # body does not exist, but service was provided
                #         return Mocap_IdResponse(False,True)
                #     else:
                #         # body exists
                        
                #         # set flag to on
                #         self.flagMOCAP = True

                #         # body EXISTS, and service was provided
                #         return Mocap_IdResponse(True,True)
                # else:
                #     self.flagMOCAP = False
                #     # service was NOT provided
                #     return Mocap_IdResponse(False,False)


        else:
            # if Mocap is turned off, and we are requested to turn it on
            if req.On == True:
                # establish connection to qualisys
                self.Qs = mocap_source.Mocap(info=0)

                # stop subscription to data from simulator
                # unsubscribe to topic
                self.SubToSim.unregister()

                self.flagMOCAP_On = True

                # service was provided
                return Mocap_IdResponse(False,True)

    # return list of bodies detected by mocap or numbers 1 to 99 if not available
    def handle_available_bodies(self, dummy):
        if self.flagMOCAP_On:
            try: # sometimes mocap causes unpredictable errors
                bodies = self.Qs.find_available_bodies(False)
                if len(bodies) > 0:
                    return {"bodies": bodies[0]}
            except:
                pass

        return {"bodies": range(0,100)}


    def PublishToGui(self,states_d,Input_to_Quad):

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

            # state of quad comes from QUALISYS, or other sensor
            st_cmd.x     = self.state_quad[0]; st_cmd.y     = self.state_quad[1]; st_cmd.z     = self.state_quad[2]
            st_cmd.vx    = self.state_quad[3]; st_cmd.vy    = self.state_quad[4]; st_cmd.vz    = self.state_quad[5]
            st_cmd.roll  = self.state_quad[6]; st_cmd.pitch = self.state_quad[7]; st_cmd.yaw   = self.state_quad[8]
            
            st_cmd.xd    = states_d[0]; st_cmd.yd    = states_d[1]; st_cmd.zd    = states_d[2]
            st_cmd.vxd   = states_d[3]; st_cmd.vyd   = states_d[4]; st_cmd.vzd   = states_d[5]

            st_cmd.cmd_1 = Input_to_Quad[0]; st_cmd.cmd_2 = Input_to_Quad[1]; st_cmd.cmd_3 = Input_to_Quad[2]; st_cmd.cmd_4 = Input_to_Quad[3]

            st_cmd.cmd_5 = 1500.0; st_cmd.cmd_6 = 1500.0; st_cmd.cmd_7 = 1500.0; st_cmd.cmd_8 = 1500.0

            self.pub.publish(st_cmd)     

            # controller state is supposed to be published
            if self.flagPublish_ctr_st:
                # publish controller state
                msg       = Controller_State()
                msg.time  = rospy.get_time()
                msg.d_est = self.ControllerObject.d_est
                self.pub_ctr_st.publish(msg) 
        
    def PublishToQuad(self,Input_to_Quad):

        # create a message of the type quad_cmd, that the simulator subscribes to 
        cmd  = quad_cmd()

        cmd.cmd_1 = Input_to_Quad[0]; cmd.cmd_2 = Input_to_Quad[1]; cmd.cmd_3 = Input_to_Quad[2]; cmd.cmd_4 = Input_to_Quad[3];
        
        cmd.cmd_5 = 1500.0; cmd.cmd_6 = 1500.0; cmd.cmd_7 = 1500.0; cmd.cmd_8 = 1500.0
        
        self.pub_cmd.publish(cmd)

    def control_compute(self):

        # node will be named quad_control (see rqt_graph)
        rospy.init_node('quad_control', anonymous=True)

        # message published by quad_control to GUI 
        self.pub = rospy.Publisher('quad_state_and_cmd', quad_state_and_cmd, queue_size=10)
        
        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('quad_cmd', quad_cmd, queue_size=10)

        # for publishing state of the controller
        self.pub_ctr_st = rospy.Publisher('ctr_state', Controller_State, queue_size=10)
        # initialize flag for publishing controller state at false
        self.flagPublish_ctr_st = False

        # controller needs to have access to STATE of the system
        # this can come from QUALISYS, a sensor, or the simulator
        self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state_from_simulator) 

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
        # SERVICE FOR SELECTING DESIRED TRAJECTORY
        # by default, STAYING STILL IN ORIGIN IS DESIRED TRAJECTORY
        # self.flagTrajDes = 0
        # Service is created, so that data is saved when GUI requests
        TrajDes_service = rospy.Service('ServiceTrajectoryDesired', SrvTrajectoryDesired, self._handle_service_trajectory_des)

        # initialize initial time for trajectory generation
        self.time_TrajDes_t0 = rospy.get_time()

        #-----------------------------------------------------------------------#
        # flag for MOCAP is initialized as FALSE
        # flag for wheter Mocap is being used or not
        self.flagMOCAP    = False
        # Flag for whether Mocap is ON or OFF
        self.flagMOCAP_On = False
        # Service is created, so that Mocap is turned ON or OFF whenever we want
        Save_MOCAP_service = rospy.Service('Mocap_Set_Id', Mocap_Id, self.handle_Mocap)

        # Service for providing list of available mocap bodies to GUI
        mocap_available_bodies = rospy.Service('MocapBodies', MocapBodies, self.handle_available_bodies)

        #-----------------------------------------------------------------------#
        # Service is created, so that user can change controller on GUI
        rospy.Service('ServiceChangeController', SrvControllerChangeByStr, self._handle_service_change_controller)
        # rospy.Service('ServiceChangeController', SrvControllerChange, self._handle_service_change_controller)


        #-----------------------------------------------------------------------#
        # Service: change neutral value that guarantees that a quad remains at a desired altitude
        rospy.Service('IrisPlusResetNeutral', IrisPlusResetNeutral, self._handle_iris_plus_reset_neutral)
        rospy.Service('IrisPlusSetNeutral', IrisPlusSetNeutral, self._handle_iris_plus_set_neutral)

        #-----------------------------------------------------------------------#
        # Service for publishing state of controller 
        # we use same type of service as above
        #Contller_St = rospy.Service('Controller_State_GUI', Controller_Srv, self.handle_Controller_State_Srv)

        #-----------------------------------------------------------------------#

        self.RotorSObject = RotorSConverter()
        self.RotorSFlag   = True


        # subscriber: to odometry
        # self.sub_odometry = rospy.Subscriber("/firefly/odometry_sensor1/odometry", Odometry, self.compute_cmd)
        self.sub_odometry = rospy.Subscriber("/firefly/ground_truth/odometry", Odometry, self.get_state_from_rotorS_simulator) 

        # # subscriber: to odometry
        # # self.sub_odometry = rospy.Subscriber("/firefly/odometry_sensor1/odometry", Odometry, self.compute_cmd)
        # self.sub_odometry_load = rospy.Subscriber("/firefly/ground_truth/odometry_load", Odometry, self.update_load_odometry) 

        # publisher: command firefly motor speeds 
        self.pub_motor_speeds = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=10)
        #-----------------------------------------------------------------------#
        
        # rc_override = rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=10)
        rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=100)

        pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=100)

        rate = rospy.Rate(self.frequency)

        t0 = rospy.get_time() 

        self.IrisPlusConverterObject = IrisPlusConverter()

        while not rospy.is_shutdown():

            time = rospy.get_time()

            # get state: if MOCAP is ON we ask MOCAP; if MOCAP in OFF, we are subscribed to Simulator topic
            self.GET_STATE_()

            #print(self.state_quad[0:3])
            # print self.state_quad[6:9]

            # states for desired trajectory
            states_d = self.traj_des()

            # compute input to send to QUAD
            # rospy.logwarn(self.ControllerObject.__class__.__name__)
            desired_3d_force_quad = self.ControllerObject.output(time,self.state_quad,states_d)
            self.DesiredZForceMedian.update_data(desired_3d_force_quad[2])


            euler_rad     = self.state_quad[6:9]*math.pi/180
            euler_rad_dot = numpy.zeros(3)

            # convert to iris + standard
            state_for_yaw_controller = numpy.concatenate([euler_rad,euler_rad_dot])
            input_for_yaw_controller = numpy.array([0.0,0.0])
            yaw_rate = self.YawControllerObject.output(state_for_yaw_controller,input_for_yaw_controller)
            
            self.IrisPlusConverterObject.set_rotation_matrix(euler_rad)
            iris_plus_rc_input = self.IrisPlusConverterObject.input_conveter(desired_3d_force_quad,yaw_rate)


            # Publish commands to Quad
            self.PublishToQuad(iris_plus_rc_input)

            # publish to GUI (it also contains publish state of Control to GUI)
            self.PublishToGui(states_d,iris_plus_rc_input)


            # ORDER OF INPUTS IS VERY IMPORTANT: roll, pitch, throttle,yaw_rate
            # create message of type OverrideRCIn
            rc_cmd          = OverrideRCIn()
            other_channels  = numpy.array([1500,1500,1500,1500])
            channels        = numpy.concatenate([iris_plus_rc_input,other_channels])
            channels        = numpy.array(channels,dtype=numpy.uint16)
            rc_cmd.channels = channels
            rc_override.publish(rc_cmd)

            # publish message
            # TOTAL_MASS = 1.66779; Force3D = numpy.array([0.0,0.0,TOTAL_MASS*9.85]); PsiStar = 0.0; self.pub_motor_speeds.publish(self.RotorSObject.rotor_s_message(Force3D,PsiStar))
            self.pub_motor_speeds.publish(self.RotorSObject.rotor_s_message(desired_3d_force_quad,yaw_rate))


            if self.SaveDataFlag == True:
                # if we want to save data
                # numpy.savetxt(self.file_handle, [concatenate([[rospy.get_time()],[self.flag_measurements], self.state_quad, states_d[0:9], Input_to_Quad,self.ControllerObject.d_est])],delimiter=' ')
                numpy.savetxt(self.file_handle, [concatenate([[rospy.get_time()],[self.flag_measurements], self.state_quad, states_d[0:9], desired_3d_force_quad])],delimiter=' ')
            # go to sleep
            rate.sleep() 


if __name__ == '__main__':
    A_Controller = quad_controller()
    try:
        A_Controller.control_compute()
    except rospy.ROSInterruptException:
        pass
