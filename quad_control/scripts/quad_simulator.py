#!/usr/bin/env python
# this is just to define file type

import rospy

from std_msgs.msg import String


# simulator will publish quad state
from quad_control.msg import quad_state
#TODO camel case for message names (they are class names)

# simulator will publish quad state
from quad_control.msg import quad_cmd

# import services defined in quad_control
# SERVICE for Starting Simulation
# SERVICE for Reseting Simulation
# SERVICE for changing Simulator
from quad_control.srv import *


# for integrating and solving differential equation
from scipy.integrate import ode

# from copy import deepcopy

# # for ploting
# import matplotlib.pyplot as plt

import numpy
from numpy import *
from numpy import cos as c
from numpy import sin as s


# from utility_functions import GetEulerAnglesDeg
from utilities.utility_functions import euler_deg_from_rot

import simulators.simulators_dictionary as shsd


from visualization_msgs.msg import Marker


simdic = shsd.simulators_dictionary

#simulators_dictionary = {0:SimulatorWithZeroDynamics,1:SimulatorWithAttitudeInnerLoop,2:SimulatorWithNoAttitudeInnerLoop}


#----------------------------------------------#
#----------------------------------------------#
# This class is necessary because of solver
# this DOES NOT work: r.set_f_params(deepcopy(self.U),parameters) if U is  ARRAY
# this works: r.set_f_params(Input(self.U),parameters) if U is  ARRAY

#class Input(object):

#    def __init__(self,U):

#        self.U0 = U[0]
#        self.U1 = U[1]
#        self.U2 = U[2]
#        self.U3 = U[3]

#        return    
#----------------------------------------------#
#----------------------------------------------#


class SimulatorNode():


    def __init__(self):
        #TODO lowercase for non-class names
        #TODO leading underscores for internal variables


        # frequency of node (Hz)
        self.frequency = 100
        # self.frequency = 1

        # delay for starting simulator (sec)
        self.TimeDelay = 2.0

        # input is initialized at zero
        # input comes from subscribing to topic that comes from controller node
        self.U = numpy.array([0.0,0.0,0.0,0.0])


        # by default, we get the class [0] which is staying still
        # SimClass = simdic["no_attitude_inner_loop"]
        SimClass = simdic["Default"]
        # sim is an instant/object of class Sim_class
        #sim       = Sim_class()
        # f is the dynamics, which is defined ALWAYS AS THE OUTPUT FUNCTION
        #f         = sim.output
        # r is the solver object used for integration

        #---------------------------------------------------------------------#
        # initial states

        # initial quad position (m)
        x0 = numpy.array([0,0,0])
        # initial quad velocity (m/s)
        v0 = numpy.array([0,0,0]) 
        # quad rotation matrix (body to inertial)
        # R0 = Rz(0).dot(Ry(0).dot(Rx(0.0*3.14/180)))
        # R0 = numpy.reshape(R0,9)
        # initial rotation matrix : Identity
        R0 = numpy.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        R0 = numpy.reshape(R0,9)

        # collecting all initial states0
        self.states0  = concatenate([x0,v0,R0])
        self.time0 = 0.0

        # self.r = ode(f).set_integrator('dopri5')
        #self.r.set_initial_value(self.states0,0.0)


        self.sim = SimClass(initial_time=self.time0, initial_state=self.states0)

        #---------------------------------------------------------------------#

    # this is the callback function that is used when input is found to be published
    def get_input(self, data):
        # create zero vector
        U = numpy.zeros(4)
        U[0] = data.cmd_1
        U[1] = data.cmd_2
        U[2] = data.cmd_3
        U[3] = data.cmd_4

        # update input 
        self.U = U
        self.sim.set_control(U)


    # callback used when starting simulator
    def handle_Start_service(self,req):
        
        if req.Start == True:
            # if GUI request simulation to be started
            self.StartFlag = True
        else:
            # if GUI request data NOT to be saved, set flag to False
            self.StartFlag = False

        # return message to Gui, to let it know resquest has been fulfilled
        return StartSimResponse(True)


    # callback used when reseting simulator
    def handle_Reset_service(self,req):
        
        # simulator has been asked to reset
        self.sim.reset(initial_state=self.state0)      

        # return message to Gui, to let it know resquest has been fulfilled
        return StartSimResponse(True)


    # callback used for changing simulator
    def __handle_simulator_change_service(self,req):
        SimulatorClass = simdic[req.jsonable_name]
        
        self.sim = SimulatorClass.from_string(req.string_parameters)

        # return message: resquest has been fulfilled
        return SrvCreateJsonableObjectByStrResponse(received = True)


    def write_state(self):

        # create a message of type quad_state_and_cmd
        state = quad_state()
        
        # get current time
        state.time = self.sim.get_time()
        simstate = self.sim.get_state()
        
        state.x  = simstate[0]
        state.y  = simstate[1]
        state.z  = simstate[2]
        state.vx = simstate[3]
        state.vy = simstate[4]
        state.vz = simstate[5]

        # rotation matrix
        R  = simstate[6:15]
        R  = numpy.reshape(R,(3,3))
        ee = euler_deg_from_rot(R)

        state.roll  = ee[0]
        state.pitch = ee[1]
        state.yaw   = ee[2]        

        return state


    def simulate_quad(self):

        # simulator node 
        rospy.init_node('simulate_quad', anonymous=True)

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#

        # Simulator subscribes to command inputs, published by a controller
        rospy.Subscriber("quad_cmd", quad_cmd, self.get_input)

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#

        # this node is a simulator, thus it will publish the state of the quad
        # it uses the commands -- that it is subscribed to -- to solve differential equations 
        pub = rospy.Publisher('quad_state', quad_state, queue_size=10)
        

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # TO SAVE FLAG
        # by default, no dynamics: everything stopped
        self.StartFlag = False
        # Service is created, so that simulation can be started or stopped
        Start_service = rospy.Service('StartSimulator', StartSim, self.handle_Start_service)
        # Service is created, so that simulation can be reseted
        Reset_service = rospy.Service('ResetSimulator', StartSim, self.handle_Reset_service)
        
        # NOTICE THAT SERVICE TYPE IS THE SAME FOR BOTH SERVICES ABOVE

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#


        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # Service is created, so that user can change simulator on GUI
        Chg_Simulator = rospy.Service('ServiceChangeSimulator', SrvCreateJsonableObjectByStr, self.__handle_simulator_change_service)


        # solve differential equations at frequency
        rate = rospy.Rate(self.frequency)
        
        pub_rviz = rospy.Publisher("visualization_marker",Marker, queue_size=10);

        marker = Marker()
        marker.header.frame_id = "map";
        marker.header.stamp = rospy.Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        # marker.type = Marker.SPHERE;
        # marker.action = Marker.ADD;
        marker.pose.position.y = 0.1;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        # marker.scale.x = 1;
        # marker.scale.y = 0.1;
        # marker.scale.z = 0.1;
        # marker.color.a = 1.0; #Don't forget to set the alpha!
        # marker.color.r = 0.0;
        # marker.color.g = 1.0;
        # marker.color.b = 0.0;
        # //only if using a MESH_RESOURCE marker type:

        # marker.color.r = 1.0;
        # marker.color.g = 1.0;
        # marker.color.b = 1.0;

        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "package://rotors_description/meshes/firefly.dae"
        # marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

        while not rospy.is_shutdown():

            
            # WARNING: IT IS VERY IMPORTANT THAT U0, U1, U2 AND U3 ARE PROVIDED THIS WAY
            # I CANNOT PROVIDE SELF.U TO THE INTEGRATION BECAUSE IT IS CHANGING 
            # AND IT MESSES UP THE INTEGRATION!!! 
            # thid DOES NOT work: r.set_f_params(deepcopy(self.U),parameters)
            
            # we delay system the initialization of the system by a TimeDelay
            simtime = self.sim.get_time()
            simstate = self.sim.get_state()
            if (simtime >= self.TimeDelay and self.StartFlag):
                self.sim.run(1.0/self.frequency)
                # set dynamics vector accordinf to current input vector
                # input vector is assumed constant during integration
                #self.r.set_f_params(Input(self.U))
                #  integrate equation for period of loop
                #self.r.integrate(self.r.t + 1.0/self.frequency);
                # reset initial state and initial time
                #self.r.set_initial_value(self.r.y, self.r.t)
            else:
                # need to update initial state and time
                #self.r.set_initial_value(self.r.y, self.r.t + 1.0/self.frequency)
                self.sim.reset(
                    initial_time=simtime+1.0/self.frequency,
                    initial_state=simstate
                )
            
            # create a message of type quad_state with current state
            state = self.write_state()
            # publish current state
            # rospy.logwarn(state)
            # rospy.logwarn('aaaaaaaaaaaaaaaaaaaaaaaaa')
            pub.publish(state)

            # marker.pose.position.x = 0.1 + numpy.cos(2*3.14/5*rospy.get_time());
            marker.pose.position.x = simstate[0]
            marker.pose.position.y = simstate[1]
            marker.pose.position.z = simstate[2]

            # quaternion
            # marker.pose.orientation.x = simstate[0]
            # marker.pose.orientation.y = simstate[1]
            # marker.pose.orientation.z = simstate[2]
            # marker.pose.orientation.w = simstate[2]          

            pub_rviz.publish( marker )

            # rospy.logwarn(marker)

            # let node sleep
            rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()        


if __name__ == '__main__':
    simulator_node = SimulatorNode()
    simulator_node.simulate_quad()
