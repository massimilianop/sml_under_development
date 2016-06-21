#!/usr/bin/env python

from utilities import jsonable as js
from utilities import utility_functions

import numpy

# for getting time
import rospy

#plots
# import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

# The children import needed dictionaries

# For example
# from controllers_hierarchical import controllers_dictionary as cd
# from TrajectoryPlanner import trajectories_dictionary as td 

# If the children need to subscribe to some topic,
# or to publish to some topic,
# then they import rospy and the needed message types.

# For example
# import rospy as rp
# from mavros.mavros_msgs.msg import OverrideRCIn

# If the mocap is needed, the module mocap must be imported
# import mocap 

#import math

# import converter from 3d_force and yaw rate into iris rc standard 
from converters.iris_plus_converter import IrisPlusConverter

class Mission(js.Jsonable):

    inner = {
        # For example: controller, reference, etc.
        'controller'     : {},        
        'yaw_controller' : {},
        'reference'      : {},
        'yaw_reference'  : {},
    } 

    # time_instant_t0 = 0.0

    """Labels for the columns of a file that stores data from this mission."""
    file_labels = [
        'time',
        'position_x',
        'position_y',
        'position_z',
        'velocity_x',
        'velocity_y',
        'velocity_z',
        'position_x_desired',
        'position_y_desired',
        'position_z_desired',
        'velocity_x_desired',
        'velocity_y_desired',
        'velocity_z_desired',        
        'roll',
        'pitch',
        'yaw',
        'force_x',
        'force_y',
        'force_z'
    ]

    def get_complete_data(self):
        """Get all data relevant to the mission
        from this data, mission should be able to do data post-analysis, 
        like ploting or computing average errors
        """        
        default_array = numpy.concatenate([
            [rospy.get_time()],
            self.get_pv(),
            self.get_pv_desired(),
            self.get_euler_angles(),
            self.desired_3d_force_quad])
        
        # default_array = numpy.concatenate([default_array,self.get_complementary_data()])
        return default_array    


    @classmethod
    def plot_from_string(cls, string):
        
        times        = []
        
        positions_x  = []
        positions_y  = []
        positions_z  = []

        velocities_x = []
        velocities_y = []
        velocities_z = []

        positions_x_d  = []
        positions_y_d  = []
        positions_z_d  = []

        velocities_x_d = []
        velocities_y_d = []
        velocities_z_d = []

        rolls        = []
        pitches      = []
        yaws         = []

        forces_x   = []
        forces_y   = []
        forces_z   = []
        

        for line in string.split('\n'):
            # ignore empty lines           
            if line:
                numbers = line.split(' ')
                
                times.append(float(numbers[0]))

                positions_x.append(float(numbers[1]))
                positions_y.append(float(numbers[2]))
                positions_z.append(float(numbers[3]))
                
                velocities_x.append(float(numbers[4]))
                velocities_y.append(float(numbers[5]))
                velocities_z.append(float(numbers[6]))

                positions_x_d.append(float(numbers[7]))
                positions_y_d.append(float(numbers[8]))
                positions_z_d.append(float(numbers[9]))
                
                velocities_x_d.append(float(numbers[10]))
                velocities_y_d.append(float(numbers[11]))
                velocities_z_d.append(float(numbers[12]))

                rolls.append(float(numbers[13]))
                pitches.append(float(numbers[14]))
                yaws.append(float(numbers[15]))
                
                forces_x.append(float(numbers[16]))
                forces_y.append(float(numbers[17]))
                forces_z.append(float(numbers[18]))
                
                #yaw_rates.append(float(numbers[13]))
        
        fig1 = plt.figure()
        plt.plot(times, positions_x, 'r-', label=r'$x$')
        plt.plot(times, positions_y, 'g-', label=r'$y$')
        plt.plot(times, positions_z, 'b-', label=r'$z$')
        plt.plot(times, positions_x_d, 'r--', label=r'$x^{\star}$')
        plt.plot(times, positions_y_d, 'g--', label=r'$y^{\star}$')
        plt.plot(times, positions_z_d, 'b--', label=r'$z^{\star}$')        
        plt.title('Positions (m)')
        plt.legend(loc='best')
        plt.grid()

        fig2 = plt.figure()
        plt.plot(times, velocities_x, 'r-', label=r'$v_x$')
        plt.plot(times, velocities_y, 'g-', label=r'$v_y$')
        plt.plot(times, velocities_z, 'b-', label=r'$v_z$')
        plt.plot(times, velocities_x_d, 'r--', label=r'$v_x^{\star}$')
        plt.plot(times, velocities_y_d, 'g--', label=r'$v_y^{\star}$')
        plt.plot(times, velocities_z_d, 'b--', label=r'$v_z^{\star}$')        
        plt.title('Velocities (m/s)')
        plt.legend(loc='best')
        plt.grid()        
        
        fig3 = plt.figure()
        plt.plot(times, rolls, 'r-', label=r'$\phi$')
        plt.plot(times, pitches, 'g-', label=r'$\theta$')
        plt.plot(times, yaws, 'b-', label=r'$\psi$')
        plt.title('Attitudes (deg)')
        plt.legend(loc='best')
        plt.grid()
        # plt.draw()
        
        fig4 = plt.figure()
        plt.plot(times, forces_x, 'r-', label=r'$F_x$')
        plt.plot(times, forces_y, 'g-', label=r'$F_y$')
        plt.plot(times, forces_z, 'b-', label=r'$F_z$')
        plt.title('Forces (N)')
        plt.legend(loc='best')
        plt.grid()
        # plt.draw()

        return fig1,fig2,fig3,fig4


    def description(cls):
        return "Abstract Mission"
    
    
    def __init__(self):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        # initialize initial time
        self.reset_initial_time()         
        
        # initialize state: ultimately defined by child class
        self.initialize_state()

        # for reseting neutral value that makes iris+ stay at desired altitude
        self.DesiredZForceMedian = utility_functions.MedianFilter(10)
              
        self.rc_output = numpy.zeros(4)

        # converting our controlller standard into iris+ standard
        self.iris_plus_converter_object_mission = IrisPlusConverter()

        # initialize desired_3d_force
        self.desired_3d_force_quad = numpy.zeros(3)

        return
        
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        pass


    def initialize_state(self):
        raise NotImplementedError()


    def real_publish(self,desired_3d_force_quad,yaw_rate,rc_output):
        return NotImplementedError()


    def get_quad_ea_rad(self):
        '''Get euler angles (roll, pitch, yaw) in radians,
        and get their time derivatives'''
        # return numpy.zeros(3+3)
        return NotImplementedError()


    def get_desired_yaw_rad(self,time_instant):
        '''Get desired yaw in radians, and its time derivative'''
        return numpy.zeros(1+1)
        #return NotImplementedError()


    def get_reference(self,time_instant):
        """
        Get reference that must be accepted by controller
        A reference might not exist
        """
        return None
  
    def get_state(self):
        """
        Get state that is accepted by controller.
        It is the job of a child class to write a state
        that is accepted by the controllers that it imports. 
        """
        return NotImplementedError()


    #TODO: error here somewhere
    def get_state_string(self):
        """In the main cycle, call this function to get a line that can be
        written on a file:
        filehandle.write(mission.get_state_string())
        """
        lst = list(self.get_state())
        string = ""
        for element in lst:
            string.append(str(element)+" ")
        return string 


    def get_pv(self):
        """Get position (m) and velocity (m/s) of quadrotor"""
        return NotImplementedError()


    def get_pv_desired(self):
        """Get desired position (m) and velocity (m/s) for the quadrotor"""
        return NotImplementedError()

    def get_ea_desired(self):
        """Get desired euler angles in degrees for UAV"""
        return numpy.array([0.0,0.0,0.0])
        # return NotImplementedError()        

    def get_rc_output(self):
        """Get rc output"""
        return numpy.zeros(8)


    def get_euler_angles(self):
        # TODO: CHECK WHETHER IT SHOULD BE RAD OR NOT
        """Get euler angles of the quadrotor (rad)"""
        return NotImplementedError()

    def get_complementary_data(self):
        """Get complementary data that is not included 
        in get_complete_data() already
        """
        # by default return empty array
        # return numpy.array([])
        return NotImplementedError()

    def get_labels_complementary_data(self):
        """Get labels of the complementary data as defined in get_complementary_data()
        """
        # by default return empty list of labels
        # return []
        return NotImplementedError()

    def reset_initial_time(self,time_instant = None):
        if time_instant == None:
            # TODO
            # self.time_instant_t0 = rospy.get_time()
            self.time_instant_t0 = 0
        else:
            self.time_instant_t0 = time_instant

   
    def yaw_rate(self,time_instant):

        state_for_yaw_controller = self.get_quad_ea_rad()
        input_for_yaw_controller = self.get_desired_yaw_rad(time_instant)

        yaw_rate = self.yaw_controller.output(state_for_yaw_controller,
            input_for_yaw_controller)

        return yaw_rate


    def compute_desired_3d_force(self,time_instant):

        # reference
        reference = self.get_reference(time_instant)

        # state
        state = self.get_state()

        # compute input to send to QUAD
        desired_3d_force_quad = self.controller.output(time_instant,
            state, reference)

        return desired_3d_force_quad


    def publish(self):

        time_instant = rospy.get_time() - self.time_instant_t0

        desired_3d_force_quad = self.compute_desired_3d_force(time_instant)

        self.desired_3d_force_quad = desired_3d_force_quad

        self.DesiredZForceMedian.update_data(desired_3d_force_quad[2])

        yaw_rate = self.yaw_rate(time_instant)

        self.rc_command(desired_3d_force_quad,yaw_rate)

        self.real_publish(desired_3d_force_quad,yaw_rate,self.rc_output)


    def rc_command(self,desired_3d_force_quad,yaw_rate):
        euler_rad          = self.get_euler_angles()*numpy.pi/180
        self.iris_plus_converter_object_mission.set_rotation_matrix(euler_rad)
        iris_plus_rc_input = self.iris_plus_converter_object_mission.input_conveter(desired_3d_force_quad,yaw_rate)
        self.rc_output     = iris_plus_rc_input