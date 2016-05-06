#!/usr/bin/env python

from utilities import jsonable as js
from utilities import utility_functions

import numpy

# for getting time
import rospy

#plots
import matplotlib.pyplot as plt

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
        'roll',
        'pitch',
        'yaw',
        'velocity_x',
        'velocity_y',
        'velocity_z',
        'control_x',
        'control_y',
        'control_z'
    ]


    @classmethod
    def plot_from_string(cls, string):
        
        times = []
        positions_x = []
        positions_y = []
        positions_z = []
        rolls = []
        pithces = []
        yaws = []
        velocities_x = []
        velocities_y = []
        velocities_z = []
        controls_x = []
        controls_y = []
        controls_z = []
        yaw_rates = []
        
        for line in string.split('\n'):
            numbers = line.split(' ')
            times.append(float(numbers[0]))
            positions_x.append(float(numbers[1]))
            positions_y.append(float(numbers[2]))
            positions_z.append(float(numbers[3]))
            rolls.append(float(numbers[4]))
            pitches.append(float(numbers[5]))
            yaws.append(float(numbers[6]))
            velocities_x.append(float(numbers[7]))
            velocities_y.append(float(numbers[8]))
            velocities_z.append(float(numbers[9]))
            controls_x.append(float(numbers[10]))
            controls_y.append(float(numbers[11]))
            controls_z.append(float(numbers[12]))
            yaw_rates.append(float(numbers[13]))
        
        plt.figure()
        plt.plot(times, positions_x, label=r'$x$')
        plt.plot(times, positions_y, label=r'$y$')
        plt.plot(times, positions_z, label=r'$z$')
        plt.title('Positions')
        plt.legend(pos='best')
        plt.grid()
        plt.draw()
        
        plt.figure()
        plt.plot(times, rolls, label=r'$\phi$')
        plt.plot(times, pitches, label=r'$\theta$')
        plt.plot(times, yaws, label=r'$\psi$')
        plt.title('Attitudes')
        plt.legend(pos='best')
        plt.grid()
        plt.draw()
        
        plt.figure()
        plt.plot(times, velocities_x, label=r'$x$')
        plt.plot(times, velocities_y, label=r'$y$')
        plt.plot(times, velcocities_z, label=r'$z$')
        plt.title('Velocities')
        plt.legend(pos='best')
        plt.grid()
        plt.draw()
        
        plt.figure()
        plt.plot(times, controls_x, label=r'$x$')
        plt.plot(times, controls_y, label=r'$y$')
        plt.plot(times, controls_z, label=r'$z$')
        plt.title('Controls')
        plt.legend(pos='best')
        plt.grid()
        plt.draw()


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

        pass
        
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        pass


    def initialize_state(self):
        raise NotImplementedError()


    def real_publish(self,desired_3d_force_quad,yaw_rate):
        return NotImplementedError()


    def get_quad_ea_rad(self):
        '''Get euler angles (roll, pitch, yaw) in radians,
        and get their time derivatives'''
        # return numpy.zeros(3+3)
        NotImplementedError()


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


    def get_rc_output(self):
        """Get rc output"""
        return numpy.zeros(8)    


    def get_euler_angles(self):
        # TODO: CHECK WHETHER IT SHOULD BE RAD OR NOT
        """Get euler angles of the quadrotor (rad)"""
        return NotImplementedError()


    def change_reference(self,key,string):
        """Change reference trajectory"""
        if key in self.inner['reference'].keys():
            TrajectoryClass   = self.inner['reference'][key]
            self.TrajGenerator = TrajectoryClass.from_string(string)


    def change_yaw_reference(self,key,string):
        """Change yaw reference trajectory"""
        if key in self.inner['yaw_reference'].keys():
            YawTrajectoryClass   = self.inner['yaw_reference'][key]
            self.YawTrajGenerator = YawTrajectoryClass.from_string(string)


    def change_controller(self, key, string):
        """Change controller"""
        if key in self.inner['controller'].keys():
            ControllerClass       = self.inner['controller'][key]
            self.ControllerObject = ControllerClass.from_string(string)


    def change_yaw_controller(self,key,string):
        """Change yaw controller"""
        if key in self.inner['yaw_controller'].keys():
            YawControllerClass      = self.inner['yaw_controller'][key]
            self.YawControllerObject = YawControllerClass.from_string(string)


    def reset_initial_time(self,time_instant = None):
        if time_instant == None:
            self.time_instant_t0 = rospy.get_time()
        else:
            self.time_instant_t0 = time_instant

   
    def yaw_rate(self,time_instant):

        state_for_yaw_controller = self.get_quad_ea_rad()
        input_for_yaw_controller = self.get_desired_yaw_rad(time_instant)

        yaw_rate = self.YawControllerObject.output(state_for_yaw_controller,
            input_for_yaw_controller) 

        return yaw_rate


    def compute_desired_3d_force(self,time_instant):

        # reference
        reference = self.get_reference(time_instant)

        # state
        state = self.get_state()

        # compute input to send to QUAD
        desired_3d_force_quad = self.ControllerObject.output(time_instant,
            state, reference)

        return desired_3d_force_quad


    def publish(self):

        time_instant = rospy.get_time() - self.time_instant_t0

        desired_3d_force_quad = self.compute_desired_3d_force(time_instant)
            
        self.DesiredZForceMedian.update_data(desired_3d_force_quad[2])

        yaw_rate = self.yaw_rate(time_instant)

        self.real_publish(desired_3d_force_quad,yaw_rate)
