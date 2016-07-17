#!/usr/bin/env python
# this line is just used to define the type of document

# for ploting
import matplotlib.pyplot as plt

import numpy

import rospy

from utilities import jsonable as js

# import utilities.utility_functions as uts

class PlanXYMotionLoadLifting(js.Jsonable):

    @classmethod
    def description(cls):
        return '''
        Plan xy trajectory of uav that removes oscillations
        '''


def sat(x):
    return 0.1*x/numpy.sqrt(x**2 + 0.1**2)

# def sat(x):
#     return x

@js.add_to_database(default=True)
class PlanXYMotionLoadLiftingQI(PlanXYMotionLoadLifting):   
    '''Plan xy trajectory of uav that removes oscillations
    '''

    @classmethod
    def description(cls):
        return '''
        Plan xy trajectory of uav that removes oscillations
        '''


    def __init__(self,
    gains = numpy.array([-1, -6, 0, 1]),
    cable_length=rospy.get_param("cable_length",0.6)):

        self.gains = gains
        self.gravity = 9.81
        self.cable_length = cable_length

        # initialize the memory
        self.position_xy_desired = numpy.array([0.0,0.0])
        self.velocity_xy_desired = numpy.array([0.0,0.0])

    def object_descrition(self):
        description = ''
        return description

    def output(self,state,reference,Dt):

        self.position_xy_desired = reference['position']
        self.velocity_xy_desired = reference['velocity']
        reference['acceleration'] = numpy.array([0.0,0.0])

        self.output = self.output_after_initialization

        return reference

    def output_after_initialization(self,state,reference,Dt):

        p_old = self.position_xy_desired
        v_old = self.velocity_xy_desired

        p_star_tilde = reference['position']
        v_star_tilde = reference['velocity']

        # if numpy.linalg.norm(p_old - p_star_tilde) + numpy.linalg.norm(v_old - v_star_tilde) < 0.3:
        if numpy.linalg.norm(p_old - p_star_tilde) < 0.2:
            
            # direction of cable
            n = state['unit_vector']
            phi   =  -numpy.arcsin(n[[1]])
            theta =  numpy.arctan(n[[0]]/n[[2]])

            # angular velocity of cable
            w  = state['angular_velocity']
            w_phi   = w[0]/numpy.cos(theta) - w[1]*numpy.tan(phi)*numpy.tan(theta)
            w_theta = w[1]/(numpy.cos(theta)**2)

            p  = sat(p_old[0] - p_star_tilde[0])
            v  = sat(v_old[0] - v_star_tilde[0])
            tt = theta
            w  = w_theta
            ux = self.compute_one_d_acceleration(p,v,tt,w)

            px_new = p_star_tilde[0] + p + v*Dt + 0.5*ux*(Dt**2)
            vx_new = v_star_tilde[0] + v + ux*Dt

            p  = -sat(p_old[1] - p_star_tilde[1])
            v  = -sat(v_old[1] - v_star_tilde[1])
            tt = phi
            w  = w_phi
            uy = -self.compute_one_d_acceleration(p,v,tt,w)

            py_new = p_star_tilde[1] - (p + v*Dt + 0.5*(-uy)*(Dt**2))
            vy_new = v_star_tilde[1] - (v + (-uy)*Dt)

            p_new = numpy.array([px_new,py_new])
            v_new = numpy.array([vx_new,vy_new])
            u = numpy.array([ux,uy])

            # p_new = p_star_tilde + p_old + v_old*Dt + 0.5*u*(Dt**2)
            # v_new = v_star_tilde + u*Dt

            reference = {}
            # array is not mutable
            reference['position'] = p_new
            reference['velocity'] = v_new
            reference['acceleration'] = 0.0*numpy.array([ux,uy])

            # reference['position'] = p_new
            # reference['velocity'] = v_new
            # reference['acceleration'] = numpy.array([ux,uy])

            self.position_xy_desired = p_new
            self.velocity_xy_desired = v_new

        else:
            self.position_xy_desired = reference['position']
            self.velocity_xy_desired = reference['velocity']
            reference['acceleration'] = numpy.array([0.0,0.0])

        return reference


    def compute_one_d_acceleration(self,p,v,tt,w):
        '''p = position'''

        g  = self.gravity
        L  = self.cable_length

        xi = numpy.array([p - L*tt,v - L*w,g*tt,g*w])

        #gains = numpy.array([-2.0,-6.0,-7.0,-4.0])
        gains = self.gains
        v     = numpy.dot(gains,xi)

        acceleration = 0.0*xi[2] + L/g*v[0]

        return acceleration[0]


    @classmethod
    def get_data_size(self):
        return 2+2

    def get_data(self):
        """Get all data relevant to the mission
        from this data, mission should be able to do data post-analysis, 
        like ploting or computing average errors
        """        
        default_array =self.position_xy_desired.tolist()
        default_array+=self.velocity_xy_desired.tolist()
        return default_array


    @classmethod
    def plot_from_string(cls, string,starting_point):
        
        times        = []
        
        positions_x  = []
        positions_y  = []

        velocities_x = []
        velocities_y = []

        for line in string.split('\n'):
            # ignore empty lines           
            if line:
                numbers = line.split(' ')

                times.append(float(numbers[0]))

                numbers = numbers[starting_point[0]:]
                
                positions_x.append(float(numbers[0]))
                positions_y.append(float(numbers[1]))
                
                velocities_x.append(float(numbers[2]))
                velocities_y.append(float(numbers[3]))

        fig1 = plt.figure()
        plt.plot(times, positions_x, 'r-', label=r'$x$')
        plt.plot(times, positions_y, 'g-', label=r'$y$')      
        plt.title('Planned xy positions (m)')
        plt.legend(loc='best')
        plt.grid()

        fig2 = plt.figure()
        plt.plot(times, velocities_x, 'r-', label=r'$v_x$')
        plt.plot(times, velocities_y, 'g-', label=r'$v_y$')    
        plt.title('Planned xy velocities (m/s)')
        plt.legend(loc='best')
        plt.grid()

        return fig1,fig2