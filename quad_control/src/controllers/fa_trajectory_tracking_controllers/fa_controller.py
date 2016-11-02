#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
import numpy as np

from utilities import utility_functions as uts
from utilities import utility_functions

import rospy

from utilities import jsonable as js

class FAController(js.Jsonable):

    
    @classmethod
    def description(cls):
        return "Controller for **fully actuated** quadrotor model"
        
    def __str__(self):
        return self.description()
        
    def get_mass(self):
        return NotImplementedError()

    @classmethod
    def get_data_size(self):
        return 3

    def get_data(self):
        """Get all data relevant to the mission
        from this data, mission should be able to do data post-analysis, 
        like ploting or computing average errors
        """        
        default_array = self.disturbance_estimate.tolist()
        
        # default_array = np.concatenate([default_array,self.get_complementary_data()])
        return default_array

    # despite not saving anything, we can plot the rc commands
    # based on info saving by mission
    @classmethod
    def plot_from_string(cls, string,starting_point):

        #plots
        # import matplotlib.pyplot as plt
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import pyplot as plt

        times       = []
        
        disturbance_estimate_x = []
        disturbance_estimate_y = []
        disturbance_estimate_z = []

        for line in string.split('\n'):
            # ignore empty lines 
            if line:
                numbers = line.split(' ')

                # print(numbers)
                times.append(float(numbers[0]))

                numbers = numbers[starting_point[0]:]

                disturbance_estimate_x.append(float(numbers[0]))
                disturbance_estimate_y.append(float(numbers[1]))
                disturbance_estimate_z.append(float(numbers[2]))
                
                #yaw_rates.append(float(numbers[13]))

        # it is not bad to import it here, since plotting is done aposteriori
        import operator
        times = map(operator.sub,times,len(times)*[times[0]])

        fig1 = plt.figure()
        plt.plot(times, disturbance_estimate_x, 'r-', label=r'$F_x$')
        plt.plot(times, disturbance_estimate_y, 'g-', label=r'$F_y$')
        plt.plot(times, disturbance_estimate_z, 'b-', label=r'$F_z$')
        plt.title('Disturbance estimate')
        plt.legend(loc='best')
        plt.grid()

        # one element tuple
        return fig1,



@js.add_to_database(default=True)
class SimplePIDController(FAController):
    """ This is a dynamic controller, not a static controller """

    @classmethod
    def description(cls):
        return "PID Controller, with saturation on integral part"

    def __init__(self,              \
    proportional_gain_xy = rospy.get_param("proportional_gain_xy",6.0), \
    derivative_gain_xy   = rospy.get_param("derivative_gain_xy",2*1.6*numpy.sqrt(6.0)), \
    integral_gain_xy     = rospy.get_param("integral_gain_xy",0.0), \
    bound_integral_xy    = rospy.get_param("bound_integral_xy",0.0), \
    proportional_gain_z  = rospy.get_param("proportional_gain_z",1.0), \
    derivative_gain_z    = rospy.get_param("derivative_gain_z",numpy.sqrt(2*1.0)), \
    integral_gain_z      = rospy.get_param("integral_gain_z",0.0), \
    bound_integral_z     = rospy.get_param("bound_integral_z",0.0),
    quad_mass            = rospy.get_param("total_mass",1.442)
    ):

        self.proportional_gain_xy = proportional_gain_xy
        self.derivative_gain_xy   = derivative_gain_xy
        self.integral_gain_xy     = integral_gain_xy
        self.bound_integral_xy    = bound_integral_xy
        self.proportional_gain_z  = proportional_gain_z
        self.derivative_gain_z    = derivative_gain_z
        self.integral_gain_z      = integral_gain_z
        self.bound_integral_z     = bound_integral_z
        self.quad_mass            = quad_mass

        #TODO get from utilities?
        self.MASS = quad_mass
        self.GRAVITY = 9.81

        self.disturbance_estimate   = numpy.array([0.0,0.0,0.0])
        self.t_old  = 0.0

    def get_total_weight(self):
        return self.MASS*self.GRAVITY

    def object_description(self):
        string =  "<p>PID Controller, with saturation on integral part</p>"
        string += "<p>force(&#916t, p,pd) = " + str(self.quad_mass) +"*( pd<sup>(2)</sup> + u(p<sup>(0)</sup> - pd<sup>(0)</sup>,p<sup>(1)</sup> - pd<sup>(1)</sup>) + g e<sub>3</sub> - d<sup>est</sup>), where</p>"
        string +="<ul>"
        string += "<li>u<sub>xy</sub>(p,v) = -"+str(self.proportional_gain_xy)+"*p"+"-"+str(self.derivative_gain_xy)+"*v"+"</li>"
        string += "<li>u<sub>z</sub>(p,v) = -"+str(self.proportional_gain_z)+"*p"+"-"+str(self.derivative_gain_z)+"*v"+"</li>"
        string += "<li>d<sub>xy</sub><sup>est(1)</sup> = "+str(self.integral_gain_xy)+"*(kp/2*ep + ev)"+"</li>"
        string += "<li>|d<sub>xy</sub><sup>est(0)</sup>| &#8804 "+str(self.bound_integral_xy)+"</li>"        
        string += "<li>d<sub>z</sub><sup>est(1)</sup> = "+str(self.integral_gain_z)+"*(kp/2*ep + ev)"+"</li>"
        string += "<li>|d<sub>z</sub><sup>est(0)</sup>| &#8804 "+str(self.bound_integral_z)+"</li>"
        string +="</ul>"
        return string
        
    def __str__(self):
        #TODO add all the parameters and the state
        string = Controller.__str__(self)
        string += "\nProportional gain xy: " + str(self.proportional_gain_xy)
        string += "\nDerivative gain xy: " + str(self.derivative_gain_xy)
        return string


    def output(self, delta_t, state, reference):

        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])        
        
        #--------------------------------------#
        # position and velocity
        x  = state[0:3]
        v  = state[3:6]
        # thrust unit vector and its angular velocity
        # R  = state[6:15]; R  = numpy.reshape(R,(3,3))

        #--------------------------------------#
        # desired quad trajectory
        xd = reference[0:3]
        vd = reference[3:6]
        ad = reference[6:9]
        
        #--------------------------------------#
        # position error and velocity error
        ep = x - xd
        ev = v - vd

        u, V_v = self.input_and_gradient_of_lyapunov(ep,ev)

        Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3 - self.disturbance_estimate)

        # -----------------------------------------------------------------------------#
        # update disturbance estimate

        gains_integral_action    = numpy.array([self.integral_gain_xy ,self.integral_gain_xy ,self.integral_gain_z ])
        max_disturbance_estimate = numpy.array([self.bound_integral_xy,self.bound_integral_xy,self.bound_integral_z])

        # derivatice of disturbance estimate (ELEMENT WISE PRODUCT)
        

        # d_est_dot  = gains_integral_action*V_v
        # # new disturbance estimate
        # t_new = delta_t
        # disturbance_estimate = self.disturbance_estimate + d_est_dot*(t_new - self.t_old) 
        # # saturate estimate just for safety (element wise bound)
        # self.disturbance_estimate = numpy.clip(disturbance_estimate,max_disturbance_estimate,-1.0*max_disturbance_estimate)
        # # update old time
        # self.t_old = t_new
        # -----------------------------------------------------------------------------#

        return Full_actuation


    def input_and_gradient_of_lyapunov(self,ep,ev):

        u    = numpy.array([0.0,0.0,0.0])
        V_v  = numpy.array([0.0,0.0,0.0])

        kp     = self.proportional_gain_xy
        kv     = self.derivative_gain_xy
        u[0]   = -kp*ep[0] - kv*ev[0]
        u[1]   = -kp*ep[1] - kv*ev[1]
        V_v[0] = (kp/2*ep[0] + ev[0])
        V_v[1] = (kp/2*ep[1] + ev[1])

        kp     = self.proportional_gain_z
        kv     = self.derivative_gain_z
        u[2]   = -kp*ep[2] - kv*ev[2]
        V_v[2] = (kp/2*ep[2] + ev[2])

        return u, V_v


    def reset_estimate_xy(self):
        self.disturbance_estimate[0] = 0.0
        self.disturbance_estimate[1] = 0.0
        return


    def reset_estimate_z(self):
        self.disturbance_estimate[2] = 0.0
        return         

@js.add_to_database()
class NeutalController(FAController):
    """ This is a dynamic controller, not a static controller """

    @classmethod
    def description(cls):
        return "Do nothing"

    def __init__(self,              \
    quad_mass = rospy.get_param("quadrotor_mass",1.442)
    ):
        #TODO get from utilities?
        self.MASS = quad_mass
        self.GRAVITY = 9.81

        self.disturbance_estimate   = numpy.array([0.0,0.0,0.0])

    def get_total_weight(self):
        return self.MASS*self.GRAVITY

    def object_description(self):
        string =  "No nothing"
        return string
        
    def output(self, delta_t, state, reference):
        '''No actuation'''

        Full_actuation = numpy.zeros(3)
        return Full_actuation
    
# Test
#string = SimpleBoundedIntegralPIDController.to_string()
#print string
#con = SimpleBoundedIntegralPIDController.from_string(string)
#print con
#print con.output(0.1, numpy.zeros(9), numpy.zeros(9))


# import controllers.double_integrator_controllers.double_integrator_controller
# CONTROLLERS_DATABASE = controllers.double_integrator_controllers.double_integrator_controller.database

# # @js.inherit_methods_list_from_parents
# @js.add_to_database()
# class ThreeDPIDController(FAController):
#     """This is a dynamic controller, not a static controller"""

#     js.Jsonable.add_inner('double_integrator_controller',CONTROLLERS_DATABASE)
    
#     @classmethod
#     def description(cls):
#         return "PID Controller, with saturation on integral part"
        
#     def __init__(self,\
#             integral_gain_xy     = 0.0        ,\
#             bound_integral_xy    = 0.0        ,\
#             integral_gain_z      = 0.5        ,\
#             bound_integral_z     = 0.0        ,\
#             quad_mass            = rospy.get_param("total_mass",1.442)
#             ):

#         self.add_inner_defaults()

#         self.integral_gain_xy     = integral_gain_xy
#         self.bound_integral_xy    = bound_integral_xy

#         self.integral_gain_z      = integral_gain_z
#         self.bound_integral_z     = bound_integral_z

#         #TODO should these two be inherited by a parent instead?
#         # Should the mass be passed as a parameter?
#         # We can always use 1.66779 as a default value
#         self.quad_mass = quad_mass
        
#         self.MASS = quad_mass
#         self.GRAVITY = 9.81

#         self.disturbance_estimate   = numpy.array([0.0,0.0,0.0])
#         self.d_est = self.disturbance_estimate

#         self.t_old  = 0.0

#         pass

#     def get_total_weight(self):
#         return self.MASS*self.GRAVITY

#     def __str__(self):
#         #TODO add the remaining parameters
#         string = Controller.__str__(self)
#         string += "\nDouble-integrator controller: " + str(self.double_integrator_controller)
#         return string
        

#     def output(self, delta_t, state, reference):

#         # third canonical basis vector
#         e3 = numpy.array([0.0,0.0,1.0])        
        
#         #--------------------------------------#
#         # position and velocity
#         x  = state[0:3]; v  = state[3:6]

#         #--------------------------------------#
#         # desired quad trajectory
#         xd = reference[0:3];
#         vd = reference[3:6];
#         ad = reference[6:9];
        
#         #--------------------------------------#
#         # position error and velocity error
#         ep = x - xd
#         ev = v - vd

#         u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.double_integrator_controller.output(ep,ev)

#         Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3 - self.d_est)


#         # -----------------------------------------------------------------------------#
#         # update disturbance estimate

#         gains_integral_action    = numpy.array([self.integral_gain_xy ,self.integral_gain_xy ,self.integral_gain_z ])
#         max_disturbance_estimate = numpy.array([self.bound_integral_xy,self.bound_integral_xy,self.bound_integral_z])

#         # derivatice of disturbance estimate (ELEMENT WISE PRODUCT)
        

#         # d_est_dot  = gains_integral_action*V_v
#         # # new disturbance estimate
#         # t_new = delta_t
#         # disturbance_estimate = self.disturbance_estimate + d_est_dot*(t_new - self.t_old) 
#         # # saturate estimate just for safety (element wise bound)
#         # self.disturbance_estimate = numpy.clip(disturbance_estimate,max_disturbance_estimate,-1.0*max_disturbance_estimate)
#         # # update old time
#         # self.t_old = t_new

#         return Full_actuation


#     def reset_disturbance_estimate(self):
#         self.disturbance_estimate = numpy.array([0.0,0.0,0.0])
#         return


import controllers.double_integrator_controllers.double_integrator_controller
CONTROLLERS_DATABASE = controllers.double_integrator_controllers.double_integrator_controller.database
CONTROLLERS_DATABASE_ONE_D = controllers.double_integrator_controllers.double_integrator_controller.database_one_d

@js.add_to_database()
class ControllerPIDXYAndZBounded(FAController):
    """This is a dynamic controller, not a static controller"""

    js.Jsonable.add_inner('double_integrator_xy',CONTROLLERS_DATABASE)
    js.Jsonable.add_inner('double_integrator_z',CONTROLLERS_DATABASE_ONE_D)

    @classmethod
    def description(cls):
        string = "PID Controller, with saturation on integral part."
        string+= "<ul>"
        string+= "<li>double_integrator_xy: controller for xy</li>"
        string+= "<li>double_integrator_xy: controller for z</li>"
        string+= "</ul>"
        return string

    def __init__(self,\
    integral_gain_xy     = rospy.get_param("integral_gain_xy",0.0), \
    bound_integral_xy    = rospy.get_param("bound_integral_xy",0.0), \
    integral_gain_z      = rospy.get_param("integral_gain_z",0.0), \
    bound_integral_z     = rospy.get_param("bound_integral_z",0.0),
    quad_mass            = rospy.get_param("total_mass",1.442)
    ):

        self.add_inner_defaults()

        self.integral_gain_xy     = integral_gain_xy
        self.bound_integral_xy    = bound_integral_xy

        self.integral_gain_z      = integral_gain_z
        self.bound_integral_z     = bound_integral_z

        #TODO should these two be inherited by a parent instead?
        # Should the mass be passed as a parameter?
        # We can always use 1.66779 as a default value
        self.quad_mass = quad_mass
        
        self.MASS = quad_mass
        self.GRAVITY = 9.81

        self.disturbance_estimate   = numpy.array([0.0,0.0,0.0])

        # self.t_old  = 0.0


    def object_description(self):
        string = """
        Parameters:
        <ul>
          <li>integral_gain_xy: """ + str(self.integral_gain_xy) +"""(N)</li>
          <li>bound_integral_xy: """ + str(self.bound_integral_xy) +"""</li>
          <li>integral_gain_z: """ + str(self.integral_gain_z) +"""</li>
          <li>bound_integral_z: """ + str(self.bound_integral_z) +"""(N)</li>
          <li>quad_mass: """ + str(self.quad_mass) +"""</li>
        </ul>
        """
        return string

    def get_total_weight(self):
        return self.MASS*self.GRAVITY

    @js.add_to_methods_list
    def reset_estimate_xy(self):
        self.disturbance_estimate[0] = 0.0
        self.disturbance_estimate[1] = 0.0
        return

    @js.add_to_methods_list
    def reset_estimate_z(self):
        self.disturbance_estimate[2] = 0.0
        return     

    def update_parameters(self,parameters):
        self.parameters = parameters


    def output(self, time_instant, states, states_d):
        # convert euler angles from deg to rotation matrix
        ee = states[6:9]
        R  = uts.rot_from_euler_deg(ee)
        R  = numpy.reshape(R,9)
        # collecting states
        states  = numpy.concatenate([states[0:6],R])

        # initiliaze initial time instant
        self.t_old = time_instant

        self.output = self.output_after_initialization

        return self.controller(0.0,states,states_d)

    def output_after_initialization(self, time_instant, states, states_d):
        # convert euler angles from deg to rotation matrix
        ee = states[6:9]
        R  = uts.rot_from_euler_deg(ee)
        R  = numpy.reshape(R,9)
        # collecting states
        states  = numpy.concatenate([states[0:6],R])

        delta_t = time_instant - self.t_old

        output = self.controller(delta_t,states,states_d)

        self.t_old = time_instant

        return output

    # Controller
    def controller(self,delta_t,states,states_d):
         
        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])        
        
        #--------------------------------------#
        # position and velocity
        x  = states[0:3]; v  = states[3:6]
        # thrust unit vector and its angular velocity
        # R  = states[6:15]; R  = numpy.reshape(R,(3,3))

        #--------------------------------------#
        # desired quad trajectory
        xd = states_d[0:3];
        vd = states_d[3:6];
        ad = states_d[6:9];
        
        #--------------------------------------#
        # position error and velocity error
        ep = x - xd
        ev = v - vd

        u,V_v = self.input_and_gradient_of_lyapunov(ep,ev)

        Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3 - self.disturbance_estimate)

        # -----------------------------------------------------------------------------#
        # update disturbance estimate

        gains_integral_action    = numpy.array([self.integral_gain_xy ,self.integral_gain_xy ,self.integral_gain_z ])
        max_disturbance_estimate = numpy.array([self.bound_integral_xy,self.bound_integral_xy,self.bound_integral_z])


        disturbance_estimate_dot  = gains_integral_action*V_v
        # new disturbance estimate
        disturbance_estimate = self.disturbance_estimate + disturbance_estimate_dot*delta_t
        # saturate estimate just for safety (element wise bound)
        self.disturbance_estimate = numpy.clip(disturbance_estimate,-1.0*max_disturbance_estimate,max_disturbance_estimate)

        return Full_actuation

    def input_and_gradient_of_lyapunov(self,ep,ev):

        u_xy,u_p_xy,u_v_xy,u_p_p_xy,u_v_v_xy,u_p_v_xy,Vpv_xy,VpvD_xy,V_p_xy,V_v_xy,V_v_p_xy,V_v_v_xy = self.double_integrator_xy.output(ep[0:2],ev[0:2])
        u_z,u_p_z,u_v_z,u_p_p_z,u_v_v_z,u_p_v_z,Vpv_z,VpvD_z,V_p_z,V_v_z,V_v_p_z,V_v_v_z = self.double_integrator_z.output(ep[2],ev[2])

        u   = numpy.concatenate([u_xy,[u_z]])
        V_v = numpy.concatenate([V_v_xy,[V_v_z]])

        return (u,V_v)