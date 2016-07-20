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


@js.add_to_database(default=True)
class SimplePIDController(FAController):
    """ This is a dynamic controller, not a static controller """

    @classmethod
    def description(cls):
        return "PID Controller, with saturation on integral part"

    def __init__(self,              \
        proportional_gain_xy = 1.0, \
        derivative_gain_xy   = numpy.sqrt(2*1.0), \
        integral_gain_xy     = 0.0, \
        bound_integral_xy    = 0.0, \
        proportional_gain_z  = 1.5, \
        derivative_gain_z    = numpy.sqrt(2*1.5), \
        integral_gain_z      = 0.5, \
        bound_integral_z     = 0.0,
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
        

        d_est_dot  = gains_integral_action*V_v
        # new disturbance estimate
        t_new = delta_t
        disturbance_estimate = self.disturbance_estimate + d_est_dot*(t_new - self.t_old) 
        # saturate estimate just for safety (element wise bound)
        self.disturbance_estimate = numpy.clip(disturbance_estimate,max_disturbance_estimate,-1.0*max_disturbance_estimate)
        # update old time
        self.t_old = t_new
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


import controllers.double_integrator_controllers.double_integrator_controller
CONTROLLERS_DATABASE = controllers.double_integrator_controllers.double_integrator_controller.database

# @js.inherit_methods_list_from_parents
@js.add_to_database()
class ThreeDPIDController(FAController):
    """This is a dynamic controller, not a static controller"""

    js.Jsonable.add_inner('double_integrator_controller',CONTROLLERS_DATABASE)
    
    @classmethod
    def description(cls):
        return "PID Controller, with saturation on integral part"
        
    def __init__(self,\
            integral_gain_xy     = 0.0        ,\
            bound_integral_xy    = 0.0        ,\
            integral_gain_z      = 0.5        ,\
            bound_integral_z     = 0.0        ,\
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
        self.d_est = self.disturbance_estimate

        self.t_old  = 0.0

        pass

    def get_total_weight(self):
        return self.MASS*self.GRAVITY

    def __str__(self):
        #TODO add the remaining parameters
        string = Controller.__str__(self)
        string += "\nDouble-integrator controller: " + str(self.double_integrator_controller)
        return string
        

    def output(self, delta_t, state, reference):

        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])        
        
        #--------------------------------------#
        # position and velocity
        x  = state[0:3]; v  = state[3:6]

        #--------------------------------------#
        # desired quad trajectory
        xd = reference[0:3];
        vd = reference[3:6];
        ad = reference[6:9];
        
        #--------------------------------------#
        # position error and velocity error
        ep = x - xd
        ev = v - vd

        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.double_integrator_controller.output(ep,ev)

        Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3 - self.d_est)


        # -----------------------------------------------------------------------------#
        # update disturbance estimate

        gains_integral_action    = numpy.array([self.integral_gain_xy ,self.integral_gain_xy ,self.integral_gain_z ])
        max_disturbance_estimate = numpy.array([self.bound_integral_xy,self.bound_integral_xy,self.bound_integral_z])

        # derivatice of disturbance estimate (ELEMENT WISE PRODUCT)
        

        d_est_dot  = gains_integral_action*V_v
        # new disturbance estimate
        t_new = delta_t
        disturbance_estimate = self.disturbance_estimate + d_est_dot*(t_new - self.t_old) 
        # saturate estimate just for safety (element wise bound)
        self.disturbance_estimate = numpy.clip(disturbance_estimate,-1.0*max_disturbance_estimate,max_disturbance_estimate)
        # update old time
        self.t_old = t_new

        return Full_actuation


    def reset_disturbance_estimate(self):
        self.disturbance_estimate = numpy.array([0.0,0.0,0.0])
        return


import controllers.double_integrator_controllers.double_integrator_controller
CONTROLLERS_DATABASE = controllers.double_integrator_controllers.double_integrator_controller.database

@js.add_to_database()
class ControllerPIDXYAndZBounded(FAController):
    """This is a dynamic controller, not a static controller"""

    js.Jsonable.add_inner('double_integrator_xy',CONTROLLERS_DATABASE)
    js.Jsonable.add_inner('double_integrator_z',CONTROLLERS_DATABASE)

    @classmethod
    def description(cls):
        return "PID Controller, with saturation on integral part"
        
    def __init__(self,\
            integral_gain_xy     = 0.0        ,\
            bound_integral_xy    = 0.0        ,\
            integral_gain_z      = 0.5        ,\
            bound_integral_z     = 0.0        ,\
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
        self.d_est = self.disturbance_estimate

        self.t_old  = 0.0

    def reset_estimate_xy(self):
        self.d_est[0] = 0.0
        self.d_est[1] = 0.0
        return

    def reset_estimate_z(self):
        self.d_est[2] = 0.0
        return        

    def update_parameters(self,parameters):
        self.parameters = parameters

    def output(self, time, states, states_d):
        # convert euler angles from deg to rotation matrix
        ee = states[6:9]
        R  = uts.rot_from_euler_deg(ee)
        R  = numpy.reshape(R,9)
        # collecting states
        states  = numpy.concatenate([states[0:6],R])
        return self.controller(time,states,states_d)     

    # Controller
    def controller(self,t_new,states,states_d):
         
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

        Full_actuation = self.MASS*(ad + u + self.GRAVITY*e3 - self.d_est)

        # -----------------------------------------------------------------------------#
        # update disturbance estimate

        # derivatice of disturbance estimate (ELEMENT WISE PRODUCT)
        d_est_dot  = self.GAINS_INTEGRAL_ACTION*V_v
        # new disturbance estimate
        self.d_est = self.d_est + d_est_dot*(t_new - self.t_old) 
        # element-wise division
        ratio      = self.d_est/self.MAX_DISTURBANCE_ESTIMATE
        # saturate estimate just for safety (element wise multiplication)
        self.d_est = self.MAX_DISTURBANCE_ESTIMATE*np.clip(ratio,1,-1)
        # update old time
        self.t_old = t_new
        # -----------------------------------------------------------------------------#

        return Full_actuation

    def input_and_gradient_of_lyapunov(self,ep,ev):

        u_x,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v_x,V_v_p,V_v_v = self.double_integrator_xy.output(ep[0],ev[0])
        u_y,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v_y,V_v_p,V_v_v = self.double_integrator_xy.output(ep[1],ev[1])
        u_z,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v_z,V_v_p,V_v_v = self.double_integrator_z.output(ep[2],ev[2])

        u   = numpy.array([u_x,u_y,u_z])
        V_v = numpy.array([V_v_x,V_v_y,V_v_z])

        return (u,V_v)
