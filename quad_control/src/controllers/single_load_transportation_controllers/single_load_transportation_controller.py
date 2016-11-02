#!/usr/bin/env python
# this line is just used to define the type of document

# for ploting
import matplotlib.pyplot as plt

import numpy

from numpy import *

import rospy

from utilities import jsonable as js

import utilities.utility_functions as uts

class SingleLoadTransportationController(js.Jsonable):

    @classmethod
    def description(cls):
        return "Abstract **Single Load Transportation Controller**: controller for single aerial vehicle transportation a load attached by cable"

    #TODO do these have special parameters?
    def __init__(self):
        pass
    
    def output(self, position, velocity): 
        raise NotImplementedError()


    @classmethod
    def get_data_size(self):
        return 3 + 3 + 2 + 2 + 1

    def get_data(self):
        """Get all data relevant to the mission
        from this data, mission should be able to do data post-analysis, 
        like ploting or computing average errors
        """        
        default_array = self.data['load_position'].tolist()
        default_array+= self.data['load_velocity'].tolist()

        # direction of cable
        n = self.data['unit_vector']
        phi   =  -numpy.arcsin(n[[1]])
        theta =  numpy.arctan(n[[0]]/n[[2]])

        # angular velocity of cable
        w  = self.data['angular_velocity']
        w_phi   = w[0]/numpy.cos(theta) - w[1]*numpy.tan(phi)*numpy.tan(theta)
        w_theta = w[1]/(numpy.cos(theta)**2)

        default_array+= [phi,theta]
        default_array+= [w_phi,w_theta]      

        default_array+= [self.disturbance_estimate]
        
        # default_array = np.concatenate([default_array,self.get_complementary_data()])
        return default_array

    def plot_from_string(cls, string,starting_point):
        
        times        = []
        
        positions_x  = []
        positions_y  = []
        positions_z  = []

        velocities_x = []
        velocities_y = []
        velocities_z = []

        phi   = []
        theta = []

        w_phi   = []
        w_theta = []

        disturbance = []

        import math

        for line in string.split('\n'):
            # ignore empty lines           
            if line:
                numbers = line.split(' ')

                times.append(float(numbers[0]))

                numbers = numbers[starting_point[0]:]

                positions_x.append(float(numbers[0]))
                positions_y.append(float(numbers[1]))
                positions_z.append(float(numbers[2]))
                
                print(numbers[2])

                velocities_x.append(float(numbers[3]))
                velocities_y.append(float(numbers[4]))
                velocities_z.append(float(numbers[5]))

                phi.append(float(numbers[6])*180/math.pi)
                theta.append(float(numbers[7])*180/math.pi)

                w_phi.append(float(numbers[8])*180/math.pi)
                w_theta.append(float(numbers[9])*180/math.pi)      

                disturbance.append(float(numbers[10]))         

        
        fig1 = plt.figure()
        plt.plot(times, positions_x, 'r-', label=r'$x$')
        plt.plot(times, positions_y, 'g-', label=r'$y$')
        plt.plot(times, positions_z, 'b-', label=r'$z$')     
        plt.title('Load position (m)')
        plt.legend(loc='best')
        plt.grid()

        fig2 = plt.figure()
        plt.plot(times, velocities_x, 'r-', label=r'$v_x$')
        plt.plot(times, velocities_y, 'g-', label=r'$v_y$')
        plt.plot(times, velocities_z, 'b-', label=r'$v_z$')       
        plt.title('Load velocity (m/s)')
        plt.legend(loc='best')
        plt.grid()        

        fig3 = plt.figure()
        plt.plot(times, phi, 'r-', label=r'$\phi$')
        plt.plot(times, theta, 'g-', label=r'$\theta$')    
        plt.title(r'$\phi$ and $\theta$ of cable in degrees')
        plt.legend(loc='best')
        plt.grid()   

        fig4 = plt.figure()
        plt.plot(times, w_phi, 'r-', label=r'$\dot{\phi}$')
        plt.plot(times, w_theta, 'g-', label=r'$\dot{\theta}$')       
        plt.title(r'$\dot{\phi}$ and $\dot{\theta}$ of cable in degrees')
        plt.legend(loc='best')
        plt.grid()           

        fig5 = plt.figure()
        plt.plot(times, disturbance, 'r-', label='disturbance')     
        plt.title('Disturbance')
        plt.legend(loc='best')
        plt.grid()

        return fig1,fig2,fig3,fig4,fig5


def skew(x):
    out = numpy.zeros((3,3))
    out[0,1] = -x[2]
    out[0,2] =  x[1]
    out[1,2] = -x[0]
    out[1,0] =  x[2]
    out[2,0] = -x[1]
    out[2,1] =  x[0]
    return out


def sat(x):
    return 0.5*x/numpy.sqrt(x**2 + 0.5**2)

# import double integrator controllers
import controllers.double_integrator_controllers.double_integrator_controller
ONE_D_DIC_DATABASE = controllers.double_integrator_controllers.double_integrator_controller.database_one_d

@js.add_to_database(default=True)
class LinearController(SingleLoadTransportationController):   
    '''Decompose control problems in two parts:
    . Control z component of load as double integrator
    . Control x and y of UAV also as double integrator
        . for controlling cable oscillations we produce a desired
        for the xy trajectory of the uav that dampens and steers
        the cable oscillations to zero
    '''

    # double integrator controller for z component of load (position)
    js.Jsonable.add_inner('z_double_integrator_ctr',ONE_D_DIC_DATABASE)    

    @classmethod
    def description(cls):
        return '''
        Linear controller for a <b>single aerial vehicle transporating load</b> attached by cable. Decompose control problems in two parts:
        <ul>
            <li>Control z component ...</li>
            <li>Control x and y ....</li>
        </ul>
        '''

    def __init__(self, 
        load_mass    = rospy.get_param("load_mass",0.1),
        quad_mass    = rospy.get_param("quadrotor_mass",1.442),
        cable_length = rospy.get_param("cable_length",0.6),
        kp           = rospy.get_param("proportional_gain_xy_load_lifting",1.0),
        kv           = rospy.get_param("derivative_gain_xy_load_lifting",1.41),
        kt           = rospy.get_param("cable_angle_gain_load_lifting",0.0),
        kw           = rospy.get_param("cable_angular_velocity_gain_xy_load_lifting",0.0),
        gain_integral_action = rospy.get_param("gain_integral_action_load_lifting",0.0),
        max_disturbance_estimate = rospy.get_param("max_disturbance_estimate_load_lifting",0.0)
        ):

        self.add_inner_defaults()

        self.kp = kp
        self.kv = kv
        self.kt = kt
        self.kw = kw

        self.quad_mass    = quad_mass
        self.load_mass    = load_mass
        self.cable_length = cable_length
        # TODO import this later from utilities
        self.g            = 9.81
        self.gravity = uts.GRAVITY

        self.data = {}

        self.e3 = numpy.array([0.0,0.0,1.0])

        self.data = {}
        self.data['load_position'] = numpy.zeros(3)
        self.data['load_velocity'] = numpy.zeros(3)
        self.data['uav_position'] = numpy.zeros(3)
        self.data['uav_velocity'] = numpy.zeros(3)
        self.data['unit_vector'] = numpy.array([0.0,0.0,1.0])
        self.data['angular_velocity'] = numpy.zeros(3)

        self.max_disturbance_estimate = max_disturbance_estimate
        self.gain_integral_action = gain_integral_action


        self.disturbance_estimate = 0.0


    def object_description(self):
        string = """
        Parameters:
        <ul>
          <li>load_mass: """ + str(self.load_mass) +"""</li>
          <li>quad_mass: """ + str(self.quad_mass) +"""</li>
          <li>cable_length: """ + str(self.cable_length) +"""</li>
        </ul>
        """
        return string

    def get_total_weight(self):
        return (self.quad_mass+self.load_mass)*self.g



    def object_descrition(self):
        description = "Controller\n"
        description = "quad mass = " + str(self.quad_mass) + "(kg), load mas = " + str(self.load_mass) + "(kg), cable length = " + str(self.cable_length) + "(m), gravity = " + str(self.g) + "(m/s/s).\n\n"
        return description

    def output(self, time_instant, states, states_d):

        # initiliaze initial time instant
        self.t_old = time_instant

        self.output = self.output_after_initialization

        return self.output_after_initialization(time_instant,states,states_d)


    def output_after_initialization(self,time_instant,state,stated):

        # masses and cable length
        m   = self.quad_mass;
        M   = self.load_mass;
        L   = self.cable_length;

        # gravity
        g   = self.g;

        e3  = numpy.array([0.0,0.0,1.0])

        # current LOAD position
        pM  = state[0:3]
        self.data['load_position'] = pM
        # current LOAD velocity
        vM  = state[3:6]
        self.data['load_velocity'] = vM
        # current QUAD position
        pm  = state[6:9]
        self.data['uav_position'] = pm
        # current QUAD velocity
        vm  = state[9:12]
        self.data['uav_velocity'] = vm

        # DESIRED LOAD position
        pd = stated[0:3]
        # DESIRED LOAD velocity
        vd = stated[3:6]

        # transformation of state
        ep = pM - pd
        ev = vM - vd

        # direction of cable
        n = (pm - pM)/numpy.linalg.norm(pm - pM)

        # alpha = 10.0*3.142/180.0
        # if numpy.dot(n,e3)<numpy.cos(alpha):
        #     naux = numpy.dot(uts.ort_proj(e3),n)
        #     n    = numpy.cos(alpha)*e3 + numpy.sin(alpha)*naux/numpy.linalg.norm(naux)

        # angular velocity of cable
        w  = dot(skew(n),(vm - vM)/numpy.linalg.norm(pM - pm))

        self.data['unit_vector'] = n
        self.data['angular_velocity'] = w

        # direction of cable
        phi   =  -numpy.arcsin(n[[1]])
        theta =  numpy.arctan(n[[0]]/n[[2]])

        # angular velocity of cable
        w_phi   = w[0]/numpy.cos(theta) - w[1]*numpy.tan(phi)*numpy.tan(theta)
        w_theta = w[1]/(numpy.cos(theta)**2)


        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.z_double_integrator_ctr.output(ep[2],ev[2])
        uz = (m+M)*g + (m + M)*(u - self.disturbance_estimate)


        delta_t = time_instant - self.t_old
        self.t_old = time_instant
        disturbance_estimate_dot  = self.gain_integral_action*V_v
        # new disturbance estimate
        disturbance_estimate = self.disturbance_estimate + disturbance_estimate_dot*delta_t
        # saturate estimate just for safety (element wise bound)
        self.disturbance_estimate = numpy.clip(disturbance_estimate,-1.0*self.max_disturbance_estimate,self.max_disturbance_estimate)


        # errors
        ep = pm - pd
        ev = vm - vd

        x1 = ep[0]
        x2 = ev[0]
        x3 = g*theta[0]
        x4 = g*w_theta[0]
        #ux = -(self.kp*x1 + self.kv*x2 + self.kw*x4)
        #ux = -(self.kp*x1 + self.kv*x2 + self.kw*w_theta[0])
        ux = -(self.kp*sat(x1) + self.kv*sat(x2) + self.kt*(n[0]) + self.kw*w[1])
        ux *= m

        x1 = ep[1]
        x2 = ev[1]
        x3 = -g*phi[0]
        x4 = -g*w_phi[0]
        #uy = -(self.kp*x1 + self.kv*x2 + self.kw*x4)
        # uy = -(self.kp*x1 + self.kv*x2 + self.kw*(-w_phi[0]))
        uy = -(self.kp*sat(x1) + self.kv*sat(x2) + self.kt*(n[1]) + self.kw*(-w[0]))
        uy *= m

        return numpy.array([ux,uy,uz])


# import double integrator controllers
import controllers.double_integrator_controllers.double_integrator_controller
DIC_DATABASE       = controllers.double_integrator_controllers.double_integrator_controller.database
ONE_D_DIC_DATABASE = controllers.double_integrator_controllers.double_integrator_controller.database_one_d


import reference.plan_motion_load_lifting
PLAN_XY_MOTION_DATABASE = reference.plan_motion_load_lifting.database

# def sat(x):
#     return 0.1*x/numpy.sqrt(x**2 + 0.1**2)

# def sat(x):
#     return x


@js.add_to_database()
class LoadzUAVxy(SingleLoadTransportationController):   
    '''Decompose control problems in two parts:
    . Control z component of load as double integrator
    . Control x and y of UAV also as double integrator
        . for controlling cable oscillations we produce a desired
        for the xy trajectory of the uav that dampens and steers
        the cable oscillations to zero
    '''

    # double integrator controller for z component of load (position)
    js.Jsonable.add_inner('z_double_integrator_ctr',ONE_D_DIC_DATABASE)    
    # double integrator controller for x and y component of load (position)
    js.Jsonable.add_inner('xy_double_integrator_ctr',DIC_DATABASE)
    # planner for xy motion of uav
    js.Jsonable.add_inner('xy_motion',PLAN_XY_MOTION_DATABASE)

    @classmethod
    def description(cls):
        return '''
        Controller for a <b>single aerial vehicle transporating load</b> attached by cable. Decompose control problems in two parts:
        <ul>
            <li>Control z component of load as double integrator</li>
            <li>Control x and y of UAV also as double integrator</li>
            <ul>
                <li>for controlling cable oscillations we produce a desired for the xy trajectory of the uav that dampens and steers the cable oscillations to zero</li>
            </ul>
        </ul>
        '''

    def __init__(self, 
        load_mass    = rospy.get_param("load_mass",0.1),
        quad_mass    = rospy.get_param("quadrotor_mass",1.442),
        cable_length = rospy.get_param("cable_length",0.6)
        ):

        self.add_inner_defaults()

        self.quad_mass    = quad_mass
        self.load_mass    = load_mass
        self.cable_length = cable_length
        # TODO import this later from utilities
        self.g            = 9.81
        self.gravity = uts.GRAVITY

        self.data = {}

        self.e3 = numpy.array([0.0,0.0,1.0])

        self.data = {}
        self.data['load_position'] = numpy.zeros(3)
        self.data['load_velocity'] = numpy.zeros(3)
        self.data['uav_position'] = numpy.zeros(3)
        self.data['uav_velocity'] = numpy.zeros(3)
        self.data['unit_vector'] = numpy.array([0.0,0.0,1.0])
        self.data['angular_velocity'] = numpy.zeros(3)


    def object_description(self):
        string = """
        Parameters:
        <ul>
          <li>load_mass: """ + str(self.load_mass) +"""</li>
          <li>quad_mass: """ + str(self.quad_mass) +"""</li>
          <li>cable_length: """ + str(self.cable_length) +"""</li>
        </ul>
        """
        return string

    def get_total_weight(self):
        return (self.quad_mass+self.load_mass)*self.g

    def output(self,time_instant,state,stated):

        x,gravity = self._state_transform(state,stated)

        reference = {}
        reference['position'] = stated[2]
        reference['velocity'] = stated[5]
        reference['acceleration'] = stated[8]
        thrust = self.compute_thrust(reference)

        reference = {}
        reference['position'] = stated[0:2]
        reference['velocity'] = stated[3:5]
        uav_xy_reference = self.xy_motion.output(self.data,reference,1.0/35.0)
        torque = self.compute_torque(uav_xy_reference,thrust)

        U = self._input_transform(thrust,torque)

        return U

    def object_descrition(self):
        description = "Controller\n"
        description = "quad mass = " + str(self.quad_mass) + "(kg), load mas = " + str(self.load_mass) + "(kg), cable length = " + str(self.cable_length) + "(m), gravity = " + str(self.g) + "(m/s/s).\n\n"
        return description


    def _state_transform(self,state,stated):

        # masses and cable length
        m   = self.quad_mass;
        M   = self.load_mass;
        L   = self.cable_length;

        # gravity
        g   = self.g;

        e3  = numpy.array([0.0,0.0,1.0])

        # current LOAD position
        pM  = state[0:3]
        self.data['load_position'] = pM
        # current LOAD velocity
        vM  = state[3:6]
        self.data['load_velocity'] = vM
        # current QUAD position
        pm  = state[6:9]
        self.data['uav_position'] = pm
        # current QUAD velocity
        vm  = state[9:12]
        self.data['uav_velocity'] = vm

        # DESIRED LOAD position
        pd = stated[0:3]
        # DESIRED LOAD velocity
        vd = stated[3:6]

        # transformation of state
        p = pM - pd
        v = vM - vd

        # direction of cable
        n = (pm - pM)/numpy.linalg.norm(pm - pM)

        # alpha = 10.0*3.142/180.0
        # if numpy.dot(n,e3)<numpy.cos(alpha):
        #     naux = numpy.dot(uts.ort_proj(e3),n)
        #     n    = numpy.cos(alpha)*e3 + numpy.sin(alpha)*naux/numpy.linalg.norm(naux)

        # angular velocity of cable
        w  = dot(skew(n),(vm - vM)/numpy.linalg.norm(pM - pm))

        self.data['unit_vector'] = n
        self.data['angular_velocity'] = w


        x = concatenate([p,v,n,w])
        # print x
        self.x = x

        #---------------------------------------------------#
        # DESIRED LOAD acceleration, jerk and snap
        ad = stated[6:9]   
        jd = stated[9:12] 
        sd = stated[12:15] 
        gravity = concatenate([g*e3 + ad,jd,sd])

        return (x,gravity)

    def compute_thrust(self,reference):

        # direction of cable
        n = self.data['unit_vector']

        # z position error
        ep = self.data['load_position'][2] - reference['position']
        # z velocity error
        ev = self.data['load_velocity'][2] - reference['velocity']

        # feedforward term: desired acceleration
        ad = reference['acceleration']

        g  = self.gravity

        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.z_double_integrator_ctr.output(ep,ev)

        thrust = 1.0/n[2]*(g + ad + u)

        return thrust

    def compute_torque(self,reference,Thrust):

        # direction of cable
        n = self.data['unit_vector']
        # angular velocity of cable
        w  = self.data['angular_velocity']

        # xy position error
        ep = self.data['uav_position'][0:2] - reference['position']
        # xy velocity error
        ev = self.data['uav_velocity'][0:2] - reference['velocity']

        ad = reference['acceleration']

        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.xy_double_integrator_ctr.output(ep,ev)
        u = ad + u
        u = numpy.concatenate([u,[0]])

        L = self.cable_length

        e3 = self.e3

        torque = -1.0/n[2]*1.0/L
        torque = torque*numpy.dot(skew(n),skew(e3))
        torque = numpy.dot(torque,u - n*(Thrust-L*numpy.dot(w,w)))

        return torque

    def _input_transform(self,Thrust,Tau):

        # masses and cable length
        m   = self.quad_mass
        M   = self.load_mass
        L   = self.cable_length
        
        n = self.x[6:9]
        w = self.x[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + numpy.dot(uts.ort_proj(n),Tau*m*L)

        return U

# import controllers for vector thrusted systems
import controllers.vector_thrust_controllers.vector_thrust_controller
VT_CONTROLLERS_DATABASE = controllers.vector_thrust_controllers.vector_thrust_controller.database

@js.add_to_database()
class SingleLoadTransportController(SingleLoadTransportationController):   

    js.Jsonable.add_inner('vector_thrust_controller',VT_CONTROLLERS_DATABASE)

    @classmethod
    def description(cls):
        return "Controller for a **single aerial vehicle transporating load** attached by cable"


    def __init__(self, 
        load_mass    = rospy.get_param("load_mass",0.1),
        quad_mass    = rospy.get_param("quadrotor_mass",1.442),
        cable_length = rospy.get_param("cable_length",0.6),
        ):

        self.add_inner_defaults()

        self.quad_mass    = quad_mass
        self.load_mass    = load_mass
        self.cable_length = cable_length
        # TODO import this later from utilities
        self.g            = 9.81

        self.data = {}
        self.data['load_position'] = numpy.zeros(3)
        self.data['load_velocity'] = numpy.zeros(3)
        self.data['uav_position'] = numpy.zeros(3)
        self.data['uav_velocity'] = numpy.zeros(3)
        self.data['unit_vector'] = numpy.array([0.0,0.0,1.0])
        self.data['angular_velocity'] = numpy.zeros(3)

        self.unit_vector_filter = uts.MedianFilter3D(3,data_initial=numpy.array([0.0,0.0,1.0]))
        self.angular_velocity_filter = uts.MedianFilter3D(3,data_initial=numpy.zeros(3))

    def object_description(self):
        string = """
        Parameters:
        <ul>
          <li>load_mass: """ + str(self.load_mass) +"""</li>
          <li>quad_mass: """ + str(self.quad_mass) +"""</li>
          <li>cable_length: """ + str(self.cable_length) +"""</li>
        </ul>
        """
        return string


    def get_total_weight(self):
        return (self.quad_mass+self.load_mass)*self.g

    def output(self,time_instant,state,stated):

        x,gravity = self._state_transform(state,stated)
        Thrust,Tau,V,VD,V_dT,V_dTau = self.vector_thrust_controller.output(x,gravity)
        # print(Thrust)
        # print(Tau)
        U = self._input_transform(Thrust,Tau)

        return U

    def report(self):
        description = "Controller for Load Lifting\n"
        parameters  = "quad mass = " + str(self.quad_mass) + "(kg), load mas = " + str(self.load_mass) + "(kg), cable length = " + str(self.cable_length) + "(m), gravity = " + str(self.g) + "(m/s/s).\n\n"
        return description + parameters + self.VT_Ctrll.report()


    def _state_transform(self,state,stated):

        # masses and cable length
        m   = self.quad_mass;
        M   = self.load_mass;
        L   = self.cable_length;

        # gravity
        g   = self.g;

        e3  = numpy.array([0.0,0.0,1.0])

        # current LOAD position
        pM  = state[0:3]
        # current LOAD velocity
        vM  = state[3:6]
        # current QUAD position
        pm  = state[6:9]
        # current QUAD velocity
        vm  = state[9:12]

        # DESIRED LOAD position
        pd = stated[0:3]
        # DESIRED LOAD velocity
        vd = stated[3:6]

        # transformation of state
        p = pM - pd
        v = vM - vd

        # direction of cable
        n = (pm - pM)/numpy.linalg.norm(pm - pM)
        n = self.unit_vector_filter.up_and_out(n)

        # alpha = 10.0*3.142/180.0
        # if numpy.dot(n,e3)<numpy.cos(alpha):
        #     naux = numpy.dot(uts.ort_proj(e3),n)
        #     n    = numpy.cos(alpha)*e3 + numpy.sin(alpha)*naux/numpy.linalg.norm(naux)

        # angular velocity of cable
        w = dot(skew(n),(vm - vM)/numpy.linalg.norm(pM - pm))
        w = self.angular_velocity_filter.up_and_out(w)

        self.data['load_position'] = pM
        self.data['load_velocity'] = vM
        self.data['uav_position'] = pm
        self.data['uav_velocity'] = vm
        self.data['unit_vector'] = n
        self.data['angular_velocity'] = w

        x = concatenate([p,v,n,w])
        # print x
        self.x = x

        #---------------------------------------------------#
        # DESIRED LOAD acceleration, jerk and snap
        ad = stated[6:9]   
        jd = stated[9:12] 
        sd = stated[12:15] 
        gravity = concatenate([g*e3 + ad,jd,sd])

        return (x,gravity)

    def _input_transform(self,Thrust,Tau):

        # masses and cable length
        m   = self.quad_mass
        M   = self.load_mass
        L   = self.cable_length
        
        n = self.x[6:9]
        w = self.x[9:12]
        
        U = n*(Thrust*(m+M) - dot(w,w)*m*L) + numpy.dot(uts.ort_proj(n),Tau*m*L)

        return U
