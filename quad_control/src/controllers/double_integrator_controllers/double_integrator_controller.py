#!/usr/bin/env python
# this line is just used to define the type of document

from utilities import jsonable as js

import numpy

# for accessing the parameters database
import rospy

class DoubleIntegratorController(js.Jsonable):
    '''
    Controller for double integrator x_2dot = u(x_0dot,x_1dot)
    user friendly parameters are natural frequency and damping
    '''

    @classmethod
    def description(cls):
        return "Abstract Double Integrator Controller"

    def __init__(self, natural_frequency=2.0, damping=numpy.sqrt(2)/2.0):
        '''user friendly parameters are natural frequency and damping'''
        self.natural_frequency = natural_frequency
        self.damping = damping
    
    # declare function as available for gui-user
    @js.add_to_methods_list
    def change_natural_frequency(self,natural_frequency=1.0):
        '''Change natural frequency (/sec/sec)'''
        self.natural_frequency = natural_frequency

    # declare function as available for gui-user
    @js.add_to_methods_list
    def change_damping(self,damping=numpy.sqrt(2)/2.0):
        '''Change damping (1: overdamped, 0: underdamped)'''
        self.damping = damping

    def output(self, position, velocity): 
        raise NotImplementedError()


@js.add_to_database(default=True)
class ComponentWise3DDIC(DoubleIntegratorController):


    @classmethod
    def description(cls):
        return "Double-integrator bounded and component-wise controller" 


    # The class "constructor" - It's actually an initializer
    def __init__(self,
    natural_frequency       = 2.0,
    damping                 = numpy.sqrt(2.0)/2.0,
    position_saturation     = 0.5,
    velocity_saturation     = 0.5
    ):
            
        DoubleIntegratorController.__init__(self,
            natural_frequency=natural_frequency,
            damping=damping)

        self.natural_frequency = natural_frequency
        self.damping = damping
            
        self.position_saturation = position_saturation
        self.velocity_saturation = velocity_saturation


    def output(self, position, velocity):
        return self._DI_Bounded_Component(position, velocity)


    def __str__(self):
        string = DoubleIntegratorController.__str__(self)
        string += "\nPosition saturation: " + str(self.position_saturation)
        string += "\nVelocity saturation: " + str(self.velocity_saturation)
        return string
        
#        description = "Bounded Double Integrator Controller u(p,v) = -sigma(p) - ro(v): simple control law, complicated Lyapunov function\n"
#        controller  = "Parameters: sat(x) = k x/sqrt(1 + x**2), with sigma(p) = kp*sat(p/sigma_p) and ro(p) = kv*sat(v/sigma_v)\n"  
#        parameters  = "kp = " + str(self.kp) + " and kv = " + str(self.kv) + " and sigma_p = " +  str(self.sigma_p) + "and sigma_v = " +  str(self.sigma_v) +"\n\n"
#        return description + controller + parameters


    def _sat(self,x):

        sat     =  x/numpy.sqrt(1.0 + x**2)
        Dsat    =  (1.0 + x**2)**(-3.0/2.0)
        D2sat   =  -3.0*x*(1.0 + x**2)**(-5.0/2.0)
        # primitive of saturation function
        sat_Int =  numpy.sqrt(1.0 + x**2) - 1.0

        return (sat,Dsat,D2sat,sat_Int)


    def _fGain(self,x):

        fgain      =  1.0
        Dfgain     =  0.0
        D2fgain    =  0.0
        # integral of x/fgain(x) from 0 to in
        fgain_int  =  1.0/2.0*x**2
        # integral of sat(x)*Dsat(x)/fgain(x) from 0 to in    
        # fgain_int2 = 1/2*sat(x)**2
        fgain_int2 = 1.0/2.0*x**2/(1.0 + x**2)   

        return (fgain,Dfgain,D2fgain,fgain_int,fgain_int2)

    # print sat(2.0)
    # print fGain(2.0)

    def  _DI_Bounded_Component(self,p,v):

        # gains
        kp = self.natural_frequency**2
        kv = 2*self.damping*self.natural_frequency

        sigma_p  = self.position_saturation
        sigma_v  = self.velocity_saturation


        sat_p,Dsat_p,D2sat_p,sat_Int_p = self._sat(p/sigma_p)
        sat_v,Dsat_v,D2sat_v,sat_Int_v = self._sat(v/sigma_v)

        fgain,Dfgain,D2fgain,fgain_int,fgain_int2   = self._fGain(v/sigma_v)


        h1     = kp*sigma_p*sat_p
        h1_p   = kp*Dsat_p
        h1_p_p = kp*D2sat_p/sigma_p

        h2     = kv*sigma_v*sat_v
        h2_v   = kv*Dsat_v
        h2_v_v = kv*D2sat_v/sigma_v

        f      = fgain
        f_v    = Dfgain/sigma_v
        f_v_v  = D2fgain/sigma_v**2


        u     = -f*h1 - h2

        u_p   = -f*h1_p
        u_p_p = -f*h1_p_p

        u_v   = -f_v*h1  - h2_v
        u_v_v = -f_v_v*h1 - h2_v_v

        u_p_v = -f_v*h1_p


        beta   = 1.0/(2.0*kp)
        h1_int = kp*(sigma_p**2)*sat_Int_p

        V  = beta*kv**2*h1_int    + \
             beta*h1*h2           + \
             sigma_v**2*fgain_int + \
             h1_int               + \
             beta*kv**2*sigma_v**2*(fgain_int - fgain_int2)

        VD = (-1)*(                    \
                   beta*h2_v*f*h1**2 + \
                   v*h2*(1.0/f - beta*h1_p) + beta/f*h2*(kv**2*v - h2*h2_v)\
                  )
              
        V_p   = beta*kv**2*h1 + beta*h2*h1_p + h1  

        V_v   = beta*h1*h2_v + v/f + beta/f*(kv**2*v - h2*h2_v)

        V_v_p = beta*h1_p*h2_v

        V_v_v = beta*h1*h2_v_v +\
                1.0/f - v/f**2*f_v +\
                (-1.0)*beta/f**2*f_v*(kv**2*v - h2*h2_v) +\
                beta/f*(kv**2 - h2_v*h2_v - h2*h2_v_v)     

        return u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v


    def  _DI_Bounded_NOT_Component(self,p,v):

        u      = numpy.zeros(3)
        u_p    = numpy.zeros((3,3))
        u_v    = numpy.zeros((3,3))
        u_p_p  = numpy.zeros((3,3,3))
        u_v_v  = numpy.zeros((3,3,3))
        u_p_v  = numpy.zeros((3,3,3))

        # V     = numpy.zeros(1)
        # VD    = numpy.zeros(1)

        V_p   = numpy.zeros(3)
        V_v   = numpy.zeros(3)
        V_v_p = numpy.zeros((3,3))
        V_v_v = numpy.zeros((3,3))

        u[0],u_p[0,0],u_v[0,0],u_p_p[0,0,0],u_v_v[0,0,0],u_p_v[0,0,0],V0,VD0,V_p[0],V_v[0],V_v_p[0,0],V_v_v[0,0] =  \
            self._DI_Bounded_Component(p[0],v[0])

        u[1],u_p[1,1],u_v[1,1],u_p_p[1,1,1],u_v_v[1,1,1],u_p_v[1,1,1],V1,VD1,V_p[1],V_v[1],V_v_p[1,1],V_v_v[1,1] =  \
            self._DI_Bounded_Component(p[1],v[1])

        u[2],u_p[2,2],u_v[2,2],u_p_p[2,2,2],u_v_v[2,2,2],u_p_v[2,2,2],V2,VD2,V_p[2],V_v[2],V_v_p[2,2],V_v_v[2,2] =  \
            self._DI_Bounded_Component(p[2],v[2])

        V  = V0  + V1  + V2
        VD = VD0 + VD1 + VD2

        return u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v
        
        
       
# Test
#string = DoubleIntegratorBoundedAndComponentWiseController.to_string()
#print string
#con = DoubleIntegratorBoundedAndComponentWiseController.from_string(string)
#print con
#print con.output(zeros(3), zeros(3))


@js.add_to_database()
class NDimensionalBoundedDIC(DoubleIntegratorController): 


    @classmethod
    def description(cls):
        return "Double-integrator bounded but not component-wise controller"


    def __init__(self,
    natural_frequency       = rospy.get_param("natural_frequency_xy",1.0),
    damping                 = rospy.get_param("damping_xy",numpy.sqrt(2.0)/2.0),
    position_saturation     = rospy.get_param("position_saturation_xy",1.0),
    velocity_saturation     = rospy.get_param("velocity_saturation_xy",1.0),
    size_di                 = 2
    ):

        DoubleIntegratorController.__init__(self,
            natural_frequency=natural_frequency,
            damping=damping)

        self.natural_frequency = natural_frequency
        self.damping = damping
            
        self.position_saturation = position_saturation
        self.velocity_saturation = velocity_saturation
        
        self.position_saturation = position_saturation
        self.velocity_saturation = velocity_saturation

        self.__eps = 0.01

        self.size_di = size_di

    def output(self,p,v):
        return self._DI_Bounded(p,v)

    def object_description(self):
        string = """
        Parameters:
        <ul>
          <li>natural_frequency: """ + str(self.natural_frequency) +"""</li>
          <li>damping: """ + str(self.damping) +"""</li>
          <li>position_saturation: """ + str(self.position_saturation) +"""</li>
          <li>velocity_saturation: """ + str(self.velocity_saturation) +"""</li>
          <li>size_di: """ + str(self.size_di) +"""</li>          
        </ul>
        """
        string+="Bounded Double Integrator Controller u(p,v) = -sigma(p) - ro(v): simple control law, complicated Lyapunov function\n"
        string+="Parameters: sat(x) = k x/sqrt(1 + x**2), with sigma(p) = kp*sat(p/sigma_p) and ro(p) = kv*sat(v/sigma_v)\n"  
        string+="kp = proportional_gain and kv = derivative_gain and sigma_p = saturation_position and sigma_v = saturation_velocity \n\n"
        # parameters  = "kp = " + str(self.__proportional_gain) + " and kv = " + str(self.__derivative_gain) + " and sigma_p = " +  str(self.__saturation_position) + "and sigma_v = " +  str(self.__saturation_velocity) + "\n\n"
        return string

    def _sat(self,x):

        sat     =  1.0/numpy.sqrt(1.0 + x**2)
        Dsat    =  -x*(1.0 + x**2)**(-3.0/2.0)
        D2sat   =  (-1.0 + 2.0*x**2)*(1.0 + x**2)**(-5.0/2.0)
        # primitive of sat(x)*x
        sat_Int =  numpy.sqrt(1.0 + x**2) - 1.0

        return (sat,Dsat,D2sat,sat_Int)

    # print sat(2.0)

    def  _DI_Bounded(self,p,v):

        # size of position of double integrator system
        # this does not work when p is a float because float does not have length
        # size_di = len(p)
        size_di = self.size_di

        # # gains
        # kp = self.get_proportional_gain()
        # kv = self.get_derivative_gain()

        # gains
        kp = self.natural_frequency**2
        kv = 2*self.damping*self.natural_frequency

        sigma_p  = self.position_saturation
        sigma_v  = self.velocity_saturation

        eps = self.__eps 

        pp  = numpy.linalg.norm(p)
        vv  = numpy.linalg.norm(v)

        sat_p,Dsat_p,D2sat_p,sat_Int_p = self._sat(pp/sigma_p)
        sat_v,Dsat_v,D2sat_v,sat_Int_v = self._sat(vv/sigma_v)       

        I  = numpy.identity(size_di) 

        # vector
        h1     = kp*sat_p*p
        if pp >= eps:
            # matrix
            h1_p   = kp*Dsat_p/sigma_p*numpy.outer(p,p)/pp + kp*sat_p*I
            # tensor
            h1_p_p = numpy.zeros((size_di,size_di,size_di));
            for i in range(size_di):
                ee = I[:,i];
                h1_p_p[:,:,i] = kp*D2sat_p/sigma_p**2*numpy.outer(p,p)/(pp**2)*numpy.dot(p,ee)       + \
                                kp*Dsat_p/sigma_p*numpy.dot(p,ee)/pp*I                         + \
                                kp*Dsat_p/sigma_p*numpy.outer(p,ee)/pp                         + \
                                (-1)*kp*Dsat_p/sigma_p*numpy.outer(p,p)/pp**3*numpy.dot(p,ee)        + \
                                kp*Dsat_p/sigma_p*numpy.outer(ee,p)/pp
        else:
            h1_p          = kp*I
            h1_p_p = numpy.zeros((size_di,size_di,size_di))

        # vector
        h2     = kv*sat_v*v
        if vv >= eps: 
            # matrix
            h2_v   = kv*Dsat_v/sigma_v*numpy.outer(v,v)/vv + kv*sat_v*I
            # tensor
            h2_v_v = numpy.zeros((size_di,size_di,size_di))
            for i in range(size_di):
                ee = I[:,i]
                h2_v_v[:,:,i] = kv*D2sat_v/sigma_v**2*numpy.outer(v,v)/(vv**2)*numpy.dot(v,ee)    + \
                                kv*Dsat_v/sigma_v*numpy.dot(v,ee)/vv*I                      + \
                                kv*Dsat_v/sigma_v*numpy.outer(v,ee)/vv                      + \
                                (-1)*kv*Dsat_v/sigma_v*numpy.outer(v,v)/(vv**3)*numpy.dot(v,ee)   + \
                                kv*Dsat_v/sigma_v*numpy.outer(ee,v)/vv
        else:
            h2_v   = kv*I
            h2_v_v = numpy.zeros((size_di,size_di,size_di))


        # vector
        u     = -h1 - h2;

        # matrix
        u_p   = -h1_p;
        # tensor
        u_p_p = -h1_p_p;

        # matrix
        u_v   = - h2_v;
        # tensor
        u_v_v = - h2_v_v;

        u_p_v = numpy.zeros((size_di,size_di,size_di));


        beta   = 1.0/(2.0*kp);
        h1_int = kp*sigma_p**2*sat_Int_p;

        # this part is not really necessary
        if pp > eps and vv > eps:
            V  = beta*kv**2*h1_int     + \
                 beta*numpy.dot(h1,h2)       + \
                 1.0/2.0*vv**2         + \
                 h1_int                + \
                 beta*1.0/2.0*(kv**2*numpy.dot(v,v) - numpy.dot(h2,h2))

            VD = (-1)*(\
                       beta*numpy.dot(h1,h1)*kv*sat_v                                                        + \
                       numpy.dot(v,h2)*beta*kp**2*pp**2*sat_p**2*numpy.dot(p/pp,v/vv)**2*(kv*Dsat_v/sigma_v)/(kv*sat_v*vv)   + \
                       numpy.dot(v,h2)*(1.0 - beta*kp*(sat_p + Dsat_p/sigma_p*pp*numpy.dot(p/pp,v/vv)**2))                   + \
                       beta*numpy.dot(v,h2)*kv**2*(1.0 - sat_v*(Dsat_v/sigma_v*vv + sat_v))   \
                      )
        else:
            V  = 0
            VD = 0

        # V_v         = dV/d(v)
        # V_v_p = d/d(p) [dV/d(v)]
        # V_v_v = d/d(v) [dV/d(v)]

        V_p   = beta*kv**2*h1 + beta*numpy.dot(h1_p.T,h2) + h1

        V_v   = beta*numpy.dot(h2_v.T,h1) + v + beta*(kv**2*v - numpy.dot(h2_v.T,h2))

        V_v_p = beta*numpy.dot(h2_v.T,h1_p)

        V_v_v = beta*numpy.dot(h2_v_v,h1).T                                 + \
                I                                                     + \
                beta*(kv**2*I - numpy.dot(h2_v_v,h2).T  - numpy.dot(h2_v.T,h2_v))
  

        return (u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v)
        
        
        
# Test
# con = NDimensionalBoundedDIC()
#print con
#print con.output(numpy.zeros(3), numpy.zeros(3))
#print con.parameters_to_string()

@js.add_to_database()
class NeutralDIC(DoubleIntegratorController):

    
    @classmethod
    def description(cls):
        return "Double-integrator neutral controller"       

    
    def output(self, position, velocity):
        #TODO make the dimension a parameter?
        return numpy.zeros(len(position))

@js.add_to_database()
class NotComponentWise3DDIC(DoubleIntegratorController): 


    @classmethod
    def description(cls):
        return "Double-integrator bounded but not component-wise controller"


    def __init__(self,
    natural_frequency       = 0.5,
    damping                 = numpy.sqrt(2.0)/2.0,
    position_saturation     = 1.0,
    velocity_saturation     = 1.0
    ):
        
        DoubleIntegratorController.__init__(self,
            natural_frequency=natural_frequency,
            damping=damping)

        self.natural_frequency = natural_frequency
        self.damping = damping
            
        self.position_saturation = position_saturation
        self.velocity_saturation = velocity_saturation
        
        self.position_saturation = position_saturation
        self.velocity_saturation = velocity_saturation

        self.__eps = 0.01


    def output(self,p,v):
        return self._DI_Bounded(p,v)

    def __str__(self):
        string = DoubleIntegratorController.__str__(self)
        string += "\nPosition saturation: " + str(self.position_saturation)
        string += "\nVelocity saturation: " + str(self.velocity_saturation)
        return string
        
        
#        description = "Bounded Double Integrator Controller u(p,v) = -sigma(p) - ro(v): simple control law, complicated Lyapunov function\n"
#        controller  = "Parameters: sat(x) = k x/sqrt(1 + x**2), with sigma(p) = kp*sat(p/sigma_p) and ro(p) = kv*sat(v/sigma_v)\n"  
#        parameters  = "kp = proportional_gain and kv = derivative_gain and sigma_p = saturation_position and sigma_v = saturation_velocity \n\n"
#        # parameters  = "kp = " + str(self.__proportional_gain) + " and kv = " + str(self.__derivative_gain) + " and sigma_p = " +  str(self.__saturation_position) + "and sigma_v = " +  str(self.__saturation_velocity) + "\n\n"
#        return description + controller + parameters


    def _sat(self,x):

        sat     =  1.0/numpy.sqrt(1.0 + x**2)
        Dsat    =  -x*(1.0 + x**2)**(-3.0/2.0)
        D2sat   =  (-1.0 + 2.0*x**2)*(1.0 + x**2)**(-5.0/2.0)
        # primitive of sat(x)*x
        sat_Int =  numpy.sqrt(1.0 + x**2) - 1.0

        return (sat,Dsat,D2sat,sat_Int)

    # print sat(2.0)

    def  _DI_Bounded(self,p,v):

        # gains
        # kp = self.get_proportional_gain()
        # kv = self.get_derivative_gain()

        # gains
        kp = self.natural_frequency**2
        kv = 2*self.damping*self.natural_frequency

        sigma_p  = self.position_saturation
        sigma_v  = self.velocity_saturation

        eps = self.__eps 

        pp  = numpy.linalg.norm(p)
        vv  = numpy.linalg.norm(v)

        sat_p,Dsat_p,D2sat_p,sat_Int_p = self._sat(pp/sigma_p)
        sat_v,Dsat_v,D2sat_v,sat_Int_v = self._sat(vv/sigma_v)       

        I  = numpy.identity(3) 

        # vector
        h1     = kp*sat_p*p
        if pp >= eps:
            # matrix
            h1_p   = kp*Dsat_p/sigma_p*numpy.outer(p,p)/pp + kp*sat_p*I
            # tensor
            h1_p_p = numpy.zeros((3,3,3));
            for i in range(3):
                ee = I[:,i];
                h1_p_p[:,:,i] = kp*D2sat_p/sigma_p**2*numpy.outer(p,p)/(pp**2)*numpy.dot(p,ee)       + \
                                kp*Dsat_p/sigma_p*numpy.dot(p,ee)/pp*I                         + \
                                kp*Dsat_p/sigma_p*numpy.outer(p,ee)/pp                         + \
                                (-1)*kp*Dsat_p/sigma_p*numpy.outer(p,p)/pp**3*numpy.dot(p,ee)        + \
                                kp*Dsat_p/sigma_p*numpy.outer(ee,p)/pp
        else:
            h1_p          = kp*I
            h1_p_p = numpy.zeros((3,3,3))

        # vector
        h2     = kv*sat_v*v
        if vv >= eps:   
            # matrix
            h2_v   = kv*Dsat_v/sigma_v*numpy.outer(v,v)/vv + kv*sat_v*I
            # tensor
            h2_v_v = numpy.zeros((3,3,3))
            for i in range(3):
                ee = I[:,i]
                h2_v_v[:,:,i] = kv*D2sat_v/sigma_v**2*numpy.outer(v,v)/(vv**2)*numpy.dot(v,ee)    + \
                                kv*Dsat_v/sigma_v*numpy.dot(v,ee)/vv*I                      + \
                                kv*Dsat_v/sigma_v*numpy.outer(v,ee)/vv                      + \
                                (-1)*kv*Dsat_v/sigma_v*numpy.outer(v,v)/(vv**3)*numpy.dot(v,ee)   + \
                                kv*Dsat_v/sigma_v*numpy.outer(ee,v)/vv
        else:
            h2_v   = kv*I
            h2_v_v = numpy.zeros((3,3,3))


        # vector
        u     = -h1 - h2;

        # matrix
        u_p   = -h1_p;
        # tensor
        u_p_p = -h1_p_p;

        # matrix
        u_v   = - h2_v;
        # tensor
        u_v_v = - h2_v_v;

        u_p_v = numpy.zeros((3,3,3));


        beta   = 1.0/(2.0*kp);
        h1_int = kp*sigma_p**2*sat_Int_p;

        # this part is not really necessary
        if pp > eps and vv > eps:
            V  = beta*kv**2*h1_int     + \
                 beta*numpy.dot(h1,h2)       + \
                 1.0/2.0*vv**2         + \
                 h1_int                + \
                 beta*1.0/2.0*(kv**2*numpy.dot(v,v) - numpy.dot(h2,h2))

            VD = (-1)*(\
                       beta*numpy.dot(h1,h1)*kv*sat_v                                                        + \
                       numpy.dot(v,h2)*beta*kp**2*pp**2*sat_p**2*numpy.dot(p/pp,v/vv)**2*(kv*Dsat_v/sigma_v)/(kv*sat_v*vv)   + \
                       numpy.dot(v,h2)*(1.0 - beta*kp*(sat_p + Dsat_p/sigma_p*pp*numpy.dot(p/pp,v/vv)**2))                   + \
                       beta*numpy.dot(v,h2)*kv**2*(1.0 - sat_v*(Dsat_v/sigma_v*vv + sat_v))   \
                      )
        else:
            V  = 0
            VD = 0

        # V_v         = dV/d(v)
        # V_v_p = d/d(p) [dV/d(v)]
        # V_v_v = d/d(v) [dV/d(v)]

        V_p   = beta*kv**2*h1 + beta*numpy.dot(h1_p.T,h2) + h1

        V_v   = beta*numpy.dot(h2_v.T,h1) + v + beta*(kv**2*v - numpy.dot(h2_v.T,h2))

        V_v_p = beta*numpy.dot(h2_v.T,h1_p)

        V_v_v = beta*numpy.dot(h2_v_v,h1).T                                 + \
                I                                                     + \
                beta*(kv**2*I - numpy.dot(h2_v_v,h2).T  - numpy.dot(h2_v.T,h2_v))
  

        return (u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v)
        
        
        
# Test
#string = DoubleIntegratorBoundedNotComponentWiseController.to_string()
#print string
#con = DoubleIntegratorBoundedNotComponentWiseController.from_string(string)
#print con
#print con.output(zeros(3),zeros(3))
#print con.output(zeros(3), zeros(3))

# @js.add_to_database()
@js.add_to_database(database_name='database_one_d',default=True)
class OneDimensionalBoundedDIC(DoubleIntegratorController): 


    @classmethod
    def description(cls):
        return "One dimenisonal Double-integrator (bounded actuation)"

    def __init__(self,
    natural_frequency       = rospy.get_param("natural_frequency_z",1.5),
    damping                 = rospy.get_param("damping_z",numpy.sqrt(2.0)/2.0),
    position_saturation     = rospy.get_param("position_saturationy_z",0.5),
    velocity_saturation     = rospy.get_param("velocity_saturation_z",0.5)
    ):
        
        DoubleIntegratorController.__init__(self,
            natural_frequency=natural_frequency,
            damping=damping)

        self.natural_frequency = natural_frequency
        self.damping = damping
            
        self.position_saturation = position_saturation
        self.velocity_saturation = velocity_saturation
        
        self.position_saturation = position_saturation
        self.velocity_saturation = velocity_saturation

        self.__eps = 0.01

    def output(self,p,v):
        return self._DI_Bounded(p,v)


    def object_description(self):
        string = """
        Parameters:
        <ul>
          <li>natural_frequency: """ + str(self.natural_frequency) +"""</li>
          <li>damping: """ + str(self.damping) +"""</li>
          <li>position_saturation: """ + str(self.position_saturation) +"""</li>
          <li>velocity_saturation: """ + str(self.velocity_saturation) +"""</li>
        </ul>
        """
        return string        

    def __str__(self):
        string = DoubleIntegratorController.__str__(self)
        string += "\nPosition saturation: " + str(self.position_saturation)
        string += "\nVelocity saturation: " + str(self.velocity_saturation)
        return string
        
        
#        description = "Bounded Double Integrator Controller u(p,v) = -sigma(p) - ro(v): simple control law, complicated Lyapunov function\n"
#        controller  = "Parameters: sat(x) = k x/sqrt(1 + x**2), with sigma(p) = kp*sat(p/sigma_p) and ro(p) = kv*sat(v/sigma_v)\n"  
#        parameters  = "kp = proportional_gain and kv = derivative_gain and sigma_p = saturation_position and sigma_v = saturation_velocity \n\n"
#        # parameters  = "kp = " + str(self.__proportional_gain) + " and kv = " + str(self.__derivative_gain) + " and sigma_p = " +  str(self.__saturation_position) + "and sigma_v = " +  str(self.__saturation_velocity) + "\n\n"
#        return description + controller + parameters


    def _sat(self,x):

        sat     =  1.0/numpy.sqrt(1.0 + x**2)
        Dsat    =  -x*(1.0 + x**2)**(-3.0/2.0)
        D2sat   =  (-1.0 + 2.0*x**2)*(1.0 + x**2)**(-5.0/2.0)
        # primitive of sat(x)*x
        sat_Int =  numpy.sqrt(1.0 + x**2) - 1.0

        return (sat,Dsat,D2sat,sat_Int)

    # print sat(2.0)

    def  _DI_Bounded(self,p,v):

        # # gains
        # kp = self.get_proportional_gain()
        # kv = self.get_derivative_gain()

        # gains
        kp = self.natural_frequency**2
        kv = 2*self.damping*self.natural_frequency

        sigma_p  = self.position_saturation
        sigma_v  = self.velocity_saturation

        eps = self.__eps 

        pp  = numpy.linalg.norm(p)
        vv  = numpy.linalg.norm(v)

        sat_p,Dsat_p,D2sat_p,sat_Int_p = self._sat(pp/sigma_p)
        sat_v,Dsat_v,D2sat_v,sat_Int_v = self._sat(vv/sigma_v)

        # vector
        h1     = kp*sat_p*p
        if pp >= eps:
            # matrix
            h1_p   = kp*Dsat_p/sigma_p*p**2/pp + kp*sat_p
            # tensor
            h1_p_p = kp*D2sat_p/sigma_p**2*p + \
                    kp*Dsat_p/sigma_p*p/pp                            + \
                    kp*Dsat_p/sigma_p*p/pp                            + \
                    (-1)*kp*Dsat_p/sigma_p               + \
                    kp*Dsat_p/sigma_p*p/pp
        else:
            h1_p   = kp
            h1_p_p = 0.0

        # vector
        h2     = kv*sat_v*v
        if vv >= eps: 
            # matrix
            h2_v   = kv*Dsat_v/sigma_v*v**2/vv + kv*sat_v
            # tensor
            h2_v_v = kv*D2sat_v/sigma_v**2*v    + \
                    kv*Dsat_v/sigma_v*v/vv                               + \
                    kv*Dsat_v/sigma_v*v/vv                               + \
                    (-1)*kv*Dsat_v/sigma_v               + \
                    kv*Dsat_v/sigma_v*v/vv
        else:
            h2_v   = kv
            h2_v_v = 0.0


        # vector
        u     = -h1 - h2

        # matrix
        u_p   = -h1_p
        # tensor
        u_p_p = -h1_p_p

        # matrix
        u_v   = - h2_v
        # tensor
        u_v_v = - h2_v_v

        u_p_v = 0


        beta   = 1.0/(2.0*kp)
        h1_int = kp*sigma_p**2*sat_Int_p

        # this part is not really necessary
        if pp > eps and vv > eps:
            V  = beta*kv**2*h1_int     + \
                 beta*numpy.dot(h1,h2)       + \
                 1.0/2.0*vv**2         + \
                 h1_int                + \
                 beta*1.0/2.0*(kv**2*numpy.dot(v,v) - numpy.dot(h2,h2))

            VD = (-1)*(\
                       beta*numpy.dot(h1,h1)*kv*sat_v                                                        + \
                       numpy.dot(v,h2)*beta*kp**2*pp**2*sat_p**2*numpy.dot(p/pp,v/vv)**2*(kv*Dsat_v/sigma_v)/(kv*sat_v*vv)   + \
                       numpy.dot(v,h2)*(1.0 - beta*kp*(sat_p + Dsat_p/sigma_p*pp*numpy.dot(p/pp,v/vv)**2))                   + \
                       beta*numpy.dot(v,h2)*kv**2*(1.0 - sat_v*(Dsat_v/sigma_v*vv + sat_v))   \
                      )
        else:
            V  = 0
            VD = 0

        # V_v         = dV/d(v)
        # V_v_p = d/d(p) [dV/d(v)]
        # V_v_v = d/d(v) [dV/d(v)]

        V_p   = beta*kv**2*h1 + beta*h1_p*h2 + h1

        V_v   = beta*h2_v*h1 + v + beta*(kv**2*v - h2_v*h2)

        V_v_p = beta*h2_v*h1_p

        V_v_v = beta*h2_v_v*h1                                 + \
                1.0                                            + \
                beta*(kv**2 - h2_v_v*h2  - h2_v*h2_v)
  

        return (u,u_p,u_v,u_p_p,u_v_v,u_p_v,V,VD,V_p,V_v,V_v_p,V_v_v)
        
        
        
# Test
# con = NDimensionalBoundedDIC()
#print con
#print con.output(numpy.zeros(3), numpy.zeros(3))
#print con.parameters_to_string()
