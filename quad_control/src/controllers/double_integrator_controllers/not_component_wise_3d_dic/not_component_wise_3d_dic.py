#!/usr/bin/python

import numpy

import json

from .. import double_integrator_controller as dic

class NotComponentWise3DDIC(dic.DoubleIntegratorController): 


    @classmethod
    def description(cls):
        return "Double-integrator bounded but not component-wise controller"


    def __init__(self,
            natural_frequency       = 0.5,
            damping                 = numpy.sqrt(2.0)/2.0,
            proportional_gain       = None,
            derivative_gain         = None,
            position_saturation     = 1.0,
            velocity_saturation     = 1.0
            ):
        
        if proportional_gain == None or derivative_gain==None:
            
            dic.DoubleIntegratorController.__init__(self,
                proportional_gain=natural_frequency**2,
                derivative_gain=2.0*damping*natural_frequency
                )
        
        else:
        
            dic.DoubleIntegratorController.__init__(self,
                proportional_gain=proportional_gain,
                derivative_gain=derivative_gain
                )
            
        self.__position_saturation = position_saturation
        self.__velocity_saturation = velocity_saturation

        self.__eps = 0.01


    def output(self,p,v):
        return self._DI_Bounded(p,v)

    def __str__(self):
        string = dic.DoubleIntegratorController.__str__(self)
        string += "\nPosition saturation: " + str(self.__position_saturation)
        string += "\nVelocity saturation: " + str(self.__velocity_saturation)
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
        kp = self.get_proportional_gain()
        kv = self.get_derivative_gain()

        sigma_p  = self.__position_saturation
        sigma_v  = self.__velocity_saturation

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

