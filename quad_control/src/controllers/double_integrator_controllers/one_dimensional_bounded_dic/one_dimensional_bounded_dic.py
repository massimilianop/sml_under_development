#!/usr/bin/python
""" 
Double Integrator Controller: 
description:
"""

import numpy

from .. import double_integrator_controller as dic

import json

class OneDimensionalBoundedDIC(dic.DoubleIntegratorController): 


    @classmethod
    def description(cls):
        return "Double-integrator bounded but not component-wise controller"

    
    @classmethod
    def parameters_to_string(cls,   \
        proportional_gain     = 1.0, \
        derivative_gain       = 1.0, \
        position_saturation   = 1.0, \
        velocity_saturation   = 1.0):

        dic = {
            'proportional_gain'   :proportional_gain    ,\
            'derivative_gain'     :derivative_gain      ,\
            'position_saturation' :position_saturation  ,\
            'velocity_saturation' :velocity_saturation
            }

        return json.dumps(dic, indent=4)    
        
    @classmethod
    def string_to_parameters(cls, string):
        
        dic = json.loads(string)
        
#        proportional_gain   = dic['proportional_gain']
#        derivative_gain     = dic['derivative_gain']
#        saturation_position = dic['saturation_position']
#        saturation_velocity = dic['saturation_velocity']

#        return proportional_gain, derivative_gain, saturation_position, saturation_velocity
        
        return dic 


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
