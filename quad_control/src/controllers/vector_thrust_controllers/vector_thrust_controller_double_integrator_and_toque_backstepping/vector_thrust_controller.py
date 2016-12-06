#!/usr/bin/python

# for ploting
import matplotlib.pyplot as plt

import numpy

from numpy import *

from numpy import zeros as zeros

from controllers.double_integrator_controllers import double_integrator_controller_database
DI_CONTROLLERS_DATABASE = double_integrator_controller_database.database

from .. import vector_thrust_controller

# import orthogonal projection operator
from utilities.utility_functions import OP as OP
# import skew symmetric matrix
from utilities.utility_functions import skew as skew

import utilities.jsonable as js

class BacksteppingVectorThrustController(vector_thrust_controller.VectorThrustController): 

    js.Jsonable.add_inner('di_controller',DI_CONTROLLERS_DATABASE)

    # The class "constructor" - It's actually an initializer
    def __init__(self,
    ktt           = 200.0,
    ktt2          = 0.5,
    kw            = 200.0,
    kw2           = 0.5
    ):

        self.add_inner_defaults()

        self.ktt     = ktt
        self.ktt2    = ktt2
        self.kw      = kw
        self.kw2     = kw2


    def output(self,x,gravity):
        return self._VectorThrustController(x,gravity)

    def report(self):
        return self.DI_Ctrll.report()

    def _Vtheta(self,x):

        #     ee = self.ee
        #     ktt = self.ktt
        #     Vtheta0 =  ktt*x*(ee**2 - x**2)**(-1.0/2.0) 
        #     Vtheta1 =  ktt*ee**2*(ee**2 - x**2)^(-3.0/2.0)
        #     Vtheta2 =  ktt*3*x*ee**2*(ee**2 - x**2)**(-5.0/2.0)

        ktt   = self.ktt
        Vtheta0 = ktt*x 
        Vtheta1 = ktt
        Vtheta2 = 0

        return (Vtheta0,Vtheta1,Vtheta2)

    def _VectorThrustController(self,x,gravity):

        ad  = gravity[0:3]
        jd  = gravity[3:6]
        sd  = gravity[6:9]

        # state
        # x = [p;v;n;w]
        p = x[0:3]
        v = x[3:6]
        n = x[6:9]
        w = x[9:12]


        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.di_controller.output(p,v)

        Td   = ad + u

        Td_t = jd
        Td_p = u_p
        Td_v = u_v

        nTd      = Td/numpy.linalg.norm(Td)
        # normTd  = numpy.linalg.norm(g*e3 + ad + u)
        normTd   = numpy.linalg.norm(Td)
        normTd_t = dot(nTd,jd)
        normTd_p = dot(u_p.T,nTd)
        normTd_v = dot(u_v.T,nTd)

        # TESTED:  
        # normTdDot = normTd_t + normTd_p*v + normTd_v*(u - OP(n)*Td)
        # block.OutputPort(2).Data = [normTdnormTdDot]

        # TESTED:  
        # uDot = u_p*v + u_v*(u - OP(n)*Td)
        # block.OutputPort(2).Data = [u(1)uDot(1)]

        nTd   = Td/normTd
        nTd_t = dot(OP(nTd),jd/normTd)
        nTd_p = dot(OP(nTd),u_p/normTd)
        nTd_v = dot(OP(nTd),u_v/normTd)

        # TESTED:
        # nTdDot = nTd_t + nTd_p*v + nTd_v*(u - OP(n)*Td) 
        # block.OutputPort(2).Data = [nTd(1)nTdDot(1)]

        # normTd_t_t = nTd'*sd + nTd_t'*jd
        # normTd_p_p     = diag(u_p)*nTd_p + diag(nTd)*diag(u_p_p)
        # normTd_p_v     = diag(u_p)*nTd_v + diag(nTd)*diag(u_p_v)
        # normTd_v_p     = diag(u_v)*nTd_p + diag(nTd)*diag(u_p_v)
        # normTd_v_v     = diag(u_v)*nTd_v + diag(nTd)*diag(u_v_v)

        # nTd_p_p   = -(nTd_p*n' + nTd*nTd_p)*diag(u_p_p)/normTd + OP(nTd)*diag(u_p_p)/normTd - OP(nTd)*diag(u_p)/normTd**2*normTd_p
        # nTd_p_v   = OP(nTd)*diag(u_p)/normTd
        # nTd_v_p   = OP(nTd)*diag(u_v)/normTd
        # nTd_v_v   = OP(nTd)*diag(u_v)/normTd


        xi = 1.0 - dot(n,nTd)
        Vtt0,Vtt1,Vtt2 = self._Vtheta(xi)
        xi_t = -dot(n,nTd_t)
        xi_p = -dot(n,nTd_p)
        xi_v = -dot(n,nTd_v)
        xi_n = -nTd

        # TESTED: 
        # xiDot = xi_t + xi_p*v + xi_v*(u - OP(n)*Td) + xi_n*skew(w)*n 
        # block.OutputPort(2).Data = [xixiDot]

        # TESTED:  
        # block.OutputPort(2).Data = [Vtt1Vtt2*xiDot]


        aux_w_star   = jd + dot(u_p,v) + dot(u_v,(u - dot(OP(n),Td)))
        aux_w_star_t = sd - dot(u_v,dot(OP(n),Td_t))
        aux_w_star_p = dot(u_p_p.T,v).T + dot(u_v,u_p - dot(OP(n),Td_p)) + dot(u_p_v.T,u - dot(OP(n),Td)).T
        aux_w_star_v = u_p + dot(u_p_v.T,v).T + dot(u_v,u_v - dot(OP(n),Td_v))  + dot(u_v_v.T,u - dot(OP(n),Td)).T
        aux_w_star_n = u_v*dot(n,Td) + dot(u_v,outer(n,Td))

        # TESTED:  
        # aux_w_starDot = aux_w_star_t + aux_w_star_p*v + aux_w_star_v*(u - OP(n)*Td) + aux_w_star_n*skew(w)*n 
        # block.OutputPort(2).Data = [aux_w_star(1)aux_w_starDot(1)]

        w_star   = dot(skew(nTd),aux_w_star/normTd)

        w_star_t = dot(-skew(aux_w_star/normTd),nTd_t)               + \
                   dot(skew(nTd),aux_w_star_t/normTd)                + \
                   dot((-1.0)*skew(nTd),aux_w_star/normTd**2*normTd_t)
        w_star_p = dot(-skew(aux_w_star/normTd),nTd_p)               + \
                   dot(skew(nTd),aux_w_star_p/normTd)                + \
                   dot((-1.0)*skew(nTd),outer(aux_w_star,normTd_p)/normTd**2)
        w_star_v = dot(-skew(aux_w_star/normTd),nTd_v)               + \
                   dot(skew(nTd),aux_w_star_v/normTd)                + \
                   dot((-1.0)*skew(nTd),outer(aux_w_star,normTd_v)/normTd**2)
        w_star_n = dot(skew(nTd),aux_w_star_n/normTd)    
                  
        # TESTED:  
        # w_starDot = w_star_t + w_star_p*v + w_star_v*(u - OP(n)*Td) + w_star_n*skew(w)*n 
        # block.OutputPort(2).Data = [w_star(1)w_starDot(1)]

        # Thrust
        Thrust = dot(Td,n)

        # gains for angular control
        ktt2  = self.ktt2
        # desired angular velocity
        wd = ktt2*dot(skew(n),nTd)      + \
             w_star                     + \
             (-1.0)*dot(skew(n),V_v)*normTd*1.0/Vtt1 

        # aux1    = ktt2*skew(n)*nTd
        # aux1Dot = ktt2*skew(n)*nTd_t + ktt2*skew(n)*nTd_p*v +  ktt2*skew(n)*nTd_v*(u - OP(n)*Td) - ktt2*skew(nTd)*skew(w)*n
        # block.OutputPort(2).Data = [aux1(1)aux1Dot(1)] 

        # aux1    = skew(n)*V_v
        # aux1Dot = skew(n)*diag(V_v_p)*v +  skew(n)*diag(V_v_v)*(u - OP(n)*Td) - skew(V_v)*skew(w)*n
        # block.OutputPort(2).Data = [aux1(2)aux1Dot(2)] 

        wd_t = ktt2*dot(skew(n),nTd_t)                  + \
               w_star_t                                 + \
               (-1)*dot(skew(n),V_v)*1/Vtt1*normTd_t    + \
               dot(skew(n),V_v)*normTd*1/Vtt1**2*Vtt2*xi_t
        wd_p = ktt2*dot(skew(n),nTd_p)                              + \
               w_star_p                                             + \
               (-1)*dot(skew(n),normTd*1/Vtt1*V_v_p)                + \
               (-1)*dot(skew(n),outer(V_v,normTd_p)*1/Vtt1)         + \
               dot(skew(n),outer(V_v,xi_p)*normTd*1/Vtt1**2*Vtt2)
        wd_v = ktt2*dot(skew(n),nTd_v)                              + \
               w_star_v                                             + \
               (-1)*dot(skew(n),outer(V_v,normTd_v)*1/Vtt1)         + \
               (-1)*dot(skew(n),normTd*1/Vtt1*V_v_v)                + \
               dot(skew(n),outer(V_v,xi_v)*normTd*1/Vtt1**2*Vtt2)
        wd_n = -ktt2*skew(nTd)                          + \
               w_star_n                                 + \
               skew(V_v)*normTd*1/Vtt1                  + \
               dot(skew(n),outer(V_v,xi_n)*normTd*1/Vtt1**2*Vtt2)
              
        # TESTED:
        wdDot = wd_t + dot(wd_p,v) + dot(wd_v,(u - dot(OP(n),Td))) + dot(wd_n,dot(skew(w),n))
        # block.OutputPort(2).Data = [wd(1)wdDot(1)]
         
        kw   = self.kw
        kw2  = self.kw2
        ew   = dot(skew(n),w - wd)
        Tau  = dot(skew(n),-wdDot - 1.0/kw*Vtt1*dot(skew(n),nTd) - dot(skew(n),wd)*dot(n,wd)) + kw2*ew

        ## Lyapunov check
        V  = Vpv + Vtt0 + 1.0/2.0*kw*dot(ew,ew)
        VD = VpvD - ktt2*Vtt1*numpy.linalg.norm(dot(skew(n),nTd))**2 - kw2*kw*dot(ew,ew)

        V_dT   = dot(V_v - Vtt1*dot(nTd_v.T,n) + kw*dot(wd_v.T,dot(skew(n),ew)),n)
        V_dTau = -kw*dot(OP(n),ew)

        return (Thrust,Tau,V,VD,V_dT,V_dTau)
