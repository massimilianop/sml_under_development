#!/usr/bin/python

# import os

# def cls():
#     os.system(['clear','cls'][os.name == 'nt'])

# # now, to clear the screen
# cls()

# # for ploting
# import matplotlib.pyplot as plt

# # for integrating
# from scipy.integrate import ode

# import numpy

# from numpy import *

# #--------------------------------------------------------------------------#
# # Some functions

# from numpy import cos as c
# from numpy import sin as s

# from numpy import sqrt  as sqrt
# from numpy import zeros as zeros

# from DI_Bounded_2 import DI_controller
# from DI_Bounded_1_to3D import DI_controller_3D



# import collections

# def skew(x):
#     out = numpy.zeros((3,3))
#     out[0,1] = -x[2]
#     out[0,2] =  x[1]
#     out[1,2] = -x[0]
#     out[1,0] =  x[2]
#     out[2,0] = -x[1]
#     out[2,1] =  x[0]
#     return out

# def OP(x):
#     out = numpy.zeros((3,3))
#     I   = numpy.identity(3)
#     out = I - outer(x,x)
#     return out

# class Vector_Thrust_Controller(object): 

#     # wn      = sqrt(2.0)
#     # xi      = sqrt(2.0)/2.0
#     # kv      = 2.0*xi*wn
#     # sigma_v = 0.5
#     # kp      = wn**2
#     # sigma_p = 0.5
#     # eps     = 0.001

#     # # kp = 6.0
#     # # kv = 4.7

#     # PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
#     # par = PAR(kv,sigma_v,kp,sigma_p,eps) 
#     # # print(par)
    
#     # DI_Ctrll = DI_controller(par)
    
#     # # ktt     = 20
#     # # ktt2    = 0.1*4
#     # # kw      = 20
#     # # kw2     = 0.1*1

#     # # factor  = 0.05
#     # # ktt     = 20.0
#     # # ktt2    = factor*kp
#     # # kw      = 20.0
#     # # kw2     = factor*kv

#     # ktt     = 500.0
#     # ktt2    = 1.0
#     # kw      = 500.0
#     # kw2     = 0.5        
    
#     # ktt2_1  = 1.0
#     # ktt2_2  = 1.0
    
#     # # The class "constructor" - It's actually an initializer
#     # # def __init__(self):
#     # #   self.M = 1.1


#     # Good for cable length of 0.6 m and attitude gain of 30!!!

#     wn      = sqrt(2.0)
#     xi      = sqrt(2.0)/2.0
#     kv      = 2.0*xi*wn
#     sigma_v = 0.2
#     kp      = wn**2
#     sigma_p = 0.2
#     eps     = 0.001

#     kv      = 0.3
#     sigma_v = 0.2
#     kp      = 1.0
#     sigma_p = 0.2
#     eps     = 0.001    

#     # kp = 6.0
#     # kv = 4.7

#     PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
#     par = PAR(kv,sigma_v,kp,sigma_p,eps) 
#     # print(par)
    
#     # DI_Ctrll = DI_controller(par)

#     DI_Ctrll = DI_controller_3D()
    
#     # ktt     = 20
#     # ktt2    = 0.1*4
#     # kw      = 20
#     # kw2     = 0.1*1

#     # factor  = 0.05
#     # ktt     = 20.0
#     # ktt2    = factor*kp
#     # kw      = 20.0
#     # kw2     = factor*kv

#     ktt     = 1200.0
#     ee      = 0.1
#     ktt2    = 8.0
#     kw      = 400.0
#     kw2     = 0.0        
    
#     ktt2_1  = 1.0
#     ktt2_2  = 1.0

#     # -------------- #
#     # good with attitude controller on rotors
#     ktt     = 800.0
#     ee      = 0.1
#     ktt2    = 2.0
#     kw      = 400.0
#     kw2     = 0.0        
    
#     ktt2_1  = 1.0
#     ktt2_2  = 1.0


#     ktt     = 1000.0
#     ee      = 0.1
#     ktt2    = 2.0
#     kw      = 1000.0
#     kw2     = 0.0        
    
#     ktt2_1  = 1.0
#     ktt2_2  = 1.0    

    
#     # The class "constructor" - It's actually an initializer
#     # def __init__(self):
#     #   self.M = 1.1

#     def output(self,x,gravity):
#         return self._VectorThrustController(x,gravity)

#     def output2(self,x,gravity):
#         return self._VectorThrustController2(x,gravity)


#     def _Vtheta(self,x):

#         # ee = self.ee
#         # ktt = self.ktt
#         # Vtheta0 =  ktt*x*(ee**2 - x**2)**(-1.0/2.0) 
#         # Vtheta1 =  ktt*ee**2*(ee**2 - x**2)**(-3.0/2.0)
#         # Vtheta2 =  ktt*3*x*ee**2*(ee**2 - x**2)**(-5.0/2.0)

#         ktt     = self.ktt
#         Vtheta0 = ktt*x 
#         Vtheta1 = ktt
#         Vtheta2 = 0

#         return (Vtheta0,Vtheta1,Vtheta2)

#     def _sat(self,x):

#         sat     =  x/sqrt(1.0 + x**2)
#         Dsat    =  (1.0 + x**2)**(-3.0/2.0)
#         D2sat   =  -3.0*x*(1.0 + x**2)**(-5.0/2.0)
#         # primitive of saturation function
#         sat_Int =  sqrt(1.0 + x**2) - 1.0

#         return (sat,Dsat,D2sat,sat_Int)


#     def _VectorThrustController(self,x,gravity):

#         ad  = gravity[0:3]
#         jd  = gravity[3:6]
#         sd  = gravity[6:9]

#         # state
#         # x = [p;v;n;w]
#         p = x[0:3]
#         v = x[3:6]
#         n = x[6:9]
#         w = x[9:12]

#         u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.DI_Ctrll.output(p,v)

#         Td   = ad + u

#         Td_t = jd
#         Td_p = u_p
#         Td_v = u_v

#         nTd      = Td/numpy.linalg.norm(Td)
#         # normTd  = numpy.linalg.norm(g*e3 + ad + u)
#         normTd   = numpy.linalg.norm(Td)
#         normTd_t = dot(nTd,jd)
#         normTd_p = dot(u_p.T,nTd)
#         normTd_v = dot(u_v.T,nTd)

#         # TESTED:  
#         # normTdDot = normTd_t + normTd_p*v + normTd_v*(u - OP(n)*Td)
#         # block.OutputPort(2).Data = [normTdnormTdDot]

#         # TESTED:  
#         # uDot = u_p*v + u_v*(u - OP(n)*Td)
#         # block.OutputPort(2).Data = [u(1)uDot(1)]

#         nTd   = Td/normTd
#         nTd_t = dot(OP(nTd),jd/normTd)
#         nTd_p = dot(OP(nTd),u_p/normTd)
#         nTd_v = dot(OP(nTd),u_v/normTd)

#         # TESTED:
#         # nTdDot = nTd_t + nTd_p*v + nTd_v*(u - OP(n)*Td) 
#         # block.OutputPort(2).Data = [nTd(1)nTdDot(1)]

#         # normTd_t_t = nTd'*sd + nTd_t'*jd
#         # normTd_p_p     = diag(u_p)*nTd_p + diag(nTd)*diag(u_p_p)
#         # normTd_p_v     = diag(u_p)*nTd_v + diag(nTd)*diag(u_p_v)
#         # normTd_v_p     = diag(u_v)*nTd_p + diag(nTd)*diag(u_p_v)
#         # normTd_v_v     = diag(u_v)*nTd_v + diag(nTd)*diag(u_v_v)

#         # nTd_p_p   = -(nTd_p*n' + nTd*nTd_p)*diag(u_p_p)/normTd + OP(nTd)*diag(u_p_p)/normTd - OP(nTd)*diag(u_p)/normTd**2*normTd_p
#         # nTd_p_v   = OP(nTd)*diag(u_p)/normTd
#         # nTd_v_p   = OP(nTd)*diag(u_v)/normTd
#         # nTd_v_v   = OP(nTd)*diag(u_v)/normTd


#         xi = 1 - dot(n,nTd)
#         Vtt0,Vtt1,Vtt2 = self._Vtheta(xi)
#         xi_t = -dot(n,nTd_t)
#         xi_p = -dot(n,nTd_p)
#         xi_v = -dot(n,nTd_v)
#         xi_n = -nTd

#         # TESTED: 
#         # xiDot = xi_t + xi_p*v + xi_v*(u - OP(n)*Td) + xi_n*skew(w)*n 
#         # block.OutputPort(2).Data = [xixiDot]

#         # TESTED:  
#         # block.OutputPort(2).Data = [Vtt1Vtt2*xiDot]


#         aux_w_star   = jd + dot(u_p,v) + dot(u_v,(u - dot(OP(n),Td)))
#         aux_w_star_t = sd - dot(u_v,dot(OP(n),Td_t))
#         aux_w_star_p = dot(u_p_p.T,v).T + dot(u_v,u_p - dot(OP(n),Td_p)) + dot(u_p_v.T,u - dot(OP(n),Td)).T
#         aux_w_star_v = u_p + dot(u_p_v.T,v).T + dot(u_v,u_v - dot(OP(n),Td_v))  + dot(u_v_v.T,u - dot(OP(n),Td)).T
#         aux_w_star_n = u_v*dot(n,Td) + dot(u_v,outer(n,Td))

#         # TESTED:  
#         # aux_w_starDot = aux_w_star_t + aux_w_star_p*v + aux_w_star_v*(u - OP(n)*Td) + aux_w_star_n*skew(w)*n 
#         # block.OutputPort(2).Data = [aux_w_star(1)aux_w_starDot(1)]

#         w_star   = dot(skew(nTd),aux_w_star/normTd)

#         w_star_t = dot(-skew(aux_w_star/normTd),nTd_t)               + \
#                    dot(skew(nTd),aux_w_star_t/normTd)                + \
#                    dot((-1)*skew(nTd),aux_w_star/normTd**2*normTd_t)
#         w_star_p = dot(-skew(aux_w_star/normTd),nTd_p)               + \
#                    dot(skew(nTd),aux_w_star_p/normTd)                + \
#                    dot((-1)*skew(nTd),outer(aux_w_star,normTd_p)/normTd**2)
#         w_star_v = dot(-skew(aux_w_star/normTd),nTd_v)               + \
#                    dot(skew(nTd),aux_w_star_v/normTd)                + \
#                    dot((-1)*skew(nTd),outer(aux_w_star,normTd_v)/normTd**2)
#         w_star_n = dot(skew(nTd),aux_w_star_n/normTd)    
                  
#         # TESTED:  
#         # w_starDot = w_star_t + w_star_p*v + w_star_v*(u - OP(n)*Td) + w_star_n*skew(w)*n 
#         # block.OutputPort(2).Data = [w_star(1)w_starDot(1)]

#         # Thrust
#         Thrust = dot(Td,n)


#         # gains for angular control
#         ktt2   = self.ktt2

#         # desired angular velocity
#         wd = ktt2*dot(skew(n),nTd)      + \
#              w_star                     + \
#              (-1)*dot(skew(n),V_v)*normTd*1/Vtt1 

#         # aux1    = ktt2*skew(n)*nTd
#         # aux1Dot = ktt2*skew(n)*nTd_t + ktt2*skew(n)*nTd_p*v +  ktt2*skew(n)*nTd_v*(u - OP(n)*Td) - ktt2*skew(nTd)*skew(w)*n
#         # block.OutputPort(2).Data = [aux1(1)aux1Dot(1)] 

#         # aux1    = skew(n)*V_v
#         # aux1Dot = skew(n)*diag(V_v_p)*v +  skew(n)*diag(V_v_v)*(u - OP(n)*Td) - skew(V_v)*skew(w)*n
#         # block.OutputPort(2).Data = [aux1(2)aux1Dot(2)] 

#         wd_t = ktt2*dot(skew(n),nTd_t)                  + \
#                w_star_t                                 + \
#                (-1)*dot(skew(n),V_v)*1/Vtt1*normTd_t    + \
#                dot(skew(n),V_v)*normTd*1/Vtt1**2*Vtt2*xi_t
#         wd_p = ktt2*dot(skew(n),nTd_p)                              + \
#                w_star_p                                             + \
#                (-1)*dot(skew(n),normTd*1/Vtt1*V_v_p)                + \
#                (-1)*dot(skew(n),outer(V_v,normTd_p)*1/Vtt1)         + \
#                dot(skew(n),outer(V_v,xi_p)*normTd*1/Vtt1**2*Vtt2)   
#         wd_v = ktt2*dot(skew(n),nTd_v)                              + \
#                w_star_v                                             + \
#                (-1)*dot(skew(n),outer(V_v,normTd_v)*1/Vtt1)         + \
#                (-1)*dot(skew(n),normTd*1/Vtt1*V_v_v)                + \
#                dot(skew(n),outer(V_v,xi_v)*normTd*1/Vtt1**2*Vtt2)
#         wd_n = -ktt2*skew(nTd)                          + \
#                w_star_n                                 + \
#                skew(V_v)*normTd*1/Vtt1                  + \
#                dot(skew(n),outer(V_v,xi_n)*normTd*1/Vtt1**2*Vtt2)
              
#         # TESTED:
#         wdDot = wd_t + dot(wd_p,v) + dot(wd_v,(u - dot(OP(n),Td))) + dot(wd_n,dot(skew(w),n))
#         # block.OutputPort(2).Data = [wd(1)wdDot(1)]
         
#         kw   = self.kw
#         kw2  = self.kw2
#         ew   = dot(skew(n),w - wd)
#         Tau  = dot(skew(n),-wdDot - 1/kw*Vtt1*dot(skew(n),nTd) - dot(skew(n),wd)*dot(n,wd)) + kw2*ew

#         ## Lyapunov check
#         V  = Vpv + Vtt0 + 1/2*kw*dot(ew,ew)
#         VD = VpvD - ktt2*Vtt1*numpy.linalg.norm(dot(skew(n),nTd))**2 - kw2*kw*dot(ew,ew)

#         V_dT   = dot(V_v - Vtt1*dot(nTd_v.T,n) + kw*dot(wd_v.T,dot(skew(n),ew)),n)
#         V_dTau = -kw*dot(OP(n),ew)

        
#         return (Thrust,Tau,V,VD,V_dT,V_dTau)


#     def _VectorThrustController2(self,x,gravity):

#         ad  = gravity[0:3]
#         jd  = gravity[3:6]
#         sd  = gravity[6:9]

#         # state
#         # x = [p;v;n;w]
#         p = x[0:3]
#         v = x[3:6]
#         n = x[6:9]
#         w = x[9:12]


#         u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.DI_Ctrll.output(p,v)

#         Td   = ad + u

#         Td_t = jd
#         Td_p = u_p
#         Td_v = u_v

#         nTd      = Td/numpy.linalg.norm(Td)
#         # normTd  = numpy.linalg.norm(g*e3 + ad + u)
#         normTd   = numpy.linalg.norm(Td)
#         normTd_t = dot(nTd,jd)
#         normTd_p = dot(u_p.T,nTd)
#         normTd_v = dot(u_v.T,nTd)

#         # TESTED:  
#         # normTdDot = normTd_t + normTd_p*v + normTd_v*(u - OP(n)*Td)
#         # block.OutputPort(2).Data = [normTdnormTdDot]

#         # TESTED:  
#         # uDot = u_p*v + u_v*(u - OP(n)*Td)
#         # block.OutputPort(2).Data = [u(1)uDot(1)]

#         nTd   = Td/normTd
#         nTd_t = dot(OP(nTd),jd/normTd)
#         nTd_p = dot(OP(nTd),u_p/normTd)
#         nTd_v = dot(OP(nTd),u_v/normTd)

#         # TESTED:
#         # nTdDot = nTd_t + nTd_p*v + nTd_v*(u - OP(n)*Td) 
#         # block.OutputPort(2).Data = [nTd(1)nTdDot(1)]

#         # normTd_t_t = nTd'*sd + nTd_t'*jd
#         # normTd_p_p     = diag(u_p)*nTd_p + diag(nTd)*diag(u_p_p)
#         # normTd_p_v     = diag(u_p)*nTd_v + diag(nTd)*diag(u_p_v)
#         # normTd_v_p     = diag(u_v)*nTd_p + diag(nTd)*diag(u_p_v)
#         # normTd_v_v     = diag(u_v)*nTd_v + diag(nTd)*diag(u_v_v)

#         # nTd_p_p   = -(nTd_p*n' + nTd*nTd_p)*diag(u_p_p)/normTd + OP(nTd)*diag(u_p_p)/normTd - OP(nTd)*diag(u_p)/normTd**2*normTd_p
#         # nTd_p_v   = OP(nTd)*diag(u_p)/normTd
#         # nTd_v_p   = OP(nTd)*diag(u_v)/normTd
#         # nTd_v_v   = OP(nTd)*diag(u_v)/normTd


#         xi = 1 - dot(n,nTd)
#         Vtt0,Vtt1,Vtt2 = self._Vtheta(xi)
#         xi_t = -dot(n,nTd_t)
#         xi_p = -dot(n,nTd_p)
#         xi_v = -dot(n,nTd_v)
#         xi_n = -nTd

#         # TESTED: 
#         # xiDot = xi_t + xi_p*v + xi_v*(u - OP(n)*Td) + xi_n*skew(w)*n 
#         # block.OutputPort(2).Data = [xixiDot]

#         # TESTED:  
#         # block.OutputPort(2).Data = [Vtt1Vtt2*xiDot]


#         aux_w_star   = jd + dot(u_p,v) + dot(u_v,(u - dot(OP(n),Td)))
#         aux_w_star_t = sd - dot(u_v,dot(OP(n),Td_t))
#         aux_w_star_p = dot(u_p_p.T,v).T + dot(u_v,u_p - dot(OP(n),Td_p)) + dot(u_p_v.T,u - dot(OP(n),Td)).T
#         aux_w_star_v = u_p + dot(u_p_v.T,v).T + dot(u_v,u_v - dot(OP(n),Td_v))  + dot(u_v_v.T,u - dot(OP(n),Td)).T
#         aux_w_star_n = u_v*dot(n,Td) + dot(u_v,outer(n,Td))

#         # TESTED:  
#         # aux_w_starDot = aux_w_star_t + aux_w_star_p*v + aux_w_star_v*(u - OP(n)*Td) + aux_w_star_n*skew(w)*n 
#         # block.OutputPort(2).Data = [aux_w_star(1)aux_w_starDot(1)]

#         w_star   = dot(skew(nTd),aux_w_star/normTd)

#         w_star_t = dot(-skew(aux_w_star/normTd),nTd_t)               + \
#                    dot(skew(nTd),aux_w_star_t/normTd)                + \
#                    dot((-1)*skew(nTd),aux_w_star/normTd**2*normTd_t)
#         w_star_p = dot(-skew(aux_w_star/normTd),nTd_p)               + \
#                    dot(skew(nTd),aux_w_star_p/normTd)                + \
#                    dot((-1)*skew(nTd),outer(aux_w_star,normTd_p)/normTd**2)
#         w_star_v = dot(-skew(aux_w_star/normTd),nTd_v)               + \
#                    dot(skew(nTd),aux_w_star_v/normTd)                + \
#                    dot((-1)*skew(nTd),outer(aux_w_star,normTd_v)/normTd**2)
#         w_star_n = dot(skew(nTd),aux_w_star_n/normTd)    
                  
#         # TESTED:  
#         # w_starDot = w_star_t + w_star_p*v + w_star_v*(u - OP(n)*Td) + w_star_n*skew(w)*n 
#         # block.OutputPort(2).Data = [w_star(1)w_starDot(1)]

#         # Thrust
#         Thrust = dot(Td,n)

#         # gains for angular control
#         ktt2   = self.ktt2
#         kkt2_p = numpy.array(3)

#         # pp      = numpy.linalg.norm(p)
#         # sigma_p = 2.0
#         # sat_p,Dsat_p,D2sat_p,sat_Int_p = self._sat(pp/sigma_p)
        
#         # ktt2     = self.ktt2_1 + self.ktt2_2*sat_p
#         # if pp > 0.01:
#         #     kkt2_p   = self.ktt2_1 + self.ktt2_2*Dsat_p*p/pp/sigma_p
#         # else:
#         #     kkt2_p   = self.ktt2_1

#         # desired angular velocity
#         wd = ktt2*dot(skew(n),nTd)      + \
#              w_star                     + \
#              (-1)*dot(skew(n),V_v)*normTd*1/Vtt1 

#         # aux1    = ktt2*skew(n)*nTd
#         # aux1Dot = ktt2*skew(n)*nTd_t + ktt2*skew(n)*nTd_p*v +  ktt2*skew(n)*nTd_v*(u - OP(n)*Td) - ktt2*skew(nTd)*skew(w)*n
#         # block.OutputPort(2).Data = [aux1(1)aux1Dot(1)] 

#         # aux1    = skew(n)*V_v
#         # aux1Dot = skew(n)*diag(V_v_p)*v +  skew(n)*diag(V_v_v)*(u - OP(n)*Td) - skew(V_v)*skew(w)*n
#         # block.OutputPort(2).Data = [aux1(2)aux1Dot(2)] 

#         wd_t = ktt2*dot(skew(n),nTd_t)                  + \
#                w_star_t                                 + \
#                (-1)*dot(skew(n),V_v)*1/Vtt1*normTd_t    + \
#                dot(skew(n),V_v)*normTd*1/Vtt1**2*Vtt2*xi_t
#         wd_p = ktt2*dot(skew(n),nTd_p)                              + \
#                w_star_p                                             + \
#                (-1)*dot(skew(n),normTd*1/Vtt1*V_v_p)                + \
#                (-1)*dot(skew(n),outer(V_v,normTd_p)*1/Vtt1)         + \
#                dot(skew(n),outer(V_v,xi_p)*normTd*1/Vtt1**2*Vtt2)   + \
#                outer(dot(skew(n),nTd),kkt2_p)
#         wd_v = ktt2*dot(skew(n),nTd_v)                              + \
#                w_star_v                                             + \
#                (-1)*dot(skew(n),outer(V_v,normTd_v)*1/Vtt1)         + \
#                (-1)*dot(skew(n),normTd*1/Vtt1*V_v_v)                + \
#                dot(skew(n),outer(V_v,xi_v)*normTd*1/Vtt1**2*Vtt2)
#         wd_n = -ktt2*skew(nTd)                          + \
#                w_star_n                                 + \
#                skew(V_v)*normTd*1/Vtt1                  + \
#                dot(skew(n),outer(V_v,xi_n)*normTd*1/Vtt1**2*Vtt2)
              
#         # TESTED:
#         wdDot = wd_t + dot(wd_p,v) + dot(wd_v,(u - dot(OP(n),Td))) + dot(wd_n,dot(skew(w),n))
#         # block.OutputPort(2).Data = [wd(1)wdDot(1)]
         
#         kw   = self.kw
#         kw2  = self.kw2
#         ew   = dot(skew(n),w - wd)
#         Tau  = dot(skew(n),-wdDot - 1/kw*Vtt1*dot(skew(n),nTd) - dot(skew(n),wd)*dot(n,wd)) + kw2*ew

#         # print dot(skew(n),-wdDot)
#         # print dot(skew(n),- 1/kw*Vtt1*dot(skew(n),nTd))
#         # print dot(skew(n),- dot(skew(n),wd)*dot(n,wd))
#         # print kw2*ew
        

#         ## Lyapunov check
#         V  = Vpv + Vtt0 + 1/2*kw*dot(ew,ew)
#         VD = VpvD - ktt2*Vtt1*numpy.linalg.norm(dot(skew(n),nTd))**2 - kw2*kw*dot(ew,ew)

#         V_dT   = dot(V_v - Vtt1*dot(nTd_v.T,n) + kw*dot(wd_v.T,dot(skew(n),ew)),n)
#         V_dTau = -kw*dot(OP(n),ew)

#         p_dot = v
#         v_dot = Thrust*n - gravity[0:3]
#         n_dot = dot(skew(w),n)
#         w_dot = dot(skew(n),Tau)

#         derivatives = concatenate([p_dot,v_dot,n_dot,w_dot])

#         Td_dot = aux_w_star
#         # Thrust: Thrust = dot(Td,n)
#         Thrust_dot = dot(Td_dot,n) + dot(Td,n_dot)        
#         Tau_dot    = numpy.zeros(3)

#         return (Thrust,Tau,V,VD,V_dT,V_dTau,Thrust_dot,Tau_dot)

























#!/usr/bin/python

import os

def cls():
    os.system(['clear','cls'][os.name == 'nt'])

# now, to clear the screen
cls()

# for ploting
import matplotlib.pyplot as plt

# for integrating
from scipy.integrate import ode

import numpy

from numpy import *

#--------------------------------------------------------------------------#
# Some functions

from numpy import cos as c
from numpy import sin as s

from numpy import sqrt  as sqrt
from numpy import zeros as zeros

# from DI_Bounded_2 import DI_controller
from DI_Bounded_1 import DI_controller
from DI_Bounded_1_to3D import DI_controller_3D



import collections


class QI_controller(object):

    state_dimension = 2
    Id = numpy.identity(state_dimension)

    K  = numpy.array([-2.0, -6.0, -7.0, -4.0])
    KK = numpy.kron(K,Id)

    P = numpy.array([[53.0/20.0, 59.0/20.0, 31.0/20.0, 1.0/4.0],
                     [59.0/20.0, 243.0/40.0, 37.0/10.0, 23.0/40.0],
                     [31.0/20.0,37.0/10.0, 15.0/4.0, 3.0/5.0],
                     [1.0/4.0, 23.0/40.0, 3.0/5.0, 11.0/40.0]])
    PP  = numpy.kron(P,Id)

    # A = [0 1 0 0;
    #      0 0 1 0;
    #      0 0 0 1;
    #      K'     ];
    #  
    # P*A + A'*P = - I


    # The class "constructor" - It's actually an initializer
    def __init__(self,parameters = None):
        if parameters is not None:
            self.K = parameters.K
            self.P = parameters.P


    def output(self,x1,x2,x3,x4):
        return self._quadruple_integrator(x1,x2,x3,x4)

    def  _quadruple_integrator(self,x1,x2,x3,x4):

        # state
        x   = numpy.concatenate([x1,x2,x3,x4])
        
        # control input
        u   = dot(self.KK,x)

        # gradient of Lyapunov
        V_x = dot(self.PP,x)

        V  = dot(x,dot(self.PP,x))
        VD = -dot(x,x)

        return (u,V_x,V,VD)





def skew(x):
    out = numpy.zeros((3,3))
    out[0,1] = -x[2]
    out[0,2] =  x[1]
    out[1,2] = -x[0]
    out[1,0] =  x[2]
    out[2,0] = -x[1]
    out[2,1] =  x[0]
    return out

def OP(x):
    out = numpy.zeros((3,3))
    I   = numpy.identity(3)
    out = I - outer(x,x)
    return out

class Vector_Thrust_Controller(object): 

    # # wn      = sqrt(2.0)
    # # xi      = sqrt(2.0)/2.0
    # # kv      = 2.0*xi*wn
    # # sigma_v = 0.5
    # # kp      = wn**2
    # # sigma_p = 0.5
    # # eps     = 0.001

    # # # kp = 6.0
    # # # kv = 4.7

    # # PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
    # # par = PAR(kv,sigma_v,kp,sigma_p,eps) 
    # # # print(par)
    
    # # DI_Ctrll = DI_controller(par)
    
    # # # ktt     = 20
    # # # ktt2    = 0.1*4
    # # # kw      = 20
    # # # kw2     = 0.1*1

    # # # factor  = 0.05
    # # # ktt     = 20.0
    # # # ktt2    = factor*kp
    # # # kw      = 20.0
    # # # kw2     = factor*kv

    # # ktt     = 500.0
    # # ktt2    = 1.0
    # # kw      = 500.0
    # # kw2     = 0.5        
    
    # # ktt2_1  = 1.0
    # # ktt2_2  = 1.0
    
    # # # The class "constructor" - It's actually an initializer
    # # # def __init__(self):
    # # #   self.M = 1.1


    # # Good for cable length of 0.6 m and attitude gain of 30!!!

    # wn      = sqrt(2.0)
    # xi      = sqrt(2.0)/2.0
    # kv      = 2.0*xi*wn
    # sigma_v = 0.2
    # kp      = wn**2
    # sigma_p = 0.2
    # eps     = 0.001

    # kv      = 0.3
    # sigma_v = 0.2
    # kp      = 1.0
    # sigma_p = 0.2
    # eps     = 0.001    

    # # kp = 6.0
    # # kv = 4.7

    # PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
    # par = PAR(kv,sigma_v,kp,sigma_p,eps) 
    # # print(par)
    
    # # DI_Ctrll = DI_controller(par)

    # DI_Ctrll = DI_controller_3D()
    
    # # ktt     = 20
    # # ktt2    = 0.1*4
    # # kw      = 20
    # # kw2     = 0.1*1

    # # factor  = 0.05
    # # ktt     = 20.0
    # # ktt2    = factor*kp
    # # kw      = 20.0
    # # kw2     = factor*kv

    # ktt     = 1200.0
    # ee      = 0.1
    # ktt2    = 8.0
    # kw      = 400.0
    # kw2     = 0.0        
    
    # ktt2_1  = 1.0
    # ktt2_2  = 1.0

    # # -------------- #
    # # good with attitude controller on rotors
    # ktt     = 800.0
    # ee      = 0.1
    # ktt2    = 2.0
    # kw      = 400.0
    # kw2     = 0.0        
    
    # ktt2_1  = 1.0
    # ktt2_2  = 1.0


    # ktt     = 1000.0
    # ee      = 0.1
    # ktt2    = 2.0
    # kw      = 1000.0
    # kw2     = 0.0        
    
    # ktt2_1  = 1.0
    # ktt2_2  = 1.0    

    wn      = 2.0
    xi      = sqrt(2)/2.0
    kv      = 2.0*xi*wn
    sigma_v = 0.5
    kp      = wn**2
    sigma_p = 0.5
    eps     = 0.01

    PAR = collections.namedtuple('DI_paramteres',['kv','sigma_v','kp','sigma_p','eps'])
    par = PAR(kv,sigma_v,kp,sigma_p,eps) 
    # print(par)
    
    DI_Ctrll = DI_controller(par)
    
    QI_Ctrll = QI_controller()    

    
    # The class "constructor" - It's actually an initializer
    # def __init__(self):
    #   self.M = 1.1

    def output(self,x,gravity):
        return self._VectorThrustController(x,gravity)

    def output2(self,x,gravity):
        return self._VectorThrustController2(x,gravity)


    def _Vtheta(self,x):

        # ee = self.ee
        # ktt = self.ktt
        # Vtheta0 =  ktt*x*(ee**2 - x**2)**(-1.0/2.0) 
        # Vtheta1 =  ktt*ee**2*(ee**2 - x**2)**(-3.0/2.0)
        # Vtheta2 =  ktt*3*x*ee**2*(ee**2 - x**2)**(-5.0/2.0)

        ktt     = self.ktt
        Vtheta0 = ktt*x 
        Vtheta1 = ktt
        Vtheta2 = 0

        return (Vtheta0,Vtheta1,Vtheta2)

    def _sat(self,x):

        sat     =  x/sqrt(1.0 + x**2)
        Dsat    =  (1.0 + x**2)**(-3.0/2.0)
        D2sat   =  -3.0*x*(1.0 + x**2)**(-5.0/2.0)
        # primitive of saturation function
        sat_Int =  sqrt(1.0 + x**2) - 1.0

        return (sat,Dsat,D2sat,sat_Int)


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

        u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.DI_Ctrll.output(p,v)

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


        xi = 1 - dot(n,nTd)
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
                   dot((-1)*skew(nTd),aux_w_star/normTd**2*normTd_t)
        w_star_p = dot(-skew(aux_w_star/normTd),nTd_p)               + \
                   dot(skew(nTd),aux_w_star_p/normTd)                + \
                   dot((-1)*skew(nTd),outer(aux_w_star,normTd_p)/normTd**2)
        w_star_v = dot(-skew(aux_w_star/normTd),nTd_v)               + \
                   dot(skew(nTd),aux_w_star_v/normTd)                + \
                   dot((-1)*skew(nTd),outer(aux_w_star,normTd_v)/normTd**2)
        w_star_n = dot(skew(nTd),aux_w_star_n/normTd)    
                  
        # TESTED:  
        # w_starDot = w_star_t + w_star_p*v + w_star_v*(u - OP(n)*Td) + w_star_n*skew(w)*n 
        # block.OutputPort(2).Data = [w_star(1)w_starDot(1)]

        # Thrust
        Thrust = dot(Td,n)


        # gains for angular control
        ktt2   = self.ktt2

        # desired angular velocity
        wd = ktt2*dot(skew(n),nTd)      + \
             w_star                     + \
             (-1)*dot(skew(n),V_v)*normTd*1/Vtt1 

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
        Tau  = dot(skew(n),-wdDot - 1/kw*Vtt1*dot(skew(n),nTd) - dot(skew(n),wd)*dot(n,wd)) + kw2*ew

        ## Lyapunov check
        V  = Vpv + Vtt0 + 1/2*kw*dot(ew,ew)
        VD = VpvD - ktt2*Vtt1*numpy.linalg.norm(dot(skew(n),nTd))**2 - kw2*kw*dot(ew,ew)

        V_dT   = dot(V_v - Vtt1*dot(nTd_v.T,n) + kw*dot(wd_v.T,dot(skew(n),ew)),n)
        V_dTau = -kw*dot(OP(n),ew)

        
        return (Thrust,Tau,V,VD,V_dT,V_dTau)


    def _VectorThrustController2(self,x,gravity):

    #     ad  = gravity[0:3]
    #     jd  = gravity[3:6]
    #     sd  = gravity[6:9]

    #     # state
    #     # x = [p;v;n;w]
    #     p = x[0:3]
    #     v = x[3:6]
    #     n = x[6:9]
    #     w = x[9:12]


    #     u,u_p,u_v,u_p_p,u_v_v,u_p_v,Vpv,VpvD,V_p,V_v,V_v_p,V_v_v = self.DI_Ctrll.output(p,v)

    #     Td   = ad + u

    #     Td_t = jd
    #     Td_p = u_p
    #     Td_v = u_v

    #     nTd      = Td/numpy.linalg.norm(Td)
    #     # normTd  = numpy.linalg.norm(g*e3 + ad + u)
    #     normTd   = numpy.linalg.norm(Td)
    #     normTd_t = dot(nTd,jd)
    #     normTd_p = dot(u_p.T,nTd)
    #     normTd_v = dot(u_v.T,nTd)

    #     # TESTED:  
    #     # normTdDot = normTd_t + normTd_p*v + normTd_v*(u - OP(n)*Td)
    #     # block.OutputPort(2).Data = [normTdnormTdDot]

    #     # TESTED:  
    #     # uDot = u_p*v + u_v*(u - OP(n)*Td)
    #     # block.OutputPort(2).Data = [u(1)uDot(1)]

    #     nTd   = Td/normTd
    #     nTd_t = dot(OP(nTd),jd/normTd)
    #     nTd_p = dot(OP(nTd),u_p/normTd)
    #     nTd_v = dot(OP(nTd),u_v/normTd)

    #     # TESTED:
    #     # nTdDot = nTd_t + nTd_p*v + nTd_v*(u - OP(n)*Td) 
    #     # block.OutputPort(2).Data = [nTd(1)nTdDot(1)]

    #     # normTd_t_t = nTd'*sd + nTd_t'*jd
    #     # normTd_p_p     = diag(u_p)*nTd_p + diag(nTd)*diag(u_p_p)
    #     # normTd_p_v     = diag(u_p)*nTd_v + diag(nTd)*diag(u_p_v)
    #     # normTd_v_p     = diag(u_v)*nTd_p + diag(nTd)*diag(u_p_v)
    #     # normTd_v_v     = diag(u_v)*nTd_v + diag(nTd)*diag(u_v_v)

    #     # nTd_p_p   = -(nTd_p*n' + nTd*nTd_p)*diag(u_p_p)/normTd + OP(nTd)*diag(u_p_p)/normTd - OP(nTd)*diag(u_p)/normTd**2*normTd_p
    #     # nTd_p_v   = OP(nTd)*diag(u_p)/normTd
    #     # nTd_v_p   = OP(nTd)*diag(u_v)/normTd
    #     # nTd_v_v   = OP(nTd)*diag(u_v)/normTd


    #     xi = 1 - dot(n,nTd)
    #     Vtt0,Vtt1,Vtt2 = self._Vtheta(xi)
    #     xi_t = -dot(n,nTd_t)
    #     xi_p = -dot(n,nTd_p)
    #     xi_v = -dot(n,nTd_v)
    #     xi_n = -nTd

    #     # TESTED: 
    #     # xiDot = xi_t + xi_p*v + xi_v*(u - OP(n)*Td) + xi_n*skew(w)*n 
    #     # block.OutputPort(2).Data = [xixiDot]

    #     # TESTED:  
    #     # block.OutputPort(2).Data = [Vtt1Vtt2*xiDot]


    #     aux_w_star   = jd + dot(u_p,v) + dot(u_v,(u - dot(OP(n),Td)))
    #     aux_w_star_t = sd - dot(u_v,dot(OP(n),Td_t))
    #     aux_w_star_p = dot(u_p_p.T,v).T + dot(u_v,u_p - dot(OP(n),Td_p)) + dot(u_p_v.T,u - dot(OP(n),Td)).T
    #     aux_w_star_v = u_p + dot(u_p_v.T,v).T + dot(u_v,u_v - dot(OP(n),Td_v))  + dot(u_v_v.T,u - dot(OP(n),Td)).T
    #     aux_w_star_n = u_v*dot(n,Td) + dot(u_v,outer(n,Td))

    #     # TESTED:  
    #     # aux_w_starDot = aux_w_star_t + aux_w_star_p*v + aux_w_star_v*(u - OP(n)*Td) + aux_w_star_n*skew(w)*n 
    #     # block.OutputPort(2).Data = [aux_w_star(1)aux_w_starDot(1)]

    #     w_star   = dot(skew(nTd),aux_w_star/normTd)

    #     w_star_t = dot(-skew(aux_w_star/normTd),nTd_t)               + \
    #                dot(skew(nTd),aux_w_star_t/normTd)                + \
    #                dot((-1)*skew(nTd),aux_w_star/normTd**2*normTd_t)
    #     w_star_p = dot(-skew(aux_w_star/normTd),nTd_p)               + \
    #                dot(skew(nTd),aux_w_star_p/normTd)                + \
    #                dot((-1)*skew(nTd),outer(aux_w_star,normTd_p)/normTd**2)
    #     w_star_v = dot(-skew(aux_w_star/normTd),nTd_v)               + \
    #                dot(skew(nTd),aux_w_star_v/normTd)                + \
    #                dot((-1)*skew(nTd),outer(aux_w_star,normTd_v)/normTd**2)
    #     w_star_n = dot(skew(nTd),aux_w_star_n/normTd)    
                  
    #     # TESTED:  
    #     # w_starDot = w_star_t + w_star_p*v + w_star_v*(u - OP(n)*Td) + w_star_n*skew(w)*n 
    #     # block.OutputPort(2).Data = [w_star(1)w_starDot(1)]

    #     # Thrust
    #     Thrust = dot(Td,n)

    #     # gains for angular control
    #     ktt2   = self.ktt2
    #     kkt2_p = numpy.array(3)

    #     # pp      = numpy.linalg.norm(p)
    #     # sigma_p = 2.0
    #     # sat_p,Dsat_p,D2sat_p,sat_Int_p = self._sat(pp/sigma_p)
        
    #     # ktt2     = self.ktt2_1 + self.ktt2_2*sat_p
    #     # if pp > 0.01:
    #     #     kkt2_p   = self.ktt2_1 + self.ktt2_2*Dsat_p*p/pp/sigma_p
    #     # else:
    #     #     kkt2_p   = self.ktt2_1

    #     # desired angular velocity
    #     wd = ktt2*dot(skew(n),nTd)      + \
    #          w_star                     + \
    #          (-1)*dot(skew(n),V_v)*normTd*1/Vtt1 

    #     # aux1    = ktt2*skew(n)*nTd
    #     # aux1Dot = ktt2*skew(n)*nTd_t + ktt2*skew(n)*nTd_p*v +  ktt2*skew(n)*nTd_v*(u - OP(n)*Td) - ktt2*skew(nTd)*skew(w)*n
    #     # block.OutputPort(2).Data = [aux1(1)aux1Dot(1)] 

    #     # aux1    = skew(n)*V_v
    #     # aux1Dot = skew(n)*diag(V_v_p)*v +  skew(n)*diag(V_v_v)*(u - OP(n)*Td) - skew(V_v)*skew(w)*n
    #     # block.OutputPort(2).Data = [aux1(2)aux1Dot(2)] 

    #     wd_t = ktt2*dot(skew(n),nTd_t)                  + \
    #            w_star_t                                 + \
    #            (-1)*dot(skew(n),V_v)*1/Vtt1*normTd_t    + \
    #            dot(skew(n),V_v)*normTd*1/Vtt1**2*Vtt2*xi_t
    #     wd_p = ktt2*dot(skew(n),nTd_p)                              + \
    #            w_star_p                                             + \
    #            (-1)*dot(skew(n),normTd*1/Vtt1*V_v_p)                + \
    #            (-1)*dot(skew(n),outer(V_v,normTd_p)*1/Vtt1)         + \
    #            dot(skew(n),outer(V_v,xi_p)*normTd*1/Vtt1**2*Vtt2)   + \
    #            outer(dot(skew(n),nTd),kkt2_p)
    #     wd_v = ktt2*dot(skew(n),nTd_v)                              + \
    #            w_star_v                                             + \
    #            (-1)*dot(skew(n),outer(V_v,normTd_v)*1/Vtt1)         + \
    #            (-1)*dot(skew(n),normTd*1/Vtt1*V_v_v)                + \
    #            dot(skew(n),outer(V_v,xi_v)*normTd*1/Vtt1**2*Vtt2)
    #     wd_n = -ktt2*skew(nTd)                          + \
    #            w_star_n                                 + \
    #            skew(V_v)*normTd*1/Vtt1                  + \
    #            dot(skew(n),outer(V_v,xi_n)*normTd*1/Vtt1**2*Vtt2)
              
    #     # TESTED:
    #     wdDot = wd_t + dot(wd_p,v) + dot(wd_v,(u - dot(OP(n),Td))) + dot(wd_n,dot(skew(w),n))
    #     # block.OutputPort(2).Data = [wd(1)wdDot(1)]
         
    #     kw   = self.kw
    #     kw2  = self.kw2
    #     ew   = dot(skew(n),w - wd)
    #     Tau  = dot(skew(n),-wdDot - 1/kw*Vtt1*dot(skew(n),nTd) - dot(skew(n),wd)*dot(n,wd)) + kw2*ew

    #     # print dot(skew(n),-wdDot)
    #     # print dot(skew(n),- 1/kw*Vtt1*dot(skew(n),nTd))
    #     # print dot(skew(n),- dot(skew(n),wd)*dot(n,wd))
    #     # print kw2*ew
        

    #     ## Lyapunov check
    #     V  = Vpv + Vtt0 + 1/2*kw*dot(ew,ew)
    #     VD = VpvD - ktt2*Vtt1*numpy.linalg.norm(dot(skew(n),nTd))**2 - kw2*kw*dot(ew,ew)

    #     V_dT   = dot(V_v - Vtt1*dot(nTd_v.T,n) + kw*dot(wd_v.T,dot(skew(n),ew)),n)
    #     V_dTau = -kw*dot(OP(n),ew)

    #     p_dot = v
    #     v_dot = Thrust*n - gravity[0:3]
    #     n_dot = dot(skew(w),n)
    #     w_dot = dot(skew(n),Tau)

    #     derivatives = concatenate([p_dot,v_dot,n_dot,w_dot])

    #     Td_dot = aux_w_star
    #     # Thrust: Thrust = dot(Td,n)
    #     Thrust_dot = dot(Td_dot,n) + dot(Td,n_dot)        
    #     Tau_dot    = numpy.zeros(3)

    #     return (Thrust,Tau,V,VD,V_dT,V_dTau,Thrust_dot,Tau_dot)

    # def _VectorThrustController(self,x,gravity):

        e3 = numpy.array([0.0,0.0,1.0])
        g_0t = gravity[0:3]
        g_1t = gravity[3:6]
        g_2t = gravity[6:9]

        # initialize gradient of Lyapunov w.r.t. state
        V_x = numpy.zeros(12)

        # state
        # x = [p;v;n;w]
        p = x[0:3]
        v = x[3:6]
        n = x[6:9]
        n3 = x[8]
        w = x[9:12]


        # control z direction with double integrator

        # z position
        z  = x[2]
        # z velocity
        vz = x[5]

        # self.DI_Ctrll.output(p,v)
        u_z,u_p_z,u_v_z,u_p_p_z,u_v_v_z,u_p_v_z,V_z,VD_z,V_p_z,V_v_z,V_v_p_z,V_v_v_z = self.DI_Ctrll.output(z,vz);
        
        V_x[2] = V_p_z
        V_x[5] = V_v_z

        Thrust_cl = 1.0/n3*(g_0t[2] + u_z)

        # finding quadruple integartor for xy direction

        # auxilar matrix 
        # (extracts first and second component from three dimentional vector)
        PP = numpy.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0]])

        xi1 = x[0:2]
        xi1_grad_x      = numpy.zeros((2,12))
        xi1_grad_x[0,0] = 1.0
        xi1_grad_x[1,1] = 1.0

        xi2 = x[3:5]
        xi2_grad_x = zeros((2,12))
        xi2_grad_x[0,3] = 1
        xi2_grad_x[1,4] = 1 

        xi3 = 1.0/n3*(u_z + g_0t[2])*n[0:2] - g_0t[0:2]
        # xi3 = 1.0/n3*u_z*n[0:2] + 1.0/n3*PP*skew(e3)*skew(n)*g_0t
        xi3_grad_x = numpy.zeros((2,12))
        xi3_grad_x[0:2,2]   =  1.0/n3*n[0:2]*u_p_z
        xi3_grad_x[0:2,5]   =  1.0/n3*n[0:2]*u_v_z
        xi3_grad_x[0:2,6:9] = -1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3),skew(n)))


        xi3_t =  1.0/n3*dot(PP,dot(skew(e3),dot(skew(n),g_1t)))
        xi3_x = -1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3),dot(OP(n),w))) + 1.0/n3*n[0:2]*(u_p_z*vz + u_v_z*u_z)
        xi4   = xi3_t + xi3_x
        xi4_grad_x = numpy.zeros((2,12))
        xi4_grad_x[0:2,2]     = -1.0/n3**2*u_p_z*dot(PP,dot(skew(e3),dot(OP(n),w))) + 1.0/n3*n[0:2]*(u_p_p_z*vz + u_p_v_z*u_z + u_v_z*u_p_z)
        xi4_grad_x[0:2,5]     = -1.0/n3**2*u_p_z*dot(PP,dot(skew(e3),dot(OP(n),w))) + 1.0/n3*n[0:2]*(u_p_v_z*vz + u_p_z + u_v_v_z*u_z + u_v_z*u_v_z)
        xi4_grad_x[0:2,6:9]   = -(1.0/n3*dot(PP,dot(skew(e3),skew(g_1t))) + 1.0/n3**2*dot(PP,dot(skew(e3),dot(skew(n),outer(g_1t,e3))))) + \
                                -1.0/n3**2*dot(PP,dot(skew(e3),skew(n)))*(u_p_z*vz + u_v_z*u_z) + \
                                -1.0/n3**2*(u_p_z*vz + u_v_z*u_z)*dot(PP,dot(skew(e3),skew(n))) + \
                                 2.0/n3**5*dot(e3,dot(skew(w),n))*(u_z + g_0t[2])*dot(PP,dot(skew(e3),skew(n))) + \
                                 1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3),dot(n,w)*numpy.identity(3) + outer(n,w)))
        xi4_grad_x[0:2,9:12] = -1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3),OP(n)))

        # 8 by 12
        xi_grad_x = numpy.concatenate([xi1_grad_x,xi2_grad_x,xi3_grad_x,xi4_grad_x])

        xi3_t_t =  1.0/n3*dot(PP,dot(skew(e3),dot(skew(n),g_2t)))
        xi3_t_x = -dot(1.0/n3*dot(PP,dot(skew(e3),skew(g_1t))) + 1.0/n3**2*dot(PP,dot(skew(e3),dot(skew(n),outer(g_1t,e3)))),dot(skew(w),n))
        xi3_x_t = -1.0/n3**2*g_1t[2]*dot(PP,dot(skew(e3),dot(OP(n),w)))
        xi3_x_x = -1.0/n3**2*dot(PP,dot(skew(e3),dot(OP(n),w)))*(u_p_z*vz + u_v_z*u_z) + \
                   1.0/n3*n[0:2]*((u_p_p_z*vz + u_p_v_z*u_z)*vz + (u_p_v_z*vz + u_v_v_z*u_z)*u_z + u_p_z*u_z + u_v_z*(u_p_z*vz + u_v_z*u_z)) + \
                  -1.0/n3**2*(u_p_z*vz + u_v_z*u_z)*dot(PP,dot(skew(e3),dot(OP(n),w))) + \
                   2/n3**5*dot(e3,dot(skew(w),n))*(u_z + g_0t[2])*dot(PP,dot(skew(e3),dot(OP(n),w))) + \
                   1.0/n3**2*(u_z + g_0t[2])*dot(PP,dot(skew(e3), dot(outer(dot(skew(w),n),n) + outer(n,dot(skew(w),n)) ,w))) 
               
               
        xi5 = xi3_t_t + xi3_t_x + xi3_x_t + xi3_x_x

        # control xy directions with quadruple integrator 

        [u_quadr_int,V_xi,V_of_xi,VD_of_xi] = self.QI_Ctrll.output(xi1,xi2,xi3,xi4)

        # -1.0/n3**2*(u_z + g_0t[2])*PP*skew(e3)*skew(n)*tau
        Tau_cl = n3/(u_z + g_0t[2])*dot(OP(n),numpy.concatenate([u_quadr_int - xi5,[0]]))

        # V_xi  is 8 by 1
        V_x           = V_x + dot(V_xi,xi_grad_x)

        V  = V_z  + V_of_xi
        VD = VD_z + VD_of_xi

        # Thrust_cl = 0.0
        # Tau_cl    = numpy.zeros(3)
        # V         = 0
        # VD        = 0
        # V_x       = numpy.zeros(12)

        # Thrust_dot = dot(Td_dot,n) + dot(Td,n_dot)        
        # Tau_dot    = numpy.zeros(3)

        return (Thrust_cl,Tau_cl,V,VD,numpy.zeros(1),numpy.zeros(3),numpy.zeros(1),numpy.zeros(3))              