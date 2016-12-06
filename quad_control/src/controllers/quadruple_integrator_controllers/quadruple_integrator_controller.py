#!/usr/bin/env python
# this line is just used to define the type of document

import numpy

import scipy.linalg

from utilities import jsonable as js

class QuadrupleIntegratorController(js.Jsonable):

    
    @classmethod
    def description(cls):
        return "Abstract **Quadruple Integrator Controller**: controller u(p,v,a,j) for system p^(4) = u"

    #TODO do these have special parameters?
    def __init__(self):
        pass
        
    def __str__(self):
        string = controller.Controller.__str__(self)
        return string
    
    def output(self, position, velocity, acceleration, jerk): 
        raise NotImplementedError()

def sat(x):
    return 0.10*x/numpy.sqrt(x**2 + 0.10**2)

def sat2(x):
    return 0.30*x/numpy.sqrt(x**2 + 0.30**2)

@js.add_to_database(default=True)
class LinearQuadrupleIntegratorController(QuadrupleIntegratorController):

    @classmethod
    def description(cls):
        string = "Linear Quadruple Integrator Controller: controller u(p,v,a,j) for system p^(4) = u"
        string+= "controller for fourth order integrator: x^(4) = u(x^(0),x^(1),x^(2),x^(3)), where u = K x"
        string+= "Controller gain: K, and P is found for PA + A^T P = - I (P is important if gradient of Lyapunov is used)"
        return string


    # controller_gains  = numpy.array([-2.0, -6.0, -7.0, -4.0])
    # Id = numpy.identity(2)
    # print(numpy.kron(controller_gains,Id))

    def __init__(self, 
        controller_gains  = numpy.array([-50.0, -80.0, -20.0, -5.0]),
        controller_gains_factor = 1.0
        ):

        self.controller_gains = controller_gains
        self.controller_gains_factor = controller_gains_factor

        # xi = numpy.sqrt(2)/2
        # xi = 1.0
        # wn = 0.5
        # g  = 10
        # L  = 0.6
        # controller_gains  = numpy.array([-g/L*wn**2, -g/L*2*xi*wn, -(g/L + wn**2), -(2*xi*wn + 2)])
        # print(controller_gains)

        state_dimension = 2
        Id = numpy.identity(state_dimension)

        # K  = numpy.array([-2.0, -6.0, -7.0, -4.0])
        # KK = numpy.kron(K,Id)


        controller_gains = numpy.dot(controller_gains_factor,controller_gains)
        self.KK = numpy.kron(controller_gains,Id)

        # TODO: P is solution to P*A + A'*P = - q_matrix_lyapunov       

        # P = numpy.array([[53.0/20.0, 59.0/20.0, 31.0/20.0, 1.0/4.0],
        #                  [59.0/20.0, 243.0/40.0, 37.0/10.0, 23.0/40.0],
        #                  [31.0/20.0,37.0/10.0, 15.0/4.0, 3.0/5.0],
        #                  [1.0/4.0, 23.0/40.0, 3.0/5.0, 11.0/40.0]])

        # P is solution to P*A + A'*P = - q_matrix_lyapunov
        c = controller_gains
        A = numpy.array([[0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0],
                         [c[0], c[1], c[2], c[3]]])
        Q = numpy.identity(4)
        P = scipy.linalg.solve_lyapunov(numpy.transpose(A),-Q)

        self.PP  = numpy.kron(P,Id)


    def output(self,x1,x2,x3,x4):
        return self._quadruple_integrator(x1,x2,x3,x4)

    def object_description(self):
        string = """
        Parameters:
        <ul>
          <li>controller_gains: """ + str(self.controller_gains) +"""</li>
          <li>controller_gains_factor: """ + str(self.controller_gains_factor) +"""</li>
        </ul>
        """
        return string

    def _quadruple_integrator(self,x1,x2,x3,x4):

        # state
        x   = numpy.concatenate([sat2(x1),sat2(x2),sat(x3),sat(x4)])
        
        # control input
        u   = numpy.dot(self.KK,x)

        # gradient of Lyapunov
        V_x = numpy.dot(self.PP,x)

        V  = numpy.dot(x,numpy.dot(self.PP,x))
        VD = -numpy.dot(x,x)

        return (u,V_x,V,VD)
