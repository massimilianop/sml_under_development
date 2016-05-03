"""This module implements the utilities related to coverage missions."""

import numpy as np


BEST_DISTANCE = 1.0


def landmark_visibility(x, y):
    if x <= 0.0:
        return 0.0
    dist = np.sqrt(x**2+y**2)
    if dist <= BEST_DISTANCE:
        return x
    return x/dist**2
    
    
    
def facet_visibility(x, y, theta):
    if x <= 0.0 or np.fabs(theta) <= np.pi/2:
        return 0.0
    dist = np.sqrt(x**2+y**2)
    if dist <= BEST_DISTANCE:
        return -np.cos(theta)*x
    return -np.cos(theta)*x/dist**2
    


def coordinates_to_transformation_matrix_SO2(px, py, alpha):
    return np.array([
        [np.cos(alpha), -np.sin(alpha), px],
        [np.sin(alpha), np.cos(alpha), py],
        [0.0, 0.0, 1.0]]
        )
        
        
def transformation_matrix_to_coordinates_SO2(mtx):
    px = mtx[0][0]
    py = mtx[0][1]
    theta = np.atan2(mtx[1][1], mtx[1][0])
    return px, py, theta





class Agent:

    def __init__(self, x, y, alpha):
        self.x = x
        self.y = y
        self.alpha = alpha
        
    def __str__(self):
        string = ""
        string += "\nx = " + str(self.x)
        string += "\ny = " + str(self.y)
        string += "\nalpha = " + str(self.alpha)
        return string


    
class Landmark:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def __str__(self):
        string = ""
        string += "\nx = " + str(self.x)
        string += "\ny = " + str(self.y)
        return string
        
    def compute_visibility(self, agent):
        om_pos = np.array([self.x, self.y, 1.0])
        ag_tran_mat = coordinates_to_transformation_matrix_SO2(agent.x, agent.y, agent.alpha)
        rel_om_pos = np.linalg.inv(ag_tran_mat).dot(om_pos)
        x, y, dummy = rel_om_pos
        return landmark_visibility(x, y)
        
        
        
        
class Facet(Landmark):

    def __init__(self, x, y, theta):
        Landmark.__init__(self)
        self.theta = theta
        
    def __str__(self):
        string = Landmark.__str__(self)
        string += "\ntheta = " + str(self.theta)
        return string
        
    def compute_visibility(self, agent):
        fct_tran_mat = coordinates_to_transformation_matrix_SO2([self.x, self.y, self.theta])
        ag_tran_mat = coordinates_to_transformation_matrix_SO2(agent.x, agent.y, agent.alpha)
        rel_tran_mat = np.linalg.inv(ag_tran_mat).dot(ag_tran_mat)
        x, y, th = transformation_matrix_to_coordinates_SO2(rel_tran_mat)
        return facet_visibility(x, y, th)
        
    
        
        
        
        
        
        
        
        
        
        
"""Test"""
#lmk = Landmark(3.0, 5.0)
#agt = Agent(0.0, 0.0, 0.0)
#print lmk.compute_visibility(agt)
        
        
