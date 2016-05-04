"""This module implements the utilities related to coverage missions."""

import numpy as np


BEST_DISTANCE = 1.0
TOLERANCE = 0.01


def landmark_visibility(x, y):
    if x <= 0.0:
        return 0.0
    dist = np.sqrt(x**2+y**2)
    if dist <= BEST_DISTANCE:
        return x
    return x/dist**2
    
    
    
def facet_visibility(x, y, theta):
    if x <= 0.0 or np.fabs(theta) <= np.pi/2.0:
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
    px = mtx[0][2]
    py = mtx[1][2]
    theta = np.arctan2(mtx[1][0], mtx[1][1])
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
        Landmark.__init__(self, x, y)
        self.theta = theta
        
    def __str__(self):
        string = Landmark.__str__(self)
        string += "\ntheta = " + str(self.theta)
        return string
        
    def compute_visibility(self, agent):
        fct_tran_mat = coordinates_to_transformation_matrix_SO2(self.x, self.y, self.theta)
        ag_tran_mat = coordinates_to_transformation_matrix_SO2(agent.x, agent.y, agent.alpha)
        rel_tran_mat = np.linalg.inv(ag_tran_mat).dot(fct_tran_mat)
        x, y, th = transformation_matrix_to_coordinates_SO2(rel_tran_mat)
        return facet_visibility(x, y, th)
        
    
        

def reassign_landmarks(agent1, agent2, lst1, lst2):
    new_lst1 = []
    new_lst2 = []
    for lmk in lst1:
        vis1 = lmk.compute_visibility(agent1)
        vis2 = lmk.compute_visibility(agent2)
        if vis2 > vis1 + TOLERANCE:
            new_lst2.append(lmk)
        else:
            new_lst1.append(lmk)
    for lmk in lst2:
        vis1 = lmk.compute_visibility(agent1)
        vis2 = lmk.compute_visibility(agent2)
        if vis1 > vis2 + TOLERANCE:
            new_lst1.append(lmk)
        else:
            new_lst1.append(lmk)
    return new_lst1, new_lst2
            
        
        
        
        
        
        
        
        
"""Test"""
fct = Facet(2.0, 3.0, np.pi)
agt = Agent(0.0, 0.0, 0.0)
print fct.compute_visibility(agt)
        
        
