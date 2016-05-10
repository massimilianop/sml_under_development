"""This module implements the utilities related to coverage missions."""

import numpy as np

import geometry_msgs.msg as gms

BEST_DISTANCE = 1.0
TOLERANCE = 0.01


def landmark_visibility(pose):
    """Visibility of a landmark
    from the point of view of an agent
    with pose x=0, y=0, th=0.
    """
    x = pose.x
    y = pose.y
    if x <= 0.0:
        return 0.0
    dist = np.sqrt(x**2+y**2)
    if dist <= BEST_DISTANCE:
        return x
    return x/dist**2
    
    
    
def facet_visibility(pose):
    """Visibility of a facet
    from the point of view of an agent
    with pose x=0, y=0, th=0.
    """
    x = pose.x
    y = pose.y
    theta = pose.theta
    if x <= 0.0 or np.fabs(theta) <= np.pi/2.0:
        return 0.0
    dist = np.sqrt(x**2+y**2)
    if dist <= BEST_DISTANCE:
        return -np.cos(theta)*x
    return -np.cos(theta)*x/dist**2
    


def transformation_matrix_SO2_from_pose_2D(pose):
    return np.array([
        [np.cos(pose.theta), -np.sin(pose.theta), pose.x],
        [np.sin(pose.theta), np.cos(pose.theta), pose.y],
        [0.0, 0.0, 1.0]]
        )
        
        
def pose_2D_from_transformation_matrix_SO2(mtx):
    px = mtx[0][2]
    py = mtx[1][2]
    theta = np.arctan2(mtx[1][0], mtx[1][1])
    return gms.Pose2D(px,py,theta)


def relative_pose(pose, agent):
    agt_mtx = transformation_matrix_SO2_from_pose_2D(agent)
    lmk_mtx = transformation_matrix_SO2_from_pose_2D(pose)
    rel_mtx = np.linalg.inv(agt_mtx).dot(lmk_mtx)
    rel_pos = pose_2D_from_transformation_matrix_SO2(rel_mtx)
    return rel_pos  



#class Agent:

#    def __init__(self, x, y, theta):
#        self.x = x
#        self.y = y
#        self.theta = theta
#        
#    def __str__(self):
#        string = ""
#        string += "\nx = " + str(self.x)
#        string += "\ny = " + str(self.y)
#        string += "\ntheta = " + str(self.theta)
#        return string


    
class Landmark:

    def __init__(self, pose):
        self.x = pose.x
        self.y = pose.y
        
    def __str__(self):
        string = ""
        string += "\nx = " + str(self.x)
        string += "\ny = " + str(self.y)
        return string
        
    def to_pose(self):
        return gms.Pose2D(self.x, self.y, 0.0)
        
    def compute_visibility(self, agent):
        pos = self.to_pose()
        rel_pos = relative_pose(pos, agent)
        return landmark_visibility(rel_pos)
        
        
        
        
class Facet(Landmark):

    def __init__(self, pose):
        Landmark.__init__(self, pose)
        self.theta = pose.theta
        
    def __str__(self):
        string = Landmark.__str__(self)
        string += "\ntheta = " + str(self.theta)
        return string
        
    def to_pose(self):
        return gms.Pose2D(self.x, self.y, self.theta)
        
    def compute_visibility(self, agent):
        rel_pos = relative_pose(self, agent)
        return facet_visibility(rel_pos)
        
    
        


def reassign_landmarks(
        agent1,
        agent2,
        lst1=[],
        lst2=[]
        ):
        
    success = False
        
    new_lst1 = []
    new_lst2 = []
    
    for lmk in lst1:
        vis1 = lmk.compute_visibility(agent1)
        vis2 = lmk.compute_visibility(agent2)
        if vis2 > vis1 + TOLERANCE:
            success = True
            new_lst2.append(lmk)
        else:
            new_lst1.append(lmk)
    
    for lmk in lst2:
        vis1 = lmk.compute_visibility(agent1)
        vis2 = lmk.compute_visibility(agent2)
        if vis1 > vis2 + TOLERANCE:
            new_lst1.append(lmk)
            success = True
        else:
            new_lst1.append(lmk)
    
    return success, new_lst1, new_lst2
        
        
        
#"""Test"""
#fct = Facet(gms.Pose2D(2.0, 3.0, 0.8*np.pi))
#agt = gms.Pose2D(0.0, 0.0, 0.0)
#print fct.compute_visibility(agt)
        
        
