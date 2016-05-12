"""This module implements some utilities for a
gradient-based coverage algorithm.
"""

import numpy as np
import random as rdm
import tf.transformations as tfm
import geometry_msgs.msg as gms
import quad_control.msg as qms
import matplotlib.pyplot as plt

# For rosparam
import rospy as rp



TOLERANCE = 0.01
NUM_LANDMARKS = 16
DOMAIN_SIZE = 5.0
__sq = np.sqrt(NUM_LANDMARKS)


# def pose2D_from_pos_ver(pos, ver):
#     angle = angle_from_versor(ver)
#     return gms.Pose2D(pos[0], pos[1], angle)
    
# def pos_ver_from_pose2D(pose):
#     pos = np.array([pose.x, pose.y])
#     ver = versor_from_angle(pose.theta)
#     return pos, ver
    
    

# def landmark_from_pos(pos):
#     return qms.Landmark(pos[0], pos[1])
    
# def pos_from_landmark(lmk):
#     return np.array([lmk.x, lmk.y])
    


def versor_gradient(ver):
    return np.array([-ver[1], ver[0]])
    


def distance_factor(distance):
    return 1.0/(1.0+distance**2)
     
def distance_factor_derivative_over_distance(distance):
    return -2.0/(1.0+distance**2)**2

    
    
def versor_from_angle(theta):
    return np.array([np.cos(theta), np.sin(theta)])
    
def angle_from_versor(ver):
    return np.arctan2(ver[1], ver[0])
    
    

# def visibility_function(pos, ver, q):
#     dis = np.linalg.norm(pos-q)
#     return -distance_factor(dis)*(pos-q).dot(ver)
    
# def visibility_gradient_pos(pos, ver, q):
#     dis = np.linalg.norm(pos-q)
#     mat = -distance_factor_derivative_over_distance(dis)*np.outer(pos-q,pos-q) - distance_factor(dis)*np.eye(2)
#     return mat.dot(ver)
   
# def visibility_gradient_ver(pos, ver, q):
#     dis = np.linalg.norm(q-pos)
#     return -distance_factor(dis)*versor_gradient(ver).dot(pos-q)
    
    

# def coverage_function(pos, ver, qs):
#     cov = 0.0
#     for q in qs:
#         cov += visibility_function(pos, ver, q)
#     return cov
    
# def coverage_gradient_pos(pos, ver, qs):
#     grad = np.zeros(2)
#     for q in qs:
#         grad += visibility_gradient_pos(pos, ver, q)
#     return grad
    
# def coverage_gradient_ver(pos, ver, qs):
#     grad = 0.0
#     for q in qs:
#         grad += visibility_gradient_ver(pos, ver, q)
#     return grad
    
    

# def reassign_landmarks(p1, v1, p2, v2, lst1, lst2):
#     remove_from_1 = []
#     add_to_2 = []
#     remove_from_2 = []
#     add_to_1 = []
#     success = False
#     for i, q in enumerate(lst1):
#         if visibility_function(p2, v2, q) > visibility_function(p1, v1, q) + TOLERANCE:
#             remove_from_1.append(i)
#             add_to_2.append(q)
#             success = True
#     for i, q in enumerate(lst2):
#         if visibility_function(p1, v1, q) > visibility_function(p2, v2, q) + TOLERANCE:
#             remove_from_2.append(i)
#             add_to_1.append(q)
#             success = True
#     return success, remove_from_1, add_to_1, remove_from_2, add_to_2
    
    
    


class Landmark:


    global DOMAIN_SIZE
    global NUM_LANDMARKS


    @classmethod
    def from_pose2d(cls, pose):
        return cls(pose.x, pose.y)


    def __init__(self, x, y):
        self.__x = x
        self.__y = y


    def __str__(self):
        string = ''
        string += '\nx: ' + str(self.__x)
        string += '\ny: ' + str(self.__y)
        return string


    def to_pose2d(self):
        return gms.Pose2D(self.__x, self.__y, 0.0)


    def copy(self):
        return Landmark(self.__x, self.__y)


    def visibility(self, x, y, theta):
        q = np.array([self.__x, self.__y])
        p = np.array([x, y])
        v = versor_from_angle(theta)
        d = np.linalg.norm(p-q)
        return -distance_factor(d)*(p-q).dot(v)


    def position_visibility_gradient(self, x, y, theta):
        q = np.array([self.__x, self.__y])
        p = np.array([x, y])
        v = versor_from_angle(theta)
        d = np.linalg.norm(p-q)
        mat = -distance_factor_derivative_over_distance(d)*np.outer(p-q,p-q) - distance_factor(d)*np.eye(2)
        return mat.dot(v)


    def orientation_visibility_gradient(self, x, y, theta):
        q = np.array([self.__x, self.__y])
        p = np.array([x, y])
        v = versor_from_angle(theta)
        d = np.linalg.norm(p-q)
        return -distance_factor(d)*versor_gradient(v).dot(p-q)


    def draw(self, color):
        sq = np.sqrt(NUM_LANDMARKS)
        plt.scatter(self.__x, self.__y,
            color=color,
            s=1.2e4*DOMAIN_SIZE/NUM_LANDMARKS,
            alpha=0.3,
            edgecolor=color,
            linewidth=2,
            marker='s')




class Agent:


    global TOLERANCE


    # @classmethod
    # def from_pose2d(cls, pose):
    #     return cls(pose.x, pose.y, pose.theta)


    def __init__(self, x=0.0, y=0.0, theta=0.0, landmarks=[], color='black'):
        self.__x = x
        self.__y = y
        self.__theta = theta
        self.__landmarks = [lmk.copy() for lmk in list(landmarks)]
        self.__color = color


    def __str__(self):
        string = ''
        string += '\nx: ' + str(self.__x)
        string += '\ny: ' + str(self.__y)
        string += '\ntheta: ' + str(self.__theta)
        string += '\nnum landmarks: ' + str(len(self.__landmarks))
        return string


    def get_pose(self):
        return self.__x, self.__y, self.__theta

    def set_pose(self, x, y, theta):
        self.__x = x
        self.__y = y
        self.__theta = theta


    def get_pose2d(self):
        return gms.Pose2D(*self.get_pose())


    def set_landmarks(self, landmarks):
        self.__landmarks = [lmk.copy() for lmk in list(landmarks)]


    def get_landmarks(self):
        return [lmk.copy() for lmk in list(self.__landmarks)]


    def get_landmark_array(self):
        array = qms.LandmarkArray([lmk.to_pose2d() for lmk in self.get_landmarks()])
        assert len(array.data) == len(self.__landmarks)
        return array


    # def to_coverage_agent(self):
    #     msg = qms.CoverageAgent()
    #     msg.pose = gms.Pose2D(*self.get_pose())
    #     msg.landmarks = [lmk.to_pose2d() for lmk in self.__landmarks]
    #     return msg


    def coverage(self):
        cov = 0.0
        for lmk in self.__landmarks:
            cov += lmk.visibility(self.__x, self.__y, self.__theta)
        return cov


    def position_coverage_gradient(self):
        grad = np.zeros(2)
        for lmk in self.__landmarks:
            grad += lmk.position_visibility_gradient(self.__x, self.__y, self.__theta)
        return grad
        

    def orientation_coverage_gradient(self):
        grad = 0.0
        for lmk in self.__landmarks:
            grad += lmk.orientation_visibility_gradient(self.__x, self.__y, self.__theta)
        return grad


    def trade(self, x, y, theta, landmarks):
        for_me = list()
        for_you = list()
        success = False
        for lmk in self.get_landmarks():
            if lmk.visibility(x, y, theta) > lmk.visibility(self.__x, self.__y, self.__theta) + TOLERANCE:
                for_you.append(lmk)
                success = True
            else:
                for_me.append(lmk)
        for lmk in list(landmarks):
            if lmk.visibility(self.__x, self.__y, self.__theta) > lmk.visibility(x, y, theta) + TOLERANCE:
                for_me.append(lmk)
                success = True
            else:
                for_you.append(lmk)
        assert len(self.get_landmarks())+len(landmarks) == len(for_me) + len(for_you)
        # if success:
        if True:
            self.set_landmarks(list(for_me))
        return success, for_you


    # def update_landmarks(self, indexes_i_remove, landmarks_i_add):
    #     filtered_landmarks = [self.__landmarks[i] for i in filter(lambda i: not i in indexes_i_remove, range(len(self.__landmarks)))]
    #     self.__landmarks = list(filtered_landmarks)
    #     # for landmark in landmarks_i_add:
    #     #     self.__landmarks.append(landmark)


    def draw(self):
        x = self.__x
        y = self.__y
        th = self.__theta
        color = self.__color
        al = 1.0
        plt.scatter(x, y, c=color, marker="o", facecolor='k')
        plt.axes().arrow(x, y, al*np.cos(th), al*np.sin(th),
                 head_width=0.35*al, head_length=0.35*al, fc=color, ec='k')
        for lmk in self.__landmarks:
            lmk.draw(self.__color)







LANDMARKS = []

for index in range(NUM_LANDMARKS):
    x = float(index//__sq)/__sq*2.0*DOMAIN_SIZE-DOMAIN_SIZE+DOMAIN_SIZE/__sq
    y = float(np.mod(index, __sq))/__sq*2.0*DOMAIN_SIZE-DOMAIN_SIZE+DOMAIN_SIZE/__sq
    LANDMARKS.append(Landmark(x, y))
    
    

AGENTS_NAMES = rp.get_param('agents_names', 'Axel Bo Calle David').split()
AGENTS_COLORS = {'Axel':'blue', 'Bo':'red', 'Calle':'green', 'David':'yellow'}

INITIAL_LANDMARKS_LISTS = {}
for name in AGENTS_NAMES:
    INITIAL_LANDMARKS_LISTS[name] = []

for lmk in LANDMARKS:
    name = rdm.choice(AGENTS_NAMES)
    INITIAL_LANDMARKS_LISTS[name].append(lmk)

    
INITIAL_POSES = {}
idx = 0
for name in AGENTS_NAMES:
    INITIAL_POSES[name] = (0.0, 0.0, 2*np.pi*idx/len(AGENTS_NAMES))
    idx += 1

# '''Test'''

# agents = [Agent(*INITIAL_POSES[name],
#     landmarks=INITIAL_LANDMARKS_LISTS[name],
#     color=AGENTS_COLORS[name]) for name in AGENTS_NAMES]

# plt.figure()
# for agent in agents:
#     agent.draw()


# success, for_you = agents[0].trade(*agents[1].get_pose(), landmarks=agents[1].get_landmarks())
# agents[1].set_landmarks(for_you)

# plt.figure()
# for agent in agents:
#     agent.draw()

# plt.show()

# plt.show()