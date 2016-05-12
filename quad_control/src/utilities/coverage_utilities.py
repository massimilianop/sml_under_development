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



TOLERANCE = 0.1
NUM_LANDMARKS = 100
DOMAIN_SIZE = 5.0
__sq = np.sqrt(NUM_LANDMARKS)




def point_2d_from_landmark(landmark):
    return qms.Point2D(*landmark.get_position())


def landmark_from_point_2d(point):
    return Landmark(point.x, point.y)


def point_2d_array_from_landmarks(landmarks):
    array = qms.Point2DArray()
    for lmk in landmarks:
        array.data.append(point_2d_from_landmark(lmk))
    return array
    

def landmarks_from_point_2d_array(array):
    return [landmark_from_point_2d(point) for point in array.data]




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
    
    
    
    


class Landmark:


    global DOMAIN_SIZE
    global NUM_LANDMARKS


    def __init__(self, x, y):
        self.__x = x
        self.__y = y


    def copy(self):
        return Landmark(self.__x, self.__y)


    def get_position(self):
        return self.__x, self.__y


    def __str__(self):
        string = ''
        string += '\nx: ' + str(self.__x)
        string += '\ny: ' + str(self.__y)
        return string


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


    def __init__(self, x, y, theta, landmarks, color):
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


    def set_landmarks(self, landmarks):
        self.__landmarks = list(landmarks)


    def get_landmarks(self):
        return self.__landmarks


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
        indexes_i_remove = []
        indexes_you_remove = []
        landmarks_i_add = []
        landmarks_you_add = []
        success = False
        for index, lmk in enumerate(self.__landmarks):
            if lmk.visibility(x, y, theta) > lmk.visibility(self.__x, self.__y, self.__theta) + TOLERANCE:
                indexes_i_remove.append(index)
                landmarks_you_add.append(lmk)
                success = True
        for index, lmk in enumerate(landmarks):
            if lmk.visibility(self.__x, self.__y, self.__theta) > lmk.visibility(x, y, theta) + TOLERANCE:
                indexes_you_remove.append(index)
                landmarks_i_add.append(lmk)
                success = True
        assert len(indexes_i_remove) == len(landmarks_you_add)
        assert len(indexes_you_remove) == len(landmarks_i_add)
        self.update_landmarks(indexes_i_remove, landmarks_i_add)
        return success, indexes_you_remove, landmarks_you_add


    def update_landmarks(self, indexes_i_remove, landmarks_i_add):
        filtered_landmarks = [self.__landmarks[i] for i in filter(lambda i: not i in indexes_i_remove, range(len(self.__landmarks)))]
        self.__landmarks = filtered_landmarks
        for landmark in landmarks_i_add:
            self.__landmarks.append(landmark)


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



'''Test'''
#agents = [Agent(
#*INITIAL_POSES[name],
#landmarks=INITIAL_LANDMARKS_LISTS[name],
#color=AGENTS_COLORS[name]
#) for name in AGENTS_NAMES]

#plt.figure()
#for agent in agents:
#    agent.draw()
#plt.xlim((-DOMAIN_SIZE, DOMAIN_SIZE))
#plt.ylim((-DOMAIN_SIZE, DOMAIN_SIZE))

#success, remove, add = agents[0].trade(*agents[1].get_pose(), landmarks=agents[1].get_landmarks())
#agents[1].update_landmarks(remove, add)

#plt.figure()
#for agent in agents:
#    agent.draw()
#plt.xlim((-DOMAIN_SIZE, DOMAIN_SIZE))
#plt.ylim((-DOMAIN_SIZE, DOMAIN_SIZE))

#plt.show()
