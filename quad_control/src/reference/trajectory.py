"""This module implements the Trajectory abstract class,
that is used for defining a desired trajectory
for a rigid body in the 3D space"""

import numpy as np
from utilities import utility_functions as uts
from utilities import jsonable as js

import numpy as np

import rospy

DEFAULT_OFFSET = np.array([
    rospy.get_param("trajectry_offset_x",0),
    rospy.get_param("trajectry_offset_y",0),
    rospy.get_param("trajectry_offset_z",0)]) 

class Trajectory(js.Jsonable):


    @classmethod
    def description(cls):
        #raise NotImplementedError()
        return "<b>Abstract Trajectory</b> with 3D offset in (m) and rotation as [roll,pitch,yaw] in (deg)"


    def __init__(self, offset=DEFAULT_OFFSET, rotation=np.zeros(3)):

        self.offset = np.array(offset)
        self.rotation = np.array(rotation)
        self.rotation_matrix = uts.rot_from_euler_rad(self.rotation)
        #TODO change the names in the utility_functions module
    
    def object_description(self):
        string = """
        Trajectory with:
        <ul>
          <li><b>offset = """+ str(self.offset) + """</b> in (m),</li>
          <li><b>rotation = """+ str(self.rotation) + """</b> in (degrees).</li>
        </ul>
        """
        return string        
    
    def get_offset(self):
        return np.array(self.offset)
        
        
    def get_rotation(self):
        return np.array(self.rotation)
        
        
    def set_offset(self, offset):
        self.offset = np.array(offset)
        
        
    def set_rotation(self):
        self.rotation = numpy.array(rotation)
        self.rotation_matrix = uts.GetRotFromEulerAnglesDeg(rotation)
        
        
    def desired_trajectory(self, time):
        raise NotImplementedError()
    
        
    def __add_offset_and_rotation(self, pos, vel, acc, jrk, snp):
        
        off = self.offset
        rot = self.rotation_matrix
        
        pos_out = rot.dot(pos) + off
        vel_out = rot.dot(vel)
        acc_out = rot.dot(acc)
        jrk_out = rot.dot(jrk)
        snp_out = rot.dot(snp)
        
        #return pos_out, vel_out, acc_out, jrk_out, snp_out
        return np.concatenate([pos_out, vel_out, acc_out, jrk_out, snp_out])
        
        
    def output(self, time):
        return self.__add_offset_and_rotation(*self.desired_trajectory(time))


# """Test"""
#string = Trajectory.to_string()
#print string
#tr = Trajectory.from_string(string)
#print tr

@js.add_to_database(default=True)
class FixedPointTrajectory(Trajectory):
    """This class implements a fixed-point trajectory,
    i.e., a trajectory that stays in the same point forever.
    """

    @classmethod
    def description(cls):
        return "<b>Stay at rest at speficied point</b>"
        
    def __init__(self, point=np.array([0.0, 0.0, 1.0])):
        self.point = point
        Trajectory.__init__(self, offset=point, rotation=np.zeros(3))
        
    def object_description(self):
        string  = Trajectory.object_description(self)+"\n"
        return string
        
    def desired_trajectory(self, time):
        pos = np.zeros(3)
        vel = np.zeros(3)
        acc = np.zeros(3)
        jrk = np.zeros(3)
        snp = np.zeros(3)
        return pos, vel, acc, jrk, snp
        
# Test
#string = FixedPointTrajectory.to_string()
#print string
#tr = FixedPointTrajectory.from_string(string)
#print tr

@js.add_to_database()
class CircleTrajectory(Trajectory):
    """This class implements a fixed-point trajectory,
    i.e., a trajectory that stays in the same point forever.
    """

    @classmethod
    def description(cls):
        return "<b>Circle trajectory</b> with <b>radius</b> in (m), and <b>speed</b> (linear velocity) in (m/s)"
        
        
    def __init__(self,
            offset=np.array([0.0, 0.0, 1.0]),
            rotation=np.zeros(3),
            radius=1.0,
            speed=0.1):

        Trajectory.__init__(self, offset=offset, rotation=rotation)
        self.radius = radius
        self.speed = speed
    
    def object_description(self):
        string  = Trajectory.object_description(self)+"\n"
        string += "Circle trajectory with <b>radius = "+ str(self.radius) + "</b> in (m),"
        string += "and <b>speed = "+ str(self.speed) + "</b> (linear velocity) in (m/s)"        
        return string
        
    def desired_trajectory(self, time):
        c = np.cos
        s = np.sin
        r = self.radius
        w = self.speed
        t = time
        pos = r*w**0*np.array([ c(w*t),-s(w*t),0.0]);
        vel = r*w**1*np.array([-s(w*t),-c(w*t),0.0]);
        acc = r*w**2*np.array([-c(w*t), s(w*t),0.0]);
        jrk = r*w**3*np.array([ s(w*t), c(w*t),0.0]);
        snp = r*w**4*np.array([ c(w*t),-s(w*t),0.0]);
        return pos, vel, acc, jrk, snp

    
# """Test"""
#string = CircleTrajectory.to_string()
#print string
#tr = CircleTrajectory.from_string(string)
#print tr
