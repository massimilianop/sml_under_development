import numpy as np

from utilities import utility_functions

import rospy

from .. import simulator as sim

# simulator will publish quad state
from quad_control.msg import quad_state

from visualization_msgs.msg import Marker

class ZeroSimulator(sim.Simulator):

    @classmethod
    def description(cls):
        string = """Quad simulator with zero dynamics.
            The quad stays where it is.
            The state is a 6d array,
            and corresponds to the position and velocity of the quad.
            There is no control.
            """
        return string

    def __init__(self,
        initial_time = 0.0,
        initial_position = np.array([0.0, 0.0, 1.0])
        ):
        
        self.initial_position = initial_position

        sim.Simulator.__init__(self, initial_time = initial_time)

        # this node is a simulator, thus it will publish the state of the quad
        # it uses the commands -- that it is subscribed to -- to solve differential equations 
        self.publisher = rospy.Publisher('quad_state', quad_state, queue_size=1)

        self.publisher_rviz = rospy.Publisher("visualization_marker",Marker, queue_size=1)

        self.marker_setup_rviz()
                
    def __del__(self):

        print("stop publishing uav state, and marker to rviz")
        self.publisher.unregister()
        self.publisher_rviz.unregister()

    def __str__(self):
        string = self.description()
        string += "\nTime: " + str(self.time)
        string += "\nPosition: " + str(self.state[0:3])
        return string

    def marker_setup_rviz(self):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        # marker.ns = "/simulator"
        marker.id = 0
        # marker.type = Marker.SPHERE
        # marker.action = Marker.ADD
        marker.pose.position.x = self.state[0]
        marker.pose.position.y = self.state[1]
        marker.pose.position.z = self.state[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # marker.scale.x = 1
        # marker.scale.y = 0.1
        # marker.scale.z = 0.1
        # marker.color.a = 1.0 #Don't forget to set the alpha!
        # marker.color.r = 0.0
        # marker.color.g = 1.0
        # marker.color.b = 0.0
        # //only if using a MESH_RESOURCE marker type:

        # marker.color.r = 1.0
        # marker.color.g = 1.0
        # marker.color.b = 1.0

        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "package://rotors_description/meshes/firefly.dae"
        # marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"

        self.marker = marker

    def update_rviz_marker(self):

        self.marker.pose.position.x = self.state[0]
        self.marker.pose.position.y = self.state[1]
        self.marker.pose.position.z = self.state[2]

        # quaternion
        # marker.pose.orientation.x = simstate[0]
        # marker.pose.orientation.y = simstate[1]
        # marker.pose.orientation.z = simstate[2]
        # marker.pose.orientation.w = simstate[2]

    @classmethod
    def get_state_size(cls):
        return 3+3
        
    @classmethod
    def get_control_size(cls):
        return 0

    def get_state(self):
        return np.array(self.state)

    def get_initial_state(self):
        # inital position
        p0 = self.initial_position
        # initial velocity
        v0 = np.zeros(3)

        return np.concatenate([p0,v0])
    
    def get_position(self):
        return self.state[0:3]

    def get_attitude(self):
        return None
                                
    def vector_field(self, time, state):
        return np.zeros(self.get_state_size())

    def create_message(self):

        # create a message of type quad_state_and_cmd
        state = quad_state()
        
        # get current time
        state.time = self.get_time()

        state.x  = self.state[0]
        state.y  = self.state[1]
        state.z  = self.state[2]
        state.vx = self.state[3]
        state.vy = self.state[4]
        state.vz = self.state[5]

        state.roll  = 0.0
        state.pitch = 0.0
        state.yaw   = 0.0  

        return state


        
    def publish(self):

        # create a message of type quad_state with current state
        state = self.create_message()
        # publish current state
        self.publisher.publish(state)

        self.update_rviz_marker()
        self.publisher_rviz.publish(self.marker)

#"""Test"""
#my_sim = Simulator()
#print my_sim
