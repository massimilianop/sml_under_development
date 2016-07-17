#!/usr/bin/env python
"""Subscribers"""

#node will subscribe to uav_odometry measurements
from nav_msgs.msg import Odometry

# for subscribing to topics, and publishing
import rospy

import math
import numpy
import numpy as np

import utilities.jsonable as js


class Subscriber(js.Jsonable):

    @classmethod
    def description(cls):
        description = """
        <b>Subscriber</b>
        """
        return description

    trajectory_tracking_database = {}
    @classmethod
    def add2trajectory_tracking(cls,Class):
        cls.trajectory_tracking_database[Class.__name__] = Class

    load_lifting_database = {}
    @classmethod
    def add2load_lifting(cls,Class):
        cls.load_lifting_database[Class.__name__] = Class


"""Subscribers for Firefly in Gazebo"""

from converter_firefly import RotorSConverter

class FireflyGazeboSubscriber():
    """this is just a namespace"""

    # parent class
    class FireflyGazeboSubscriber(Subscriber):
            
        @classmethod
        def description(cls):
            description = """
            <b>Subscriber for firefly</b>
            """
            return description

    #-----------------------------------------------------------#
    #-----------------------------------------------------------#
    # for trajectory traking


    # add class to database
    @js.add_to_database(default=True)
    class TrajectoryTracking(FireflyGazeboSubscriber):

        @classmethod
        def description(cls):
            description = """
            <b>Subscriber for trajectory tracking with firefly in gazebo</b>
            """
            return description
        
        def __init__(self):
            # Copy inner_keys into self variables
            # Subscribe to the necessary topics, if any

            self.initialize_state()

            # subscriber to uav_odometry from RotorS
            dictionary = {}
            dictionary['name'] = "/firefly/ground_truth/odometry"
            dictionary['data_class'] = Odometry
            dictionary['callback'] = self.get_state_from_rotorS_simulator
            self.sub_uav_odometry = rospy.Subscriber(**dictionary)

            # TODO: correct this
            self.uav_odometry =  Odometry()

        def object_description(self):
            description = """
            <b>Firefly</b>, from RotorS, to track a desired trajectory. This mission depends on:
            <ul>
              <li>controller: a trajectory tracking controller</li>
              <li>reference: a reference position trajectory to be tracked</li>
              <li>yaw_controller: a yaw controller</li>
              <li>yaw_reference: a yaw reference</li>
            </ul>
            """
            return description

        def initialize_state(self):
            # state of quad: position, velocity and attitude
            # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
            self.state_quad  = numpy.zeros(3+3+3)

        def __del__(self):
            # TODO: delete from parent as well
            # Unsubscribe from all the subscribed topics
            self.sub_uav_odometry.unregister()


        # callback when ROTORS simulator publishes states
        def get_state_from_rotorS_simulator(self,uav_odometry_rotor_s):
            self.uav_odometry = uav_odometry_rotor_s
            # RotorS has attitude inner loop that needs to known attitude of quad
            #self.rotors_object.rotor_s_attitude_for_control(uav_odometry_rotor_s)
            # get state from rotorS simulator
            self.state_quad = RotorSConverter.get_quad_state(uav_odometry_rotor_s)

        # all subscribers must provide the functionalities below
        def get_controller_state(self):
            return self.state_quad

        def get_yaw_controller_state(self):
            euler_rad     = self.state_quad[6:9]*math.pi/180
            euler_rad_dot = numpy.zeros(3)
            return numpy.concatenate([euler_rad,euler_rad_dot]) 

        def get_uav_odometry(self):
            return self.uav_odometry


    #-----------------------------------------------------------#
    #-----------------------------------------------------------#
    # for load lifting

    # to estimate load velocity: recall that velocity from rotorS comes w.r.t. the body reference frame
    from utilities.utility_functions import VelocityFilter


    # add class to database
    @js.add_to_database()
    class LoadLifting(FireflyGazeboSubscriber):

        @classmethod
        def description(cls):
            string = """Firefly and load, from RotorS, to track a desired trajectory
            This mission depends on:
            . a trajectory tracking controller
            . a reference trajectory to be tracked
            . a yaw controller
            """
            return string
            
        def __init__(self):
            # Copy the parameters into self variables
            # Subscribe to the necessary topics, if any

            # subscriber to odometry of firefly center of mass from RotorS
            self.sub_odometry = rospy.Subscriber(
                "/firefly/ground_truth/odometry",
                Odometry,
                self.update_firefly_odometry
                )

            # subscriber: to odometry of load attached to firefly
            self.sub_odometry_load = rospy.Subscriber(
                "/firefly/ground_truth/odometry_load",
                Odometry,
                self.update_load_odometry) 

            self.load_velocity_estimator = VelocityFilter(3,numpy.zeros(3),0.0)
            # self.quad_velocity_estimator = VelocityFilter(3,numpy.zeros(3),0.0)

            self.initialize_state()


        def initialize_state(self):
            # state of quad: position, velocity and attitude
            # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
            self.state_quad             = numpy.zeros(3+3+3)
            #self.load_odometry_position = numpy.array([0.0,0.0,-self.cable_length]) 
            self.load_odometry_position = numpy.array([0.0,0.0,-1.0]) 
            self.load_odometry_velocity = numpy.zeros(3)

            self.uav_odometry = Odometry()


        def __del__(self):
            # Unsubscribe from all the subscribed topics
            self.sub_odometry.unregister()

        # callback when ROTORS simulator publishes states
        def update_firefly_odometry(self,odometry_rotor_s):

            self.uav_odometry = odometry_rotor_s
            # get state from rotorS simulator
            self.state_quad = RotorSConverter.get_quad_state(odometry_rotor_s)


        def update_load_odometry(self,data_odometry):

            self.load_odometry_position = numpy.array([data_odometry.pose.pose.position.x,\
                                                       data_odometry.pose.pose.position.y,\
                                                       data_odometry.pose.pose.position.z])

            # beware that velocity obtained in this way is expressed in body reference frame
            # self.load_odometry_velocity = numpy.array([data_odometry.twist.twist.linear.x,\
            #                                            data_odometry.twist.twist.linear.y,\
            #                                            data_odometry.twist.twist.linear.z])

            current_time                = data_odometry.header.stamp.secs + data_odometry.header.stamp.nsecs/1e9
            self.load_odometry_velocity = self.load_velocity_estimator.out(self.load_odometry_position,current_time)


        # all subscribers must provide the functionalities below
        def get_controller_state(self):

            position_load = self.load_odometry_position
            velocity_load = self.load_odometry_velocity

            position_quad = self.state_quad[0:3]
            velocity_quad = self.state_quad[3:6]


            state  = numpy.concatenate([position_load, \
                                        velocity_load, \
                                        position_quad, \
                                        velocity_quad ])

            return state

        def get_yaw_controller_state(self):
            euler_rad     = self.state_quad[6:9]*math.pi/180
            euler_rad_dot = numpy.zeros(3)
            return numpy.concatenate([euler_rad,euler_rad_dot]) 

        def get_uav_odometry(self):
            return self.uav_odometry

"""Subscribers for Iris in Rviz"""


import utilities.utility_functions as uts

class IrisRvizSubscriber():
    """this is just a namespace"""

    # parent class
    class IrisRvizSubscriber(Subscriber):

        @classmethod
        def description(cls):
            description = """
            <b>Subscriber for Iris Rviz</b>
            """
            return description

    #-----------------------------------------------------------#
    #-----------------------------------------------------------#
    # for trajectory traking

    # add class to database
    @js.add_to_database(default=True)
    class TrajectoryTracking(IrisRvizSubscriber):

        @classmethod
        def description(cls):
            return "Iris, simulated, to track a desired trajectory"

        
        def __init__(self):
            # Copy the parameters into self variables
            # Subscribe to the necessary topics, if any

            # controller needs to have access to STATE: comes from simulator
            self.SubToSim = rospy.Subscriber('simulator/uav_odometry', Odometry, self.get_state_from_simulator)

            self.uav_odometry =  Odometry()
            self.state_quad = np.zeros(3+3+3)

        # def initialize_state(self):
        #     # state of quad: position, velocity and attitude 
        #     # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
            
        def __str__(self):
            return self.description()
            # Add the self variables
            
             
        def __del__(self):
            # Unsubscribe from all the subscribed topics
            self.sub_odometry.unregister()

        # callback when simulator publishes states
        def get_state_from_simulator(self, uav_odometry):

            self.uav_odometry = uav_odometry

            # position
            odometry = uav_odometry
            p = np.array([odometry.pose.pose.position.x,\
                          odometry.pose.pose.position.y,\
                          odometry.pose.pose.position.z])

            quaternion = np.array([odometry.pose.pose.orientation.x,\
                                   odometry.pose.pose.orientation.y,\
                                   odometry.pose.pose.orientation.z,\
                                   odometry.pose.pose.orientation.w])    

            rotation_matrix  = uts.rot_from_quaternion(quaternion)
            # ee = np.array([roll,pitch,yaw]) in DEGREES
            ee               = uts.euler_deg_from_rot(rotation_matrix)

            v_body = np.array([odometry.twist.twist.linear.x,\
                               odometry.twist.twist.linear.y,\
                               odometry.twist.twist.linear.z])

            v = np.dot(rotation_matrix,v_body)                

            self.state_quad = np.concatenate([p,v,ee])

        # all subscribers must provide the functionalities below
        def get_controller_state(self):
            return self.state_quad

        def get_yaw_controller_state(self):
            euler_rad     = self.state_quad[6:9]*math.pi/180
            euler_rad_dot = np.zeros(3)
            return np.concatenate([euler_rad,euler_rad_dot]) 

        def get_uav_odometry(self):
            return self.uav_odometry


    @js.add_to_database()
    class LoadLifting(IrisRvizSubscriber):

        @classmethod
        def description(cls):
            return "Iris, simulated, to track a desired trajectory"

        
        def __init__(self):
            # Copy the parameters into self variables
            # Subscribe to the necessary topics, if any

            # controller needs to have access to STATE: comes from simulator
            self.SubToSim = rospy.Subscriber('simulator/uav_odometry', Odometry, self.get_state_from_simulator)

            self.uav_odometry =  Odometry()
            self.state_quad = np.zeros(3+3+3)

        # def initialize_state(self):
        #     # state of quad: position, velocity and attitude 
        #     # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
            
        def __str__(self):
            return self.description()
            # Add the self variables
            
             
        def __del__(self):
            # Unsubscribe from all the subscribed topics
            self.sub_odometry.unregister()

        # callback when simulator publishes states
        def get_state_from_simulator(self, uav_odometry):

            self.uav_odometry = uav_odometry

            # position
            odometry = uav_odometry
            p = np.array([odometry.pose.pose.position.x,\
                          odometry.pose.pose.position.y,\
                          odometry.pose.pose.position.z])

            quaternion = np.array([odometry.pose.pose.orientation.x,\
                                   odometry.pose.pose.orientation.y,\
                                   odometry.pose.pose.orientation.z,\
                                   odometry.pose.pose.orientation.w])    

            rotation_matrix  = uts.rot_from_quaternion(quaternion)
            # ee = np.array([roll,pitch,yaw]) in DEGREES
            ee               = uts.euler_deg_from_rot(rotation_matrix)

            v_body = np.array([odometry.twist.twist.linear.x,\
                               odometry.twist.twist.linear.y,\
                               odometry.twist.twist.linear.z])

            v = np.dot(rotation_matrix,v_body)                

            self.state_quad = np.concatenate([p,v,ee])

        # all subscribers must provide the functionalities below
        def get_controller_state(self):
            return self.state_quad

        def get_yaw_controller_state(self):
            euler_rad     = self.state_quad[6:9]*math.pi/180
            euler_rad_dot = np.zeros(3)
            return np.concatenate([euler_rad,euler_rad_dot]) 

        def get_uav_odometry(self):
            return self.uav_odometry

"""Subscribers for Real Iris"""

from utilities.utility_functions import rot_from_euler_rad,quaternion_from_rot

def mocap_to_odometry(position,velocity,euler_angles,angular_velocities):

    # create a message of type quad_state with current state
    odometry = Odometry()

    odometry.pose.pose.position.x = position[0]
    odometry.pose.pose.position.y = position[1]
    odometry.pose.pose.position.z = position[2]

    VERIFY RAD OR DEGREE!!!!!
    VERIFY BODY TO INERTIAL!!!!!
    R_body_to_inertial = rot_from_euler_rad(euler_angles)

    quaternion = quaternion_from_rot(R_body_to_inertial)
    odometry.pose.pose.orientation.x = quaternion[0]
    odometry.pose.pose.orientation.y = quaternion[1]
    odometry.pose.pose.orientation.z = quaternion[2]
    odometry.pose.pose.orientation.w = quaternion[3]       

    v_body = numpy.dot(numpy.transpose(R_body_to_inertial),velocity)
    odometry.twist.twist.linear.x = v_body[0]
    odometry.twist.twist.linear.y = v_body[1]
    odometry.twist.twist.linear.z = v_body[2]

    omega_body = numpy.dot(numpy.transpose(R_body_to_inertial),angular_velocities)
    odometry.twist.twist.angular.x = omega_body[0]
    odometry.twist.twist.angular.y = omega_body[1]
    odometry.twist.twist.angular.z = omega_body[2]

    return odometry

import utilities.mocap_source as mocap_source

from utilities.utility_functions import VelocityFilter

class IrisSubscriber():
    """just a namespace"""

    # parent class
    class IrisSubscriber(Subscriber):

        @classmethod
        def description(cls):
            description = """
            <b>Subscriber for Iris Rviz</b>
            """
            return description

    #-----------------------------------------------------------#
    #-----------------------------------------------------------#
    # for trajectory traking

    # add class to database
    @js.add_to_database(default=True)
    class TrajectoryTracking(IrisSubscriber):

        @classmethod
        def description(cls):
            return "Iris to track a desired trajectory"

        def __init__(self,body_id = 29):
            # Copy the parameters into self variables
            # Subscribe to the necessary topics, if any
            
            # establish connection to qualisys
            self.Qs = mocap_source.Mocap(info=0)

            self.body_id = body_id

            # intiialization should be done in another way,
            # but median will take care of minimizing effects
            self.velocity_estimator = VelocityFilter(3,numpy.zeros(3),0.0)

            self.uav_odometry =  Odometry()
            self.state_quad = np.zeros(3+3+3)

            pass
            
             
        def __del__(self):
            # Unsubscribe from all the subscribed topics
            # self.sub_odometry.unregister()
            pass

        def get_state(self):

            bodies = self.Qs.get_body(self.body_id)

            if bodies != 'off':  

                self.flag_measurements = True

                # position
                x=bodies["x"]; y=bodies["y"]; z=bodies["z"]   
                p = numpy.array([x,y,z])

                # velocity
                #v = numpy.array([data.vx,data.vy,data.vz])
                v = self.velocity_estimator.out(p,rospy.get_time())

                # attitude: euler angles THESE COME IN DEGREES
                roll = bodies["roll"]
                pitch=-bodies["pitch"]
                yaw  = bodies["yaw"]
                ee = numpy.array([roll,pitch,yaw])

                # collect all components of state
                self.state_quad = numpy.concatenate([p,v,ee])
                
                self.uav_odometry = mocap_to_odometry(position=p,velocity=v,euler_angles=ee,angular_velocities=numpy.zeros(3))

            return self.state_quad

        # all subscribers must provide the functionalities below
        def get_controller_state(self):
            self.get_state()
            return self.state_quad

        def get_yaw_controller_state(self):
            euler_rad     = self.state_quad[6:9]*math.pi/180
            euler_rad_dot = np.zeros(3)
            return np.concatenate([euler_rad,euler_rad_dot]) 

        def get_uav_odometry(self):
            return self.uav_odometry


    @js.add_to_database()
    class LoadLifting(IrisSubscriber):

        @classmethod
        def description(cls):
            return "Iris to track a desired trajectory"

        
        def __init__(self,body_id = 29,load_id = 12):
            # Copy the parameters into self variables
            # Subscribe to the necessary topics, if any

            # establish connection to qualisys
            self.Qs = mocap_source.Mocap(info=0)

            self.body_id = body_id
            self.load_id = load_id

            # intiialization should be done in another way,
            # but median will take care of minimizing effects
            self.velocity_estimator = VelocityFilter(3,numpy.zeros(3),0.0)

            self.load_velocity_estimator = VelocityFilter(3,numpy.zeros(3),0.0)

            self.uav_odometry =  Odometry()
            self.state_quad = np.zeros(3+3+3)

            # initialize stuff
            # state of quad: position, velocity and attitude
            # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
            self.state_quad             = numpy.zeros(3+3+3)
            #self.load_odometry_position = numpy.array([0.0,0.0,-self.cable_length]) 
            self.load_odometry_position = numpy.array([0.0,0.0,-1.0]) 
            self.load_odometry_velocity = numpy.zeros(3)

            self.uav_odometry = Odometry()
             
        def __del__(self):
            # Unsubscribe from all the subscribed topics
            # self.sub_odometry.unregister()
            pass

        def get_state(self):

            bodies = self.Qs.get_body(self.body_id)

            if bodies != 'off':  

                self.flag_measurements = True

                # position
                x=bodies["x"]; y=bodies["y"]; z=bodies["z"]   
                p = numpy.array([x,y,z])

                # velocity
                #v = numpy.array([data.vx,data.vy,data.vz])
                v = self.velocity_estimator.out(p,rospy.get_time())

                # attitude: euler angles THESE COME IN DEGREES
                roll = bodies["roll"]
                pitch=-bodies["pitch"]
                yaw  = bodies["yaw"]

                VERIFY RAD OR DEGREE!!!!!

                ee = numpy.array([roll,pitch,yaw])
                
                # collect all components of state
                self.state_quad = numpy.concatenate([p,v,ee])

                self.uav_odometry = mocap_to_odometry(position=p,velocity=v,euler_angles=ee,angular_velocities=numpy.zeros(3))

            #---------------------------------------#
            # get load odometry

            bodies = self.Qs.get_body(self.load_id)

            if bodies != 'off':  

                self.flag_measurements = True

                # position
                x=bodies["x"]; y=bodies["y"]; z=bodies["z"]   
                p = numpy.array([x,y,z])
                self.load_odometry_position  = p

                # velocity
                #v = numpy.array([data.vx,data.vy,data.vz])
                v = self.load_velocity_estimator.out(p,rospy.get_time())
                self.load_odometry_velocity = v


                # self.uav_odometry = mocap_to_odometry(position=p,velocity=v,euler_angles=numpy.zeros(3),angular_velocities=numpy.zeros(3))

        # all subscribers must provide the functionalities below
        def get_controller_state(self):

            position_load = self.load_odometry_position
            velocity_load = self.load_odometry_velocity

            position_quad = self.state_quad[0:3]
            velocity_quad = self.state_quad[3:6]


            state  = numpy.concatenate([position_load, \
                                        velocity_load, \
                                        position_quad, \
                                        velocity_quad ])

            return state

        def get_yaw_controller_state(self):
            euler_rad     = self.state_quad[6:9]*math.pi/180
            euler_rad_dot = numpy.zeros(3)
            return numpy.concatenate([euler_rad,euler_rad_dot]) 

        def get_uav_odometry(self):
            return self.uav_odometry