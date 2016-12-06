#!/usr/bin/env python
# this line is just used to define the type of document

import utilities.mocap_source as mocap_source

#node will subscribe to uav_odometry measurements
from nav_msgs.msg import Odometry

# for obtaining rotation and quaternion from euler angles
import utilities.utility_functions as uts

# set up publisher
import rospy

import numpy

class BodyPublisher():

    def __init__(self):

        # frequency of publishing
        self.frequency = rospy.get_param(param_name='publishing_frequency',default=50)

        self.bodies_names = ['Iris5','Iris2','Load1','Truck','Truck2']
        self.child_frame_id = ['base_uav','base_uav','base_world','base_world','base_world']

        # self.bodies_names = ['Iris5/Iris5','Iris2/Iris2','Load1','Truck','Truck2']
        # self.child_frame_id = ['base_uav','base_uav','base_world','base_world','base_world']
        # self.bodies_names = rosy.get_param(param_name = '', default =['Load1','Iris5','Iris2'])

    def body_publish(self,body,body_name,child_frame_id):

        time = rospy.get_time()

        # position
        position = numpy.array([body["x"],body["y"],body["z"]])
        # velocity
        velocity = getattr(self,'velocity_estimator'+body_name).out(position,time)

        # attitude: euler angles THESE COME IN DEGREES
        roll = body["roll"]
        pitch=-body["pitch"]
        yaw  = body["yaw"]
        euler_angles = numpy.array([roll,pitch,yaw])

        angular_velocity = numpy.zeros(3)

        # create a message of type quad_state with current state
        odometry = Odometry()

        # odometry.header.time = 
        # global frame
        odometry.header.frame_id = 'base_world'

        odometry.pose.pose.position.x = position[0]
        odometry.pose.pose.position.y = position[1]
        odometry.pose.pose.position.z = position[2]

        R_body_to_inertial = uts.rot_from_euler_deg(euler_angles)

        quaternion = uts.quaternion_from_rot(R_body_to_inertial)
        odometry.pose.pose.orientation.x = quaternion[0]
        odometry.pose.pose.orientation.y = quaternion[1]
        odometry.pose.pose.orientation.z = quaternion[2]
        odometry.pose.pose.orientation.w = quaternion[3]       

        # child frame
        odometry.child_frame_id = child_frame_id
        if child_frame_id == 'base_world':

            odometry.twist.twist.linear.x = velocity[0]
            odometry.twist.twist.linear.y = velocity[1]
            odometry.twist.twist.linear.z = velocity[2]

            odometry.twist.twist.angular.x = angular_velocity[0]
            odometry.twist.twist.angular.y = angular_velocity[1]
            odometry.twist.twist.angular.z = angular_velocity[2]

        else:

            v_body = numpy.dot(numpy.transpose(R_body_to_inertial),velocity)
            odometry.twist.twist.linear.x = v_body[0]
            odometry.twist.twist.linear.y = v_body[1]
            odometry.twist.twist.linear.z = v_body[2]

            angular_velocity_body = numpy.dot(numpy.transpose(R_body_to_inertial),angular_velocity)
            odometry.twist.twist.angular.x = angular_velocity_body[0]
            odometry.twist.twist.angular.y = angular_velocity_body[1]
            odometry.twist.twist.angular.z = angular_velocity_body[2]

        getattr(self,'pub'+body_name).publish(odometry)
        return

    def publish(self):

        # node will be named quad_control (see rqt_graph)
        rospy.init_node('mocap_publisher', anonymous=True)

        for body_name in self.bodies_names:
            # message published by body
            setattr(self,'pub'+body_name,rospy.Publisher(body_name+'Odometry', Odometry, queue_size=1))
            setattr(self,'velocity_estimator'+body_name,uts.VelocityFilter(3,numpy.zeros(3),0.0))

        # establish connection to qualisys
        Qs = mocap_source.Mocap(info=0)

        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():

            bodies_list = Qs.get_updated_bodies()

            if not bodies_list == 'off':
                # check if they come in order
                for body in bodies_list:
                    self.body_publish(body = body, 
                        body_name = self.bodies_names[body['id']-1],
                        child_frame_id = self.child_frame_id[body['id']-1])

            # go to sleep
            rate.sleep()

if __name__ == '__main__':
    body_publisher = BodyPublisher()
    try:
        body_publisher.publish()
    except rospy.ROSInterruptException:
        pass
