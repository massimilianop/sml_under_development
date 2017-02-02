#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import nav_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool

from utilities import utility_functions as uts
from utilities import utility_functions

import change_flight_mode_client as cfm


class BarReferencePublisher():

    def __init__(self):
        self.frequency = 0.5

        self.emergency_button = False

        rospy.init_node('bar_reference_publisher', anonymous=True)

        self.publish_bar_reference = rospy.Publisher(
            name = 'bar_reference_pose_path',
            data_class = nav_msgs.msg.Path,
            queue_size = 1)

        self.publish_uav_1_reference = rospy.Publisher(
            name = 'uav_1_reference_path',
            data_class = nav_msgs.msg.Path,
            queue_size = 1)

        self.publish_uav_2_reference = rospy.Publisher(
            name = 'uav_2_reference_path',
            data_class = nav_msgs.msg.Path,
            queue_size = 1)

        self.rate = rospy.Rate(hz = self.frequency)

        self.subscribe_emergency_button = rospy.Subscriber(
            name = '/arena/killswitch',
            data_class = Bool,
            callback =self.callback_sub_emergency_button)


    def callback_sub_emergency_button(self, msg):
        self.emergency_button = msg.data


    def cycle(self):
        while not rospy.is_shutdown():

            # Paths initialization
            uav1_path   = nav_msgs.msg.Path()
            uav2_path   = nav_msgs.msg.Path()
            bar_path    = nav_msgs.msg.Path() 

            # Lift-off sequence
            cfm.change_flight_mode_client('lift_off')
            uav1_path.poses = self.liftOff(xInit=1,yInit=0)
            uav2_path.poses = self.liftOff(xInit=-1,yInit=0)

            self.publish_uav_1_reference.publish(uav1_path)
            self.publish_uav_2_reference.publish(uav2_path)

            # Switch to normal flight mode
            # ISSUE: the command below is executed immediately, effectively making the quads ignore the uav_reference path!!
            #self.change_flight_mode_client('normal')

            # self.publish_bar_reference.publish(message_instance)

            # go to sleep
            self.rate.sleep()


    def liftOff(self,xInit,yInit,step=0.05,timeStep=2,n_iterations=16):
        path = nav_msgs.msg.Path()

        # offset = rospy.get_param("/firefly/cable_length")
        # n = offset // step
        # for i in range(0,int(n+1)):
        for i in range(0,n_iterations):
            path.poses.append(geometry_msgs.msg.PoseStamped())
            path.poses[i].pose.position.z = step * (i+1)
            path.poses[i].pose.position.x = xInit
            path.poses[i].pose.position.y = yInit
            path.poses[i].header.stamp.secs    = timeStep * (i)

        return path.poses


    def getOrientation(self,psi=0.0,theta=0.0):
        r_matrix = uts.rotation_matrix_from_euler_angles(0.0,theta,psi)
        quaternion = uts.quaternion_from_rot(r_matrix)

        return quaternion


    def getPose(self,x=0.0,y=0.0,z=0.0,psi=0.0,theta=0.0):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quaternion = self.getOrientation(psi,theta)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose


    def getTime(self, currentPath):
        lastIndex = len(currentPath.poses) -1
        latestTime = currentPath.poses[lastIndex].header.stamp.secs

        return latestTime


    def addPoseStamped(self,x,y,z,psi,theta,currentPath,timeOffset):
        latestTime = self.getTime(currentPath)
        newTime = latestTime + timeOffset

        poseStampd = geometry_msgs.msg.PoseStamped()
        poseStampd.pose = self.getPose(x,y,z,psi,theta)
        poseStampd.header.stamp.secs = newTime

        currentPath.poses.append(poseStampd)

        return currentPath



if __name__ == '__main__':
    bar_reference_publisher = BarReferencePublisher()
    try:
        bar_reference_publisher.cycle()
    except rospy.ROSInterruptException:
        pass


