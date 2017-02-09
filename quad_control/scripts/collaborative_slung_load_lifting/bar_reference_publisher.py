#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import nav_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool

from utilities import utility_functions as uts
from utilities import utility_functions

import change_flight_mode_client as cfm

from math import radians
from math import cos
from math import sin


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

        # Paths initialization
        uav1_path   = nav_msgs.msg.Path()
        uav2_path   = nav_msgs.msg.Path()
        bar_path    = nav_msgs.msg.Path()

        # Lift-off sequence
        cfm.change_flight_mode_client('lift_off')
        uav1_path.poses = self.liftOff(xInit=0.9,yInit=0)
        uav2_path.poses = self.liftOff(xInit=-0.9,yInit=0)

        liftOff_time = self.getTime(uav1_path)

        while not rospy.is_shutdown() and rospy.get_time() < liftOff_time and not self.emergency_button:
            self.publish_uav_1_reference.publish(uav1_path)
            self.publish_uav_2_reference.publish(uav2_path)

        # Switch to normal flight mode
        cfm.change_flight_mode_client('normal')

        # The first bar reference is where the bar already is
        bar_path.poses.append(geometry_msgs.msg.PoseStamped())
        bar_path.poses[0].pose.position.z = 0.1
        bar_path.poses[0].header.stamp.secs = liftOff_time

        # Custom path for the bar

        # Lift to z=0.5
        bar_path = self.addPoseStamped([0,0,0.5],0,0,bar_path,5)
        # Cicle around the arena
        arena_radius = 2.0
        bar_path = self.demo_circumference([0,0,0.5],arena_radius,bar_path,5)
        # Move along x
        bar_path = self.addPoseStamped([-arena_radius,0,0.5],0,0,bar_path,10)
        # Move diagonally
        bar_path = self.addPoseStamped([0.0,arena_radius,0.5],0,0,bar_path,10)
        # Move along y
        bar_path = self.addPoseStamped([0.0,-arena_radius,0.5],0,0,bar_path,10)
        # Back to the center
        bar_path = self.addPoseStamped([0,0,0.5],0,0,bar_path,10)
        # Changes to the theta angle
        bar_path = self.addPoseStamped([0,0,0.5],0,radians(30),bar_path,5)s
        bar_path = self.addPoseStamped([0,0,0.5],0,radians(-30),bar_path,5)
        bar_path = self.addPoseStamped([0,0,0.5],0,0,bar_path,5)
        # Changes to the psi angle
        bar_path = self.addPoseStamped([0,0,0.5],radians(85),0,bar_path,5)
        ar_path = self.addPoseStamped([0,0,0.5],radians(185),0,bar_path,5)
        bar_path = self.addPoseStamped([0,0,0.5],radians(275),0,bar_path,5)
        bar_path = self.addPoseStamped([0,0,0.5],0,0,bar_path,5)

        demo_time = self.getTime(bar_path) + 5

        while not rospy.is_shutdown() and rospy.get_time() >= liftOff_time and rospy.get_time() < demo_time and not self.emergency_button:
            self.publish_bar_reference.publish(bar_path)

        # Landing 
        cfm.change_flight_mode_client('landing')
        uav1_path = self.addPoseStamped([0.9,0,0],0,0,uav1_path,demo_time - liftOff_time)
        uav2_path = self.addPoseStamped([-0.9,0,0],0,0,uav2_path,demo_time - liftOff_time)

        while not rospy.is_shutdown() and rospy.get_time() >= demo_time and not self.emergency_button:
            self.publish_uav_1_reference.publish(uav1_path)
            self.publish_uav_2_reference.publish(uav2_path)


        if emergency_button :
            cfm.change_flight_mode_client('emergency')
            # Emergency landing


        #     # go to sleep
        #     self.rate.sleep()


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


    def addPoseStamped(self,position,psi,theta,currentPath,timeOffset):
        x=position[0]
        y=position[1]
        z=position[2]
        latestTime = self.getTime(currentPath)
        newTime = latestTime + timeOffset

        poseStampd = geometry_msgs.msg.PoseStamped()
        poseStampd.pose = self.getPose(x,y,z,psi,theta)
        poseStampd.header.stamp.secs = newTime

        currentPath.poses.append(poseStampd)

        return currentPath


    # --------------- DEMO PATHS ---------------

    def demo_circumference(self,center,radius,currentPath,timeOffset):

        path = currentPath

        cx = center[0]
        cy = center[1]
        cz = center[2]

        angle_step = 30
        nTot = 360 / angle_step + 1

        alpha = radians(angle_step+0.1) # the +0.1 was added to avoid singularities

        for i in range(0,nTot):

            psi_i = i * alpha
            target_x = cx + radius * cos(psi_i)
            target_y = cy + radius * sin(psi_i)
            target_z = cz

            # Add the position to the path
            path = self.addPoseStamped([target_x,target_y,target_z],psi_i,0,path,2)

        return path



if __name__ == '__main__':
    bar_reference_publisher = BarReferencePublisher()
    try:
        bar_reference_publisher.cycle()
    except rospy.ROSInterruptException:
        pass