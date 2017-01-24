#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import nav_msgs.msg
import geometry_msgs.msg

from utilities import utility_functions as uts
from utilities import utility_functions


class BarReferencePublisher():

    def __init__(self):
        self.frequency = 0.5

        rospy.init_node('bar_reference_publisher', anonymous=True)

        self.publish_reference = rospy.Publisher(
            name = 'bar_reference_pose_path',
            data_class = nav_msgs.msg.Path,
            queue_size = 1)

        self.rate = rospy.Rate(hz = self.frequency)


    def cycle(self):
        while not rospy.is_shutdown():

            message_instance = nav_msgs.msg.Path()

            # Lift-off sequence
            message_instance.poses = self.liftOff()

            # After lift-off, go to a certain pose
            message_instance = self.addPoseStamped(1.0,2.0,1.1,0.3,0.1,message_instance,10)

            print message_instance

            self.publish_reference.publish(message_instance)

            # go to sleep
            self.rate.sleep()


    def liftOff(self,step=0.1,timeStep=5):
        path = nav_msgs.msg.Path()

        offset = rospy.get_param("/firefly/cable_length")

        n = offset // step

        # for i in range(0,int(n+1)):   # correct code
        for i in range(0,11):     # test code
            path.poses.append(geometry_msgs.msg.PoseStamped())
            path.poses[i].pose.position.z = step * (i+1) - offset
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


