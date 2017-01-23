#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import nav_msgs.msg
import geometry_msgs.msg

class BarReferencePublisher():

    def __init__(self):
        self.frequency = 0.5

    def cycle(self):

        rospy.init_node('bar_reference_publisher', anonymous=True)

        publish_reference = rospy.Publisher(
            name = 'bar_reference_pose_path',
            data_class = nav_msgs.msg.Path,
            queue_size = 1)

        rate = rospy.Rate(hz = self.frequency)

        while not rospy.is_shutdown():

            message_instance = nav_msgs.msg.Path()
            # p1 = geometry_msgs.msg.PoseStamped()
            # p2 = geometry_msgs.msg.PoseStamped()
            # # p2.pose.position.x = 1
            # p1.pose.position.z = 0.5
            # p2 = p1
            # message_instance.poses = [p1,p2]
            message_instance.poses = self.liftOff()
            publish_reference.publish(message_instance)

            # go to sleep
            rate.sleep()


    def liftOff(self,n=5,step=0.1,timeStep=10):
        path = nav_msgs.msg.Path()

        for i in range(1,n+1):
            path.poses.append(geometry_msgs.msg.PoseStamped())
            path.poses[i-1].pose.position.z = step * i
            path.poses[i-1].header.stamp.secs    = timeStep * i

        return path.poses




if __name__ == '__main__':
    bar_reference_publisher = BarReferencePublisher()
    try:
        bar_reference_publisher.cycle()
    except rospy.ROSInterruptException:
        pass


