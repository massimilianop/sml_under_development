import quad_control.msg as qms
import rospy as rp
import utilities.coverage_utilities as cov


def __callback(array):
    rp.logwarn('in the callback')
    landmarks = cov.landmarks_from_point_2d_array(array)
    for lmk in landmarks:
        rp.logwarn(lmk)


rp.init_node('landmarks_listener')
rp.Subscriber('coverage_landmarks', qms.Point2DArray, __callback)
rp.spin()
