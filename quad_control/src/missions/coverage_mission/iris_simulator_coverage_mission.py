"""This file implements a coverage mission."""

from missions import mission as ms

import utilities.coverage as cov
from converters.iris_plus_converter as ipc

from controllers.fa_trajectory_tracking_controllers import fa_trajectory_tracking_controllers_database as fattcs
from yaw_rate_controllers import yaw_controllers_database as ycs
#from simulators import simulators_dictionary as sms

import rospy as rp

import geometry_msgs.msg as gms
import quad_control.msg as qms


class IrisSimulatorCoverageMission(ms.Mission):
    
    
    inner = {}
    inner['controller'] = fattcs.database
    inner['yaw_controller'] = ycs.database
    #inner['simulator'] = sms.simulators_dictionary
    
    
    def __trade_service(self, trade_request_msg):
        other_pose = trade_request_msg.pose
        other_landmarks = trade_request_msg.landmarks
        for lmk in self.__landmarks:
            lmk = cov.Landmark(lmk)
        for lmk in other_landmarks:
            lmk = cov.Landmark(lmk)
        my_new_landmarks, other_new_landmarks = cov.reassign_landmarks(self.__waypoint, other_pose, self.__landmarks, other_landmarks)
        self.__landmarks = []
        for lmk in my_new_landmarks:
            self.__landmarks.append(lmk.to_pose())
        for lmk in other_new_landmarks:
            lmk = lmk.to_pose()
        return other_new_landmarks
        
        
    def __objective_function(waypoint):
        vis = 0.0
        for lmk in self.__landmarks:
            vis += lmk.compute_visibility(waypoint)
        return vis
    
    
    def __simulator_callback(self, msg):
        self.__sim_state = msg
    
    
    def __init__(self,
        controller = fattcs.database['Default'],
        yaw_controller = ycs.database['Default'],
        #simulator = sms.simulators_dictionary['Default']
        other_quads_names = ['Iris2', 'Iris3', 'Iris4'],
        landmarks = [],
        ):
        
        ms.Mission.__init__(self)
        
        self.__controller = controller
        self.__yaw_controller = yaw_controller
        self.__converter = ipc.IrisPlusConverter()
        self.__cmd_pub = rospy.Publisher('quad_cmd', qms.quad_cmd, queue_size=10)
        
        self.__other_quads_names = other_quads_names
        self.__landmarks = landmarks
        self.__waypoint = gms.Pose2D(0.0, 0.0, 0.0)
        
        self.__sim_state = qms.quad_state()
        self.__sim_sub = rospy.Subscriber('quad_state', qms.quad_state, self.__simulator_callback)
        
        
        
        
#        
#    
#    def __del__(self):
#        # Unsubscribe from all the subscribed topics
#        self.sub_odometry.unregister()


    def get_quad_ea_rad(self):
    	euler_rad     = self.state_quad[6:9]*math.pi/180
    	euler_rad_dot = numpy.zeros(3)
    	return numpy.concatenate([euler_rad,euler_rad_dot])


    def get_reference(self,time_instant):
        self.reference = self.TrajGenerator.output(time_instant)
        return self.reference
        # return numpy.zeros(3*5)


    def get_state(self):
        return self.state_quad


    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.reference[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]


    def real_publish(self,desired_3d_force_quad,yaw_rate):

        euler_rad     = self.state_quad[6:9]*math.pi/180 

        self.IrisPlusConverterObject.set_rotation_matrix(euler_rad)
        iris_plus_rc_input = self.IrisPlusConverterObject.input_conveter(desired_3d_force_quad,yaw_rate)

        # create a message of the type quad_cmd, that the simulator subscribes to 
        cmd  = quad_cmd()
        
        cmd.cmd_1 = iris_plus_rc_input[0]
        cmd.cmd_2 = iris_plus_rc_input[1]
        cmd.cmd_3 = iris_plus_rc_input[2]
        cmd.cmd_4 = iris_plus_rc_input[3]

        cmd.cmd_5 = 1500.0
        cmd.cmd_6 = 1500.0
        cmd.cmd_7 = 1500.0
        cmd.cmd_8 = 1500.0
        
        self.pub_cmd.publish(cmd)

        
