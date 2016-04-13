
# import Misson abstract class
from .. import Mission

from ConverterBetweenStandards.RotorSConverter import RotorSConverter

class FireflyTrajectoryTracking(Mission):


    @classmethod
    def description(cls):
        return "Firefly, from RotorS, to track a desired trajectory"
    
    
    def __init__(self, params):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any
        
        # converting our controlller standard into rotors standard
        self.RotorSObject = RotorSConverter()

        # subscriber to odometry from RotorS
        self.sub_odometry = rospy.Subscriber("/firefly/ground_truth/odometry", Odometry, self.get_state_from_rotorS_simulator)

        # publisher: command firefly motor speeds 
        self.pub_motor_speeds = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=10)      

        self.reset_initial_time()  

        pass  
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        self.sub_odometry.unregister()
        pass

    def get_quad_ea_rad(self):
    	euler_rad     = self.state_quad[6:9]*math.pi/180
    	euler_rad_dot = numpy.zeros(3)
    	return numpy.concatenate([euler_rad,euler_rad_dot])

    @classmethod
    def get_reference(cls,time_instant):
        return self.TrajGenerator.output(time)


    @classmethod
    def get_state(cls):
        return self.state_quad

    @classmethod
    def real_publish(self,desired_3d_force_quad,yaw_rate):

		# publish message
		# TOTAL_MASS = 1.66779; Force3D = numpy.array([0.0,0.0,TOTAL_MASS*9.85]); PsiStar = 0.0; self.pub_motor_speeds.publish(self.RotorSObject.rotor_s_message(Force3D,PsiStar))
		self.pub_motor_speeds.publish(self.RotorSObject.rotor_s_message(desired_3d_force_quad,yaw_rate))

    # callback when ROTORS simulator publishes states
    def get_state_from_rotorS_simulator(self,odometry_rotor_s):

        # RotorS has attitude inner loop that needs to known attitude of quad 
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        # get state from rotorS simulator
        self.state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)       
        return           