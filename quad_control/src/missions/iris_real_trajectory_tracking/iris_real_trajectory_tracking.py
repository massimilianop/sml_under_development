
# import Misson abstract class
from .. import Mission

from mavros_msgs.msg import OverrideRCIn
# from mavros_msgs.srv import ParamSet,ParamGet,CommandBool,SetMode

# from quadrotor_tracking_controllers_hierarchical import controllers_dictionary
from controllers_hierarchical.fully_actuated_controllers import controllers_dictionary

from ConverterBetweenStandards.IrisPlusConverter import IrisPlusConverter

class FireflyTrajectoryTracking(Mission):


    @classmethod
    def description(cls):
        return "Firefly, from RotorS, to track a desired trajectory"
    
    
    def __init__(self, body_id = 12):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any
        
        # converting our controlller standard into iris+ standard
        self.IrisPlusConverterObject = IrisPlusConverter()

        # establish connection to qualisys
        self.Qs = mocap_source.Mocap(info=0)

        # publisher: command firefly motor speeds    
        self.rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=100)

        self.reset_initial_time()  

        self.body_id = body_id

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

        bodies = self.Qs.get_body(self.body_id)

        if bodies != 'off':  

            self.flag_measurements = True

            # position
            x=bodies["x"]; y=bodies["y"]; z=bodies["z"]   
            p = numpy.array([x,y,z])

            # velocity
            #v = numpy.array([data.vx,data.vy,data.vz])
            v = self.VelocityEstimator.out(p,rospy.get_time())

            # attitude: euler angles THESE COME IN DEGREES
            roll = bodies["roll"]; pitch=-bodies["pitch"]; yaw  = bodies["yaw"]
            ee = numpy.array([roll,pitch,yaw])

            # collect all components of state
            self.state_quad = numpy.concatenate([p,v,ee])  
        else:
            # do nothing, keep previous state
            self.flag_measurements = False

        return self.state_quad

    @classmethod
    def real_publish(self,desired_3d_force_quad,yaw_rate):

        self.IrisPlusConverterObject.set_rotation_matrix(euler_rad)
        iris_plus_rc_input = self.IrisPlusConverterObject.input_conveter(desired_3d_force_quad,yaw_rate)

        # ORDER OF INPUTS IS VERY IMPORTANT: roll, pitch, throttle,yaw_rate
        # create message of type OverrideRCIn
        rc_cmd          = OverrideRCIn()
        other_channels  = numpy.array([1500,1500,1500,1500])
        channels        = numpy.concatenate([iris_plus_rc_input,other_channels])
        channels        = numpy.array(channels,dtype=numpy.uint16)
        rc_cmd.channels = channels
        rc_override.publish(rc_cmd)

    # callback when ROTORS simulator publishes states
    def get_state_from_rotorS_simulator(self,odometry_rotor_s):

        # RotorS has attitude inner loop that needs to known attitude of quad 
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        # get state from rotorS simulator
        self.state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)       
        return           