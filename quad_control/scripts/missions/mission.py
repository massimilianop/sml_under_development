from utilities import jsonable as js

# The children import needed dictionaries

# For example
# from controllers_hierarchical import controllers_dictionary as cd
# from TrajectoryPlanner import trajectories_dictionary as td 

# If the children need to subscribe to some topic,
# or to publish to some topic,
# then they import rospy and the needed message types.

# For example
# import rospy as rp
# from mavros.mavros_msgs.msg import OverrideRCIn

# If the mocap is needed, the module mocap must be imported
# import mocap 



class Mission(js.Jsonable):

    
    inner = {
        # For example: controller, reference, etc.
    }
    
    
    @classmethod
    def description(cls):
        return "Abstract Mission"
    
    
    def __init__(self, params):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any
        pass
        
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        pass
