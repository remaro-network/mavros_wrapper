import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class MavrosWrapper:
    def __init__(self):
        self.status = State()
        self.state_sub = rospy.Subscriber(
            'mavros/state',
            State,
            self.status_cb)

    def status_cb(self, msg):
        self.status = msg

    def set_mode(self, mode):
        rospy.wait_for_service('mavros/set_mode')
        try:
            # Create a proxy for the set_mode service
            set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

            # Call set_mode service
            return set_mode(custom_mode=mode)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def arm_motors(self, bool):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            # Create a proxy for the arming service
            arm_motors = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

            # Call arm_motors service
            return arm_motors(bool)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
