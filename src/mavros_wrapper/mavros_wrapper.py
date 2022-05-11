import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


class MavrosWrapper(Node):
    def __init__(self, node_name='mavros_wrapper'):
        super().__init__(node_name)
        self.status = State()
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)

    def status_cb(self, msg):
        self.status = msg

    def call_service(self, srv_type, srv_name, request):
        service = self.create_client(srv_type, srv_name)
        while not service.wait_for_service(timeout_sec=1.0):
           self.get_logger().info(srv_name + ' not available, waiting again...')
        resp = service.call_async(request)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.call_service(SetMode, 'mavros/set_mode', req)

    def arm_motors(self, arm_motors_bool):
        req = CommandBool.Request()
        req.value = arm_motors_bool
        self.call_service(CommandBool, 'mavros/cmd/arming', req)
