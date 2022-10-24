import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


class MavrosWrapper(Node):
    def __init__(self, node_name='mavros_wrapper'):
        super().__init__(node_name)
        self.status = State()
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)

        # TODO: this should be optional
        self.local_pos = PoseStamped()
        local_position_sub_qos = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=5)
        self.local_position_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.local_pos_cb,
            local_position_sub_qos)

    def status_cb(self, msg):
        self.status = msg

    def local_pos_cb(self, msg):
        self.local_pos = msg

    def call_service(self, srv_type, srv_name, request):
        service = self.create_client(srv_type, srv_name)
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(srv_name + ' not available, waiting...')
        resp = service.call_async(request)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.call_service(SetMode, 'mavros/set_mode', req)

    def arm_motors(self, arm_motors_bool):
        req = CommandBool.Request()
        req.value = arm_motors_bool
        self.call_service(CommandBool, 'mavros/cmd/arming', req)

    def setpoint_position_local(
            self, x=.0, y=.0, z=.0, rx=.0, ry=.0, rz=.0, rw=1.0):

        self.setpoint_poisition_local_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = rx
        pose.pose.orientation.y = ry
        pose.pose.orientation.z = rz
        pose.pose.orientation.w = rw

        self.setpoint_poisition_local_pub.publish(pose)
