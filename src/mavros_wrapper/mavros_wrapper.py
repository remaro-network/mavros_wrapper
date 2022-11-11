import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType


class MavrosWrapper(Node):
    def __init__(self, node_name='mavros_wrapper'):
        super().__init__(node_name)
        self.status = State()
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)

        # TODO: this should be optional
        self.local_pos = PoseStamped()
        self.local_pos_received = False
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
        self.local_pos_received = True

    def call_service(self, srv_type, srv_name, request):
        service = self.create_client(srv_type, srv_name)
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(srv_name + ' not available, waiting...')
        future = service.call_async(request)
        return future

    def set_mavros_param(self, name, type, value):
        parameter_dict = {
            ParameterType.PARAMETER_BOOL: 'bool_value',
            ParameterType.PARAMETER_INTEGER: 'integer_value',
            ParameterType.PARAMETER_DOUBLE: 'double_value',
            ParameterType.PARAMETER_STRING: 'string_value',
            ParameterType.PARAMETER_BYTE_ARRAY: 'byte_array_value',
            ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array_value',
            ParameterType.PARAMETER_INTEGER_ARRAY: 'integer_array_value',
            ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value',
            ParameterType.PARAMETER_STRING_ARRAY: 'string_array_value'
        }

        req = SetParameters.Request()

        parameter = Parameter()
        parameter.name = name
        parameter.value.type = type
        setattr(parameter.value, parameter_dict[type], value)
        req.parameters.append(parameter)

        return self.call_service(
            SetParameters, 'mavros/param/set_parameters', req)

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

        pose = self.pose_stamped(x, y, z, rx, ry, rz, rw)
        print('setpoint_postion_local value ', pose.pose.position)

        self.setpoint_poisition_local_pub.publish(pose)

    def pose_stamped(
            self, x=.0, y=.0, z=.0, rx=.0, ry=.0, rz=.0, rw=1.0):

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = rx
        pose.pose.orientation.y = ry
        pose.pose.orientation.z = rz
        pose.pose.orientation.w = rw

        return pose

    def check_setpoint_reached(self, pose, delta=0.1):
        return abs(
            self.local_pos.pose.position.x -
            pose.pose.position.x) <= delta and abs(
            self.local_pos.pose.position.y -
            pose.pose.position.y) <= delta and abs(
            self.local_pos.pose.position.z -
            pose.pose.position.z) <= delta
