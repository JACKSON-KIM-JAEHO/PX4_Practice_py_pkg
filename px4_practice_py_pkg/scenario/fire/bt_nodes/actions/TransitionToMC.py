import rclpy
import math
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandVtolTransition
from mavros_msgs.msg import State, ExtendedState, Altitude, NavControllerOutput
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_practice_py_pkg.modules.base_bt_nodes import *
from px4_practice_py_pkg.modules.base_bt_nodes import BTNode, Status
from mavros_msgs.msg import AttitudeTarget

class TransitionToMC(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared
        self.sent = False
        self.start_time = None
        self.TIMEOUT = 20000.0

        self.client = self.shared.create_client(
            CommandVtolTransition,
            '/mavros/cmd/vtol_transition'
        )

    def tick(self):
        # 이미 MC면 성공
        if self.shared.vtol_state == ExtendedState.VTOL_STATE_MC:
            self.shared.get_logger().info("[TransitionToMC] Already MC")
            return Status.SUCCESS

        if not self.client.service_is_ready():
            return Status.RUNNING

        if self.start_time is None:
            self.start_time = self.shared.get_clock().now()

        dt = (self.shared.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if dt > self.TIMEOUT:
            self.shared.get_logger().error("[TransitionToMC] TIMEOUT")
            return Status.FAILURE

        if not self.sent:
            req = CommandVtolTransition.Request()
            req.state = ExtendedState.VTOL_STATE_MC
            self.client.call_async(req)
            self.sent = True
            self.shared.get_logger().info("[TransitionToMC] FW → MC requested")
            return Status.RUNNING

        return Status.RUNNING
