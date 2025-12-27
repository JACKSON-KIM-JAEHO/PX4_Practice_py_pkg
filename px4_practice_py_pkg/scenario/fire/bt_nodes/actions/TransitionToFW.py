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

class TransitionToFW(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared
        self.sent = False
        self.start_time = None
        self.TIMEOUT = 10.0  # 현실적인 타임아웃

        self.client = self.shared.create_client(
            CommandVtolTransition,
            '/mavros/cmd/vtol_transition'
        )

    def tick(self):
        # ✅ FW로 완전히 전환되었으면 성공
        if self.shared.vtol_state == ExtendedState.VTOL_STATE_FW:
            self.shared.get_logger().info("[TransitionToFW] FW confirmed")
            return Status.SUCCESS

        # 서비스 준비 대기
        if not self.client.service_is_ready():
            return Status.RUNNING

        # 타임아웃 시작
        if self.start_time is None:
            self.start_time = self.shared.get_clock().now()

        dt = (self.shared.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if dt > self.TIMEOUT:
            self.shared.get_logger().error(
                "[TransitionToFW] FAILED: PX4 did not enter transition"
            )
            return Status.FAILURE

        # 최초 1회만 요청
        if not self.sent:
            req = CommandVtolTransition.Request()
            req.state = ExtendedState.VTOL_STATE_FW
            self.client.call_async(req)
            self.sent = True
            self.shared.get_logger().info("[TransitionToFW] Transition requested")
            return Status.RUNNING

        # ✅ 핵심: 전환이 '시작'되었는지도 확인
        if self.shared.vtol_state == ExtendedState.VTOL_STATE_TRANSITION_TO_FW:
            self.shared.get_logger().info("[TransitionToFW] Transition started")
            return Status.RUNNING

        return Status.RUNNING