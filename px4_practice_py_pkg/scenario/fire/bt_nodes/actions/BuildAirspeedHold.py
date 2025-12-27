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

class BuildAirspeedHold(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared

        self.PITCH_DEG = 12.0
        self.THRUST = 0.75

        self.logged_fw = False

    def tick(self):
        now = self.shared.get_clock().now()

        # 🔴 이미 FW로 전환되었으면 여기서 종료
        if self.shared.vtol_state == ExtendedState.VTOL_STATE_FW:
            if not self.logged_fw:
                self.shared.get_logger().info(
                    "[BuildAirspeedHold] FW transition detected → SUCCESS"
                )
                self.logged_fw = True
            return Status.SUCCESS

        # 아직 FW 아니면 계속 에어스피드 빌드
        msg = AttitudeTarget()
        msg.header.stamp = now.to_msg()

        # rate 무시 (자세만 준다)
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )

        pitch_rad = math.radians(self.PITCH_DEG)

        # pitch만 준 quaternion
        msg.orientation.w = math.cos(pitch_rad / 2.0)
        msg.orientation.x = 0.0
        msg.orientation.y = math.sin(pitch_rad / 2.0)
        msg.orientation.z = 0.0

        msg.thrust = self.THRUST

        self.shared.att_pub.publish(msg)

        self.shared.get_logger().info(
            f"[BuildAirspeedHold] RUNNING | pitch={self.PITCH_DEG} deg, thrust={self.THRUST}"
        )

        return Status.RUNNING
