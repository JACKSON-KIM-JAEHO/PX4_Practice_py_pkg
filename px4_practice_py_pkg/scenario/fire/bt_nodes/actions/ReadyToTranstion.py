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

class ReadyToTranstion(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared
        self.start_time = None
        self.HOLD_TIME = 1.0
        self.MAX_TILT = 15.0
        self.done = False   # 🔑 핵심

    def tick(self):
        # 이미 준비 완료되었으면 계속 SUCCESS
        if self.done:
            return Status.SUCCESS

        # 위치/고도 유지
        self.shared.desired_pose.pose.position.x = self.shared.x
        self.shared.desired_pose.pose.position.y = self.shared.y
        self.shared.desired_pose.pose.position.z = self.shared.fast_target_z

        # 자세 안정 체크
        if abs(self.shared.roll) > self.MAX_TILT or abs(self.shared.pitch) > self.MAX_TILT:
            self.start_time = None
            return Status.RUNNING

        # 안정화 타이머 시작
        if self.start_time is None:
            self.start_time = self.shared.get_clock().now()
            return Status.RUNNING

        dt = (self.shared.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if dt >= self.HOLD_TIME:
            self.shared.get_logger().info("[ReadyToTransition] READY (latched)")
            self.done = True           # 🔑 여기서 고정
            return Status.SUCCESS

        return Status.RUNNING
