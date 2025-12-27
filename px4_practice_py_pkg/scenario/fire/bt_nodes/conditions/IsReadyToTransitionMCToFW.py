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

class IsReadyToTransitionMCToFW(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared

        self.MIN_ALT = 20.0
        self.MIN_VX  = 12.0    # PX4 VT_ARSP_TRANS 기본값 근처
        self.MAX_TILT = 15.0   # deg

    def tick(self):
        if not self.shared.local_pose_received:
            return Status.FAILURE

        z = self.shared.local_pose.pose.position.z
        vx = self.shared.vx
        alt = abs(self.shared.local_pose.pose.position.z)
        if alt < self.MIN_ALT:
            return Status.FAILURE

        if vx < self.MIN_VX:
            return Status.FAILURE

        if abs(self.shared.roll) > self.MAX_TILT:
            return Status.FAILURE

        if abs(self.shared.pitch) > self.MAX_TILT:
            return Status.FAILURE

        return Status.SUCCESS