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

class IsFWMode(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared

    def tick(self):
        if self.shared.vtol_state == ExtendedState.VTOL_STATE_FW:
            return Status.SUCCESS
        return Status.FAILURE