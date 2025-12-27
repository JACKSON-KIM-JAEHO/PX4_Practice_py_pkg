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

class OffboardTakeoff(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared
        #self.TAKEOFF_Z = 30.0   # 목표 이륙 고도 (local frame)
        self.X_TOL = 1.0  # ±1m 허용 오차
        self.Y_TOL = 1.0  # ±1m 허용 오차
        self.Z_TOL = 1.0  # ±1m 허용 오차
        self.done = False

    def tick(self):
        target_x = self.shared.desired_pose.pose.position.x
        target_y = self.shared.desired_pose.pose.position.y
        target_z = self.shared.desired_pose.pose.position.z
        self.shared.get_logger().info(
            f"[BT] tick pre={self.shared.pre_send_counter}/"
            f"{self.shared.pre_send_limit} "
            f"conn={self.shared.current_state.connected} "
            f"armed={self.shared.current_state.armed} "
            f"mode={self.shared.current_state.mode}"
        )
        if self.done:
            return Status.SUCCESS

        if not self.shared.local_pose_received:
            return Status.RUNNING

        if not self.shared.current_state.connected:
            return Status.RUNNING

        if self.shared.pre_send_counter < self.shared.pre_send_limit:
            return Status.RUNNING
        
        if not self.shared.current_state.armed:
            self.shared.bt_request_arming = True
            return Status.RUNNING
        
        if self.shared.current_state.mode != "OFFBOARD":
            self.shared.bt_request_offboard = True
            return Status.RUNNING

        

        # ✅ local position z로 이륙 판단
        x = self.shared.local_pose.pose.position.x
        y = self.shared.local_pose.pose.position.y
        z = self.shared.local_pose.pose.position.z
        self.shared.get_logger().info(f"[BT] local x = {x:.2f}")
        self.shared.get_logger().info(f"[BT] local y = {y:.2f}")
        self.shared.get_logger().info(f"[BT] local z = {z:.2f}")

        if abs(x - target_x) <= self.X_TOL and abs(y - target_y) <= self.Y_TOL and abs(z - target_z) <= self.Z_TOL:
            self.shared.get_logger().info("[BT][Takeoff] SUCCESS (within ±1m)")
            self.done = True
            return Status.SUCCESS

        return Status.RUNNING