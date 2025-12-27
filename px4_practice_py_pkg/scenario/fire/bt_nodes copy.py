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

# BT Node List
CUSTOM_ACTION_NODES = [
    'ExplorationNode',
    'CheckingitemsNode',
    'DeliveryexecutingNode',
    'RightplacecheckingNode',
    'DropoffexecutingNode',
    'CheckingnomoreTask',
    'GatheringNode',
]

def quaternion_to_euler_deg(x, y, z, w):
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )

class ROS2NodeShared(Node):
    def __init__(self, node_name='bt_shared_node'):
        super().__init__(node_name)
        # BT 노드에서 사용하는 내부 플래그

        self.bt_request_arming = False
        self.bt_request_offboard = False

        self.vtol_state = ExtendedState.VTOL_STATE_MC
        self.altitude_rel = 0.0
        self.vx = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.declare_parameter('drone.position_x', 0.0)
        self.declare_parameter('drone.position_y', 0.0)
        self.declare_parameter('drone.position_z', 0.0)

        target_x = self.get_parameter('drone.position_x').get_parameter_value().double_value
        target_y = self.get_parameter('drone.position_y').get_parameter_value().double_value
        target_z = self.get_parameter('drone.position_z').get_parameter_value().double_value

        # ✅ MAVROS는 BEST_EFFORT로 맞추는 게 안전 (센서류/상태류 다 포함)
        mavros_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # MAVROS: 드론의 현재 상태 (connected, armed, mode) 확인용 (필수)
        self.current_state = State() 
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            mavros_qos
        )
        
        # VTOL 확장 상태: 천이 모드 (Transition) 및 착륙 상태 확인용 (VTOL 임무의 핵심)
        self.extended_state = ExtendedState()
        
        self.extended_sub = self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self._extended_state_callback,
            mavros_qos
        )
        self.altitude_sub = self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self._altitude_callback,
            mavros_qos
        )


        # Local Position: 현재 위치 (x, y, z) 확인용 (Base 도착 판단 및 모든 비행 임무의 핵심)
        self.local_pose = PoseStamped()
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._local_pose_callback,
            qos
        )

        self.imudata_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self._imu_callback,
            qos
        )


        # Nav Controller Output: 웨이포인트 거리 (wp_dist) 및 에러 확인용 (순찰/복귀 임무의 완료 판단)
        self.nav_output = NavControllerOutput()
        self.nav_output_sub = self.create_subscription(
            NavControllerOutput, '/mavros/nav_controller_output/output', self._nav_output_callback, 10)
            
        # Velocity Body: 속도 확인용 (비행 중 감시/안전성 확인)
        self.velocity_body = TwistStamped()
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_body',
            self._velocity_callback,
            qos
        )                                                                        

        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.cmd_vx = 0.0
        # ----------------------------------------------------
        # 2. 제어 발행 및 서비스 클라이언트 (Publish/Client)
        # ----------------------------------------------------
        
        # 목표 Setpoint: 위치 제어용 (필수)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
            
        # Arming 서비스 클라이언트 (필수)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        # Mode 설정 서비스 클라이언트 (필수: OFFBOARD, AUTO.MISSION, LAND 등)
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        
        self.altitude_received = False
        self.local_pose_received = False


        # 목표 setpoint
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = target_x
        self.target_pose.pose.position.y = target_y
        self.target_pose.pose.position.z = target_z

        # 상태
        self.pre_send_counter = 0 # Setpoint 사전 전송 횟수를 세는 카운터
        self.pre_send_limit = 40 # Offboard 모드 진입 전 Setpoint를 2초(20Hz 기준) 동안 사전 전송하기 위한 횟수 제한 값
        #TODO: 나중에 손 보자

        self.local_pose_received = False
        self.altitude_received = False
        self.bt_request_arming = False
        self.bt_request_offboard = False

        # timer 20Hz 
        self.timer = self.create_timer(0.05, self._timer_loop)


     # 3. 콜백 함수 정의 (Callback Methods)

    def _state_callback(self, msg):
        self.current_state = msg

    def _extended_state_callback(self, msg: ExtendedState):
        self.extended_state = msg
        self.vtol_state = msg.vtol_state

    
    def _imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(
            q.x, q.y, q.z, q.w
        )

        self.roll = roll
        self.pitch = pitch


    def _nav_output_callback(self, msg: NavControllerOutput):
        self.nav_output = msg
        
    def _velocity_callback(self, msg: TwistStamped):
        self.velocity_body = msg
        self.vx = msg.twist.linear.x
    
    def _timer_loop(self):
        vel = TwistStamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        vel.twist.linear.x = self.cmd_vx
        self.vel_pub.publish(vel)
        # setpoint는 항상
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.target_pose)

        if self.pre_send_counter < self.pre_send_limit:
            self.pre_send_counter += 1
            return

        # ✅ ARM은 armed 될 때까지 계속
        if self.bt_request_arming and not self.current_state.armed:
            self._arm()

        # ✅ OFFBOARD는 mode 바뀔 때까지 계속
        if self.bt_request_offboard and self.current_state.mode != "OFFBOARD":
            self._set_offboard_mode()

        
    def _altitude_callback(self, msg):
        self.altitude_data = msg
        self.altitude_rel = msg.relative
        self.altitude_received = True

    def _local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_pose_received = True


    def _arm(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn("[Shared] arming service not ready")
            return

        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)

        self.get_logger().info("[Shared] arming requested")
        # ❌ 여기서 bt_request_arming = False 하지 마라



    def _set_offboard_mode(self):
        if not self.mode_client.service_is_ready():
            self.get_logger().warn("[Shared] set_mode service not ready")
            return

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.mode_client.call_async(req)

        self.get_logger().info("[Shared] offboard requested")
        # ❌ bt_request_offboard = False 제거



class IsMCMode(BTNode):
    '''MC(멀티콥터 모드)인지 확인하는 용도이며, 멀티콥터 모드일 때 SUCCESS 반환'''
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared

    def tick(self):
        if self.shared.vtol_state == ExtendedState.VTOL_STATE_MC:
            return Status.SUCCESS
        return Status.FAILURE

class IsFWMode(BTNode):
    '''FW(고정익 모드)인지 확인하는 용도이며, 고정익 모드일 때 SUCCESS 반환
       참고: Python에서는 보통 True == SUCCESS, False == FAILURE로 매핑해서 씀'''

    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared

    def tick(self):
        if self.shared.vtol_state == ExtendedState.VTOL_STATE_FW:
            return Status.SUCCESS
        return Status.FAILURE


class IsReadyToTransitionMCToFW(BTNode):
    '''Pitch 조건도 넣어야 할까?'''
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared
        # 최신 상태
        self._vtol_state = ExtendedState.VTOL_STATE_MC
        self.MIN_ALT_M = 10.0
        self.V_MIN_MS = 8.0
        self.V_MAX_MS = 14.0
        #self.PITCH_MIN = -15.0
        #self.PITCH_MAX = -5.0

    # ---------- BT Tick ----------

    def tick(self):

        if self.shared.vtol_state != ExtendedState.VTOL_STATE_MC:
            return Status.FAILURE

        if self.shared.altitude_rel < self.MIN_ALT_M:
            return Status.FAILURE

        if not (self.V_MIN_MS <= self.shared.vx <= self.V_MAX_MS):
            return Status.FAILURE

        '''if not (self.PITCH_MIN <= self.shared.pitch <= self.PITCH_MAX):
            return Status.FAILURE'''

        return Status.SUCCESS
    
class IsReadyToTransitionFWToMC(BTNode):
    '''테스트용'''
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared

        self.MIN_ALT_M = 10.0
        self.V_MIN_MS = 8.0
        self.V_MAX_MS = 14.0
        #self.PITCH_MIN = -15.0
        #self.PITCH_MAX = -5.0

    def tick(self):

        if self.shared.vtol_state != ExtendedState.VTOL_STATE_FW:
            return Status.FAILURE

        if self.shared.altitude_rel < self.MIN_ALT_M:
            return Status.FAILURE

        if self.shared.vx > self.V_MAX_MS:
            return Status.FAILURE

        '''if not (self.PITCH_MIN <= self.shared.pitch <= self.PITCH_MAX):
            return Status.FAILURE'''

        return Status.SUCCESS
    
class ReadyToTransitionMCToFW(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared
        self.node = shared

        

        # 목표 값
        self.V_TARGET = 10.0        # m/s
        #self.PITCH_TARGET = -10.0   # deg

        # 허용 오차
        self.V_TOL = 1.0
        #self.PITCH_TOL = 2.0

        

    def tick(self):

        
        # MC 상태가 아니면 실패
        if self.shared.vtol_state != ExtendedState.VTOL_STATE_MC:
            return Status.FAILURE
        
        self.node.cmd_vx = self.V_TARGET

        # ---------- 속도 제어 ----------
        v_err = self.V_TARGET - self.shared.vx

        # 단순 P 제어 (테스트용)
        throttle = 0.5 + 0.05 * v_err
        throttle = max(0.3, min(0.7, throttle))

        # ---------- Pitch 제어 ----------
        #pitch_err = self.PITCH_TARGET - self.shared.pitch
        #pitch_cmd = self.PITCH_TARGET + 0.1 * pitch_err

        # 👉 실제 제어는 shared node가 수행
        #self.node.blackboard.desired_pitch = pitch_cmd
        #self.node.blackboard.desired_throttle = throttle

        # ---------- 수렴 판단 ----------
        speed_ok = abs(v_err) < self.V_TOL
        #pitch_ok = abs(pitch_err) < self.PITCH_TOL

        if speed_ok:
            self.node.get_logger().info(
                "[ReadyToTransitionMCToFW] 조건 만족 → 전환 가능"
            )
            return Status.SUCCESS

        return Status.RUNNING                         

class TransitionToFW(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.node = shared
        self.shared = shared
        self.client = self.node.create_client(CommandVtolTransition, '/mavros/cmd/vtol_transition')
        self.sent = False
        self.start_time = None
        self.TIMEOUT = 8.0

    def reset(self):
        self.sent = False
        self.start_time = None

    def tick(self):
        if self.start_time is None:
            self.start_time = self.node.get_clock().now()
        if self.shared.vtol_state == ExtendedState.VTOL_STATE_FW:
            return Status.SUCCESS

        if not self.sent:
            if not self.client.service_is_ready():
                return Status.RUNNING
            req = CommandVtolTransition.Request()
            req.state = CommandVtolTransition.Request.STATE_FW  # 또는 4
            self.client.call_async(req)
            self.sent = True
            self.node.get_logger().info("[TransitionToFW] /cmd/vtol_transition 요청")

        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed > self.TIMEOUT:
            return Status.FAILURE

        return Status.RUNNING




class MoveToTestTransitionPoint(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.node = shared

        self.target_x = 100.0
        self.target_y = 0.0
        self.target_z = 50.0

    def tick(self):
        # 목표 위치 지정
        self.node.target_pose.pose.position.x = self.target_x
        self.node.target_pose.pose.position.y = self.target_y
        self.node.target_pose.pose.position.z = self.target_z

        # 현재 위치
        cur = self.node.local_pose.pose.position

        dx = self.target_x - cur.x
        dy = self.target_y - cur.y
        dz = self.target_z - cur.z

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        self.node.get_logger().info(f"[MoveToTest] dist={dist:.2f}")

        if dist < 2.0:
            return Status.SUCCESS

        return Status.RUNNING





class OffboardTakeoff(BTNode):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared
        #self.TAKEOFF_Z = 30.0   # 목표 이륙 고도 (local frame)
        self.Z_TOL = 1.0  # ±1m 허용 오차

    def tick(self):
        target_z = self.shared.target_pose.pose.position.z
        self.shared.get_logger().info(
            f"[BT] tick pre={self.shared.pre_send_counter}/"
            f"{self.shared.pre_send_limit} "
            f"conn={self.shared.current_state.connected} "
            f"armed={self.shared.current_state.armed} "
            f"mode={self.shared.current_state.mode}"
        )

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
        z = self.shared.local_pose.pose.position.z
        self.shared.get_logger().info(f"[BT] local z = {z:.2f}")

        if abs(z - target_z) <= self.Z_TOL:
            self.shared.get_logger().info(
                "[BT][Takeoff] SUCCESS (within ±1m)"
            )
            return Status.SUCCESS

        return Status.RUNNING



