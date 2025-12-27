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

from .utils.math_utils import *

class ROS2NodeShared(Node):
    def __init__(self, node_name='bt_shared_node'):
        super().__init__(node_name)
        # BT 노드에서 사용하는 내부 플래그
        #self.control_mode = "POSITION" 

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

        target_x = self.get_parameter('drone.position_x').value
        target_y = self.get_parameter('drone.position_y').value
        target_z = self.get_parameter('drone.position_z').value

        self.takeoff_goal_X = target_x
        self.takeoff_goal_Y = target_y
        self.takeoff_goal_z = target_z

        # 🔥 반드시 여기서 setpoint에 반영
        #self.target_pose = PoseStamped()
        #self.target_pose.pose.position.x = target_x
        #self.target_pose.pose.position.y = target_y
        #self.target_pose.pose.position.z = target_z

        self.desired_pose = PoseStamped()
        self.desired_pose.pose.position.x = target_x
        self.desired_pose.pose.position.y = target_y
        self.desired_pose.pose.position.z = target_z

        self.current_setpoint = PoseStamped()
        
        self.current_setpoint.pose.position.x = target_x
        self.current_setpoint.pose.position.y = target_y
        self.current_setpoint.pose.position.z = target_z 

        self.declare_parameter('safe_transition_point.x', 0.0)
        self.declare_parameter('safe_transition_point.y', 0.0)
        self.declare_parameter('safe_transition_point.z', 0.0)
        self.declare_parameter('safe_transition_point.tol_xy', 0.0)
        self.declare_parameter('safe_transition_point.tol_z', 0.0)

        self.x = self.get_parameter('safe_transition_point.x').value
        self.y = self.get_parameter('safe_transition_point.y').value
        self.z = self.get_parameter('safe_transition_point.z').value
        self.tol_xy = self.get_parameter('safe_transition_point.tol_xy').value
        self.tol_z = self.get_parameter('safe_transition_point.tol_z').value

        self.att_pub = self.create_publisher(AttitudeTarget,'/mavros/setpoint_raw/attitude',10)

        self.control_mode = 'POSITION'


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

        #self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
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

        # 상태
        self.pre_send_counter = 0 # Setpoint 사전 전송 횟수를 세는 카운터
        self.pre_send_limit = 40 # Offboard 모드 진입 전 Setpoint를 2초(20Hz 기준) 동안 사전 전송하기 위한 횟수 제한 값
        #TODO: 나중에 손 보자

        self.local_pose_received = False
        self.altitude_received = False
        self.bt_request_arming = False
        self.bt_request_offboard = False

        self.max_step_xy = 0.2
        self.min_step_xy = 0.05
        self.max_step_z = 0.2
        self.min_step_z = 0.05

        self.slow_radius_xy = 25.0
        self.slow_radius_z = 10.0

        self.fast_target_z = self.takeoff_goal_z

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
    
    def _smooth_step(self, cur, goal, max_step):
        diff = goal - cur
        if abs(diff) <= max_step:
            return goal
        return cur + max_step * (1 if diff > 0 else -1)
    
    def _update_setpoint_smoothly(self):
        MAX_STEP_XY = self.max_step_xy
        MIN_STEP_XY = self.min_step_xy
        MAX_STEP_Z  = self.max_step_z
        MIN_STEP_Z  = self.min_step_z

        SLOW_RADIUS_XY = self.slow_radius_xy
        SLOW_RADIUS_Z  = self.slow_radius_z


        cur = self.current_setpoint.pose.position
        goal = self.desired_pose.pose.position

        dx = goal.x - cur.x
        dy = goal.y - cur.y
        dz = goal.z - cur.z

        dist_xy = math.hypot(dx, dy)
        dist_z  = abs(dz)

        # --- XY step 크기 계산 (cosine easing) ---
        t_xy = min(dist_xy / SLOW_RADIUS_XY, 1.0)
        scale_xy = 0.5 * (1.0 - math.cos(math.pi * t_xy))
        step_xy = MIN_STEP_XY + scale_xy * (MAX_STEP_XY - MIN_STEP_XY)

        # --- Z step ---
        t_z = min(dist_z / SLOW_RADIUS_Z, 1.0)
        scale_z = 0.5 * (1.0 - math.cos(math.pi * t_z))
        step_z = MIN_STEP_Z + scale_z * (MAX_STEP_Z - MIN_STEP_Z)

        # ✅ XY는 벡터 단위 이동
        if dist_xy > 1e-3:
            move_xy = min(step_xy, dist_xy)
            cur.x += dx / dist_xy * move_xy
            cur.y += dy / dist_xy * move_xy
        else:
            cur.x = goal.x
            cur.y = goal.y

        # Z는 기존 방식 유지 (축 이동이 자연스러움)
        cur.z = self._smooth_step(cur.z, goal.z, step_z)

    
    
    def _timer_loop(self):
        now = self.get_clock().now().to_msg()

        if self.control_mode == "POSITION":
            self._update_setpoint_smoothly()
            self.current_setpoint.header.stamp = now
            self.pose_pub.publish(self.current_setpoint)

        # ❗ ATTITUDE 모드는 BT 노드에서 직접 publish
        # 여기서 아무것도 안 함

        if self.pre_send_counter < self.pre_send_limit:
            self.pre_send_counter += 1
            return

        if self.bt_request_arming and not self.current_state.armed:
            self._arm()

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
