import rclpy
import threading
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_trees.blackboard import Blackboard 
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 

'''나중에 BT로 구현할 때는 떠 있으면 success도 만들고 계속 떠 있는 지 지속적으로 확인해서 Running도 구현해야함.'''
class ROS2NodeShared(Node):
    """
    모든 BT 노드가 ROS 2 통신을 위해 공유하는 단일 노드 역할을 합니다.
    기존 OffboardTakeoff 노드의 __init__ 로직을 포함합니다.
    """
    
    def __init__(self, node_name='bt_shared_node'):
        super().__init__(node_name)
        
        # PyTrees 블랙보드 및 Executor 설정
        self.blackboard = Blackboard() 
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self)
        self.spin_thread = threading.Thread(target=self._spin_thread_func, daemon=True)
        self.spin_thread.start()

        # MAVROS 통신 자원 초기화 (기존 OffboardTakeoff 클래스의 __init__ 로직)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.current_state = State()
        self.state_sub = self.create_subscription(State, '/mavros/state', self._state_callback, 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Setpoint 메시지 (3m 이륙 목표)
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 3.0
        
        # Setpoint 사전 전송 카운터
        self.setpoint_sent_count = 0
        self.pre_send_threshold = 40

    def _state_callback(self, msg):
        self.current_state = msg

    def _spin_thread_func(self):
        """별도의 스레드에서 ROS 2 콜백을 처리합니다."""
        try:
            self.executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception as e:
            self.get_logger().error(f"ROS 2 spin thread failed: {e}")


# ----------------------------------------------------------------------
# 2. 🚀 BT 액션 노드 구현: OffboardTakeoff
# ----------------------------------------------------------------------
class OffboardTakeoff(Behaviour):
    """
    기존 OffboardTakeoff 클래스의 timer_callback 로직을 BT로 순차적으로 구현합니다.
    (SendSetpoint -> Arming -> SetMode -> Takeoff 확인 로직을 통합하거나,
     간결한 예시를 위해 BT가 하나의 시퀀스를 실행하도록 로직을 분리합니다.)
    """
    
    def __init__(self, name: str, shared_resources: ROS2NodeShared):
        super().__init__(name)
        self.shared = shared_resources
        self.service_in_progress = False

    def update(self) -> Status:
        # 1. Setpoint 사전 전송 (2초 동안)
        if self.shared.setpoint_sent_count < self.shared.pre_send_threshold:
            self._publish_setpoint()
            self.shared.setpoint_sent_count += 1
            self.logger.debug(f"Setpoint 사전 전송 중: {self.shared.setpoint_sent_count}")
            return Status.RUNNING

        # 2. Arming 시도
        if self.shared.current_state.connected and not self.shared.current_state.armed:
            self._arm_drone()
            return Status.RUNNING # 서비스 응답 대기

        # 3. Offboard Mode 설정 시도
        elif self.shared.current_state.armed and self.shared.current_state.mode != 'OFFBOARD':
            self._set_offboard_mode()
            return Status.RUNNING # 서비스 응답 대기

        # 4. 이륙 및 고도 유지 (Offboard Mode 진입 완료)
        elif self.shared.current_state.armed and self.shared.current_state.mode == 'OFFBOARD':
            self._publish_setpoint() # 계속 Setpoint 유지
            self.logger.info("오프보드 모드 진입 완료 및 이륙 중. (RUNNING)")
            # TODO: 실제 고도 확인 로직 추가 시 SUCCESS 반환 가능
            return Status.RUNNING
        
        # 5. 모든 조건 불만족 시 (연결 대기 등)
        else:
            self._publish_setpoint() # 최소한 Setpoint는 계속 보냄
            self.logger.warning("드론 연결 또는 상태 대기 중. (RUNNING)")
            return Status.RUNNING
    
    def _publish_setpoint(self):
        self.shared.target_pose.header.stamp = self.shared.get_clock().now().to_msg()
        self.shared.pose_pub.publish(self.shared.target_pose)

    def _arm_drone(self):
        if not self.service_in_progress and self.shared.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            future = self.shared.arming_client.call_async(req)
            future.add_done_callback(lambda f: self._service_callback(f, "Arming"))
            self.service_in_progress = True
            self.logger.info("Arming 요청 보냈습니다!")

    def _set_offboard_mode(self):
        if not self.service_in_progress and self.shared.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.shared.mode_client.call_async(req)
            future.add_done_callback(lambda f: self._service_callback(f, "Offboard Mode"))
            self.service_in_progress = True
            self.logger.info("Offboard 모드 전환 요청 보냈습니다!")
    
    def _service_callback(self, future, service_name):
        """서비스 응답 처리 콜백"""
        self.service_in_progress = False
        try:
            response = future.result()
            success = response.success if service_name == "Arming" else response.mode_sent
            if success:
                self.logger.info(f"[{service_name}] 성공 응답 수신!")
            else:
                self.logger.error(f"[{service_name}] 실패 응답 수신.")
        except Exception as e:
            self.logger.error(f"[{service_name}] 서비스 호출 실패: {e}")
