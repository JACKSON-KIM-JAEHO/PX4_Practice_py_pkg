import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

'''나중에 BT로 구현할 때는 떠 있으면 success도 만들고 계속 떠 있는 지 지속적으로 확인해서 Running도 구현해야함.'''
class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__('offboard_takeoff')

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Subscription
        self.current_state = State()
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Setpoint 메시지 생성
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 3.0  # 이륙 고도 3m

        # 주기적으로 퍼블리시 시작
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # flag
        self.setpoint_sent = 0
        #self.armed = False
        #self.offboard_mode_set = False
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def timer_callback(self):
        '''1. Arming
           2. Offboard Mode 실행
           3. Takeoff --> Setpoint로 설정'''
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.target_pose)

        if self.setpoint_sent < 40:  # 약 2초 동안 Setpoint 보내기
            self.setpoint_sent += 1
            return

        if self.current_state.connected and not self.current_state.armed:
            self.arm_drone()
        
        elif self.current_state.armed and self.current_state.mode != 'OFFBOARD':
            self.set_offboard_mode()

        elif self.current_state.armed and self.current_state.mode == 'OFFBOARD':
             self.get_logger().info("오프보드 모드 진입 완료 및 이륙 중.")
             # TODO: 나중에 위치(position) 토픽을 구독해서 3m에 도달했는지 확인하는 로직 추가


    def arm_drone(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            self.get_logger().info("Arming 요청 보냈습니다!")

    def set_offboard_mode(self):
        if self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.mode_client.call_async(req)
            self.get_logger().info("Offboard 모드 전환 요청 보냈습니다!")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()