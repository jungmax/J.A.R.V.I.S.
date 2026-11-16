import rclpy    
from rclpy.node import Node
from std_msgs.msg import String
import threading                                                #spin말고 독립적으로 일을 시킬수있는 코드
from pymodbus.client import ModbusTcpClient as ModbusClient
import time

# [1] 두산 로보틱스 라이브러리 설정
import DR_init
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# [2] 서비스 메시지 타입
try:
    from dsr_msgs2.srv import SetRobotControl
except ImportError:
    from dsr_msgs.srv import SetRobotControl

# [3] 전역변수
movel = None
movej = None
get_current_posx = None
posx = None
posj = None
set_safe_stop_reset_type = None
# -----------------------------------------------------------------------------------------------------------------  
class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.get_logger().info('🚀 robot Controller 초기화 상태!!. 명령 대기중..')

        # 1. 구독자 생성
        self.create_subscription(String, '/robot/cmd', self.robot_callback, 10)
        # self.create_subscription(String, '/robot/jog', self.jog_callback, 10)
        self.create_subscription(String, '/robot/gripper', self.gripper_callback, 10)
        self.create_subscription(String, '/robot/mode', self.mode_callback, 10)
        

        # 2. [그리퍼 연동] Modbus 클라이언트 초기화 (실제 그리퍼 IP와 포트에 맞게 설정)
        # 예시: IP "192.168.137.100", Port 502
        self.client = ModbusClient('192.168.137.100', port=502)
        # help(self.client.write_registers)
        self.client.connect()
        self.get_logger().info("🔌 그리퍼 Modbus 통신 연결 준비 완료")

        # 3. 복구용 서비스 클라이언트
        self.cli_control = self.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')
        self.SetRobotControl = SetRobotControl
# -----------------------------------------------------------------------------------------------------------------        
    def robot_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"📢 들어온 음성 명령: {command}")

        if command == "home":
            self.get_logger().info("🏠 홈 포지션으로 이동합니다.")
            # x=400mm, y=0mm, z=500mm, 회전각(rx, ry, rz)은 현재 자세 유지 등
            target_pose = posx(400.0, 0.0, 500.0, 0.0, 180.0, 0.0)
            movel(target_pose, vel=60, acc=60)
            
        elif command == "stop":
            self.get_logger().info("🛑 비상 정지 실행.")
        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 명령입니다: {command}")

    def gripper_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"📢 들어온 음성 명령: {command}")
        
        if command == "open":
            self.get_logger().info("🖐️ 그리퍼를 엽니다.")
            self.open_gripper() # 그리퍼 열기 함수 호출
                    
        elif command == "close":
            self.get_logger().info("✊ 그리퍼를 닫습니다.")
            self.close_gripper() # 그리퍼 닫기 함수 호출
        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 명령입니다: {command}")   

    def mode_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"📢 들어온 음성 명령: {command}")

        if command == "recover":
            self.get_logger().info("✅복구모드")
            self.recover_robot_team_style()
# ------------------------------------------------------------------------------------------------------------------ 
    def open_gripper(self):
        try: 
            self.client.write_registers(0, [400, 1100, 16], device_id=65)
            self.get_logger().info("👉 [그리퍼] 열기 명령 패킷 전송 성공")
        except Exception as e: 
            self.get_logger().error(f"⚠️ 그리퍼 열기 에러: {e}")

    def close_gripper(self):
        try:
            self.client.write_registers(0, [0, 1100, 16], device_id=65)
            self.get_logger().info("👉 [그리퍼] 닫기 명령 패킷 전송 성공")
        except Exception as e:
            self.get_logger().error(f"⚠️ 그리퍼 닫기 에러: {e}")
# ------------------------------------------------------------------------------------------------------------------ 
    def wait_and_spin(node, duration):
            end_time = time.time() + duration
            while rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
                time.sleep(0.01)

    def call_control(self, val, name):
        if not self.cli_control.wait_for_service(timeout_sec=1.0): 
            return False
        req = self.SetRobotControl.Request()
        req.robot_control = val
        future = self.cli_control.call_async(req)
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if future.done(): 
                return True
            if time.time() - start > 2.0: 
                return False
            
    # 팀 고유 복구 (3 -> 2 -> 1)
    def recover_robot_team_style(self):
        print("\n🔄 [복구] 팀 고유 시퀀스 시작 (3 -> 2 -> 1)...")
        if set_safe_stop_reset_type: 
            set_safe_stop_reset_type(1)
        
        self.call_control(3, "Safe Off Reset"); 
        self.get_logger().info("3번, Safe Off Reset")
        self.wait_and_spin(1.0)

        self.call_control(2, "Safe Stop Reset"); 
        self.get_logger().info("2번, Safe Stop Reset")
        self.wait_and_spin(1.0)

        self.call_control(1, "Servo On")
        self.get_logger().info("1번, Servo On")
        self.wait_and_spin(1.0)

        self.get_logger().info("   -> 서보 켜지는 중 (3초)...")
        self.wait_and_spin(3.0)
        self.get_logger().info("recovery_success")
# ------------------------------------------------------------------------------------------------------------------ 
    # 큐에 적재된 목록리스트 확인함수
    def print_queue_status(self):
        # 큐 객체를 직접 잠그지 않고 list로 변환하여 안전하게 복사
        with self.cmd_queue.mutex:
            current_items = list(self.cmd_queue.queue)
        
        if current_items:
            # 보기 좋게 포맷팅 (예: [('CMD', 'UP'), ('GRIPPER', 'close'), ('CMD', 'SCAN')])
            items_str = " ➔ ".join([f"({m_type}: {m_val})" for m_type, m_val in current_items])
            self.get_logger().info(f"📦 [큐 대기열 현황] (총 {len(current_items)}개 대기 중) ➔ {items_str}")
        else:
            self.get_logger().info("📦 [큐 대기열 현황] 텅 빔 (Idle)")
# -------------------------------------------------------------------------------------------------------------            
def main(args = None):
    rclpy.init(args=args)
    node = RobotController()

    global DR_init
    DR_init.__dsr__node = node  # 내가 만든 노드 객체를 두산 라이브러리에 연결!
    global movel, movej, get_current_posx, posx, posj
    from DSR_ROBOT2 import movel, movej, get_current_posx
    from DR_common2 import posx, posj

    try:
        rclpy.spin(node)  # 평소에는 여기서 영원히 돌면서 대기함
        
    except KeyboardInterrupt:
        print("🛑 사용자가 Ctrl + C를 눌러서 종료를 요청함!")
    finally:
        # spin이 끝나거나 종료 신호가 오면, 안전하게 뒤처리 실행!
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()    