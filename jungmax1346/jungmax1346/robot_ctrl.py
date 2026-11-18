import rclpy    
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
import threading                                                #spin말고 독립적으로 일을 시킬수있는 코드
from pymodbus.client import ModbusTcpClient as ModbusClient
import time
from rclpy.executors import MultiThreadedExecutor

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
set_robot_mode = None
# -----------------------------------------------------------------------------------------------------------------  
class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.get_logger().info('🚀 robot Controller 초기화 상태!!. 명령 대기중..')
        self.pending = None
        # 1. 구독자 생성
        self.create_subscription(String, '/robot/cmd', self.robot_callback, 10)
        # self.create_subscription(String, '/robot/jog', self.jog_callback, 10)
        self.create_subscription(String, '/robot/gripper', self.gripper_callback, 10)
        self.create_subscription(String, '/robot/mode', self.mode_callback, 10)
        self.create_subscription(PoseStamped, '/robot/target_pose', self.pose_callback, 10)
        self.create_subscription(Float64MultiArray, '/robot/target_joint', self.joint_callback, 10)
        

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
    # Callback 함수
    def robot_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"📢 들어온 음성 명령: {command}")

        if command == "home":
            self.pending = command

        elif command == "init":
            self.pending = command

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
            self.get_logger().info("⚠️ 복구모드 시작")
            self.recover_robot_team_style()

    def joint_callback(self, msg):
        # msg.data에 들어 있는 숫자 배열을 파이썬 리스트로 변환
        target_joints = list(msg.data)
        # 로그로 잘 들어왔는지 확인 (f-string 문법 살짝 수정!)
        self.get_logger().info(f"📢 들어온 타겟 관절 각도: {target_joints}")
        

    def pose_callback(self, msg): 
        # 1. 메시지 상자에서 3차원 공간 위치(x, y, z) 꺼내기
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z

        # 2. 메시지 상자에서 회전 자세(Orientation - 쿼터니언) 꺼내기
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # 3. 로그로 잘 들어왔는지 확인
        self.get_logger().info(f"📢 들어온 목표 위치 -> X: {tx}, Y: {ty}, Z: {tz}")

        # 4. 단위 변환 (미터 단위를 밀리미터로 변환하는 경우)
        if abs(tx) < 10.0: 
            tx *= 1000 
            ty *= 1000 
            tz *= 1000

        # 5. 현재 로봇의 위치/자세를 가져와서 회전 값 등 부족한 부분을 채워넣기
        curr_pos, _ = get_current_posx()
        if curr_pos:
            # 보통 x, y, z는 새로운 목표값으로 바꾸고, 
            # 회전(rx, ry, rz 또는 쿼터니언)은 현재 로봇의 자세나 필요한 값을 조합합니다.
            # (두산 로봇 라이브러리에 따라 posx 구조에 맞춰 리스트나 오일러 각으로 변환할 수 있습니다)
            
            target = [tx, ty, tz, curr_pos[3], curr_pos[4], curr_pos[5]]
               
# ------------------------------------------------------------------------------------------------------------------ 
    # 로봇 상태 확인
    def state_check(self):
        try:
            state = get_robot_state()
            if state in [3, 5, 6, 9, 10]:
                self.get_logger().warn(f"⚠️ 로봇 이상 상태 감지됨: {state}")
        except Exception as e:
            self.get_logger().error(f"state_check 에러: {e}")
# ------------------------------------------------------------------------------------------------------------------ 
    # 그리퍼 함수
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
    # 복구 함수
    def wait_and_spin(self, duration):
        end_time = time.time() + duration
        
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)

    def call_control(self, val, name):
        # 1. 함수가 호출되어 진입했는지 확인하는 로그
        self.get_logger().info(f"🔄 [요청 시도] {val}번, {name}")
        
        # 서비스 서버가 살아있는지 확인
        if not self.cli_control.wait_for_service(timeout_sec=1.0): 
            # ⚠️ 여기서 튕기면 "아, 서버 연결 자체가 안 되었구나!" 바로 알 수 있음
            self.get_logger().error(f"❌ [서버 연결 실패] {val}번, {name} 서버를 찾을 수 없습니다.")
            return False
        
        req = self.SetRobotControl.Request() #두산 로봇 제어 명령을 내리기 위한 공식 빈 신청서 한 장을 뽑아서 req라는 변수에 담아둔다
        req.robot_control = val  # 뽑아둔 빈 신청서의 '제어 명령' 칸에 val 값을 적고
        future = self.cli_control.call_async(req) # 그 작성된 신청서를 제어기 우체통에 쏙 집어넣는 것
        start = time.time()

        # 2. 메시지 발사가 정상적으로 끝났음을 알리는 로그
        self.get_logger().info(f"📤 [전송 완료] {val}번, {name} ➔ 응답 대기 시작")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if future.done(): 
                return True
            if time.time() - start > 2.0: 
                return False
            
    # 팀 고유 복구 (3 -> 2 -> 1)
    def recover_robot_team_style(self):
        print("\n🔄 [복구] 팀 고유 시퀀스 시작 (3 -> 2 -> 1)...")
        
        # 1. 3번 단계 실행 및 결과 검사
        if not self.call_control(3, "Safe Off Reset"):
            self.get_logger().error("❌ [FAIL] 3번 단계 실패! 복구를 중단합니다.")
            return False
        self.wait_and_spin(1.0)

        # 2. 2번 단계 실행 및 결과 검사
        if not self.call_control(2, "Safe Stop Reset"):
            self.get_logger().error("❌ [FAIL] 2번 단계 실패! 복구를 중단합니다.")
            return False
        self.wait_and_spin(1.0)

        # 3. 1번(서보 온) 단계 실행 및 결과 검사
        if not self.call_control(1, "Servo On"):
            self.get_logger().error("❌ [FAIL] 1번 단계 실패! 복구를 중단합니다.")
            return False

        # 모든 단계를 무사히 통과했을 때만 실행됨
        self.get_logger().info("   -> 서보 켜지는 중 (3초)...")
        self.wait_and_spin(3.0)
        self.get_logger().info("✅ recovery_success (복구 완료)")
        return True
# ------------------------------------------------------------------------------------------------------------------     
    # 동작 루프 실행 함수
    def run_pending(self):
        cmd = self.pending
        if cmd is None:
            return                 # 할 일 없으면 그냥 리턴
        try:
            if cmd == "home":
                self.move_home()
            elif cmd == "init":
                self.move_init()
        finally:
            self.pending = None    # 끝나면 비움
# ------------------------------------------------------------------------------------------------------------------     
    # 실제 이동 함수
    def move_home(self):
        try:
            self.get_logger().info("🏠 홈 포지션으로 이동 시작...")
            ret = movej([0, 0, 90, 0, 90, 0], vel=60, acc=60) 
            time.sleep(3.0); 
            self.get_logger().info(f"movel ret={ret}")
            self.get_logger().info(f"이동 후 state={get_robot_state()}, pos={get_current_posx()[0]}")
            self.get_logger().info("✅ 홈 포지션 완료") 
        except Exception as e:
            self.get_logger().error(f"❌ 홈 이동 중 에러 발생: {e} ")
        
    def move_init(self):
        try:
            self.get_logger().info("초기 포지션으로 이동 시작...")
            ret = movej([0, 0, 0, 0, 90, 0], vel=60, acc=60) 
            time.sleep(3.0); 
            self.get_logger().info(f"movel ret={ret}")
            self.get_logger().info(f"이동 후 state={get_robot_state()}, pos={get_current_posx()[0]}") 
            self.get_logger().info("✅ 초기 포지션 완료") 
        except Exception as e:
            self.get_logger().error(f"❌ 초기 포지션 이동 중 에러 발생: {e} ")

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
    DR_init.__dsr__node = node  # 내가 만든 노드 객체를 두산 라이브러리에 연결!
    global movel, movej, get_current_posx, posx, posj, set_safe_stop_reset_type, get_robot_state, set_robot_mode
    from DSR_ROBOT2 import movel, movej, get_current_posx, set_safe_stop_reset_type, get_robot_state, set_robot_mode
    from DR_common2 import posx, posj

    print("🚀 멀티스레드 백그라운드 스핀 + 메인 상태 체크 루프 시작!")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01) 
            node.state_check()
            node.run_pending() 
    except KeyboardInterrupt:
        print("🛑 사용자가 Ctrl + C를 눌러서 종료를 요청함!")

    finally:
        # 4. 안전한 자원 해제
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()    