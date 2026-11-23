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
get_current_posj = None
posx = None
posj = None
set_safe_stop_reset_type = None
set_robot_mode = None
# -----------------------------------------------------------------------------------------------------------------
class StopRequested(BaseException):
    pass  
# -----------------------------------------------------------------------------------------------------------------
class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.get_logger().info("\n" + "=" * 50 + '\n 🚀 robot Controller 초기화 상태!!. 명령 대기중..\n' + "=" * 50)

        # 변수
        self.pending = None
        self.stop_requested = False  # 반복 함수 멈추기 위한 변수
        self.activing = None
        self.running = False

        # 1. 구독자 생성
        self.create_subscription(String, '/robot/cmd', self.robot_callback, 10)
        self.create_subscription(String, '/robot/gripper', self.gripper_callback, 10)
        self.create_subscription(String, '/robot/mode', self.mode_callback, 10)
        self.create_subscription(PoseStamped, '/robot/target_pose', self.pose_callback, 10)
        self.create_subscription(Float64MultiArray, '/robot/target_joint', self.joint_callback, 10)
        

        # 2. [그리퍼 연동] Modbus 클라이언트 초기화 (실제 그리퍼 IP와 포트에 맞게 설정)
        # 예시: IP "192.168.137.100", Port 502
        self.client = ModbusClient('192.168.137.100', port=502)
        # help(self.client.write_registers)
        self.client.connect()
        self.get_logger().info("\n" + "=" * 50 + '\n 🔌 그리퍼 Modbus 통신 연결 준비 완료\n' + "=" * 50)

        # 3. 복구용 서비스 클라이언트
        self.cli_control = self.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')
        self.SetRobotControl = SetRobotControl
# -----------------------------------------------------------------------------------------------------------------        
    # Callback 함수
    def robot_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"📢 들어온 음성 명령: {command}")

        if command == "stop":
            self.get_logger().info("🛑 비상 정지 실행.")
            self.stop_requested = True
            return
        
        if command in ["home", "init", "circle"]:
            self.def_pending(command)
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
            self.def_pending(command)
            

    def joint_callback(self, msg):
        # msg.data에 들어 있는 숫자 배열을 파이썬 리스트로 변환
        target_joint = list(msg.data)
        self.get_logger().info(f"📢 들어온 타겟 관절 각도: {target_joint}")
        self.target_joint_data = target_joint
        self.def_pending("move_custom_joint")

    def pose_callback(self, msg): 
        self.get_logger().info("📥 [포즈 토픽 수신됨!] 데이터 처리 시작...")
        
        # 1. 메시지에서 위치(x, y, z) 꺼내기
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z

        # 2. 단위 변환 (미터 단위를 밀리미터로 변환)
        if abs(tx) < 10.0: 
            tx *= 1000 
            ty *= 1000 
            tz *= 1000

        # 3. 콜백에서는 통신하지 않고 값만 변수에 저장한 뒤 즉시 펜딩 걸기!
        self.target_tx = tx
        self.target_ty = ty
        self.target_tz = tz
        
        self.get_logger().info(f"✅ 타겟 포즈 접수 완료 -> X: {tx}, Y: {ty}, Z: {tz}")
# ------------------------------------------------------------------------------------------------------------------     
    # 동작 루프 실행 함수
    def run(self):
        command = self.pending
        self.running = True
        self.activing = command

        try:
            if command == "home":
                self.move_home()
            elif command == "init":
                self.move_init()
            elif command == "circle":
                self.circle()      
            elif command == "move_custom_joint":         
                self.move_custom_joint() 
            elif command == "recover":
                self.mode_recover()

        except StopRequested:  
            self.get_logger().info(f"⏹ {command} 정지됨")
        
        finally:
            self.pending = None    # 끝나면 비움
            self.running = False
            self.activing = False
# ------------------------------------------------------------------------------------------------------------------     
    # 실제 이동 함수
    def move_home(self):
        try:
            self.get_logger().info("\n" + "-" * 50 + "\n 🏠 홈 포지션으로 이동 시작...\n" + "-" * 50)
            ret = movej([0, 0, 90, 0, 90, 0], vel=60, acc=60) 
            time.sleep(1.0); 
            self.get_logger().info(f"movel ret={ret}")
            self.get_logger().info(f"이동 후 state={get_robot_state()}, pos={get_current_posj()}")
            self.get_logger().info("\n" + "-" * 50 + "\n ✅ 홈 포지션 완료\n" + "-" * 50)
        except Exception as e:
            self.get_logger().error(f"❌ 홈 이동 중 에러 발생: {e} ")
        
    def move_init(self):
        try:
            self.get_logger().info("\n" + "-" * 50 + "\n 초기 포지션으로 이동 시작...\n" + "-" * 50)
            ret = movej([0, 0, 0, 0, 0, 0], vel=60, acc=60) 
            time.sleep(1.0); 
            self.get_logger().info(f"movel ret={ret}")
            self.get_logger().info(f"이동 후 state={get_robot_state()}, pos={get_current_posj()}") 
            self.get_logger().info("✅ 초기 포지션 완료") 
        except Exception as e:
            self.get_logger().error(f"❌ 초기 포지션 이동 중 에러 발생: {e} ")
              
    def circle(self):
        self.stop_requested = False   # 시작할 때 초기화
        angles = [0, 120, 240, 360]
        try:
            self.get_logger().info("\n" + "-" * 50 + f"\n{self.activing} 행동 시작\n" + "-" * 50)
            while rclpy.ok():
                for a in angles:
                    movej([a, 90, 0, 0, 0, 0], vel=60, acc=60)
                    self.wait_and_spin(1.0)
                    self.check_stop() 
            self.get_logger().info("⏹ 돌기 정지")
        except Exception as e:
            self.get_logger().error(f"❌ 돌기 중 에러 발생: {e}")

    def move_custom_joint(self):
        try:
            self.get_logger().info(f"🎯 지정한 각도(좌표)로 이동 시작: {self.target_joint_data}")
            # movej 함수를 이용해 해당 좌표로 직선 이동 (속도, 가속도 60)
            ret = movej(self.target_joint_data, vel=60, acc=60) 
            time.sleep(1.0) 
            self.get_logger().info(f"movej ret={ret}")
            self.get_logger().info(f"이동 후 state={get_robot_state()}, pos={get_current_posx()[0]}")
            self.get_logger().info("✅ 커스텀 각도 이동 완료") 
        except Exception as e:
            self.get_logger().error(f"❌ 커스텀 포즈 이동 중 에러 발생: {e}")
    
    def mode_recover(self):
                try:
                    self.get_logger().info("⚠️ 복구모드 시작")
                    self.recover_robot_team_style()
                except Exception as e:
                    self.get_logger().error(f"❌ 복구모드 중 에러 발생: {e} ")              
# ------------------------------------------------------------------------------------------------------------------       
    # 헬퍼 함수 (클래스 안에 추가)
    def def_pending(self, command):
        if self.running:
            self.get_logger().warn(
                f'⚠️ 현재 {self.activing} 동작 진행중. 멈추려면 stop 명령을 보내세요!')
            return False
        self.pending = command
        return True                      
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
    # 딜레이 스핀 함수
    def wait_and_spin(self, duration):
        end_time = time.time() + duration
        
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)
# ------------------------------------------------------------------------------------------------------------------ 
    # 복구 함수
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
                res = future.result()
                return res is not None and res.success
            
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

# -------------------------------------------------------------------------------------------------------------
    # STOP 함수
    def check_stop(self):
        if self.stop_requested:
            raise StopRequested()
# -------------------------------------------------------------------------------------------------------------            
def main(args = None):
    rclpy.init(args=args)
    node = RobotController()
    
    DR_init.__dsr__node = node  # 내가 만든 노드 객체를 두산 라이브러리에 연결!
    global movel, movej, get_current_posx, posx, posj, set_safe_stop_reset_type, get_robot_state, set_robot_mode, get_current_posj
    from DSR_ROBOT2 import movel, movej, get_current_posx, set_safe_stop_reset_type, get_robot_state, set_robot_mode, get_current_posj
    from DR_common2 import posx, posj

    print("\n"+ "=" * 60 + "\n 🚀 멀티스레드 백그라운드 스핀 + 메인 상태 체크 루프 시작!\n" + "=" * 60)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec = 1) 
            node.state_check()
            node.run() 
    except KeyboardInterrupt:
        print("🛑 사용자가 Ctrl + C를 눌러서 종료를 요청함!")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()    