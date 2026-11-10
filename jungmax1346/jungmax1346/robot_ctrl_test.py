import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
import time
import queue
import threading

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

# --- 그리퍼 컨트롤러 ---
class GripperController:
    def __init__(self, ip, port):
        self.client = ModbusClient(ip, port=int(port), stopbits=1, bytesize=8, parity='E', baudrate=115200, timeout=1)
        try: 
            self.client.connect()
        except: 
            pass
    def open(self):
        try: 
            self.client.write_registers(0, [400, 1100, 16], unit=65)
        except: 
            pass
    def close(self):
        try: 
            self.client.write_registers(0, [300, 0, 16], unit=65)
        except: 
            pass

# --- 로봇 제어 노드 (클래스 구조 적용) ---
class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller", namespace=ROBOT_ID)
        self.get_logger().info("🤖 Robot Controller Node Started (Class-based)")

        # DSR 함수 바인딩용 변수 초기화
        self.init_dsr_globals()

        # 그리퍼 초기화
        self.gripper = GripperController("192.168.1.1", 502)

        # 통신 인터페이스 (구독/발행)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        
        self.create_subscription(String, '/robot/nudge_cmd', self.nudge_callback, 10)
        self.create_subscription(String, '/robot/gripper', self.gripper_callback, 10)
        self.create_subscription(String, '/robot/jog', self.jog_callback, 10)
        self.create_subscription(PoseStamped, '/robot/target_pose', self.pose_callback, 10)
        self.create_subscription(Float64MultiArray, '/robot/target_joint', self.joint_callback, 10)

        # 복구용 서비스 클라이언트
        self.cli_control = self.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')

        # 명령 처리용 큐 및 스레드
        self.cmd_queue = queue.Queue()
        self.is_error_mode = False

        # 실행 백그라운드 스레드 시작 (블로킹 API 처리용)
        self.worker_thread = threading.Thread(target=self.process_command_loop, daemon=True)
        self.worker_thread.start()

    def init_dsr_globals(self):
        global movej, movel, get_current_posx, get_current_posj, get_robot_state, set_robot_mode, set_safe_stop_reset_type
        try:
            from DSR_ROBOT2 import (
                movej, movel, get_current_posx, get_current_posj, 
                get_robot_state, set_robot_mode, set_safe_stop_reset_type
            )
            from DR_common2 import posx, posj
            self.posx = posx
            self.posj = posj
            self.get_logger().info("✅ DSR 라이브러리 로드 성공")
        except ImportError as e:
            self.get_logger().error(f"❌ 라이브러리 로드 실패: {e}")

    def publish_status(self, msg):
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(f"📤 Status: {msg}")

    # --- 토픽 콜백 함수들 (큐에 명령 적재) ---
    def nudge_callback(self, msg):
        cmd = msg.data.upper().strip()
        if cmd == "RECOVERY":
            self.cmd_queue.put(("RECOVERY", None))
        else:
            self.cmd_queue.put(("CMD", cmd))

    def gripper_callback(self, msg):
        self.cmd_queue.put(("GRIPPER", msg.data.lower().strip()))

    def jog_callback(self, msg):
        self.cmd_queue.put(("JOG", msg.data.upper().strip()))

    def joint_callback(self, msg):
        self.cmd_queue.put(("JOINT", list(msg.data)))

    def pose_callback(self, msg):
        self.cmd_queue.put(("POSE", msg.pose))

    # --- 복구 및 유틸리티 ---
    def call_control(self, val):
        if not self.cli_control.wait_for_service(timeout_sec=1.0): return False
        req = SetRobotControl.Request()
        req.robot_control = val
        future = self.cli_control.call_async(req)
        start = time.time()
        while rclpy.ok():
            if future.done(): return True
            if time.time() - start > 2.0: return False
            time.sleep(0.01)

    def recover_robot_team_style(self):
        self.get_logger().info("🔄 [복구] 팀 고유 시퀀스 시작 (3 -> 2 -> 1)...")
        if set_safe_stop_reset_type: set_safe_stop_reset_type(1)
        self.call_control(3); time.sleep(1.0)
        self.call_control(2); time.sleep(1.0)
        self.call_control(1)
        self.get_logger().info("   -> 서보 켜지는 중 (3초)...")
        time.sleep(3.0)
        self.publish_status("recovery_success")

    # --- 메인 실행 루프 (스레드에서 동작하여 ROS2 스핀을 방해하지 않음) ---
    def process_command_loop(self):
        while rclpy.ok():
            # 로봇 에러 상태 모니터링
            try:
                current_state = get_robot_state()
                if current_state in [3, 5, 6, 9, 10]:
                    if not self.is_error_mode:
                        self.is_error_mode = True
                        self.get_logger().error(f"🚨 에러 감지! (State: {current_state})")
                        self.publish_status("fail: collision_detected")
            except Exception:
                pass

            try:
                msg_type, msg_val = self.cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # 복구 명령 처리
            if msg_type == "RECOVERY":
                self.recover_robot_team_style()
                self.is_error_mode = False
                continue

            if self.is_error_mode:
                if get_robot_state() == 1: 
                    self.is_error_mode = False
                else: 
                    self.get_logger().warn("⛔ 에러 상태. RECOVERY 필요.")
                    continue

            self.get_logger().info(f"⚙️ [실행] {msg_type} -> {msg_val}")

            try:
                if msg_type == "GRIPPER":
                    if msg_val == "open": 
                        self.gripper.open(); time.sleep(0.5); self.publish_status("opened")
                    elif msg_val == "close": 
                        self.gripper.close(); time.sleep(1.0); self.publish_status("gripped")

                elif msg_type == "CMD":
                    curr_pos, _ = get_current_posx()
                    if curr_pos is None: continue
                    target = list(curr_pos)
                    d = 30.0
                    
                    if msg_val == "UP": target[2] += d
                    elif msg_val == "DOWN": target[2] -= d
                    elif msg_val == "LEFT": target[0] -= d
                    elif msg_val == "RIGHT": target[0] += d
                    elif msg_val == "FORWARD": target[1] += d
                    elif msg_val == "BACKWARD": target[1] -= d
                    
                    elif msg_val == "SEARCH_TOOL":
                        movej([47.861, 31.503, 43.848, -0.071, 104.867, 47.664], vel=60, acc=60)
                        time.sleep(0.5); self.publish_status("arrived"); continue

                    elif msg_val == "HANDOVER":
                        movej([-7.522, -28.045, 113.411, -7.961, 53.899, 9.936], vel=60, acc=60)
                        time.sleep(1.0); self.publish_status("arrived"); continue

                    elif msg_val == "SCAN":
                        movej([-51.291, -38.599, 129.907, 55.324, 68.009, 9.435], vel=60, acc=60)
                        time.sleep(1.0); self.publish_status("arrived"); continue

                    elif msg_val == "MAGNIFY_PRE":
                        movej([-30.98, -1.46, 96.56, 24.10, 81.27, 76.75], vel=60, acc=60)
                        time.sleep(1.0); self.publish_status("arrived"); continue

                    elif msg_val == "START_TASK":
                        movej([0,0,90,0,90,0], vel=60, acc=60)
                        self.gripper.open(); self.publish_status("arrived"); continue

                    movel(self.posx(target), vel=60, acc=60)
                    self.publish_status("arrived")

                elif msg_type == "POSE":
                    curr_pos, _ = get_current_posx()
                    if curr_pos:
                        tx, ty, tz = msg_val.position.x, msg_val.position.y, msg_val.position.z
                        if abs(tx) < 10.0: tx*=1000; ty*=1000; tz*=1000
                        target = [tx, ty, tz, curr_pos[3], curr_pos[4], curr_pos[5]]
                        movel(self.posx(target), vel=100, acc=100)
                        time.sleep(0.5); self.publish_status("arrived_target")

                elif msg_type == "JOINT":
                    movej(msg_val, vel=60, acc=60); time.sleep(0.5); self.publish_status("joint_arrived")

                elif msg_type == "JOG":
                    curr_j = get_current_posj()
                    if curr_j:
                        t = list(curr_j)
                        if msg_val == "TURN_FRONT": t[5] -= 15.0
                        elif msg_val == "TURN_BACK": t[5] += 15.0
                        movej(t, vel=60, acc=60); self.publish_status("jog_done")

            except Exception as e:
                self.get_logger().error(f"💥 동작 실패: {e}")
                self.is_error_mode = True
                self.publish_status("fail: collision_detected")

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotController()
    
    # 두산 라이브러리 노드 바인딩
    global DR_init
    DR_init.__dsr__node = node 

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()