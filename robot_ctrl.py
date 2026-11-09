import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading               #spin말고 독립적으로 일을 시킬수있는 코드
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.get_logger('🚀 robot Controller 초기화 상태!!. 명령 대기중..')

        # 1. 구독자 생성
        self.create_subscription(String, '/robot/cmd', self.robot_callback, 10)

        # 2. [그리퍼 연동] Modbus 클라이언트 초기화 (실제 그리퍼 IP와 포트에 맞게 설정)
        # 예시: IP "192.168.137.100", Port 502
        self.client = ModbusClient('192.168.137.100', port=502)
        self.client.connect()
        self.get_logger().info("🔌 그리퍼 Modbus 통신 연결 준비 완료")

    def robot_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"📢 들어온 음성 명령: {command}")

        if command == "home":
            self.get_logger().info("🏠 홈 포지션으로 이동합니다.")
            # 여기에 두산 로봇 홈 이동 함수 연결 예정
            
        elif command == "open":
            self.get_logger().info("🖐️ 그리퍼를 엽니다.")
            self.open_gripper() # 그리퍼 열기 함수 호출
            
        elif command == "close":
            self.get_logger().info("✊ 그리퍼를 닫습니다.")
            self.close_gripper() # 그리퍼 닫기 함수 호출
            
        elif command == "stop":
            self.get_logger().info("🛑 비상 정지 실행.")
        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 명령입니다: {command}")
    
    def open_gripper(self):
        try: 
            self.client.write_registers(0, [400, 1100, 16], unit=65)
            self.get_logger().info("👉 [그리퍼] 열기 명령 패킷 전송 성공")
        except Exception as e: 
            self.get_logger().error(f"⚠️ 그리퍼 열기 에러: {e}")

    def close_gripper(self):
        try:
            self.client.write_registers(0, [0, 1100, 16], unit=65)
            self.get_logger().info("👉 [그리퍼] 닫기 명령 패킷 전송 성공")
        except Exception as e:
            self.get_logger().error(f"⚠️ 그리퍼 열기 에러: {e}")
            
def main(args = None):
    rclpy.init(args=args)
    node = RobotController()
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