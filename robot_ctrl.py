import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.get_logger('🚀 robot Controller 초기화 상태!!. 명령 대기중..')

        self.create_subscription(String, '/robot/cmd', self.robot_callback, 10)

        def robot_callback(self, msg):
            command = msg.data.lower().strip()
            self.get_logger().info(f"📢 들어온 음성 명령: {command}")

            if command == "home":
                self.get_logger().info(f"📢 들어온 음성 명령: {command}")
            elif command == "stop":
                self.get_logger().info(f"📢 들어온 음성 명령: {command}")
            else:
                self.get_logger().warn(f"⚠️ 알 수 없는 명령입니다: {command}")

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