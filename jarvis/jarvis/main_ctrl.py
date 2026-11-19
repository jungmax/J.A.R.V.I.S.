import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.get_logger().info('🚀 Main Controller 초기화 상태!!')

        # -------------------------------------------------------------
        # 1. 귀 달기 (구독자, Subscriber): 남의 말을 듣는 채널 개설
        # -------------------------------------------------------------
        # 누군가 '/voice/cmd'라는 우체통에 String(문자열) 메시지를 넣으면,
        # 자동으로 self.voice_callback 함수를 실행해라! (대기열 크기 10)
        # self.create_subscription(자료형, '채널이름', 실행할함수, 대기열크기)
        self.create_subscription(String, '/voice/cmd', self.voice_callback, 10)

        # -------------------------------------------------------------
        # 2. 입 달기 (발행자, Publisher): 내가 명령을 내리는 채널 개설
        # -------------------------------------------------------------
        # 나는 앞으로 하위 제어기에게 '/robot/nudge_cmd'라는 우체통을 통해 
        # String(문자열) 메시지를 보낼 것이다!
        self.robot_cmd_pub = self.create_publisher(String, '/robot/cmd', 10)

    def voice_callback(self, msg):
        # -------------------------------------------------------------
        # 3. 귀가 소리를 들었을 때 실행되는 행동(콜백 함수)
        # -------------------------------------------------------------
        # msg.data 속에는 상대방이 보낸 진짜 텍스트 알맹이가 들어있습니다.
        command = msg.data.lower().strip()
        self.get_logger().info(f"📢 들어온 음성 명령: {command}")

        # 만약 "home"이라는 단어가 들어오면 하위 제어기로 명령 발사!
        if "home" in command:
            self.get_logger().info("🏠 홈 위치로 이동 명령 퍼블리시 시작!")
            
            # 입(Publisher)을 통해 메시지 객체를 만들어서 밖으로 쏘아 보냄
            out_msg = "HOME"
            self.robot_cmd_pub.publish(String(data=out_msg))

    

def main(args=None):
    rclpy.init()
    node = MainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 강제 종료 감지")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()