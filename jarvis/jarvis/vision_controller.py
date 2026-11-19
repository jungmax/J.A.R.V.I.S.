import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
from collections import deque
import cv2

from jungmax1346.vision_detector import ObjectDetector, HandDetector

import DR_init

ROBOT_ID = "dsr01"


class CommandVisionNode(Node):
    def __init__(self, yolo_model_path: str, calibration_npy: str, cap):
        super().__init__('command_vision_node')
        
        self.get_logger().info("=== Command Vision Node (Webcam Test Mode) 초기화 ===")
        
        # 1. 웹캠 및 카메라 설정 (웹캠은 고정 파라미터 사용)
        self.cap = cap
        self.intrinsics = {'fx': 600.0, 'fy': 600.0, 'ppx': 320.0, 'ppy': 240.0}
        
        # 2. CV 모듈 초기화
        self.object_detector = ObjectDetector(yolo_model_path, conf_threshold=0.6)
        self.hand_detector = HandDetector()
        
        # 3. 캘리브레이션 (파일이 없으면 단위행렬 사용)
        if os.path.exists(calibration_npy):
            self.gripper2cam = np.load(calibration_npy)
            self.get_logger().info(f"✓ 캘리브레이션 로드 성공")
        else:
            self.gripper2cam = np.eye(4)
            self.get_logger().warn(f"⚠ 캘리브레이션 파일 없음, 단위 행렬 사용")
        
        # 4. 물체별 Offset 설정 (픽셀 단위)
        self.object_offsets = {
            'magnifier': {
                'pixel': (0, 60),
                'description': '손잡이 위치',
                'enabled': True
            },
            'pcb': {'pixel': (0, 0), 'description': '중앙', 'enabled': False},
            'flux': {'pixel': (0, 0), 'description': '중앙', 'enabled': False},
            'pump': {'pixel': (0, 0), 'description': '중앙', 'enabled': False}
        }
        
        # 5. ROS2 Publisher
        self.bridge = CvBridge()
        self.debug_image_pub = self.create_publisher(Image, '/vision/debug_image', 10)
        self.cmd_sub = self.create_subscription(String, '/vision/cmd', self.command_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/target_pose', 10)
        self.status_pub = self.create_publisher(String, '/vision/status', 10)
        
        # 6. 상태 관리
        self.current_mode = "IDLE"
        self.target_object = None
        self.hit_buffer = deque(maxlen=3)
        self.last_published = None
        
        self.frame_counter = 0
        self.frame_skip = 5
        
        # 7. 시각화 설정
        self.enable_visualization = True
        self.debug_frame = None
        
        # 8. 메인 루프 (15fps)
        self.create_timer(1/15.0, self.main_loop)
        
        self.get_logger().info("✓ Command Vision Node 준비 완료 (웹캠 모드)")
    
    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        self.get_logger().info(f"[CMD] 명령 수신: '{command}'")
        self._parse_command(command)
    
    def _parse_command(self, command: str):
        if command == "track_hand":
            self.current_mode = "FINGER_POINTING"
            self.target_object = None
            self.hit_buffer.clear()
            self.last_published = None
            self.frame_counter = 0
            self.get_logger().info("[MODE] FINGER_POINTING - 손가락 가리킴")
            self._publish_status("손가락으로 가리켜주세요")
            return
        
        object_keywords = {
            'pcb': ['pcb', '기판'],
            'flux': ['flux', '플럭스'],
            'magnifier': ['magnifier', '돋보기'],
            'pump': ['pump', '흡입기']
        }
        
        for obj_name, keywords in object_keywords.items():
            if any(kw in command for kw in keywords):
                self.current_mode = "OBJECT_DETECT"
                self.target_object = obj_name
                self.last_published = None
                self.get_logger().info(f"[MODE] OBJECT_DETECT - '{self.target_object}' 찾기")
                self._publish_status(f"'{self.target_object}' 검색 중...")
                return

        if any(word in command for word in ['중지', '취소', '멈춰', 'stop', 'cancel']):
            self.current_mode = "IDLE"
            self.target_object = None
            self.hit_buffer.clear()
            self.get_logger().info("[MODE] IDLE")
            self._publish_status("대기 중")
            return
        
        self.get_logger().warn(f"[CMD] 인식 불가: '{command}'")
        self._publish_status("명령을 인식할 수 없습니다")
    
    def main_loop(self):
        ret, color_img = self.cap.read()
        if not ret or color_img is None:
            return
        
        if self.enable_visualization:
            self.debug_frame = color_img.copy()
        
        if self.current_mode == "OBJECT_DETECT":
            self._process_object_detection(color_img)
        elif self.current_mode == "FINGER_POINTING":
            self._process_finger_pointing(color_img)
        
        if self.enable_visualization and self.debug_frame is not None:
            self._publish_debug_image()
    
    def _apply_offset(self, center_2d, obj_class_name):
        offset_config = self.object_offsets.get(obj_class_name.lower(), None)
        if offset_config is None or not offset_config['enabled']:
            return center_2d
        x_offset, y_offset = offset_config['pixel']
        return (int(center_2d[0] + x_offset), int(center_2d[1] + y_offset))
    
    def _draw_detections(self, detections):
        if self.debug_frame is None:
            return
        for det in detections:
            x1, y1, x2, y2 = map(int, det.bbox)
            cv2.rectangle(self.debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det.class_name} {det.confidence:.2f}"
            cv2.putText(self.debug_frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.circle(self.debug_frame, det.center_2d, 5, (0, 255, 0), -1)
    
    def _draw_hand(self, hand_landmarks):
        if self.debug_frame is None or hand_landmarks is None:
            return
        try:
            tip_x, tip_y = hand_landmarks.fingertip_2d
            wrist_x, wrist_y = hand_landmarks.wrist_2d
            cv2.circle(self.debug_frame, (tip_x, tip_y), 10, (255, 0, 0), -1)
            cv2.circle(self.debug_frame, (wrist_x, wrist_y), 10, (0, 0, 255), -1)
            cv2.line(self.debug_frame, (wrist_x, wrist_y), (tip_x, tip_y), (255, 255, 0), 3)
        except Exception as e:
            self.get_logger().error(f"손 시각화 실패: {e}")
    
    def _draw_intersection(self, intersection_2d):
        if self.debug_frame is None or intersection_2d is None:
            return
        x, y = intersection_2d
        cv2.circle(self.debug_frame, (int(x), int(y)), 12, (0, 0, 255), 4)
        cv2.circle(self.debug_frame, (int(x), int(y)), 6, (255, 255, 255), -1)
    
    def _draw_buffer_status(self):
        if self.debug_frame is None:
            return
        h, w, _ = self.debug_frame.shape
        cv2.putText(self.debug_frame, f"Mode: {self.current_mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if self.target_object:
            cv2.putText(self.debug_frame, f"Target: {self.target_object}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    def _publish_debug_image(self):
        if self.debug_frame is None:
            return
        try:
            self._draw_buffer_status()
            img_msg = self.bridge.cv2_to_imgmsg(self.debug_frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera"
            self.debug_image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 발행 실패: {e}")
    
    def _process_finger_pointing(self, color_img):
        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            hand_landmarks = self.hand_detector.detect(color_img)
            self._draw_hand(hand_landmarks)
            return
        
        hand_landmarks = self.hand_detector.detect(color_img)
        self._draw_hand(hand_landmarks)
        if hand_landmarks is None:
            self.hit_buffer.clear()
            return
        
        u, v = hand_landmarks.fingertip_2d
        self._draw_intersection((u, v))
        
        # 웹캠 모드이므로 가상의 고정 깊이(Z=500mm) 사용
        dummy_3d = np.array([0.0, 0.0, 500.0])
        self._publish_pose(dummy_3d, "fingertip_tracking")
        self.current_mode = "IDLE"
        self._publish_status("검지 끝 추적 완료 (웹캠 가상 좌표)")
        self.hit_buffer.clear()
    
    def _process_object_detection(self, color_img):
        detections = self.object_detector.detect(color_img)
        self._draw_detections(detections)
        
        target_detection = None
        for det in detections:
            if self.target_object.lower() in det.class_name.lower():
                target_detection = det
                break
        
        if target_detection is None:
            return
        
        u, v = self._apply_offset(target_detection.center_2d, target_detection.class_name)
        self.get_logger().info(f"[OBJ] 감지됨: {target_detection.class_name} at pixel=({u}, {v})")
        
        # 웹캠 모드 가상 3D 좌표 발행
        dummy_3d = np.array([0.0, 0.0, 500.0])
        self._publish_pose(dummy_3d, target_detection.class_name)
        self.current_mode = "IDLE"
        self._publish_status("대기 중")
    
    def _publish_pose(self, position, object_name):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(msg)
        self.get_logger().info(f"[PUBLISH] 좌표 발행 완료: {object_name}")
        self._publish_status(f"{object_name} 검출 완료")
    
    def _publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)


def main(args=None):
    import argparse
    
    parser = argparse.ArgumentParser(description="Command-based Vision Node (Webcam)")
    parser.add_argument("--model", type=str, default=None, help="YOLO 모델 경로")
    parsed = parser.parse_args()
    
    rclpy.init(args=args)
    
    print("\n" + "="*50)
    print("  Command-based Vision Node (Webcam Mode)")
    print("="*50)
    
    robot_node = rclpy.create_node("vision_interface_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = robot_node
    
    # 웹캠 연결 (ID: 0)
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Error: 웹캠을 켤 수 없습니다.")
        rclpy.shutdown()
        return
    print("✓ 웹캠 연결 성공")
    
    model_path = parsed.model if parsed.model else os.path.expanduser("~/ros2_ws/src/jungmax1346/models/result_4.pt")
    calib_path = os.path.expanduser("~/ros2_ws/src/jungmax/calibration/T_gripper2camera.npy")
    
    if not os.path.exists(model_path):
        print(f"⚠ 경고: 모델 파일 없음 - {model_path} (객체 검출 불가)")
    
    vision_node = CommandVisionNode(model_path, calib_path, cap)
    
    executor = MultiThreadedExecutor()
    executor.add_node(vision_node)
    
    print("\n구독: /vision/cmd (또는 /dsr01/vision/cmd)")
    print("발행: /vision/target_pose, /vision/debug_image\n")
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        cap.release()
        rclpy.shutdown()


if __name__ == "__main__":
    main()