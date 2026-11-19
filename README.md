# Robot Controller (ROS2 + Doosan M0609)

ROS2 토픽으로 두산 로보틱스 M0609 협동로봇과 그리퍼를 제어하는 노드입니다.
반복 동작(`circle`)을 `stop` 토픽으로 멈추고, 동작 중 들어온 다른 명령은 거절하는 구조를 포함합니다.

## 주요 기능

- 토픽 기반 동작 명령 (`home`, `init`, `circle`, `stop`)
- 그리퍼 제어 (Modbus TCP)
- 관절 각도 직접 지정 이동
- 로봇 에러 복구 시퀀스 (Safe Off Reset → Safe Stop Reset → Servo On)
- 반복 동작 정지 기능 (`stop_requested` 플래그)

# 기본 설정

~.py 수정했을 때 마다 항상 colcon build 해주기

특정 폴더만 실행할 때
```bash
colcon build --packages-select 프로젝트폴더
```
새로운 터미널을 열었을 때
```bash
source install/setup.bash 
```
# 로봇 제어 실행

가상 시뮬레이션 런치 파일
```bash
 ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609 host:=127.0.0.1
```
로봇 파일 실행
```bash
 ros2 run jungmax1346  robot
```
# 토픽 문자 및 데이터 

초기상태
```bash
 ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'init'}"
```
집으로 이동시킬 때
```bash
 ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'home'}"
```
원하는 각도로 이동시킬 때
```bash
 ros2 topic pub --once /robot/target_joint std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
