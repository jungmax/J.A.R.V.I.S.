# Ycjeong_project

메인 컨트롤러 -> 로봇 컨트롤러

# 기본 설정

~.py 수정했을 때는 colcon build 해주기

특정 폴더만 실행할 때

colcon build --packages-select 프로젝트폴더

새로운 터미널을 열었을 때

source install/setup.bash 

# 로봇 제어 실행

가상 시뮬레이션 런치 파일

 ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609 host:=127.0.0.1

로봇 파일 실행

 ros2 run jungmax1346  robot

# 토픽 문자 및 데이터 

초기상태

 ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'init'}"

집으로 이동시킬 때

 ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'home'}"

원하는 각도로 이동시킬 때

 ros2 topic pub --once /robot/target_joint std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
