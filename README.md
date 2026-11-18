# Ycjeong_project

메인 컨트롤러 -> 로봇 컨트롤러

# 기본 설정

.py 수정했을 때는 colcon build 해주기

특정 폴더만 실행할 때

colcon build --packages-select 프로젝트폴더

새로운 터미널을 열었을 때

source install/setup.bash 

# 토픽 문자 및 데이터 

초기상태

ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'home'}"

집으로 이동시킬 때

ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'home'}"

원하는 포즈로 이동 시킬 때

ros2 topic pub --once /robot/target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}"

원하는 각도로 이동시킬 때

ros2 topic pub --once /robot/target_joint std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
