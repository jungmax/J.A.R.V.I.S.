# Ycjeong_project

메인 컨트롤러 -> 로봇 컨트롤러

초기상태

ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'home'}"

집으로 이동시킬 때

ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'home'}"

원하는 각도로 이동시킬 때

ros2 topic pub --once /robot/target_joint std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
