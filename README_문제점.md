# Robot Controller (ROS2 + Doosan M0609)

ROS2 토픽으로 두산 로보틱스 M0609 협동로봇과 그리퍼를 제어하는 노드입니다.
반복 동작(`circle`)을 `stop` 토픽으로 멈추고, 동작 중 들어온 다른 명령은 거절하는 구조를 포함합니다.

## 주요 기능

- 토픽 기반 동작 명령 (`home`, `init`, `circle`, `stop`)
- 그리퍼 제어 (Modbus TCP)
- 관절 각도 직접 지정 이동
- 로봇 에러 복구 시퀀스 (Safe Off Reset → Safe Stop Reset → Servo On)
- 반복 동작 정지 기능 (`stop_requested` 플래그)
- 동작 중 중복 명령 거절 (`busy` 플래그)

## 구독 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/robot/cmd` | `std_msgs/String` | `home`, `init`, `circle`, `stop` |
| `/robot/gripper` | `std_msgs/String` | `open`, `close` |
| `/robot/mode` | `std_msgs/String` | `recover` |
| `/robot/target_joint` | `std_msgs/Float64MultiArray` | 관절 각도 6개로 이동 |
| `/robot/target_pose` | `geometry_msgs/PoseStamped` | 타겟 포즈 수신 (이동 동작은 미구현) |

## 사용 방법

```bash
# 반복 동작 시작
ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'circle'}"

# 반복 동작 정지
ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'stop'}"

# 홈 위치로 이동
ros2 topic pub --once /robot/cmd std_msgs/msg/String "{data: 'home'}"

# 에러 복구
ros2 topic pub --once /robot/mode std_msgs/msg/String "{data: 'recover'}"
```

## 동작 구조

```
토픽 수신 → 콜백이 pending에 명령 저장 → main 루프의 run()이 pending을 읽어 실행
```

- 콜백은 `pending`에 "할 일 메모"만 남기고 바로 끝납니다.
- 실제 동작은 `main()` 루프의 `run()`이 실행합니다.
- 반복 동작 중에는 `wait_and_spin()`이 `spin_once`를 대신 돌려 `stop` 콜백을 받을 수 있게 합니다.

## 트러블슈팅

| # | 문제 | 원인 | 해결 방법 |
|---|---|---|---|
| 1 | circle이 무한히 돌고 멈출 방법이 없음 | `while True`라 빠져나올 조건이 없음 | `self.stop_requested` 플래그를 만들고 `while rclpy.ok() and not self.stop_requested`로 변경 |
| 2 | stop 토픽을 보내도 콜백이 실행되지 않음 | `time.sleep()` 동안 `spin_once`가 안 돌아서 콜백이 호출되지 않음 | `time.sleep(1.0)`을 `self.wait_and_spin(1.0)`으로 교체 |
| 3 | `pending`에 stop을 넣으면 처리되지 않음 | `circle()`이 `run()` 안에 갇혀 있어서 pending을 읽는 곳까지 못 돌아옴 | `pending`과 별개로 `stop_requested`라는 신호 변수를 따로 사용 |
| 4 | 360 → 0으로 갈 때 역회전해서 원이 이어지지 않음 | `movej`는 목표 각도와의 숫자 차이만큼 그대로 이동함 | 누적 방식(`angle += step`)은 J1 한계(±360° 추정)를 넘으면 에러가 나므로, 한계 안에서 되돌아오는 기존 방식 유지 |
| 5 | circle 도는 중 home을 보내면 로그만 뜨고 실행이 안 됨 | 콜백이 `pending`에 넣어도 `circle()`이 끝난 뒤 `finally`에서 `pending = None`으로 지워짐 | `self.busy` 플래그를 추가하고, busy일 때는 경고 로그를 찍고 `return` |
| 6 | busy 체크 코드가 콜백 안에 길게 들어가 지저분함 | 콜백 세 개에 같은 체크를 반복해야 함 | `reject_if_busy()` 헬퍼 함수로 분리 |
| 7 | 실행 후 두 번째 명령부터 계속 거절됨 | `run()`의 `finally`에서 `busy = False`를 안 해서 busy가 계속 True로 남음 | `finally`에 `self.busy = False` 추가 |
| 8 | busy일 때 stop도 거절됨 | stop 처리가 busy 체크보다 뒤에 있음 | stop을 busy 체크보다 **먼저** 처리 |
| 9 | `self.busy` 관련 에러 가능성 | `__init__`에 `self.busy = False` 선언이 없음 | `__init__`에서 `self.busy = False` 선언 |

## 학습한 개념

| 개념 | 핵심 |
|---|---|
| `self.` 변수 | 객체에 붙은 메모리 칸 하나를 모든 메서드가 공유함. 값이 바뀌는 건 칸을 공유하기 때문이지 while 위치와는 무관 |
| 콜백 실행 시점 | 저절로 실행되지 않고 `spin_once`가 호출될 때만 실행됨 |
| `return` | 그 함수 하나만 종료하고, 클래스나 노드는 영향 없음 |
| while 조건 vs if 체크 | while은 for가 전부 끝나야 다시 검사되므로, 이동마다 확인하려면 for 안의 if가 필요함 |
| `break` vs `return` | `break`는 가장 안쪽 반복문 하나만 탈출하고, `return`은 함수 전체를 종료함 |

## 알려진 한계

- `movej`가 실행되는 동안에는 stop이 반응하지 않습니다 (이동 하나가 끝나야 멈춤).
- 즉시 정지가 필요하면 스레드 분리와 두산 `stop()` 명령이 필요합니다.
- J1 한계 수치(±360°)는 추정값이므로 두산 M0609 매뉴얼에서 확인이 필요합니다.
- `target_pose` 토픽은 값을 저장만 하고 이동 동작은 아직 구현되지 않았습니다.

## 향후 개선

- `MultiThreadedExecutor` 또는 액션 서버로 구조 변경
- 동작 중 즉시 정지 (`stop(DR_QSTOP)` 연동)
- 명령 큐 방식으로 변경 (stop 후 다음 명령 자동 실행)
