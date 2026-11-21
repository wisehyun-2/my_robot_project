# ros2_control 학습 가이드 (bumperbot)

참조 저장소의 완성된 로봇 패키지(bumperbot)를 기능 단위로 나눠서, **빈 작업 공간에서 파일을 하나씩 손으로 치며** ros2_control 패키지를 완성하는 가이드다(2026-11-21 작성). 단계마다 "왜 필요한가 → 코드 → 코드 설명 → 빌드와 시험" 순서로 간다. 이 공부가 끝나면 v5 로봇의 패키지를 쓴다(13장).

- 기준 코드: `references/bumperbot_odometry_control`(AntoBrandi, "Self Driving and ROS 2 - Learn by Doing! Odometry & Control", Apache-2.0, 커밋 `30c9361`). 섹션 폴더마다 그 시점의 `bumperbot_ws` 전체가 들어 있다.
- 가이드의 코드는 그 코드를 **ROS 2 Jazzy**에 맞게 고친 것이다. 노드는 C++만 쓴다(launch는 Python). 달라진 곳은 각 장 끝의 "참조 코드와 다른 점"과 00장 7절에 모았다.

## 준비

1. 참조 저장소를 받는다(저장소 맨 위에서). 메시(STL)와 대조용 원본 코드가 여기 있다.

   ```bash
   git submodule update --init references/bumperbot_odometry_control references/ros2_control_demos
   ```

2. 패키지를 설치한다(Ubuntu 24.04, ROS 2 Jazzy).

   ```bash
   sudo apt install ros-jazzy-desktop ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard ros-jazzy-robot-localization libserial-dev socat python3-colcon-common-extensions
   ```

   조이스틱을 쓸 때만 `ros-jazzy-joy ros-jazzy-joy-teleop`을 더한다.

3. 연습 작업 공간은 저장소 **밖** `~/bumperbot_ws`에 만든다(01장). 연습 코드는 v5 코드가 아니라서 저장소에 넣지 않는다.

가이드의 명령은 저장소가 `~/my_robot_project`에 있다고 보고 쓴다. 다른 곳에 있으면 그 경로만 바꾼다.

## 장 목록

| 장 | 참조 섹션 | 만드는 것 | 비중 |
|---|---|---|---|
| [00 큰 그림](00_big_picture.md) | — | ros2_control의 구조, 시뮬·실물의 노드 구성, 필요한 파일, 패키지 구조 | 먼저 읽는다 |
| [01 ROS 2 기초](01_section3_ros2_basics.md) | 3 (+4 일부) | 작업 공간, `package.xml`·`CMakeLists.txt`·`setup.py`, pub/sub, 파라미터 | 패키지 구조는 자세히 |
| [02 로봇 모델과 Gazebo](02_section4_description_gazebo.md) | 4 | URDF/xacro, RViz, Gazebo에 스폰 | 핵심 |
| [03 ros2_control 붙이기](03_section5_ros2_control.md) | 5 | `<ros2_control>` 태그, gz 플러그인, 제어기 yaml, 스포너 | **핵심** |
| [04 운동학 맛보기](04_section6_kinematics.md) | 6 | turtlesim으로 상대 자세 | 짧다 |
| [05 차동 구동과 simple_controller](05_section7_differential_kinematics.md) | 7 | 직접 쓴 노드 + 범용 제어기, `diff_drive_controller`, 키보드 조종 | 핵심 |
| [06 메시지 패키지, 서비스, TF2](06_section8_tf2_services.md) | 8 | 인터페이스 패키지, 서비스, TF | 중간 |
| [07 오도메트리](07_section9_odometry.md) | 9 | 바퀴 각도 → odom 메시지와 TF | 핵심 |
| [08 노이즈 제어기](08_section10_noisy_controller.md) | 10 | 일부러 틀린 오도메트리 | 짧다 |
| [09 센서 융합](09_section11_sensor_fusion.md) | 11 | Gazebo IMU, 칼만 필터, robot_localization | 중간 |
| [10 라이프사이클 노드](10_section12_lifecycle_node.md) | 12 | 상태 기계(11장 준비) | 짧다 |
| [11 하드웨어 인터페이스 플러그인](11_section12_hardware_interface.md) | 12 | 시리얼 노드, 가짜 아두이노, `SystemInterface` 플러그인, pluginlib 등록 | **핵심** |
| [12 실물 구성 띄우기와 bringup](12_section12_bringup.md) | 12 | xacro로 시뮬/실물 전환, `ros2_control_node`, bringup 패키지 | **핵심** |
| [13 v5 로봇으로 옮길 때](13_to_v5.md) | — | 그대로 쓰는 것, 달라지는 것, 이 가이드에 없는 것 | 마지막에 읽는다 |

순서대로 가는 것이 기본이다. 뒤 장의 코드는 앞 장에서 만든 파일 위에 쌓인다. ros2_control의 줄기는 02 → 03 → 05 → 07 → 11 → 12이고, 나머지 장은 그 줄기를 받치는 곁가지다(장마다 무엇을 받치는지는 00장 6절에 있다).

## 읽는 법

- **코드 블록 위의 표기.** `> 파일: bumperbot_controller/CMakeLists.txt — 전체`는 그 파일 전체다(경로는 `~/bumperbot_ws/src/` 기준). `— 수정`은 파일의 일부이고, 내용이 `...`뿐인 줄(`# ...`, `// ...`, `<!-- ... -->`)은 "원래 있던 줄을 그대로 둔다"는 뜻이라 파일에 치지 않는다. `(단계 N 시점)`은 그 단계가 끝났을 때의 모습이다.
- **기대 결과.** 명령 아래의 출력은 이 가이드를 쓸 때 실제로 본 것이다. 시각, 프로세스 번호, 실행 시점에 따라 달라지는 숫자는 다를 수 있다. 통과 조건은 글로 따로 적혀 있다.
- **터미널.** 끝나지 않는 명령(`ros2 run`, `ros2 launch` 등)은 "터미널 A, B"처럼 나눠 적었다. 새 터미널을 열면 `source ~/bumperbot_ws/install/setup.bash`부터 한다.
- **출처 표기.** "(일반 지식)"은 이 컴퓨터에서 확인한 것이 아니라 일반적으로 알려진 내용이라는 뜻이고, "확인하지 못했다"는 말 그대로다. 표기가 없는 사실은 설치된 헤더나 실제 실행으로 확인한 것이다.
- 막히면 그 장의 "자주 나는 오류"를 먼저 보고, 참조 저장소의 같은 파일과 나란히 놓고 본다(다른 곳은 "참조 코드와 다른 점" 표에 있다).

## 진행 체크리스트

- [ ] 00 큰 그림을 한 번 읽었다
- [ ] 01 `simple_publisher`가 `/chatter`를 내고 `ros2 param set`이 된다. Python 패키지도 한 번 빌드했다
- [ ] 02 `check_urdf`가 링크 트리를 내고, Gazebo에 로봇이 뜨고 `/clock`이 흐른다
- [ ] 03 `ros2 control list_controllers`에 제어기 둘이 active이고, 숫자 배열 토픽으로 바퀴가 돈다
- [ ] 04 두 거북이의 상대 위치와 회전 행렬이 출력된다
- [ ] 05 `/bumperbot_controller/cmd_vel`로 로봇이 움직인다(직접 쓴 노드 구성과 `diff_drive_controller` 구성 둘 다). 키보드로 조종했다
- [ ] 06 서비스가 응답하고 `tf2_echo`가 변환을 낸다
- [ ] 07 odom 토픽과 TF `odom → base_footprint`가 Gazebo의 실제 위치와 1% 안쪽으로 맞는다
- [ ] 08 `odom_noisy`가 `odom`에서 점점 벌어진다
- [ ] 09 `/imu/out`이 나오고, EKF의 방향이 `odom_noisy`보다 실제에 가깝다
- [ ] 10 `ros2 lifecycle set`으로 상태를 옮기면 `active`에서만 구독이 동작한다
- [ ] 11 플러그인이 경고 없이 빌드되고 pluginlib에 등록됐다. 시리얼 노드가 가짜 아두이노와 왕복한다
- [ ] 12 `real_robot.launch.py`(가짜 아두이노)에서 하드웨어가 active이고, 직진·회전 명령이 맞는 좌우로 시리얼에 나간다. `simulated_robot.launch.py`도 된다
- [ ] 13 v5에서 새로 배워야 할 것(제어기 플러그인 등)을 확인했다

## 이 가이드를 어디까지 믿어도 되는가

- **확인한 것.** 가이드의 코드는 모두 이 노트북(Ubuntu 24.04, Jazzy, `ros2_control` 4.50.0)에서 장 순서대로 만들어 빌드하고 돌려 본 것이다. 최종 여덟 패키지는 한 작업 공간에서 처음부터 경고 없이 빌드된다. 시뮬레이션(두 제어기 구성, IMU, EKF)과 실물 구성(가상 시리얼 + 가짜 아두이노)을 끝까지 돌렸다. 가이드 속 코드 블록은 그 검증된 파일과 글자 그대로 같은지 기계로 대조했다.
- **확인하지 못한 것.** 창이 뜨는 프로그램(Gazebo 화면, RViz, `joint_state_publisher_gui`, turtlesim 창)은 창 없이 서버만 띄우거나 토픽으로 확인했다. 화면에 무엇이 보이는지는 직접 보지 못했다. 진짜 아두이노와 `/dev/ttyUSB0`, 실제 조이스틱, 실제 키 입력도 시험하지 못했다(가상 터미널과 가짜 메시지로 대신했다).
- 실행 환경이 다르면(창이 뜬 Gazebo가 실시간보다 느릴 때 등) 숫자가 달라질 수 있다. 통과 조건은 숫자가 아니라 관계로 적었다.
