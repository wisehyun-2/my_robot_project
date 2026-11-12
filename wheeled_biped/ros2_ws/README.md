# ros2_ws — v5 로봇의 ROS 2 작업 공간 (Jazzy)

**2026-11-12 현재 `src/`에 패키지가 없다.** 이 문서는 그날 정한 것, 패키지 구조 안, 작업 순서를 적은 것이다. 실물은 v5 로봇(Raspberry Pi 4B + pi3hat + moteus r4.11, AKE60-8 × 6)이고, ROS 2 Jazzy 위에서 TITA의 `ros2_control` 패키지 구조를 따른다(2026-11-08 결정).

옛 패키지(`upkie_ros2`, `diablo_lite_ros2`, 메시지 3종)는 Plan 1~3용으로 작성만 하고 빌드하지 못한 채 2026-11-11 삭제했다(git 이력에 있음).

## 0. 두 컴퓨터로 작업하기

- 작업 브랜치는 **`ros2-v5`** 하나다. Windows PC와 Ubuntu 노트북이 같이 쓴다.
- 역할은 나누지 않는다. 두 컴퓨터를 동시에 쓰지 않는다(2026-11-12 사용자).
- 시작할 때 `git pull`, 끝낼 때 커밋하고 `git push`. 올리지 않고 다른 컴퓨터로 넘어가면 충돌이 난다.
- Claude Code의 메모는 컴퓨터마다 따로다. 다른 컴퓨터가 알아야 할 결정은 이 문서나 `CLAUDE.md`에 적는다.

## 1. 노트북(Ubuntu 24.04 + ROS 2 Jazzy) 준비

(a) Python 코어. ROS의 Python 패키지가 보이도록 `--system-site-packages`로 만든다.

```bash
python3 -m venv --system-site-packages ~/wbr_venv
source ~/wbr_venv/bin/activate
pip install -e wheeled_biped        # 저장소 최상위에서
cd wheeled_biped
python -c "from v5_lqi.controller import make_plan; p = make_plan(); p.reset(); print(p.run(2.0))"   # True가 나오면 정상
python -m unittest discover -s tests
```

(b) ROS 패키지.

```bash
sudo apt update
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-mujoco-ros2-control ros-jazzy-mujoco-ros2-control-demos \
  ros-jazzy-generate-parameter-library ros-jazzy-xacro python3-colcon-common-extensions
ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py     # MuJoCo 창이 뜨면 설치 확인 끝
```

(c) 빌드. `colcon`은 venv를 켜지 않은 터미널에서 돌린다(venv의 Python이 섞이지 않게).

```bash
source /opt/ros/jazzy/setup.bash
cd wheeled_biped/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

(d) 참조 저장소. 저장소 밖 `../references/`(`my_robot_project`와 같은 폴더 아래)에 둔다. 아래 커밋에서 확인한 내용이다.

| 저장소 | 커밋 | 쓰는 곳 |
|---|---|---|
| `https://github.com/mjbots/pi3hat` | 12ab9af | 하드웨어 플러그인(`lib/cpp/mjbots/pi3hat`) |
| `https://github.com/mjbots/moteus` | aec0333 | moteus 프로토콜(`lib/cpp/mjbots/moteus`) |
| `https://github.com/upkie/upkie` | ffa9e4d | pi3hat IMU 변환, 스파인 주기 참고 |
| `https://github.com/DDTRobot/ddt_ros2_control` | beb05df | 제어기 골격(`rl_controller`), 키보드, 토픽 이름 |
| `https://github.com/DDTRobot/TITA_ROS2_Control_Sim` | 3c782e4 | 시뮬 bringup 참고, `template_ros2_controller` |

아래 다섯 개는 2026-11-12에 더한 것이다. 저장소 정보와 README만 확인했고 코드는 아직 읽지 않았다. 모두 Humble 기준이거나(위 DDT 포함) Jazzy 공식 예제다.

| 저장소 | 받을 폴더 이름 | 라이선스 | 쓰는 곳 |
|---|---|---|---|
| `https://github.com/DDTRobot/tita_hardware_ros2_control` | `tita_hardware_ros2_control` | 파일 없음 | 실기 launch·`controllers.yaml` 틀(`hw_bringup`, 저장소 README로 확인). `read()`·`write()`에서 통신을 부르는 자리(옵시디언 `ROS2 패키지 선별` 메모의 평가, 코드는 아직 안 읽음). Humble |
| `https://github.com/BartlomiejK2/pi3hat_hardware_interface` | `pi3hat_hw_bartlomiej` | **GPL-3.0** | 우리가 만들 플러그인과 가장 가깝다: moteus 전용, 관절별 CAN 버스·ID, `mode`·`fault`·전압·온도 상태, IMU z축 변환, 시험 코드. Ubuntu 22.04 실시간 커널 + Humble |
| `https://github.com/CentroEPiaggio/pi3hat_robotic_systems` | `pi3hat_robotic_systems` | 파일 없음 | moteus + power dist(전압·전류)를 읽는 예. 우리 JC3의 power dist r4.5b |
| `https://github.com/G-Levine/pi3hat_hardware_interface` | `pi3hat_hw_glevine` | MIT | root·DDS 문제 해결 메모. Mini Cheetah 프로토콜 전용, 2024-02 이후 갱신 없음 |
| `https://github.com/ros-controls/ros2_control_demos` (브랜치 `jazzy`) | `ros2_control_demos` | Apache-2.0 | Jazzy 방식의 하드웨어·제어기 공식 예제. 예제 9가 시뮬·실물 전환(`hw_env`와 같은 구조) |

```bash
cd ~/references        # my_robot_project 옆의 references 폴더
git clone https://github.com/DDTRobot/tita_hardware_ros2_control
git clone https://github.com/BartlomiejK2/pi3hat_hardware_interface pi3hat_hw_bartlomiej
git clone https://github.com/CentroEPiaggio/pi3hat_robotic_systems
git clone https://github.com/G-Levine/pi3hat_hardware_interface pi3hat_hw_glevine
git clone -b jazzy https://github.com/ros-controls/ros2_control_demos
```

참조 코드를 쓸 때의 규칙:

- **GPL-3.0과 라이선스 파일이 없는 저장소는 읽고 구조만 참고한다. 코드를 복사하지 않는다.** GPL 코드를 복사하면 우리 패키지도 GPL이 되고, 라이선스가 없는 코드는 사용이 허락되지 않은 상태로 본다. 복사해도 되는 것은 MIT(G-Levine)와 Apache-2.0(ros-controls, 헤더에 Apache-2.0이 적힌 DDT 소스)이고, 그때는 출처 표기를 남긴다.
- DDT 코드와 pi3hat 인터페이스 세 개는 모두 Humble API다. Jazzy에서 달라진 것은 공식 이전 안내(control.ros.org의 "Humble to Jazzy")와 `ros2_control_demos`의 `jazzy` 브랜치를 기준으로 맞춘다. 안내에서 확인한 것: 제어기 파라미터는 스포너의 `--param-file`로 넘겨야 한다(yaml을 controller_manager에만 넘기면 안 됨), `get_state()` → `get_lifecycle_state()`, URDF는 `robot_description` 토픽으로 받는다, 값이 있는지 확인하는 `get_optional()`이 새로 생겼다. 하드웨어 인터페이스를 내보내는 방식은 `jazzy` 예제의 것을 따른다.
- pi3hat 인터페이스 세 개는 Ubuntu 22.04 + Humble에서 쓰던 것이다. 24.04 + Jazzy에서 도는지는 확인된 곳이 없다.
- 쓰지 않는 것: DDT의 MuJoCo 브리지 3종(공식 `mujoco_ros2_control`로 대신함), `TITA-SDK-ROS2`·`TITA_CAN_Inerface`(TITA 보드의 CAN 규약), 강화학습 저장소(지금 범위 밖).

## 2. 정해진 것

| 항목 | 결정 | 언제 |
|---|---|---|
| ROS 배포판 | Jazzy (Ubuntu 24.04) | 11-08 사용자 |
| 패키지 구조 | TITA `ros2_control` 구조, 균형 제어기 자리만 `wbr` 이식 | 11-08 사용자 |
| 시뮬레이터 | MuJoCo, Gazebo, **Isaac Sim** 셋 다. 공식 `mujoco_ros2_control` 먼저, Gazebo와 Isaac Sim은 그 뒤 | MuJoCo·Gazebo 11-08 사용자, Isaac Sim 11-12 사용자, 순서는 11-12 제안 |
| 제어기 출력 | 토크(effort)만. 모든 환경(실물·MuJoCo·Gazebo·Isaac Sim)에서 같게 | 11-08 |
| CAN 버스 | 왼쪽 다리 JC1, 오른쪽 다리 JC2, JC3은 power dist. ID는 후보 1·2·3 / 4·5·6 | 11-12 사용자 |
| 범위 | **주행 중 균형 잡기만.** 시작 자세·스스로 일어서기는 나중에 정함 | 11-12 사용자 |
| 상태 기계 | `idle → balance → fault`. 받침 위에서 세운 뒤 `arm()` (Python `DiabloMcu`와 같음) | 11-12 |
| 제어 주기 | **목표 1 kHz**, 주기는 파라미터. Pi에서 안 나오면 500 / 200 Hz로 내림 | 11-12, Pi 실측 뒤 확정 |
| 균형 제어기 | LQI8(`v5_lqi`). MPC(`v5_mpc`)는 나중에 같은 자리에 끼움 | 11-12 |
| 균형 제어기 구성 | **전체 제어기만 쓴다**(LQI8 상태 8개: ∫위치·∫기울기 포함, 바깥 루프 네 개 모두 켬). 순수 LQR과 적분을 하나씩 켜는 단계는 쓰지 않는다. Python 시험도 `--stage 4`만 돌린다. C++로 옮기는 것도 이 구성 하나다 | 11-07 사용자 |

제어 주기 근거: Python 단계 시험과 게인이 모두 1 kHz 기준이다. upkie도 같은 구성(Pi 4 + pi3hat + moteus 6개)에서 스파인 1000 Hz, IMU 1000 Hz가 기본값이다(`spines/pi3hat_spine.cpp` 147·177행). 다만 upkie의 균형 제어는 200 Hz 에이전트에서 돌고, 스파인은 CAN 한 주기가 끝나기를 기다린다. 우리는 그 위에 `ros2_control`과 IMU 상세 읽기가 얹히므로 실측이 필요하다. 구조도에 있던 500 Hz는 TITA / DDT의 값이다.

## 3. 패키지 구조 (안)

사용자가 준 구조도에 11-12 점검 결과를 반영한 안이다. `[수정]`, `[추가]`는 그 점검의 제안이고 아직 확정이 아니다.

```
ros2_ws/src/
├─ wbr_description/                      로봇 모델 + hw_env 전환
│  ├─ urdf/wheelbiped_v5.urdf.xacro       models/wheelbiped_v5의 URDF에서 (build_v5가 생성, 손으로 안 고침)
│  ├─ urdf/ros2_control.xacro             hw_env = real | mujoco | gazebo 로 플러그인 선택(Isaac Sim은 아래 isaac/ 참고), 관절별 CAN 버스·ID, IMU 센서
│  ├─ urdf/gazebo.xacro, worlds/flat.sdf  MuJoCo 다음 순서
│  ├─ isaac/                              [추가] Isaac Sim용 USD와 IMU 센서를 더하는 덧붙임 URDF (7절: Isaac Sim은 xacro의 플러그인 선택을 쓰지 않는다)
│  ├─ mujoco/wheelbiped_v5.xml, scene.xml  [수정] meshdir 경로 (기립 키프레임은 나중)
│  ├─ meshes/*.stl, rviz/v5.rviz
├─ wbr_hardware/                         실물 플러그인 (read / write)
│  ├─ src/pi3hat_system.cpp               [수정] 출발점: mjbots pi3hat_moteus_transport.h + moteus.h
│  ├─ (third_party 복사 대신)              [수정] CMake FetchContent, 커밋 고정
│  └─ config/moteus/*.cfg                 [추가] 드라이버 6개 설정과 덤프
├─ wbr_controller/                       update(): 추정 → 균형 → 바깥 루프 → 관절 토크
│  ├─ core/                               [추가] ROS 없는 C++ 라이브러리 (Eigen만)
│  │    estimator      ← diablo_arch/mcu.py(추정부) + observers/body_attitude.py + upkie_observers.py
│  │    serial_leg     ← controllers/lqi8/serial_leg.py
│  │    lqi            ← controllers/lqi8/lqi8.py, law.py (전체 제어기, 상태 8개)
│  │    outer_loops    ← mcu.py의 다리 길이·요·롤·벌어짐·VMC·마찰 보상·감속·다리 범위 벽
│  │    state_machine  idle → balance → fault, arm 거부 조건
│  │    gain_table     내보낸 표 읽기 + 다리 길이 보간
│  │    balance_law.hpp  [추가] LQI8 / 나중의 MPC를 끼우는 자리 (Python `_make_law`)
│  ├─ src/wbr_controller.cpp              얇은 래퍼: 인터페이스, 토픽, 500 ms 명령 타임아웃, 상태 발행
│  ├─ src/wbr_controller_parameters.yaml  generate_parameter_library
│  └─ test/test_equivalence.cpp           core만 링크, Python 기준 기록과 토크 비교
├─ wbr_msgs/msg/BalanceStatus.msg        state, fault 사유, v, v_ref, pitch, roll, L0, 전압 …
├─ wbr_teleop/                           키보드 (DDT keyboard_controller 수정), 우선순위 낮음
└─ wbr_bringup/
   ├─ launch/robot.launch.py              hw_env 인자 하나로 실물·MuJoCo·Gazebo·Isaac Sim
   ├─ launch/record.launch.py             rosbag2
   ├─ launch/hardware_check.launch.py     [추가] 제어기 없이 토크 0 + 상태 확인 (0단계 시험)
   ├─ config/controllers.yaml             주기, 제어기, 브로드캐스터, thread_priority·cpu_affinity
   ├─ config/wbr_params_<주기>hz_<이름>.yaml  [수정] export_params.py가 만드는 파일 (게인표 포함). 이름 = 게인 세트(기본, 저게인)
   ├─ config/fastdds_udp.xml              [추가] root로 돌 때의 DDS 설정
   └─ scripts/                            [추가] Pi 설정 확인, 실행 스크립트

wheeled_biped/docs/pi_setup.md           [추가] Pi 4 준비: Ubuntu 24.04, 실시간 커널, CPU 격리, 빌드, root 실행

wheeled_biped/wbr/tools/                 ROS 패키지 아님, 아직 없음
├─ export_params.py                       PlanV5를 만든 뒤 최종 McuParams·LQI8 설정·표를 통째로 내보냄
└─ dump_golden.py                         Python 제어기의 입력·출력 기록 (C++ 일치 시험용)
```

C++로 옮기지 않는 것: `wbr/observers/pi3hat_ars.py`(pi3hat 펌웨어 필터 흉내, 시뮬 전용), 시뮬 하네스(`plan4_lqi8.py`, `plan_v5.py`의 모델·노이즈·지연 부분), `host_link.py`의 DIABLO 직렬 링크 흉내(타임아웃 규칙만 래퍼에 옮김).

## 4. 구조도에 없지만 실물에 필요한 것

1. **전원 직후 영점.** moteus는 전원을 켜면 관절각을 ±22.5° 안으로 접어 읽는다. 플러그인이 활성화될 때 hip·무릎에 `MakeOutputNearest`를 보내고 `home_state`(레지스터 0x00c)가 2인지 읽는다. 없으면 `arm()`이 항상 거부한다(`docs/bringup_step0.md` 7절).
2. **모터 상태 인터페이스.** `mode`, `fault`, 전압, 온도, `home_state`. moteus가 자체 시간 초과에 들어가면 `stop`으로 푼다.
3. **IMU 상세값과 표본 시각.** 제어기는 pi3hat 자세를 시작값으로만 쓰고 원시 자이로·가속도·바이어스로 자세를 구한다. 표준 IMU 값 10개 외에 바이어스 3개와 표본 시각이 필요하다(`request_attitude_detail`). 시뮬레이터에는 없으므로 제어기가 선택 사항으로 받는다.
4. **IMU 좌표 규약.** pi3hat은 z가 아래인 좌표계(ARS)로 자세를 주고 MuJoCo·Gazebo는 z가 위다. 플러그인에서 ROS 규약(z가 위, IMU 축)으로 바꿔 내보낸다. 몸통 좌표로 바꾸는 일(`R_base_to_imu` = diag(−1, −1, 1), URDF `imu_fix`)은 제어기가 한다(5절).
5. **root와 DDS.** pi3hat 라이브러리는 `/dev/mem`을 직접 열어 root가 필요하다(`pi3hat.cc:361`). root 노드와 일반 사용자 노드는 기본 공유 메모리 전송으로 통신이 안 될 수 있다.
6. **높이·롤·피치 명령.** DDT는 `command/cmd_twist`, `command/cmd_pose`, `command/cmd_key` 세 토픽을 쓴다. 우리 제어기도 높이(`up`), 롤, 피치 명령을 받는다.
7. **받침 위 점검 실행.** 제어기 없이 부호·영점·주기를 확인하는 launch.
8. **배터리 전압.** moteus 전압 레지스터나 power dist(CAN ID 32, JC3)에서 읽어 상태에 싣는다.
9. **pi3hat에 넣을 값 세 개.** 설정 `attitude_rate_hz = 1000`(라이브러리 기본값은 400, `pi3hat.h:178`)과 `mounting_deg = 0, 0, 0`(축 변환은 우리 쪽에서 한다), 그리고 매 주기 입력의 `request_attitude_detail = true`(바이어스를 받는다, `pi3hat.h:242`). 기본값 그대로 두면 IMU가 시뮬과 다른 주기로 돌고 바이어스가 오지 않는다.
10. **꺼질 때와 죽었을 때 토크 끊기.** 플러그인의 비활성화·종료·오류 경로에서 모든 관절에 `stop`을 보낸다. launch를 Ctrl+C로 끌 때도 이 경로를 탄다. 프로그램이 죽어 명령이 끊기는 경우는 moteus가 맡는다: 설정 `servo.default_timeout_s`와 명령마다 넣는 `watchdog_timeout`.
11. **응답이 빠진 주기의 규약.** 관절 응답이 빠지면 그 관절 값을 NaN으로 두거나 표본 시각을 갱신하지 않는다. 그러면 제어기의 가드가 잡는다(Python: NaN 5주기 초과 또는 오래된 표본 50 ms 초과면 FAULT, `max_bad_samples`, `stale_fault_s`). 플러그인이 지난 값을 새 값처럼 다시 내보내면 이 가드가 무력해진다.
12. **pi3hat은 한 번에 한 프로그램만 쓴다**(잠금 파일 `/tmp/.pi3hat-lock`, `pi3hat.cc:1152`). `ros2_control_node`가 떠 있는 동안에는 `tview`, `pi3hat_tool`, `moteus_tool`을 쓸 수 없다. 드라이버 설정과 점검은 ROS를 끄고 한다.
13. **연결하지 않은 CAN 포트로 프레임을 보내지 않는다.** 플러그인은 설정에 있는 버스로만 보낸다. 옵시디언 메모에 따르면 2025-09-20 이전 pi3hat 펌웨어는 빈 포트로 보내면 그 프로세서가 멈추고 전원을 껐다 켜야 풀린다(mjbots 블로그 출처, 저장소의 pi3hat 소스에서는 확인하지 못함). `pi3hat_tool --info`로 펌웨어를 확인한다.
14. **전 주기 기록.** 제어에 쓰인 모든 신호(관절, IMU, 추정값, 기준, 토크, 상태)를 제어 주기 그대로 기록한다. 부호 오류, 시뮬과의 차이, 포화·지연·잡음을 여기서 본다. 기록은 토크를 보낸 뒤에 하거나 실시간이 아닌 스레드로 넘긴다(`realtime_tools`). rosbag만으로 1 kHz가 버거우면 메모리 버퍼에 모았다가 파일로 쓴다.
15. **FAULT와 안전 동작(Python과 같게).** 몸통 피치·롤 34° 초과, NaN 표본, 오래된 표본, 유한하지 않은 토크 명령이면 FAULT이고 모든 관절 토크 0. 명령이 0.5 s 끊기면 속도 0으로 서 있는다(균형은 계속). `arm()`은 영점 안 맞춘 다리, 피치 3° 이상, 다리 길이·각도 범위 밖을 거부한다.

## 5. 옮길 때 조심할 것

- **파라미터 출처가 네 곳이다.** `models/wheelbiped_v5/description.json`(`plan_overrides`), `PlanV5Config`(서스펜션·감속), `Plan4Config`(속도 창·마찰 보상·요 적분 한계), LQI8 가중치(`plan4_lqi8.py`의 `lqi_kwargs` + `PlanV5._lqi_design`). 손으로 옮기지 말고 `PlanV5`를 만든 뒤의 `mcu.p`와 `law.ctrl`에서 내보낸다.
- **표로 내보낼 것.** LQI8 게인 K(다리 길이 격자), `gM(L0)`, `theta_eq(L0)`, 정적 다리 힘의 기울기(`SerialPlant.static_leg_force`), `Tp_ff`. 게인은 주기(`Ts`)마다 다르다.
- **스텝 수는 정수로 내보낸다.** 속도 창 5 ms는 500 Hz에서 2.5스텝인데 Python `round`는 2, C++ `std::round`는 3이다. `_freeze_hold_steps`도 같다.
- **`ros2_control` 한 주기 지연.** 순서가 읽기 → 제어 → 쓰기인데 pi3hat은 보내기와 받기가 한 번의 `Cycle`이다. 상태가 한 주기 늦는다. 시뮬레이션 한계는 12 ms(1 kHz, 속도 창 5 ms), 시험은 10 ms로 돌린다.
- **위치 적분은 고정 `dt`, 자세 필터는 표본 시각 간격.** Python과 같게 둔다(`docs/bringup_step0.md` 9절).
- **`core`는 Python `obs`와 같은 입력을 받는다(제안).** 관절 위치·속도, IMU 자세(ARS 좌표의 쿼터니언), IMU 축의 각속도·가속도·바이어스, 표본 시각. 그러면 기준 기록을 그대로 넣어 Python과 줄 단위로 비교할 수 있다. ROS 규약(z가 위)에서 이 규약으로 바꾸는 일은 래퍼가 한다. 부호만 바뀌는 변환이라 오차가 없다.
- **IMU는 IMU 축 그대로 내보낸다.** 11-08 메모(`전체 로봇 구조도`)에는 플러그인이 몸통 좌표로 바꾼다고 적혀 있다. 시뮬레이터가 IMU 축으로 내보내므로(MJCF의 `imu` 사이트가 보드 축이다) 실물도 IMU 축으로 맞추고, 몸통 좌표로 바꾸는 일(`R_base_to_imu`)은 제어기가 한다.
- **피치·롤 속도, 요 속도, 가속도의 경로가 서로 다르다.** 피치·롤 속도는 보드의 `rate`(보드가 바이어스를 보정한 자이로), 요 속도는 원시 z 자이로에서 제어기가 배운 오프셋을 뺀 값, 가속도는 원시 값으로 되돌린 것이다(`accel_mps2 + 9.81 × IMU에서 본 위 방향`, hat이 위인 정지 상태에서 (0, 0, +1). 아래 방향으로 넣으면 부호가 틀린다. 식은 `docs/bringup_step0.md` 8절). 보드 자세각은 첫 표본의 시작값으로만 쓴다(`docs/robot_v5_model.md` 5절).
- **기준 기록과 파라미터 파일은 같은 실행에서 만들어 같이 커밋한다.** 따로 만들면 Python·numpy·scipy 버전 차이로 게인이 조금 달라져 일치 시험이 깨질 수 있다. 파일 머리에 만든 날짜, 커밋, 패키지 버전을 적는다.
- **실물 질량을 잰 뒤에도 yaml을 손으로 고치지 않는다.** `robot_cad_v5/calc/mass_properties.json`(Fusion의 `save_mass_json()`이 쓰는 파일)을 잰 값으로 맞춘 뒤 `python -m wbr.models.build_v5` → 내보내기 순서로 다시 만든다. 다리 힘 피드포워드(`F_ff`)가 틀리면 다리 길이가 어긋난다: 몸통 질량 오차 1 kg에 약 3.3 mm(다리 하나에 0.5 × 9.81 N, `kp_leg` 1503 N/m로 나눈 값).
- **저게인 세트가 필요하다.** 실물의 첫 자립은 토크 가중치 R을 키운 낮은 게인으로 하고 단계적으로 올린다(사용자 절차 5-3, 5-4). 내보내기 도구가 이름 붙인 게인 세트를 여러 개 만들 수 있어야 한다.
- **MJCF는 지금 그대로 공식 `mujoco_ros2_control`에 맞는다**(문서 기준, 실행은 안 해 봄): `general` 구동기는 effort를 그대로 받고, 센서 이름 `imu_quat` / `imu_gyro` / `imu_accel`은 `mujoco_sensor_name = imu`로 잡힌다. 이 패키지는 자기 `ros2_control_node`를 써야 한다.
- **MJCF의 `stand` 키프레임은 다리를 편 자세(500 mm)다.** `arm()` 허용 범위(326~485 mm) 밖이다. 시작 자세는 나중에 정하기로 했다.
- **ROS 시뮬은 연결 확인용이다.** 노이즈·지연·pi3hat 필터 흉내는 Python 하네스에만 있다. 제어기의 합격 판정은 Python 단계 시험(`v5_lqi/TEST`)과 C++ 일치 시험으로 한다.

## 6. 작업 순서 (2026-11-12 최종안)

단계마다 통과 조건이 있고, 통과하기 전에는 다음으로 가지 않는다. 사용자의 단계별 절차(옵시디언 `DIABLO 방식을 로봇에 적용하는 절차`)와 `docs/bringup_step0.md`를 ROS 작업 순서로 옮긴 것이다. 균형 로봇의 실수는 대부분 부호와 영점에서 나오므로, 부호 확인을 시뮬(A3)과 실물(C5)에서 두 번 한다.

### A. 노트북 (실물 없이)

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| A0 | 기준 확인: `python v5_lqi/TEST/run_stages.py --stage 4 --keep-going` (시드 0 하나, 약 3분) | 넘어짐과 무릎 꺾임(`knee_flipped`)이 없다. 보고서의 단계 판정은 "실패"로 나오는 것이 지금 상태다: 저장소 보고서(시드 5개)에서 시험 27개 중 7개가 일부 기준을 넘고, 시드 0에서는 2-3, 2-9, 2-10, 2-12, 2-13 다섯 개다(다리 토크 rms, 턱에서의 방향·롤). 노트북에서도 넘는 시험이 이와 같거나 비슷하면 된다. 경계에 있는 값은 MuJoCo 버전 차이로 달라질 수 있다. 실행하면 보고서 파일이 덮어써지므로 커밋하지 않는다 |
| A1 | `wbr/tools/export_params.py`, `dump_golden.py` | 1 kHz의 파라미터·표 파일과 기준 기록이 나온다. 내보낸 파일만 읽어 다시 계산한 토크가 원래 Python 제어기와 같다. 전체 제어기(상태 8개) 하나를 내보낸다 |
| A2 | `wbr_description` (xacro, `ros2_control` 태그, MuJoCo) | `colcon build` 통과. MuJoCo 창에 모델이 뜨고 `joint_states`와 IMU 토픽이 나온다 |
| A3 | ROS 시뮬에서 부호 시험: 토크를 그대로 넘기는 표준 제어기(`forward_command_controller/ForwardCommandController`, 인터페이스 effort)로 관절마다 + 토크 | 움직이는 방향이 `docs/bringup_step0.md` 4절 표와 같다. 모델을 기울였을 때 IMU 부호가 8절 표의 규약 값과 같다 |
| A4 | `wbr_controller/core` + `test_equivalence`. 옮기는 순서: serial_leg → gain_table·lqi → estimator → outer_loops → state_machine | 기준 기록의 모든 스텝에서 관절 토크 6개가 Python과 일치한다(허용 오차 후보 1e-6 N·m) |
| A5 | `wbr_controller` 래퍼 + `wbr_bringup` | MuJoCo에서 1 kHz로 60 s 서 있고, 밀어도 돌아오고, 0.5 m/s 속도 명령을 따라간다. 처음부터 전체 제어기로 한다 |
| A6 | 안전 동작 시험 | 명령이 0.5 s 끊기면 속도 0으로 서 있다. 피치 34° 초과, NaN, 오래된 표본이면 토크 0. 제어기를 끄거나 launch를 끝내면 토크 0 |
| A7 | `wbr_msgs`, `wbr_teleop`, 기록(rosbag과 전 주기 기록) | 키보드로 전진·회전·높이를 바꾼다. 기록에 제어 주기 신호가 빠짐없이 있다 |
| A8 | 노트북에서 루프 측정 | `update()` 계산 시간과 주기 지터를 적어 둔다(Pi와 비교할 기준) |

- A2·A3과 A4는 서로 기다리지 않는다.
- A5의 시작 자세는 아직 정하지 않았다. 시험할 때만 Python `reset()`과 같은 기립 자세(다리 460 mm)에서 일시정지로 시작하고, 제어기가 켜진 뒤 푼다.
- 래퍼의 골격은 `ros2_control_demos`(`jazzy`)의 예제를 따르고, 명령 토픽과 상태 기계의 틀은 `ddt_ros2_control`의 `rl_controller`와 `TITA_ROS2_Control_Sim`의 `template_ros2_controller`를 참고한다. 두 DDT 제어기에는 안전 검사와 명령 타임아웃이 없으므로 4절 15번을 반드시 넣는다.

### B. Pi 4 (pi3hat 없이)

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| B1 | Ubuntu 24.04 Server(arm64), 실시간 커널(아래 둘 중 하나), `isolcpus`, ROS 2 Jazzy | `uname -a`에 `PREEMPT_RT`가 보이고 `cat /sys/kernel/realtime`이 1이다. 전원 부족 경고가 없다 |
| B2 | `cyclictest`로 1 kHz 타이머 지터 | 최대 지터를 적는다 |
| B3 | 작업 공간을 Pi에서 빌드. `mock_components/GenericSystem` + `wbr_controller`를 1 kHz로 | 지터 2 ms 이하, 평균 주기와 설정 주기의 차이 5 % 이내(`docs/bringup_step0.md` 9절의 후보 기준). `update()` 시간을 A8과 비교 |

여기까지는 "Pi의 CPU와 ROS가 1 kHz를 버티는가"만 확인된다. 모터 통신을 포함한 주기는 C4·C6에서 잰다.

실시간 커널을 얻는 방법은 두 가지다(2026-11-12 확인, 둘 다 Pi에서 해 보지는 않았다).

| 방법 | 하는 일 | 비고 |
|---|---|---|
| Ubuntu Pro의 실시간 커널 | 보통의 Ubuntu 24.04에 `sudo pro attach <토큰>` 뒤 `sudo pro enable realtime-kernel --variant=raspi` | Ubuntu Pro는 다른 운영체제가 아니라 구독이다. 개인은 5대까지 무료이고 Ubuntu One 계정과 토큰이 필요하다. Pi 4·5를 지원하고 apt로 갱신된다 |
| `ros-realtime/linux-real-time-kernel-builder` | Ubuntu `raspi` 커널(6.8)에 PREEMPT_RT 패치를 입힌 deb를 받거나 직접 빌드해 `dpkg -i`로 설치 | Ubuntu Pro가 필요 없다. 직접 빌드는 x86 Ubuntu 24.04의 Docker에서 하고 디스크 16 GB가 든다. 커널 갱신은 스스로 다시 해야 한다 |

### C. pi3hat과 moteus가 온 뒤

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| C1 | pi3hat 단독 점검. 전원을 끈 채 장착하고 켠다 | `/sys/firmware/devicetree/base/hat/product`가 `mjbots quad pi3 hat`으로 시작한다(라이브러리가 이 값을 검사하고, 아니면 `No pi3hat detected`로 멈춘다. `pi3hat.cc:964`). `sudo pi3hat_tool --info`에 프로세서 세 줄이 나온다. `--read-att`로 IMU 부호가 8절 표와 같다(바로 놓으면 롤 ±180°가 정상) |
| C2 | moteus 1대를 JC1에 연결 | 종단 저항을 꽂고 전원을 끈 상태에서 CANH–CANL 약 60 Ω. `tview`에서 값이 갱신된다 |
| C3 | moteus 6대 설정. 순서를 바꾸지 않는다(`docs/bringup_step0.md` 3절에 명령 타임아웃 설정을 더한 것): 펌웨어 → ID → 보정(링크 달기 전) → 감속비 0.125 → 조립 → 부호 → 위치 한계 → 영점 → 명령 타임아웃(`servo.default_timeout_s`) → 덤프 보관 | 4절의 기록표(관절별 부호)가 채워지고, 위치 한계와 영점 값을 적어 두고, 덤프가 `wbr_hardware/config/moteus/`에 있다 |
| C4 | 통신 주기 측정(9절): JC1·JC2, IMU 포함 | 평균·99 %·최대 주기를 적는다 |
| C5 | `wbr_hardware` + `hardware_check.launch.py`(토크 0) | 관절·IMU·전압·`home_state`가 ROS로 올라온다. ROS를 거쳐 A3의 부호 시험을 실물로 반복한다 |
| C6 | Pi에서 전체 루프의 주기와 센서 → 토크 지연 측정 | 지터 2 ms 이하, 지연 8 ms 이하(후보). 1 kHz가 안 나오면 500 / 200 Hz 파일로 바꾸고 그 주기로 A1·A4를 다시 한다 |
| C7 | 받침 위에서 순서대로: 전원 직후 영점 → `arm()` 조건 → 매달기(몸통을 앞으로 기울이면 바퀴가 앞으로 돈다) → 저게인 자립(테더) → 게인 올리기 → 주행. 모두 전체 제어기로 한다 | 1분 이상 자립, 가벼운 밀기에서 복귀, 지속 진동 없음 |

`wbr_hardware`의 코드는 C1 전에도 쓸 수 있다. 노트북에서 컴파일까지 되는지는 해 봐야 안다. 실행은 Pi에서만 된다.

### D. 그 뒤

Gazebo → Isaac Sim → 시작 자세와 스스로 일어서기 → MPC(`v5_mpc`) → 상위 기능(SLAM, 로봇팔).

## 7. 확인하지 못한 것, 정할 것

- Ubuntu 24.04의 Pi 4B에서 pi3hat 라이브러리가 도는지, 모터 6개 + IMU 상세 읽기로 1 kHz가 나오는지. pi3hat과 moteus가 있어야 잴 수 있다.
- JC1과 JC2는 pi3hat의 같은 프로세서가 맡는다. 주기에 영향이 있으면 오른쪽만 JC3으로 옮겨 비교한다.
- CAN ID(후보 1·2·3 / 4·5·6), moteus 위치 한계 후보, 무릎 완전 신장 보호.
- 시작 자세와 스스로 일어서기(나중).
- Isaac Sim(11-12 사용자가 추가). Isaac Sim 6.1 문서에는 공식 확장 `isaacsim.ros2.control`이 있다: Controller Manager를 Isaac Sim 안에서 띄우고, USD의 관절 구동 설정에서 인터페이스를 만든다(강성·감쇠가 없는 구동이면 effort). IMU는 `IsaacImuSensor`와 덧붙임 URDF로 표준 값 10개를 낸다. `update_rate`는 물리 스텝보다 빠를 수 없으므로 1 kHz면 물리도 1 kHz 이상이어야 한다. 그래서 Isaac Sim은 xacro의 `<hardware>` 플러그인 선택을 쓰지 않고, USD + 덧붙임 URDF + 같은 `controllers.yaml`로 붙는다. 확인하지 못한 것: 우리가 빌드한 `wbr_controller` 플러그인이 Isaac Sim 안의 Controller Manager에 실리는지(문서는 UR10과 표준 제어기로만 확인), 타이어 접촉과 IMU 축을 MuJoCo 모델과 맞추는 방법, 어느 컴퓨터에서 돌릴지(RTX 그래픽 카드 필요).
- 사용자 절차와 Python 제어기가 다른 곳(정해야 함): 토크를 끊는 각도가 절차에는 |θ| > 25°(가상 다리 각), 코드에는 몸통 피치·롤 34°(`fall_pitch`, `fall_roll`)다. 절차에 있는 "전원 투입 직후 토크 램프업"과 "하드웨어 비상정지"는 코드에 없다. 넣는다면 Python에 먼저 넣고 단계 시험을 다시 돌린 뒤 C++로 옮긴다.
- Pi의 전원. pi3hat은 Pi에 5 V 2.5 A까지 준다고 제품 설명에 적혀 있다(Pi 4의 권장은 3 A). 팬과 USB 장치까지 물리면 전압이 떨어져 CPU가 느려질 수 있으므로 B1에서 전원 부족 경고를 확인한다.
- 옵시디언 `Study/Calibration 단계/0단계.md`는 11-08 새벽의 사본이라 옛 내용이 남아 있다(JC1·JC3, CAD 이름 반대, `arm()`이 접힌 관절을 막지 못함). 기준은 저장소의 `docs/bringup_step0.md`다.
- 3절의 `[수정]`·`[추가]` 항목을 그대로 갈지.
