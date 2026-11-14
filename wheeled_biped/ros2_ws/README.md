# ros2_ws — v5 로봇의 ROS 2 작업 공간 (Jazzy)

**2026-11-13 현재 `src/`에 패키지가 없다.** 이 문서는 정한 것, 패키지 구조 안, 옮길 때 지킬 것, 작업 순서를 적은 것이다. 실물은 v5 로봇(Raspberry Pi 4B + pi3hat + moteus r4.11, AKE60-8 × 6)이고, ROS 2 Jazzy 위에서 TITA의 `ros2_control` 패키지 구조를 따른다(2026-11-08 결정).

옛 패키지(`upkie_ros2`, `diablo_lite_ros2`, 메시지 3종)는 Plan 1~3용으로 작성만 하고 빌드하지 못한 채 2026-11-11 삭제했다(git 이력에 있음).

2026-11-13에 Opus 5.5 검토자 세 명이 이 문서를 우리 코드, 참조 소스, 공식 문서와 대조했고, 그 지적을 다시 원본에서 확인해 반영했다. 근거는 `파일:줄`로 적었다. 참조 저장소의 줄 번호는 1절 (d)의 커밋 기준이다. "제안"·"후보"라고 적은 것은 아직 정하지 않은 것이다.

## 0. 두 컴퓨터로 작업하기

- 작업 브랜치는 **`ros2-v5`** 하나다. Windows PC와 Ubuntu 노트북이 같이 쓴다.
- 역할은 나누지 않는다. 두 컴퓨터를 동시에 쓰지 않는다(2026-11-12 사용자).
- 시작할 때 `git pull`, 끝낼 때 커밋하고 `git push`. 올리지 않고 다른 컴퓨터로 넘어가면 충돌이 난다.
- Claude Code의 메모는 컴퓨터마다 따로다. 다른 컴퓨터가 알아야 할 결정은 이 문서나 `CLAUDE.md`에 적는다.
- Pi 4는 세 번째 컴퓨터다. 같은 브랜치를 받아 Pi에서 빌드한다(6절 B3).

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

(d) 참조 저장소. **2026-11-14부터 이 저장소의 `references/` 아래 git 서브모듈**로 둔다(사용자: 여러 컴퓨터가 같은 커밋을 보게). 서브모듈은 아래 커밋에 고정되어 있고, 받는 법과 전체 목록은 `references/README.md`에 있다(예전 위치 `../references/`는 쓰지 않는다). 아래 커밋에서 확인한 내용이다.

| 저장소 | 커밋 | 쓰는 곳 |
|---|---|---|
| `https://github.com/mjbots/pi3hat` | 12ab9af | 하드웨어 플러그인(`lib/cpp/mjbots/pi3hat`) |
| `https://github.com/mjbots/moteus` | aec0333 | moteus 프로토콜(`lib/cpp/mjbots/moteus`) |
| `https://github.com/upkie/upkie` | ffa9e4d | pi3hat IMU 변환, 스파인 구조 참고 |
| `https://github.com/DDTRobot/ddt_ros2_control` | beb05df | 제어기 골격(`rl_controller`), 키보드, 토픽 이름 |
| `https://github.com/DDTRobot/TITA_ROS2_Control_Sim` | 3c782e4 | 시뮬 bringup 참고, `template_ros2_controller` |

아래 다섯 개는 2026-11-12에 더한 것이다. 저장소 정보와 README만 확인했고 코드는 아직 읽지 않았다.

| 저장소 | 받을 폴더 이름 | 라이선스 | 쓰는 곳 |
|---|---|---|---|
| `https://github.com/DDTRobot/tita_hardware_ros2_control` | `tita_hardware_ros2_control` | 파일 없음 | 실기 launch·`controllers.yaml` 틀(`hw_bringup`, 저장소 README로 확인). `read()`·`write()`에서 통신을 부르는 자리(옵시디언 `ROS2 패키지 선별` 메모의 평가, 코드는 아직 안 읽음). Humble |
| `https://github.com/BartlomiejK2/pi3hat_hardware_interface` | `pi3hat_hw_bartlomiej` | **GPL-3.0** | 우리가 만들 플러그인과 가장 가깝다: moteus 전용, 관절별 CAN 버스·ID, `mode`·`fault`·전압·온도 상태, IMU z축 변환, 시험 코드. Ubuntu 22.04 실시간 커널 + Humble |
| `https://github.com/CentroEPiaggio/pi3hat_robotic_systems` | `pi3hat_robotic_systems` | 파일 없음 | moteus + power dist(전압·전류)를 읽는 예. 우리 JC3의 power dist r4.5b. Humble |
| `https://github.com/G-Levine/pi3hat_hardware_interface` | `pi3hat_hw_glevine` | MIT | root에서 FastDDS 공유 메모리가 안 되는 문제와 UDP 프로필 해결법. Mini Cheetah 프로토콜 전용. 코드는 2023-05 이후 그대로다(2024-02의 갱신은 LICENSE 추가) |
| `https://github.com/ros-controls/ros2_control_demos` (브랜치 `jazzy`) | `ros2_control_demos` | Apache-2.0 | Jazzy 방식의 하드웨어·제어기 공식 예제. 예제 9가 시뮬·실물 전환(`hw_env`와 같은 구조) |

이 다섯 개도 `references/` 아래 서브모듈이다(폴더 이름은 위 표). 받는 법:

```bash
cd my_robot_project    # 저장소 맨 위
git submodule update --init references/tita_hardware_ros2_control references/pi3hat_hw_bartlomiej references/pi3hat_robotic_systems references/pi3hat_hw_glevine references/ros2_control_demos
```

참조 코드를 쓸 때의 규칙:

- **GPL-3.0과 라이선스 파일이 없는 저장소는 읽고 구조만 참고한다. 코드를 복사하지 않는다.** GPL 코드를 복사하면 우리 패키지도 GPL이 되고, 라이선스가 없는 코드는 사용이 허락되지 않은 상태로 본다. 복사해도 되는 것은 MIT(G-Levine)와 Apache-2.0(ros-controls)이고, 그때는 출처 표기를 남긴다. DDT 저장소에는 최상위 LICENSE 파일이 없다. DDT 코드는 그 파일의 머리말과 그 패키지의 `package.xml`이 둘 다 Apache-2.0일 때만 복사한다.
- upkie의 `Pi3HatInterface.cpp`를 그대로 쓰지 않는다. IMU 상세 요청을 켜지 않고 바이어스를 읽는다(4.2절 3번).
- DDT 코드와 pi3hat 인터페이스 세 개는 모두 Humble API다. 공식 이전 안내(control.ros.org jazzy의 "Humble to Jazzy")에서 우리에게 걸리는 것:
  - 제어기 파라미터는 스포너의 `--param-file`로 넘긴다. yaml을 controller_manager에만 넘기는 방식은 안 된다.
  - `get_state()` → `get_lifecycle_state()`.
  - URDF는 `robot_description` 토픽으로 받는다. `<ros2_control>` 태그의 관절은 모두 URDF에 있어야 한다.
  - 인터페이스 핸들의 `get_value()`는 사용 중단이다. `get_optional()`이나 `bool get_value(double&)`를 쓰고, `set_value()`의 bool 반환을 확인한다.
  - 하드웨어의 인터페이스는 `<ros2_control>` 태그에서 자동으로 만들어진다(`on_export_state_interfaces()` 등). 태그에 없는 인터페이스는 `export_unlisted_state_interfaces()`로 낸다.
  - `controller_interface::init()`은 `init(ControllerInterfaceParams)`로 바뀌었다.
- pi3hat 인터페이스 세 개는 Ubuntu 22.04 + Humble에서 쓰던 것이다. 24.04 + Jazzy에서 도는지는 확인된 곳이 없다.
- 쓰지 않는 것: DDT의 MuJoCo 브리지 3종(공식 `mujoco_ros2_control`로 대신함), `TITA-SDK-ROS2`·`TITA_CAN_Inerface`(TITA 보드의 CAN 규약), 강화학습 저장소(지금 범위 밖).

## 2. 정해진 것

| 항목 | 결정 | 언제 |
|---|---|---|
| ROS 배포판 | Jazzy (Ubuntu 24.04) | 11-08 사용자 |
| 패키지 구조 | TITA `ros2_control` 구조, 균형 제어기 자리만 `wbr` 이식 | 11-08 사용자 |
| 시뮬레이터 | MuJoCo, Gazebo, **Isaac Sim** 셋 다. 공식 `mujoco_ros2_control` 먼저, Gazebo와 Isaac Sim은 그 뒤 | MuJoCo·Gazebo 11-08 사용자, Isaac Sim 11-12 사용자, 순서는 11-12 제안 |
| 제어기 출력 | 토크(effort)만. 모든 환경(실물·MuJoCo·Gazebo·Isaac Sim)에서 같게 | 11-08 |
| CAN 버스 | 왼쪽 다리 JC1(프로세서 1), 오른쪽 다리 **JC4**(프로세서 2), JC3은 power dist(프로세서 2). 두 다리를 pi3hat의 두 프로세서에 나눈다(upkie와 같은 방식). ID는 후보 1·2·3 / 4·5·6, `--pi3hat-cfg "1=1,2,3;4=4,5,6"` | 11-12 사용자(JC1·JC2), 11-14 사용자(오른쪽을 JC4로) |
| pi3hat 대기 값 | 라이브러리(v1.1.0) 기본값 그대로: `timeout_ns` 0, `min_tx_wait_ns` 200 µs, `rx_baseline_wait_ns` 1 ms, `rx_extra_wait_ns` 0. 바꿀 일이 생겨도 다시 빌드하지 않게 설정 파일에서 읽는다(제안) | 11-14 사용자 |
| 범위 | **주행 중 균형 잡기만.** 시작 자세·스스로 일어서기는 나중에 정함 | 11-12 사용자 |
| 상태 기계 | `idle → balance → fault`, 받침 위에서 세운 뒤 arm. Python에는 idle 상태가 따로 없고 `armed`·`fault` 플래그다. C++에서 정할 규칙은 4.4절 3번 | 11-12 |
| 제어 주기 | **목표 1 kHz**, 주기는 파라미터. Pi에서 안 나오면 500 / 200 Hz로 내림(6절 C4의 주기 결정) | 11-12, Pi 실측 뒤 확정 |
| 균형 제어기 | LQI8(`v5_lqi`). MPC(`v5_mpc`)는 나중에 같은 자리에 끼움 | 11-12 |
| 균형 제어기 구성 | **전체 제어기만 쓴다**(LQI8 상태 8개: ∫위치·∫기울기 포함, 바깥 루프 네 개 모두 켬). 순수 LQR과 적분을 하나씩 켜는 단계는 쓰지 않는다. Python 시험도 `--stage 4`만 돌린다. C++로 옮기는 것도 이 구성 하나다 | 11-07 사용자 |

제어 주기 근거: Python 단계 시험과 게인이 모두 1 kHz 기준이다. upkie도 같은 구성(Pi 4 + pi3hat + moteus 6개)에서 스파인 1000 Hz, IMU 1000 Hz가 기본값이다(`spines/pi3hat_spine.cpp` 147·177행). upkie는 CAN 주기를 별도 스레드에서 돌리고, 스파인은 직전 주기의 응답으로 계산한 뒤 그 주기가 끝나기를 기다렸다가 다음 주기를 시작한다(`Spine.cpp` 185~264행). 균형 제어는 200 Hz 에이전트에서 돈다. 우리는 그 위에 `ros2_control`과 IMU 상세 읽기가 얹히므로 실측이 필요하다. 구조도에 있던 500 Hz는 TITA / DDT의 값이다.

## 3. 패키지 구조 (안)

사용자가 준 구조도에 점검 결과를 반영한 안이다. `[수정]`, `[추가]`는 제안이고 아직 확정이 아니다.

```
ros2_ws/src/
├─ wbr_description/                      로봇 모델 + hw_env 전환
│  ├─ urdf/wheelbiped_v5.urdf.xacro       models/wheelbiped_v5의 URDF에서 (build_v5가 생성, 손으로 안 고침)
│  ├─ urdf/ros2_control.xacro             hw_env = real | mujoco | gazebo 로 플러그인 선택(Isaac Sim은 아래 isaac/ 참고),
│  │                                      관절별 CAN 버스·ID, IMU 센서, 실물용 추가 상태 인터페이스(4.3절)
│  ├─ urdf/gazebo.xacro, worlds/flat.sdf  MuJoCo 다음 순서
│  ├─ isaac/                              [추가] Isaac Sim용 USD와 IMU 센서를 더하는 덧붙임 URDF (7절)
│  ├─ mujoco/wheelbiped_v5.xml, scene.xml  [수정] 패키지로 옮길 때 meshdir 경로
│  │                                      [추가] 시뮬 시험용 기립 키프레임, 몸통을 고정한 변형(둘 다 build_v5가 생성, 6절 A2)
│  ├─ meshes/*.stl, rviz/v5.rviz
├─ wbr_hardware/                         실물 플러그인 (read / write)
│  ├─ src/pi3hat_system.cpp               [수정] 출발점: mjbots pi3hat_moteus_transport.h + moteus.h
│  ├─ (third_party 복사 대신)              [수정] CMake FetchContent, 커밋 고정. moteus(aec0333)를 pi3hat보다 먼저 선언한다
│  │                                      (pi3hat의 CMakeLists가 moteus를 다른 커밋 25c847a로 가져오기 때문, pi3hat/CMakeLists.txt:33-41)
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
│  ├─ src/wbr_controller.cpp              얇은 래퍼: 인터페이스, 명령 토픽, 명령 타임아웃, arm / disarm, 상태 발행,
│  │                                      비활성화 때 토크 0 쓰기
│  ├─ src/wbr_controller_parameters.yaml  generate_parameter_library
│  └─ test/test_equivalence.cpp           core만 링크, Python 기준 기록과 토크·내부 상태 비교
├─ wbr_msgs/msg/BalanceStatus.msg        state, fault 사유, arm 거부 사유, v, v_ref, pitch, roll, L0, 전압 …
├─ wbr_teleop/                           키보드 (DDT keyboard_controller 수정), 우선순위 낮음
└─ wbr_bringup/
   ├─ launch/robot.launch.py              hw_env 인자 하나로 실물·MuJoCo·Gazebo·Isaac Sim
   ├─ launch/record.launch.py             rosbag2
   ├─ launch/hardware_check.launch.py     [추가] 균형 제어기 없이 상태 확인과 관절별 토크 시험 (bringup 0단계)
   ├─ config/controllers.yaml             주기, 제어기, 브로드캐스터, thread_priority·cpu_affinity
   ├─ config/wbr_params_<주기>hz_<이름>.yaml  [수정] export_params.py가 만드는 파일 (게인표 포함). 이름 = 게인 세트(기본, 저게인)
   ├─ config/fastdds_udp.xml              [추가] root로 돌 때의 DDS 설정
   └─ scripts/                            [추가] Pi 설정 확인, 실행 스크립트

wheeled_biped/docs/pi_setup.md           [추가] Pi 4 준비: Ubuntu 24.04, 실시간 커널, CPU 격리, 빌드, root 실행, 시간 동기

wheeled_biped/wbr/tools/                 ROS 패키지 아님, 아직 없음
├─ export_params.py                       PlanV5를 만든 직후의 객체들에서 설정·표를 통째로 내보냄 (5.1절)
└─ dump_golden.py                         Python 제어기의 입력·적용된 명령·출력·내부 상태 기록 (5.3절)
```

C++로 옮기지 않는 것: `wbr/observers/pi3hat_ars.py`(pi3hat 펌웨어 필터 흉내, 시뮬 전용), 시뮬 하네스(`plan4_lqi8.py`, `plan_v5.py`의 모델·노이즈·지연 부분). `host_link.py`는 DIABLO 직렬 링크 흉내이지만 그 규칙(4.4절 2번)은 래퍼에 옮긴다.

## 4. 실물에 필요한 것

### 4.1 moteus 명령과 질의 (C++ 기본값의 함정)

1. **토크만 보내는 명령의 정확한 형태.** C++ `PositionMode`의 기본값은 `Command`가 `position = 0.0`, `kp_scale = 1.0`, `kd_scale = 1.0`, `watchdog_timeout = NaN`이고, `Format`은 `position`·`velocity`만 보내고 `feedforward_torque`·`kp_scale`·`kd_scale`·`maximum_torque`·`watchdog_timeout`은 보내지 않는다(`kIgnore`, `moteus_protocol.h:896-927`). 기본 `Format`으로 보내면 토크는 전달되지 않고 "위치 0, 설정된 kp 그대로"인 위치 명령이 나간다. 그래서 이렇게 보낸다:
   - `Command`: `position = NaN`, `velocity = 0`, `feedforward_torque = τ`, `kp_scale = 0`, `kd_scale = 0`, `maximum_torque = τ_max`, `watchdog_timeout = 유한한 양수`
   - `Format`: 위 필드를 모두 `kIgnore`가 아닌 값(예: `kFloat`)으로 둔다.
   - Python `make_position`은 준 인자만 보내므로 `docs/bringup_step0.md` 9절의 Python 예와 C++ 기본 동작이 다르다.
2. **`watchdog_timeout`에 NaN을 넣지 않는다.** NaN은 "시간 초과 없음", 0은 설정 `servo.default_timeout_s`(기본 0.1 s)를 쓴다는 뜻이다(`docs/protocol/registers.md:401-409`, `fw/bldc_servo_control.h:232`). 유한한 값(예: 몇 주기)을 넣는다.
3. **질의에 `home_state`를 켠다.** 기본 `Query::Format`은 `mode`, 위치·속도·토크, 전압, 온도, `fault`만 읽고 `home_state`는 읽지 않는다(`moteus_protocol.h:336-350`). 켜지 않으면 결과가 0으로 남아 `arm()`이 이유 없이 계속 거부한다. `query_format.home_state = kInt8`로 두면 한 응답에 모두 온다.
4. **영점 전에 위치 모드 명령을 보내지 않는다.** moteus는 전원을 켜면 관절각을 ±22.5° 안으로 접어 읽는다. 접힌 값은 위치 한계 밖일 수 있고(기립 자세: 무릎 −0.1282 → −0.0032 회전, hip 0.0738 → −0.0512 회전), 한계 밖에서 위치 모드로 들어가면 fault 39가 난다(`fw/bldc_servo_control.h:1637-1641`, 토크 0 명령도 위치 모드다). fault는 `stop`으로만 풀린다. 플러그인의 활성화 순서: 질의만 → hip·무릎에 `MakeOutputNearest`(기준 자세 값, `docs/bringup_step0.md` 7절) → `home_state`(레지스터 0x00c)가 2인지 확인 → 그 뒤에 토크 명령.
5. **시간 초과는 토크를 끊지 않는다(결정 필요).** 기본 `servo.timeout_mode`는 12(속도 0을 유지하는 감쇠, 최대 `timeout_max_torque_Nm` 5 N·m)이고 0이 구동 해제, 15가 단락 제동이다(`fw/bldc_servo_structs.h:613-623`). 시간 초과 상태는 `stop`을 보내야 풀린다. Python의 안전 상태는 모든 관절 토크 0이다. 프로그램이 죽었을 때 다리가 감쇠로 버틸지 풀릴지를 정해 `timeout_mode`와 `timeout_max_torque_Nm`을 설정한다(6절 C3).
6. **위치 한계는 토크만 쓰는 명령에도 걸린다.** 한계를 0.02 회전 넘으면 바깥으로 미는 토크가 0이 된다(`docs/bringup_step0.md` 5절). 바퀴는 한계를 NaN으로 둔다.
7. **상태 인터페이스.** `mode`, `fault`, 전압, 온도, `home_state`를 제어기까지 올린다. 배터리 전압은 moteus 전압이나 power dist(CAN ID 32, JC3)에서 읽는다.

### 4.2 pi3hat

1. **root.** 라이브러리는 `/dev/mem`을 직접 연다(`pi3hat.cc:361`). 한 번에 한 프로그램만 쓸 수 있다(잠금 파일 `/tmp/.pi3hat-lock`, `pi3hat.cc:1152`). `ros2_control_node`가 떠 있는 동안에는 `tview`, `pi3hat_tool`, `moteus_tool`을 쓸 수 없다. 드라이버 설정과 점검은 ROS를 끄고 한다.
2. **넣을 값.** 설정 `attitude_rate_hz = 1000`(기본 400, `pi3hat.h:178`), `mounting_deg = 0, 0, 0`. 매 주기 입력에는 `request_attitude`와 `request_attitude_detail`. `Pi3HatMoteusTransport`는 `Attitude*`를 넘기면 `request_attitude`를 스스로 켜지만(`pi3hat_moteus_transport.h:201-202`) `request_attitude_detail`은 `Options::default_input`에 직접 넣어야 한다. 플러그인에서 끄지 않고 항상 켠다(11-14 사용자): `opts.default_input.request_attitude_detail = true;` 이 값은 `Pi3Hat::Input`의 멤버라 C++에서만 켤 수 있고, `pi3hat_tool`과 Python `Pi3HatRouter`에는 이 옵션이 없다.
3. **바이어스는 조용히 0이 된다.** 상세 요청 없이 읽으면 42바이트만 읽고 바이어스 칸은 이전 값(처음에는 0)을 그대로 복사한다(`pi3hat.cc:1268-1284`). NaN이 아니라서 제어기의 가드가 잡지 못한다. upkie `Pi3HatInterface.cpp`가 이 상태다.
4. **표본 시각은 플러그인이 찍는다.** `Attitude`에는 시각이 없다(`pi3hat.h:105-113`). 새 표본이 왔는지는 `Output::attitude_present`로 안다. 플러그인은 새 표본일 때만 단조 시계로 시각을 찍는다. 새 표본이 없는 주기에는 시각을 갱신하지 않는다(그러면 제어기가 추정값을 유지하고, 50 ms 넘게 이어지면 FAULT). 매 주기 새 표본을 기다릴지(`wait_for_attitude`, upkie 방식)는 정한다.
5. **대기 값은 라이브러리 기본값을 쓴다(11-14 사용자).** `min_tx_wait_ns` 200 µs, `rx_baseline_wait_ns` 1 ms, `rx_extra_wait_ns` 0, `timeout_ns` 0(`pi3hat.h:215-236`). 응답이 다 오면 최소 200 µs 뒤 바로 끝나므로 평소 주기에는 영향이 없다. 응답이 하나라도 빠지면 "계산된 전송 시간 + `rx_baseline_wait_ns`"까지 기다리므로(`pi3hat.cc:1619-1628, 1687-1693`) 그 주기는 1 ms를 넘는다. 그래서 7번의 처리가 필요하다. 주석은 CPU를 격리한 환경이면 `rx_baseline_wait_ns`를 200 µs까지 줄일 수 있다고 하지만, 바꾸려면 C++ 코드에서 넣어야 한다(`pi3hat_tool`, C++ 통신 계층의 명령줄, Python `Pi3HatRouter` 어디에도 이 옵션이 없다).
6. **응답은 출처 ID로 짝짓는다.** `Cycle`은 보내기 전에 남아 있던 프레임을 먼저 거두어 같은 버퍼에 담는다(`pi3hat.cc:1707-1709`). 늦게 온 지난 주기의 응답이 이번 주기 값처럼 보일 수 있으므로 순서로 짝짓지 않는다. 트랜스포트의 첫 주기는 수신 버퍼가 5칸이라(`pi3hat_moteus_transport.h:178`) 6대 중 1대의 응답이 빠진다.
7. **응답이 빠진 관절.** 그 관절 값을 NaN으로 두거나 표본 시각을 갱신하지 않는다. 지난 값을 새 값처럼 다시 내보내면 제어기의 가드가 무력해진다.
8. **응답할 장치가 없는 버스로 보내지 않는다.** 옛 펌웨어는 아무도 응답하지 않는 프레임을 보내면 멈춘다(`pi3hat.h:290-295`의 주석). 연결하지 않은 포트뿐 아니라 전원이 꺼진 moteus도 해당한다. 플러그인은 시작할 때 `device_info().can_unknown_address_safe`를 검사하고, 설정에 있는 버스로만 보낸다.
9. **스레드.** `Pi3HatMoteusTransport`는 자체 작업 스레드에서 `Cycle`을 돌리고 끝을 콜백으로 알린다(`pi3hat_moteus_transport.h:64-66, 101-160`). `write()`에서 시작하고 다음 `read()`에서 거두면 controller_manager 스레드가 `Cycle`에 막히지 않는다. 작업 스레드는 격리된 CPU가 있으면 거기에 고정된다.

### 4.3 플러그인과 제어기 사이의 규약

| 인터페이스 | 값 | 실물에서 만드는 법 | 시뮬 |
|---|---|---|---|
| 관절 `effort` 명령 | 관절 토크 [N·m], 로봇 왼쪽에서 보아 반시계 + | moteus 피드포워드 토크 | 구동기 |
| 관절 `position`, `velocity`, `effort` 상태 | rad, rad/s, N·m | 회전 → rad | 시뮬레이터 |
| 관절 `mode`, `fault`, `home_state`, 전압, 온도 | moteus 값 | 질의 응답 | 없음 |
| IMU `orientation` | IMU 축에서 세계(z가 위)로의 회전 | pi3hat 쿼터니언(ARS, z가 아래)을 x축 180° 회전 | 시뮬레이터 |
| IMU `angular_velocity` | 보드의 `rate`(보드가 바이어스를 보정한 값), rad/s, IMU 축 | `rate_dps` × π/180. **바이어스를 또 빼지 않는다** | 자이로 |
| IMU `linear_acceleration` | 원시 가속도(중력 포함), IMU 축 | `accel_mps2 + 9.81 × IMU에서 본 위 방향` | 가속도계 |
| IMU 자이로 바이어스 | 보드의 `bias`, rad/s, IMU 축 | `bias_dps` × π/180 | 0 |
| 표본 시각 | 단조 시계 [s] | 4.2절 4번 | `update()`의 시각 |

- 원시 가속도의 식과 부호는 `docs/bringup_step0.md` 8절에 있다. hat이 위인 정지 상태에서 위 방향은 (0, 0, +1)이다. 아래 방향으로 넣으면 부호가 틀린다.
- 펌웨어는 `rate = 원시 자이로 + bias`로 정의한다(`fw/attitude_reference.h:142-144`). 원시 z 자이로(`rate − bias`)는 제어기가 만든다. 플러그인이 미리 빼면 z축은 두 번 빠지고 x·y축은 보정이 사라진다.
- IMU는 IMU 축 그대로 내보낸다. 몸통 좌표로 바꾸는 일(`R_base_to_imu` = diag(−1, −1, 1), URDF `imu_fix`)은 제어기가 한다. 시뮬레이터도 IMU 축으로 내보내기 때문이다(MJCF의 `imu` 사이트가 보드 축이다). 11-08 메모(`전체 로봇 구조도`)는 플러그인이 몸통 좌표로 바꾼다고 적었지만 이 문서가 기준이다.
- **Jazzy에는 "있으면 쓰는" 선택 인터페이스가 없다.** 제어기가 요청한 상태 인터페이스가 하나라도 없으면 활성화가 실패한다(`controller_manager.cpp`의 "Unable to activate controller … is not available"). 그래서 제어기 파라미터로 요청 목록 자체를 바꾼다(실물: 바이어스·표본 시각·`home_state` 등 포함, 시뮬: 표준 값만). 시뮬에서는 래퍼가 바이어스 0과 `update()`의 시각을 넣는다.
- 실물에서 표본 시각, 가속도, 바이어스는 선택 사항이 아니다. 시각이 없으면 오래된 표본 가드가 꺼지고 자세 필터가 고정 dt로 돈다(`mcu.py:372`). 가속도가 없거나 받던 바이어스가 빠지면 FAULT다(`mcu.py:319-326`).

### 4.4 제어기 쪽

1. **명령.** DDT의 세 토픽을 쓴다: `command/cmd_twist`(전진 속도 [m/s], 요 속도 [rad/s]), `command/cmd_pose`(높이·롤·피치), `command/cmd_key`. Python의 환산: 전진은 ±1.0 m/s, 요는 ±2.0 rad/s로 자른다. 높이는 `L_target = L_stand + 0.06 × up`을 다리 범위로 자른다. 롤·피치 기준은 rad이고 ±0.3으로 자른다(`mcu.py:487-500`, `McuParams`).
2. **명령이 끊겼을 때(`host_link.py`의 규칙).** 0.5 s 넘게(`>`) 명령이 없으면 전진·요는 0, 속도 모드 입력은 0, 각도 모드 값과 모드 플래그는 유지한다. 균형은 계속한다(`disconnect_off_s` = 0). 유한하지 않은 값이 든 명령은 버리고 수신으로 세지 않는다.
3. **arm과 상태(정해야 함).** arm / disarm을 부르는 수단이 구조도에 없었다. 제안: 거부 사유를 문자열로 돌려주는 서비스(`std_srvs/Trigger`)와 `cmd_key`. Python의 동작은 이렇다.
   - 하네스는 `reset(armed=True)`로 무장한 채 시작한다. 실기 진입점은 `armed=False`다(`docs/HANDOFF.md` 2.1절).
   - `arm()`은 측정값이 없을 때, 피치 3° 이상, hip·무릎의 `home_state`가 2가 아닐 때, 다리 길이가 326~485 mm 밖, 다리 각이 ±20° 밖이면 거부한다(`mcu.py:248-276`).
   - FAULT가 나도 `armed`는 그대로다. `arm()`이 fault를 지우고 바로 균형으로 돌아간다.
   - 무장하지 않은 동안에도 다리 길이 기준 `L_ref`는 목표(기본 460 mm)로 간다. 다른 길이에서 arm하면 다리 길이 PD에 계단 입력이 들어간다.
   - 무장하지 않았거나 FAULT인 동안 LQI8은 매 주기 초기화된다(`law.py:48-50`).
   - C++에서 정할 것: fault에서 나가는 규칙, arm할 때 `L_ref`를 지금 다리 길이로 맞출지. Python과 다르게 하려면 Python을 먼저 바꾸고 단계 시험을 다시 돌린다.
4. **FAULT와 안전 상태(Python과 같게).** 몸통 피치·롤 34° 초과, NaN 표본이 5주기 초과, 오래된 표본이 50 ms 초과, 균형 법칙 입력이 유한하지 않은 상태가 5주기 초과, 유한하지 않은 토크 명령이면 FAULT이고 모든 관절 토크 0이다.
5. **제어기를 끌 때 토크 0을 쓴다.** ros2_control은 제어기를 비활성화해도 명령 인터페이스의 마지막 값을 지우지 않고, 하드웨어의 `write()`는 계속 돈다(표준 `forward_command_controller`도 마지막 값을 남긴다). `wbr_controller`의 `on_deactivate`에서 effort 6개에 0을 쓴다. 플러그인은 effort를 잡은 제어기가 없어지면(`perform_command_mode_switch`) `stop`을 보낸다.
6. **꺼질 때와 죽었을 때.** launch를 Ctrl+C로 끄면 controller_manager가 하드웨어를 비활성화·종료한다. 이 경로에서 플러그인이 모든 관절에 `stop`을 보낸다. 프로그램이 죽으면 명령이 끊기고 moteus의 시간 초과가 맡는다(4.1절 5번).
7. **전 주기 기록.** 제어에 쓰인 모든 신호(관절, IMU, 추정값, 기준, 토크, 상태, 표본 시각과 `write()` 시각)를 제어 주기 그대로, 순번을 붙여 기록한다. 기록은 실시간이 아닌 스레드로 넘긴다(`realtime_tools`). rosbag만으로 1 kHz가 버거우면 메모리 버퍼에 모았다가 파일로 쓴다.
8. **root와 DDS.** root 노드와 일반 사용자 노드는 FastDDS 공유 메모리로 통신이 안 된다(G-Levine README). UDP 전용 프로필을 만들고, root 쪽과 일반 사용자 쪽 터미널 모두에 같은 `FASTRTPS_DEFAULT_PROFILES_FILE`을 준다.

## 5. 옮길 때 조심할 것

### 5.1 파라미터와 표

- **출처는 여러 곳이다. 손으로 옮기지 않는다.** `models/wheelbiped_v5/description.json`(`plan_overrides`의 게인과 마찰 보상 크기 0.096, 바퀴 반경·윤거·토크 한계·IMU 위치·`R_base_to_imu`), `plant_params.json`(다리 범위, 기립 길이, 질량), `PlanV5Config`(서스펜션·감속), `Plan4Config`(속도 창, 마찰 보상의 시간 상수와 속도 문턱, 요 적분 한계), `McuParams` 기본값(`height_gain`, `arm_pitch`, `fall_pitch`, `max_forward` 등), LQI8 가중치(`plan4_lqi8.py`의 `lqi_kwargs` + `PlanV5._lqi_design` + `ake_cad`의 `LQR_R_*`)와 `LQI8` 생성자 기본값.
- **만든 직후의 객체에서 모두 내보낸다.** `plan.mcu.p`, `plan.mcu.attitude`(`BodyAttitude`의 `deriv_tau` 0.02, `reject` 1.0, `quiet` 3.0), `plan.law`(`Tp_ff`, `theta_eq`), `plan.law.ctrl`(K 표, gM 표, 설정), `plan.leg.g`(L1 = L2 = 0.25), `plan.plant`(`tau_*_max`, 정적 다리 힘에 쓰는 표와 질량), `plan.link.p`(명령 타임아웃), `R_ARS_TO_WORLD`. `law.ctrl`의 `v_ramp_rate`, `v_ref_tau_brake`, `hold_integrators`는 실행 중에 바뀌므로(`law.py:56-59`) 한 스텝도 돌리기 전에 내보낸다.
- **코드에 박힌 상수는 내보내기로 안 나온다.** 소스에서 옮기고 목록으로 관리한다: 요 오프셋 학습의 0.1 s 저역 통과와 0.1 rad/s 문턱(`mcu.py:469-470`), 롤·피치 기준 제한 0.3 rad, 마찰 보상의 `2.0 × tanh`(`mcu.py:646`), 쿼터니언 크기 허용 0.1(`mcu.py:315`), 정지 판정 0.1 m/s와 0.026 rad(`lqi8.py:274`), `1e-9` 문턱(`lqi8.py:253`, `mcu.py:350`), 포화 판정의 토크 한계 × 2(`lqi8.py:293`).
- **표.** 다리 길이 격자는 0.312~0.497 m, 5 mm 간격, 38점이다. LQI8 게인 K(2 × 8)는 원소마다 선형 보간하고 격자 끝에서 자른다(`lqi8.py:214-220`). gM은 균일 격자의 선형 섞기다(`lqi8.py:199-212`). `theta_eq(L0)`는 표, `Tp_ff`는 기립 길이에서의 상수 하나다. 정적 다리 힘(`SerialPlant.static_leg_force`)은 c와 d의 기울기 표(`np.gradient`), 평균 `m_l`, `m_b`, `g`로 계산하고 다리마다 세계 다리각을 넣는다(`plant_serial.py:210-219`, `mcu.py:680`). 게인은 주기(`Ts`)마다 다르다.
- **스텝 수는 Python이 계산한 정수를 내보낸다.** 속도 창 스텝은 매 주기 `int(round(vel_window_s / dt))`로 계산한다(`mcu.py:288`). 5 ms 창은 500 Hz에서 2.5스텝인데 Python `round`는 2, C++ `std::round`는 3이다. 200 Hz에서는 1스텝이 되어 창이 꺼지고 드라이버 속도를 그대로 쓴다.
- **단계 시험과 같은 제어기인지 확인한다.** 단계 시험은 커밋된 `models/wheelbiped_v5`를 쓰지 않고, CAD 파일(`robot_cad_v5/calc/*.json`)에서 임시 모델과 게인을 그 자리에서 다시 만든다(`stage_tests.py:101-124`). 내보낼 때 두 경로의 값이 같은지 비교한다(6절 A1).
- **실물 질량을 잰 뒤에도 yaml을 손으로 고치지 않는다.** `robot_cad_v5/calc/mass_properties.json`(Fusion의 `save_mass_json()`이 쓰는 파일)을 잰 값으로 맞춘 뒤 `python -m wbr.models.build_v5` → 내보내기 순서로 다시 만든다. 다리 힘 피드포워드(`F_ff`)가 틀리면 다리 길이가 어긋난다: 몸통 질량 오차 1 kg에 약 3.3 mm(다리 하나에 0.5 × 9.81 N, `kp_leg` 1503 N/m로 나눈 값).
- **저게인 세트가 필요하다.** 실물의 첫 자립은 토크 가중치 R을 키운 낮은 게인으로 하고 단계적으로 올린다(사용자 절차 5-3, 5-4). 내보내기 도구가 이름 붙인 게인 세트를 여러 개 만들 수 있어야 한다.

### 5.2 숨은 상태와 순서

- **C++가 그대로 가져야 할 상태.** MCU: 시계 `t`, `L_ref`·`L_target`, `yaw_int`, `yaw_ref_limited`, `yaw_offset`, `wheel_yaw_lp`, `roll_force_int`, `roll_ref_limited`, `v_limit`·`v_limit_hold`, `fc_hip_lp`·`fc_wheel_lp`, `position`, 위치 이력(속도 창), `bad_samples`·`stale_samples`, 표본 시각 두 개, 마지막 유효 서보 값, `home_state`(`mcu.py:182-234`). 자세 필터: `up`, `gyro_lp`, `f_lp`, `vh_lp`, `a_hip`, `lateral`. LQI8: `z`, `z_att`, `z_pos`, `u_prev`, `v_ramp`, `v_filt`, `v_filt1`, 기준 가속도 저역 통과, 동결 카운터, 마지막 출력.
- **첫 표본.** 속도는 드라이버 값으로 시작하고 창이 한 스텝씩 늘어난다. `up`은 보드 쿼터니언에서 시작한다. 요 오프셋은 `−bias_z`로 시작한다. 시각 간격은 공칭 `dt`다.
- **순서.** 추정이 명령 적용보다 먼저 돈다. 그래서 요 오프셋 학습은 전 주기의 요 기준을 본다. 자세 필터는 전 표본에서 구한 hip 가속도를 쓴다.
- **시각 간격을 쓰는 곳과 고정 `dt`를 쓰는 곳.** 표본 시각 간격은 자세 필터, 요 오프셋 학습, hip 속도 미분에 쓴다(`mcu.py:426,456,461`). 위치 적분은 고정 `dt`다(`mcu.py:453`). Python과 같게 둔다(`docs/bringup_step0.md` 9절).
- **신호 경로.** 피치·롤 속도는 보드의 `rate`를 몸통 좌표로 바꾼 값이다. 요 속도는 원시 z 자이로(`rate − bias`)에서 제어기가 배운 오프셋을 뺀 뒤, 세 축을 세계 수직 방향에 투영한 값이다(`mcu.py:417-428`). 피치·롤 각은 자세 필터에서 나온다. 보드의 자세각은 필터의 시작값으로만 쓰지만(`docs/bringup_step0.md` 8절) 쿼터니언은 매 주기 검사한다(NaN이거나 크기가 1에서 0.1 넘게 벗어나면 잘못된 표본).
- **`upkie_observers.py`의 기본값을 복사하지 않는다.** `BaseOrientation`의 기본 `R_base_to_imu` = diag(−1, 1, −1)은 upkie의 값이다. v5는 diag(−1, −1, 1)을 넘긴다.

### 5.3 기준 기록과 일치 시험

- **`core`는 Python `obs`와 같은 입력을 받는다(제안).** 관절 위치·속도·토크, IMU 자세(ARS 좌표의 쿼터니언, 순서 w x y z), IMU 축의 `rate`·원시 가속도·바이어스, 표본 시각, 있으면 `home_state`. ROS 규약에서 이 규약으로 바꾸는 일은 래퍼가 한다: 쿼터니언 순서(x y z w → w x y z)와 x축 180° 회전 하나(쿼터니언 곱). 값의 자리와 부호만 바뀌므로 오차가 없다.
- **기준 기록에 넣을 것.** 스텝마다: `mcu.tick`에 들어간 `obs`(지연선과 부호 변환을 지난 것), 그 스텝에 실제로 적용된 명령(`link.poll`이 돌려준 `MotionCtrl` 전체: 값과 모드 플래그), 관절 토크 6개, 내부 상태(`z`, `z_att`, `z_pos`, `up`, `yaw_offset`, `position`, `L_ref`), 그리고 reset·arm·disarm이 일어난 스텝 번호.
- **명령 타임아웃은 `core` 밖에서 시험한다.** `tick(obs, host_cmd)`에 적용된 명령을 직접 넣으면 링크의 시계가 빠진다. 하네스는 명령 수신 시각을 시뮬 시각으로, 타임아웃 판정을 MCU의 시계로 한다(`plan4_lqi8.py:418`, `mcu.py:515`). 타임아웃 규칙은 래퍼의 시험으로 따로 확인한다.
- **arm 경로가 든 기록을 따로 만든다.** 하네스는 무장한 채 시작하므로 보통의 기록에는 `arm()`이 한 번도 나오지 않는다. 무장하지 않은 채 시작해 arm하는 기록, FAULT가 나는 기록을 더한다.
- **토크만 비교하지 않는다.** 문턱(`1e-9`)이나 조건부 적분이 한 번 다르게 갈리면 적분기가 계속 벌어진다. 내부 상태도 스텝마다 비교한다.
- **기준 기록과 파라미터 파일은 같은 실행에서 만들어 같이 커밋한다.** 따로 만들면 Python·numpy·scipy 버전 차이로 게인이 조금 달라질 수 있다. 파일 머리에 만든 날짜, 커밋, 패키지 버전을 적는다.

### 5.4 지연과 시뮬

- **한 주기 지연.** `ros2_control`은 읽기 → 제어 → 쓰기 순서이고 pi3hat은 보내기와 받기가 한 번의 `Cycle`이다. `write()`에서 `Cycle`을 시작하면 다음 `read()`가 그 응답을 쓴다. 센서에서 토크까지는 대략 한 주기 + `Cycle` 시간 + IMU 표본 나이다.
- **지연 한계.** 시험은 10 ms로 돌린다. 한계는 문서마다 다르게 적혀 있다: `docs/robot_v5_model.md` 5절은 11 ms까지 떨림 없음, 12 ms에서 6회 중 4회 떨림. `docs/bringup_step0.md` 9절은 12 ms까지 정상, 13 ms부터 떨림. 여유를 잡을 때는 11 ms를 한계로 본다(1 kHz, 속도 창 5 ms).
- **MJCF의 구동기와 센서는 그대로 공식 `mujoco_ros2_control`에 맞는다**(문서 기준, 실행은 안 해 봄): `general` 구동기는 effort를 그대로 받고, 센서 이름 `imu_quat` / `imu_gyro` / `imu_accel`은 `mujoco_sensor_name = imu`로 잡힌다. 고칠 것은 패키지로 옮길 때의 `meshdir` 경로와 6절 A2의 추가 두 가지다. 이 패키지는 자기 `ros2_control_node`를 써야 한다.
- **MJCF의 `stand` 키프레임은 다리를 편 자세(500 mm)다.** `arm()` 허용 범위(326~485 mm) 밖이다.
- **ROS 시뮬은 연결 확인용이다.** 노이즈·지연·pi3hat 필터 흉내는 Python 하네스에만 있다. 제어기의 합격 판정은 Python 단계 시험(`v5_lqi/TEST`)과 C++ 일치 시험으로 한다.

## 6. 작업 순서 (2026-11-13)

단계마다 통과 조건이 있고, 통과하기 전에는 다음으로 가지 않는다. 사용자의 단계별 절차(옵시디언 `DIABLO 방식을 로봇에 적용하는 절차`)와 `docs/bringup_step0.md`를 ROS 작업 순서로 옮긴 것이다. 균형 로봇의 실수는 대부분 부호와 영점에서 나오므로, 부호 확인을 시뮬(A3)과 실물(C5)에서 두 번 한다. "후보"라고 적은 수치는 아직 정하지 않은 것이다.

### A. 노트북 (실물 없이)

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| A0 | 기준 확인: `python v5_lqi/TEST/run_stages.py --stage 4 --keep-going` (시드 0 하나, 약 3분) | 넘어짐과 무릎 꺾임(`knee_flipped`)이 없다. 보고서의 단계 판정은 "실패"로 나오는 것이 지금 상태다: 저장소 보고서(시드 5개)에서 시험 27개 중 7개가 일부 기준을 넘고, 시드 0에서는 2-3, 2-9, 2-10(다리 토크 rms), 2-12(방향·옆 이동·롤), 2-13(다리 토크 피크·롤) 다섯 개다. 노트북에서도 넘는 시험이 이와 같거나 비슷하면 된다. 경계에 있는 값은 MuJoCo 버전 차이로 달라질 수 있다. 실행하면 보고서 파일이 덮어써지므로 커밋하지 않는다 |
| A1 | `wbr/tools/export_params.py`, `dump_golden.py` (5.1절, 5.3절) | 1 kHz의 파라미터·표 파일과 기준 기록이 나온다. 내보낸 파일만 읽어 다시 계산한 토크가 원래 Python 제어기와 같다. 커밋된 모델에서 내보낸 값과 단계 시험 경로에서 만든 값이 같다 |
| A2 | `wbr_description`: xacro, `ros2_control` 태그, MuJoCo. `build_v5`가 시뮬 시험용 기립 키프레임(Python `reset()`과 같은 자세: 다리 460 mm, 다리 각 `theta_eq` 약 3.5°)과 몸통을 고정한 MJCF 변형을 만든다 | `colcon build` 통과. MuJoCo 창에 모델이 뜨고 `joint_states`와 IMU 토픽이 나온다 |
| A3 | ROS 시뮬에서 부호 시험. 몸통을 고정한 변형에서 `forward_command_controller/ForwardCommandController`(인터페이스 effort)로 관절마다 + 토크. IMU는 떠 있는 모델을 `set_free_joint_state` 서비스로 기울인 자세에 놓고 읽는다 | 움직이는 방향이 `docs/bringup_step0.md` 4절 표와 같다. IMU 부호가 `docs/bringup_step0.md` 8절 표의 규약 값과 같다. 이 시험 제어기는 꺼도 마지막 토크가 남으므로 0을 보낸 뒤 끈다 |
| A4 | `wbr_controller/core` + `test_equivalence`. 옮기는 순서: serial_leg → gain_table·lqi → estimator → outer_loops → state_machine | 기준 기록의 모든 스텝에서 관절 토크 6개와 내부 상태가 Python과 일치한다(후보: 절대 오차 1e-6 이하, NaN은 같은 자리에서 NaN). arm 경로와 FAULT가 든 기록도 통과한다 |
| A5 | `wbr_controller` 래퍼 + `wbr_bringup`. 처음부터 전체 제어기로 한다 | MuJoCo에서 1 kHz로: 정지 60 s 동안 피치 rms ≤ 0.5°, 위치 이동 ≤ 5 cm(Python 시험 1-1의 기준). 속도 스텝 0 → 0.5 m/s → 0에서 정상 오차 ≤ 0.05 m/s, 피치 ≤ 5°(시험 1-4의 기준). 넘어지지 않는다 |
| A6 | 안전 동작 시험. 실제 시각으로 돌린다 | 명령이 0.5 s 끊기면 속도 0으로 서 있다. 피치 34° 초과, NaN, 오래된 표본이면 FAULT와 토크 0. 세 경우 각각 토크가 0이 된다: 제어기 비활성화, launch 종료(Ctrl+C), 프로세스 강제 종료. arm 거부 사유가 서비스 응답과 상태 메시지에 나온다 |
| A7 | `wbr_msgs`, `wbr_teleop`, 기록(rosbag과 전 주기 기록) | 키보드로 전진·회전·높이를 바꾼다. 기록의 순번에 빈칸이 없고 표본 수가 시간 × 주기와 같다 |
| A8 | 계산 시간 측정: 기준 기록을 `core`에 다시 넣는 벤치(ROS 없이) | 한 틱 계산 시간의 최대와 99 % 값을 적어 둔다(Pi와 비교할 기준) |

- A2·A3과 A4는 서로 기다리지 않는다.
- **A5의 시작 상태(제안, 실행은 안 해 봄).** 공식 `mujoco_ros2_control`에는 일시정지로 시작하는 옵션이 없다. 시작 상태는 키프레임 이름(`initial_keyframe`)으로 정하고, 일시정지는 뜬 뒤에 서비스(`~/set_pause`)로만 건다. 제어기가 켜지기까지 1~2 s 동안 로봇이 쓰러지므로 순서를 이렇게 잡는다: launch → 제어기가 active가 됨 → `~/reset_world` 서비스로 기립 키프레임으로 되돌림 → 바로 arm. 이 키프레임은 시뮬 시험용이고, 실물의 시작 자세는 여전히 나중에 정한다.
- **A5의 밀기.** `mujoco_ros2_control`에는 외력을 주는 서비스가 없다. 뷰어에서 마우스로 밀거나 `set_free_joint_state`로 몸통에 속도를 주어 복귀하는지만 본다. 크기를 정한 밀기 시험은 Python 단계 시험의 판정을 쓴다.
- 래퍼의 골격은 `ros2_control_demos`(`jazzy`)의 예제를 따르고, 명령 토픽과 상태 기계의 틀은 `ddt_ros2_control`의 `rl_controller`와 `TITA_ROS2_Control_Sim`의 `template_ros2_controller`를 참고한다. `rl_controller`에는 토크 한계 자르기만 있고 안전 검사는 항상 통과이며(`fsm/FSM.cpp:94-121`) 명령 타임아웃이 없다. `template_ros2_controller`에는 둘 다 없다. 4.4절을 반드시 넣는다.

### B. Pi 4 (pi3hat 없이)

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| B1 | Ubuntu 24.04 Server(arm64), 실시간 커널(아래 둘 중 하나), `isolcpus`, ROS 2 Jazzy, 노트북과 시간 동기(chrony). root로 `/dev/mem`의 주변장치 영역(Pi 4는 0xfe000000부터, `pi3hat.cc:182`)을 mmap해 읽는 작은 시험 | `uname -a`에 `PREEMPT_RT`가 보이고 `cat /sys/kernel/realtime`이 1이다. `/dev/mem` mmap이 된다. 전원 부족 경고가 없다 |
| B2 | `cyclictest`로 1 kHz 타이머 지터 | 최대 지연을 적는다(후보: 200 µs 이하. 넘으면 CPU 격리와 우선순위 설정을 다시 본다) |
| B3 | 작업 공간을 Pi에서 빌드. (1) A8의 벤치를 Pi에서 돌린다. (2) `mock_components/GenericSystem` + 브로드캐스터 + `wbr_controller`를 1 kHz, 실제 시각으로 돌린다 | (1) 한 틱 계산 시간의 최대가 주기의 일부다(후보: 300 µs 이하). (2) 60 s 동안 주기의 평균·99 %·최대와 overrun 경고 수를 적는다(후보: 평균 오차 5 % 이내, 99 % 값 1.1 ms 이하, 최대 2 ms 이하) |

- B3의 (2)에서는 가짜 하드웨어 값으로 제어기가 무장하지 못해 균형 계산이 돌지 않는다. 그래서 계산 시간은 (1)의 벤치로 따로 잰다. controller_manager의 overrun 처리는 `use_sim_time`이 켜져 있으면 꺼지므로 실제 시각으로 잰다.
- 여기까지는 "Pi의 CPU와 ROS가 1 kHz를 버티는가"만 확인된다. 모터 통신을 포함한 주기는 C4·C6에서 잰다.
- `/dev/mem` 시험을 B1에 둔 이유: Ubuntu 커널에서 pi3hat 라이브러리가 `/dev/mem`을 쓸 수 있는지가 확인된 곳이 없고, hat 없이도 미리 볼 수 있다.

실시간 커널을 얻는 방법은 두 가지다(2026-11-12 확인, 둘 다 Pi에서 해 보지는 않았다).

| 방법 | 하는 일 | 비고 |
|---|---|---|
| Ubuntu Pro의 실시간 커널 | 보통의 Ubuntu 24.04에 `sudo pro attach <토큰>` 뒤 `sudo pro enable realtime-kernel --variant=raspi` | Ubuntu Pro는 다른 운영체제가 아니라 구독이다. 개인은 5대까지 무료이고 Ubuntu One 계정과 토큰이 필요하다. Pi 4·5를 지원하고 apt로 갱신된다 |
| `ros-realtime/linux-real-time-kernel-builder` | Ubuntu `raspi` 커널(6.8)에 PREEMPT_RT 패치를 입힌 deb를 직접 빌드해 `dpkg -i`로 설치 | Ubuntu Pro가 필요 없다. 빌드는 x86 Ubuntu 24.04의 Docker에서 하고 디스크 16 GB가 든다. 미리 만든 deb는 GitHub Actions의 산출물이라 남아 있지 않을 수 있다. 저장소는 2025년 중반 이후 갱신이 없다. 커널 갱신은 스스로 다시 해야 한다 |

### C. pi3hat과 moteus가 온 뒤

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| C1 | pi3hat 단독 점검. 전원을 끈 채 장착하고 켠다 | `/sys/firmware/devicetree/base/hat/product`가 `mjbots quad pi3 hat`으로 시작한다(라이브러리가 이 값을 검사하고, 아니면 `No pi3hat detected`로 멈춘다. `pi3hat.cc:964`). `sudo pi3hat_tool --info`에 프로세서 세 줄(CAN1, CAN2, AUX)이 나온다. `--read-att`로 IMU 부호가 `docs/bringup_step0.md` 8절 표와 같다(바로 놓으면 롤 ±180°가 정상) |
| C2 | moteus 1대를 JC1에 연결 | 종단 저항을 꽂고 전원을 끈 상태에서 CANH–CANL 약 60 Ω. `tview`에서 값이 갱신된다 |
| C3 | moteus 6대 설정. 순서를 바꾸지 않는다(`docs/bringup_step0.md` 3절에 시간 초과 설정을 더한 것): 펌웨어 → ID → 보정(링크 달기 전) → 감속비 0.125 → 조립 → 부호 → 위치 한계 → 영점 → 시간 초과(`servo.default_timeout_s`, `servo.timeout_mode`, `servo.timeout_max_torque_Nm`) → 덤프 보관 | `docs/bringup_step0.md` 4절의 기록표(관절별 부호)가 채워지고, 위치 한계와 영점 값을 적어 두고, 덤프가 `wbr_hardware/config/moteus/`에 있다 |
| C4 | 통신 주기 측정(`docs/bringup_step0.md` 9절): JC1·JC4, IMU 포함. 대기 값은 라이브러리 기본값으로 잰다(2절). **여기서 제어 주기를 정한다** | 평균·99 %·최대 주기를 적는다. 1 kHz가 안 되면 500 / 200 Hz로 정하고 `WBR_TEST_RATE=<주기>`로 A0 → A1 → A4를 다시 한다(그 주기의 Python 단계 시험 보고서는 아직 없다) |
| C5 | `wbr_hardware` + `hardware_check.launch.py`. 활성화 순서는 4.1절 4번(영점 전에는 질의만) | 관절·IMU·전압·`home_state`가 ROS로 올라온다. `can_unknown_address_safe`를 확인한다. ROS를 거쳐 A3의 부호 시험을 실물로 반복한다 |
| C6 | Pi에서 전체 루프의 주기와 센서 → 토크 지연 측정. 지연은 전 주기 기록의 표본 시각과 `write()` 시각의 차로 잰다 | 주기는 B3의 후보 기준. 지연 8 ms 이하(`docs/bringup_step0.md` 9절의 후보) |
| C7 | 실물 안전 시험. 받침 위, 바퀴를 띄운 채 낮은 토크 한계로 | 다섯 경우 각각 관절이 정한 대로 풀리거나 감쇠한다: launch 종료(Ctrl+C), 프로세스 강제 종료(moteus 시간 초과), 제어기 비활성화, CAN 케이블 뽑기, 명령 0.5 s 끊김. 전원을 끊는 스위치에 손이 닿는다 |
| C8 | 받침 위에서 순서대로: 전원 직후 영점 → arm 조건 → 매달기(몸통을 앞으로 기울이면 바퀴가 앞으로 돈다) → 저게인 자립(테더) → 게인 올리기 → 주행. 모두 전체 제어기로 한다 | 1분 이상 자립(후보: 피치 rms와 위치 이동이 A5의 기준 안), 가벼운 밀기에서 복귀, 기록에 지속 진동이 없다 |

`wbr_hardware`의 코드는 C1 전에도 쓸 수 있다. 노트북에서 컴파일까지 되는지는 해 봐야 안다. 실행은 Pi에서만 된다.

### D. 그 뒤

Gazebo → Isaac Sim → 시작 자세와 스스로 일어서기 → MPC(`v5_mpc`) → 상위 기능(SLAM, 로봇팔).

## 7. 확인하지 못한 것, 정할 것

정할 것:

- moteus 시간 초과 동작: `servo.timeout_mode` 0(풀림) / 12(감쇠) / 15(단락 제동)와 `timeout_max_torque_Nm`(4.1절 5번).
- 상태 기계의 규칙: fault에서 나가는 방법, arm할 때의 `L_ref`(4.4절 3번). arm / disarm 수단(서비스, `cmd_key`).
- 새 IMU 표본이 없는 주기의 처리와 `wait_for_attitude`(4.2절 4번).
- 전원을 켤 때의 기준 자세 값(`MakeOutputNearest`에 넣을 값, `docs/bringup_step0.md` 7절의 "결정 필요").
- CAN ID(후보 1·2·3 / 4·5·6), moteus 위치 한계 후보, 무릎 완전 신장 보호.
- 6절의 "후보" 수치들(일치 시험 허용 오차, `cyclictest` 지연, 주기 지표, 계산 시간).
- 사용자 절차와 Python 제어기가 다른 곳: 토크를 끊는 각도가 절차에는 |θ| > 25°(가상 다리 각), 코드에는 몸통 피치·롤 34°(`fall_pitch`, `fall_roll`)다. 절차에 있는 "전원 투입 직후 토크 램프업"과 "하드웨어 비상정지"는 코드에 없다. 넣는다면 Python에 먼저 넣고 단계 시험을 다시 돌린 뒤 C++로 옮긴다.
- 시작 자세와 스스로 일어서기(나중).
- 3절의 `[수정]`·`[추가]` 항목을 그대로 갈지.

확인하지 못한 것:

- 모터 6개 + IMU 상세 읽기로 1 kHz가 나오는지. moteus가 있어야 잴 수 있다. (Ubuntu 24.04의 Pi 4B에서 pi3hat 라이브러리가 `/dev/mem`으로 도는 것은 2026-11-14 확인: 기본 커널과 Ubuntu Pro 실시간 커널 `6.8.0-2056-raspi-realtime` 모두 `pi3hat_tool --info` 통과.)
- 두 다리를 다른 프로세서에 나눈 배선(JC1·JC4)이 같은 프로세서(JC1·JC2)보다 실제로 나은지는 재 보지 않았다. 원리상 차이는 작다: 두 CAN 버스는 어느 쪽이든 동시에 돌고, 프로세서 1·2는 Pi의 같은 SPI1 선을 나눠 쓴다(CS만 다름). 비교가 필요하면 C4에서 오른쪽을 JC2로 옮겨 잰다.
- 지연 한계가 11 ms인지 12 ms인지(5.4절). 다시 재야 한다.
- 커밋된 `models/wheelbiped_v5`가 지금의 CAD 파일에서 만든 것과 같은지(6절 A1에서 확인).
- `mujoco_ros2_control`에서 물리 0.5 ms와 제어 1 kHz가 어떻게 맞물리는지, 플러그인이 모르는 이름의 상태 인터페이스를 xacro에 적으면 어떻게 되는지.
- Isaac Sim(11-12 사용자가 추가). Isaac Sim 6.1 문서에는 공식 확장 `isaacsim.ros2.control`이 있다(Humble·Jazzy 지원): Controller Manager를 Isaac Sim 안에서 띄우고, USD의 관절 구동 설정에서 인터페이스를 만든다(강성·감쇠가 없는 구동이면 effort). IMU는 `IsaacImuSensor`와 덧붙임 URDF로 표준 값 10개를 낸다. `update_rate`는 물리 스텝보다 빠를 수 없으므로 1 kHz면 물리도 1 kHz 이상이어야 한다. 그래서 Isaac Sim은 xacro의 `<hardware>` 플러그인 선택을 쓰지 않고, USD + 덧붙임 URDF + 같은 `controllers.yaml`로 붙는다. 확인하지 못한 것: 시스템 Jazzy로 빌드한 `wbr_controller` 플러그인이 Isaac Sim 안의 Controller Manager에 실리는지(ros2_control 패키지가 Isaac Sim과 함께 배포되므로 라이브러리가 맞는지 알 수 없다. 문서는 UR10과 표준 제어기로만 확인), 타이어 접촉과 IMU 축을 MuJoCo 모델과 맞추는 방법, 어느 컴퓨터에서 돌릴지(RTX 그래픽 카드 필요).
- Pi의 전원. pi3hat은 Pi에 5 V 2.5 A까지 준다고 판매처 설명에 적혀 있다(Pi 4의 권장은 3 A). 팬과 USB 장치까지 물리면 전압이 떨어져 CPU가 느려질 수 있으므로 B1에서 전원 부족 경고를 확인한다.
- 추가한 참조 저장소 다섯 개의 코드 내용(1절 (d)).
- 옵시디언 `Study/Calibration 단계/0단계.md`는 11-08 새벽의 사본이라 옛 내용이 남아 있다(JC1·JC3, CAD 이름 반대, `arm()`이 접힌 관절을 막지 못함). 기준은 저장소의 `docs/bringup_step0.md`다.
