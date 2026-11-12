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
| `https://github.com/DDTRobot/tita_hardware_ros2_control` | `tita_hardware_ros2_control` | 파일 없음 | 실기 launch·`controllers.yaml` 틀(`hw_bringup`), `read()`·`write()`에서 통신을 부르는 자리. Humble |
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
- DDT 코드와 pi3hat 인터페이스 세 개는 모두 Humble API다. Jazzy에서 달라진 것(`get_value` → `get_optional`, `get_state` → `get_lifecycle_state`, `on_export_*`, 스포너의 `--param-file`)은 `ros2_control_demos`의 `jazzy` 브랜치를 기준으로 맞춘다.
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

제어 주기 근거: Python 단계 시험과 게인이 모두 1 kHz 기준이다. upkie도 같은 구성(Pi 4 + pi3hat + moteus 6개)에서 스파인 1000 Hz, IMU 1000 Hz가 기본값이다(`spines/pi3hat_spine.cpp` 147·177행). 다만 upkie의 균형 제어는 200 Hz 에이전트에서 돌고, 스파인은 CAN 한 주기가 끝나기를 기다린다. 우리는 그 위에 `ros2_control`과 IMU 상세 읽기가 얹히므로 실측이 필요하다. 구조도에 있던 500 Hz는 TITA / DDT의 값이다.

## 3. 패키지 구조 (안)

사용자가 준 구조도에 11-12 점검 결과를 반영한 안이다. `[수정]`, `[추가]`는 그 점검의 제안이고 아직 확정이 아니다.

```
ros2_ws/src/
├─ wbr_description/                      로봇 모델 + hw_env 전환
│  ├─ urdf/wheelbiped_v5.urdf.xacro       models/wheelbiped_v5의 URDF에서 (build_v5가 생성, 손으로 안 고침)
│  ├─ urdf/ros2_control.xacro             hw_env = real | mujoco | gazebo | isaac, 관절별 CAN 버스·ID, IMU 센서
│  ├─ urdf/gazebo.xacro, worlds/flat.sdf  MuJoCo 다음 순서
│  ├─ isaac/                              [추가] Isaac Sim용 파일(USD 등). 연결 방식은 아직 정하지 않음 (7절)
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
│  │    lqi            ← controllers/lqi8/lqi8.py, law.py
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
   ├─ config/wbr_params_<주기>hz.yaml     [수정] export_params.py가 만드는 파일 하나 (게인표 포함)
   └─ config/fastdds_udp.xml              [추가] root로 돌 때의 DDS 설정

wheeled_biped/wbr/tools/                 ROS 패키지 아님, 아직 없음
├─ export_params.py                       PlanV5를 만든 뒤 최종 McuParams·LQI8 설정·표를 통째로 내보냄
└─ dump_golden.py                         Python 제어기의 입력·출력 기록 (C++ 일치 시험용)
```

C++로 옮기지 않는 것: `wbr/observers/pi3hat_ars.py`(pi3hat 펌웨어 필터 흉내, 시뮬 전용), 시뮬 하네스(`plan4_lqi8.py`, `plan_v5.py`의 모델·노이즈·지연 부분), `host_link.py`의 DIABLO 직렬 링크 흉내(타임아웃 규칙만 래퍼에 옮김).

## 4. 구조도에 없지만 실물에 필요한 것

1. **전원 직후 영점.** moteus는 전원을 켜면 관절각을 ±22.5° 안으로 접어 읽는다. 플러그인이 활성화될 때 hip·무릎에 `MakeOutputNearest`를 보내고 `home_state`(레지스터 0x00c)가 2인지 읽는다. 없으면 `arm()`이 항상 거부한다(`docs/bringup_step0.md` 7절).
2. **모터 상태 인터페이스.** `mode`, `fault`, 전압, 온도, `home_state`. moteus가 자체 시간 초과에 들어가면 `stop`으로 푼다.
3. **IMU 상세값과 표본 시각.** 제어기는 pi3hat 자세를 시작값으로만 쓰고 원시 자이로·가속도·바이어스로 자세를 구한다. 표준 IMU 값 10개 외에 바이어스 3개와 표본 시각이 필요하다(`request_attitude_detail`). 시뮬레이터에는 없으므로 제어기가 선택 사항으로 받는다.
4. **IMU 좌표 규약.** pi3hat은 z가 아래인 좌표계(ARS)로 자세를 주고 MuJoCo·Gazebo는 z가 위다. 플러그인에서 ROS 규약으로 바꿔 내보내고, 제어기는 `R_base_to_imu`(diag(−1, −1, 1), URDF `imu_fix`)만 쓴다.
5. **root와 DDS.** pi3hat 라이브러리는 `/dev/mem`을 직접 열어 root가 필요하다(`pi3hat.cc:361`). root 노드와 일반 사용자 노드는 기본 공유 메모리 전송으로 통신이 안 될 수 있다.
6. **높이·롤·피치 명령.** DDT는 `command/cmd_twist`, `command/cmd_pose`, `command/cmd_key` 세 토픽을 쓴다. 우리 제어기도 높이(`up`), 롤, 피치 명령을 받는다.
7. **받침 위 점검 실행.** 제어기 없이 부호·영점·주기를 확인하는 launch.
8. **배터리 전압.** moteus 전압 레지스터나 power dist(CAN ID 32, JC3)에서 읽어 상태에 싣는다.

## 5. 옮길 때 조심할 것

- **파라미터 출처가 네 곳이다.** `models/wheelbiped_v5/description.json`(`plan_overrides`), `PlanV5Config`(서스펜션·감속), `Plan4Config`(속도 창·마찰 보상·요 적분 한계), LQI8 가중치(`plan4_lqi8.py`의 `lqi_kwargs` + `PlanV5._lqi_design`). 손으로 옮기지 말고 `PlanV5`를 만든 뒤의 `mcu.p`와 `law.ctrl`에서 내보낸다.
- **표로 내보낼 것.** LQI8 게인 K(다리 길이 격자), `gM(L0)`, `theta_eq(L0)`, 정적 다리 힘의 기울기(`SerialPlant.static_leg_force`), `Tp_ff`. 게인은 주기(`Ts`)마다 다르다.
- **스텝 수는 정수로 내보낸다.** 속도 창 5 ms는 500 Hz에서 2.5스텝인데 Python `round`는 2, C++ `std::round`는 3이다. `_freeze_hold_steps`도 같다.
- **`ros2_control` 한 주기 지연.** 순서가 읽기 → 제어 → 쓰기인데 pi3hat은 보내기와 받기가 한 번의 `Cycle`이다. 상태가 한 주기 늦는다. 시뮬레이션 한계는 12 ms(1 kHz, 속도 창 5 ms), 시험은 10 ms로 돌린다.
- **위치 적분은 고정 `dt`, 자세 필터는 표본 시각 간격.** Python과 같게 둔다(`docs/bringup_step0.md` 9절).
- **MJCF는 지금 그대로 공식 `mujoco_ros2_control`에 맞는다**(문서 기준, 실행은 안 해 봄): `general` 구동기는 effort를 그대로 받고, 센서 이름 `imu_quat` / `imu_gyro` / `imu_accel`은 `mujoco_sensor_name = imu`로 잡힌다. 이 패키지는 자기 `ros2_control_node`를 써야 한다.
- **MJCF의 `stand` 키프레임은 다리를 편 자세(500 mm)다.** `arm()` 허용 범위(326~485 mm) 밖이다. 시작 자세는 나중에 정하기로 했다.
- **ROS 시뮬은 연결 확인용이다.** 노이즈·지연·pi3hat 필터 흉내는 Python 하네스에만 있다. 제어기의 합격 판정은 Python 단계 시험(`v5_lqi/TEST`)과 C++ 일치 시험으로 한다.

## 6. 작업 순서

1. `wbr/tools/export_params.py`, `dump_golden.py` — Python만으로 검증된다. C++의 기준이 된다.
2. `wbr_description` — xacro, `ros2_control` 태그, MuJoCo에 모델이 뜨는지.
3. `wbr_controller/core` + `test_equivalence` — 2번과 따로 진행할 수 있다.
4. `wbr_controller` 래퍼 + `wbr_bringup` — MuJoCo에서 1 kHz로 균형. 시작 자세는 아직 정하지 않았으므로, 시험할 때만 Python `reset()`과 같은 기립 자세(다리 460 mm)에서 일시정지로 시작해 제어기가 켜진 뒤 푼다.
5. `wbr_msgs`, `wbr_teleop`, 기록.
6. Pi 4: Ubuntu 24.04 + 실시간 커널, `cyclictest`, 가짜 하드웨어(`mock_components`)로 1 kHz.
7. pi3hat + moteus가 온 뒤: `docs/bringup_step0.md` 9절의 주기 측정 → `wbr_hardware` → 받침 위 토크 0 → `arm()`.
8. Gazebo, Isaac Sim.

## 7. 확인하지 못한 것, 정할 것

- Ubuntu 24.04의 Pi 4B에서 pi3hat 라이브러리가 도는지, 모터 6개 + IMU 상세 읽기로 1 kHz가 나오는지. pi3hat과 moteus가 있어야 잴 수 있다.
- JC1과 JC2는 pi3hat의 같은 프로세서가 맡는다. 주기에 영향이 있으면 오른쪽만 JC3으로 옮겨 비교한다.
- CAN ID(후보 1·2·3 / 4·5·6), moteus 위치 한계 후보, 무릎 완전 신장 보호.
- 시작 자세와 스스로 일어서기(나중).
- Isaac Sim(11-12 사용자가 추가): `ros2_control`에 붙이는 방식, URDF에서 USD로 바꾸는 방법, 타이어 접촉과 IMU를 MuJoCo 모델과 맞추는 방법을 아직 조사하지 않았다. RTX 그래픽 카드가 있는 컴퓨터에서만 돈다.
- 3절의 `[수정]`·`[추가]` 항목을 그대로 갈지.
