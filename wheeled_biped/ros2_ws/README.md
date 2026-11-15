# ros2_ws — v5 로봇의 ROS 2 작업 공간 (Jazzy)

**`src/`에 패키지가 아직 없다(2026-11-15).** 이 문서는 정한 것, 패키지 구조 안, 실물에서 지킬 것, 작업 순서를 적는다.

- 실물: v5 로봇(Raspberry Pi 4B + pi3hat + moteus r4.11, AKE60-8 × 6). ROS 2 Jazzy 위에서 TITA의 `ros2_control` 패키지 구조를 따른다(11-08 결정).
- "제안"·"후보"라고 적은 것은 아직 정하지 않은 것이다.
- 실물 설정 순서·부호·영점·IMU 시험은 `docs/bringup_step0.md`에 있다. 이 문서가 `bringup_step0.md` N절이라고 적은 곳은 그 문서를 가리킨다.
- 이 문서는 11-15에 줄였다. 근거(`파일:줄`)와 자세한 설명이 든 전문: `git show 39e448f:wheeled_biped/ros2_ws/README.md`.

## 0. 두 컴퓨터로 작업하기

- 브랜치 `ros2-v5` 하나를 Windows PC, Ubuntu 노트북, Pi 4가 같이 쓴다. 동시에 쓰지 않는다(11-12 사용자).
- 시작할 때 `git pull`, 끝낼 때 커밋하고 `git push`. 올리지 않고 다른 컴퓨터로 넘어가면 충돌이 난다.
- 다른 컴퓨터가 알아야 할 결정은 해당 README에 날짜와 함께 적는다.

## 1. 노트북(Ubuntu 24.04 + ROS 2 Jazzy) 준비

(a) Python 코어. ROS의 Python 패키지가 보이게 `--system-site-packages`로 만든다.

```bash
python3 -m venv --system-site-packages ~/wbr_venv
source ~/wbr_venv/bin/activate
pip install -e wheeled_biped        # 저장소 최상위에서
cd wheeled_biped
python -m unittest discover -s tests
```

(b) ROS 패키지.

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-mujoco-ros2-control ros-jazzy-mujoco-ros2-control-demos \
  ros-jazzy-generate-parameter-library ros-jazzy-xacro python3-colcon-common-extensions
ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py     # MuJoCo 창이 뜨면 설치 확인 끝
```

(c) 빌드. `colcon`은 venv를 켜지 않은 터미널에서 돌린다.

```bash
source /opt/ros/jazzy/setup.bash
cd wheeled_biped/ros2_ws
colcon build --symlink-install && source install/setup.bash
```

(d) 참조 저장소. `references/` 아래 서브모듈이다. 목록·고정 커밋·받는 법은 `references/README.md`에 있다.

- 11-12에 더한 다섯 개(`tita_hardware_ros2_control`, `pi3hat_hw_bartlomiej`, `pi3hat_robotic_systems`, `pi3hat_hw_glevine`, `ros2_control_demos`)는 README만 확인했고 코드는 아직 읽지 않았다.
- **GPL-3.0이거나 라이선스 파일이 없는 저장소는 구조만 참고하고 코드를 복사하지 않는다.** 복사해도 되는 것은 MIT와 Apache-2.0이고 출처 표기를 남긴다. DDT 코드는 파일 머리말과 `package.xml`이 둘 다 Apache-2.0일 때만 복사한다.
- upkie의 `Pi3HatInterface.cpp`를 그대로 쓰지 않는다(4.2절 3번).
- DDT 코드와 pi3hat 인터페이스 세 개는 Humble API다(Ubuntu 22.04). Jazzy에서 바뀐 것:
  - 제어기 파라미터는 스포너의 `--param-file`로 넘긴다.
  - `get_state()` → `get_lifecycle_state()`, `init()` → `init(ControllerInterfaceParams)`.
  - URDF는 `robot_description` 토픽으로 받는다. `<ros2_control>` 태그의 관절은 모두 URDF에 있어야 한다.
  - 핸들의 `get_value()` 대신 `get_optional()`을 쓰고 `set_value()`의 반환을 확인한다.
  - 하드웨어 인터페이스는 `<ros2_control>` 태그에서 자동으로 만들어진다.
- 쓰지 않는 것: DDT의 MuJoCo 브리지(공식 `mujoco_ros2_control`로 대신함), `TITA-SDK-ROS2`·`TITA_CAN_Inerface`, 강화학습 저장소.

## 2. 정해진 것

| 항목 | 결정 | 언제 |
|---|---|---|
| ROS 배포판 | Jazzy (Ubuntu 24.04) | 11-08 사용자 |
| 패키지 구조 | TITA `ros2_control` 구조, 균형 제어기 자리만 `wbr` 이식 | 11-08 사용자 |
| 시뮬레이터 | 공식 `mujoco_ros2_control` 먼저, 그 뒤 Gazebo, Isaac Sim | 11-08, 11-12 사용자 |
| 제어기 출력 | 토크(effort)만. 실물과 모든 시뮬에서 같게 | 11-08 |
| CAN 버스 | 왼쪽 다리 JC1, 오른쪽 다리 **JC4**, power dist JC3. ID 후보 1·2·3 / 4·5·6 (`--pi3hat-cfg "1=1,2,3;4=4,5,6"`) | 11-14 사용자 |
| pi3hat 대기 값 | 라이브러리(v1.1.0) 기본값. 설정 파일에서 읽게 한다(제안) | 11-14 사용자 |
| 범위 | **주행 중 균형 잡기만.** 시작 자세·스스로 일어서기는 나중 | 11-12 사용자 |
| 상태 기계 | `idle → balance → fault`, 받침 위에서 세운 뒤 arm (4.4절 3번) | 11-12 |
| 제어 주기 | **목표 1 kHz**, 파라미터로 둔다. Pi에서 안 나오면 500 / 200 Hz (6절 C4) | 11-12, 실측 뒤 확정 |
| 균형 제어기 | LQI8(`v5_lqi`) **전체 제어기 하나**(상태 8개, 바깥 루프 모두 켬). MPC는 나중에 같은 자리 | 11-07, 11-12 사용자 |

## 3. 패키지 구조 (안)

`[추가]`·`[수정]`은 사용자 구조도에 더한 제안이고 확정이 아니다.

```
ros2_ws/src/
├─ wbr_description/     URDF xacro(build_v5가 만든 것), ros2_control.xacro(hw_env = real | mujoco | gazebo),
│                       MJCF, [추가] 기립 키프레임·몸통 고정 변형(6절 A2), [추가] isaac/
├─ wbr_hardware/        실물 플러그인. 출발점은 mjbots pi3hat_moteus_transport.h + moteus.h
│                       [수정] third_party 복사 대신 FetchContent(커밋 고정, moteus를 pi3hat보다 먼저 선언)
│                       [추가] config/moteus/*.cfg (드라이버 6개 설정과 덤프)
├─ wbr_controller/
│  ├─ core/             [추가] ROS 없는 C++ 라이브러리(Eigen만): estimator, serial_leg, lqi, outer_loops,
│  │                    state_machine, gain_table, balance_law.hpp(LQI8·MPC를 끼우는 자리)
│  ├─ src/              얇은 래퍼: 인터페이스, 명령 토픽·타임아웃, arm / disarm, 상태 발행, 끌 때 토크 0
│  └─ test/test_equivalence.cpp   core만 링크해 Python 기준 기록과 비교
├─ wbr_msgs/            BalanceStatus.msg (상태, fault·arm 거부 사유, v, 피치, L0, 전압 …)
├─ wbr_teleop/          키보드(DDT keyboard_controller 수정), 우선순위 낮음
└─ wbr_bringup/         robot.launch.py(hw_env 하나로 전환), [추가] hardware_check.launch.py, controllers.yaml,
                        wbr_params_<주기>hz_<이름>.yaml(export_params.py가 만듦), [추가] fastdds_udp.xml

wheeled_biped/wbr/tools/   아직 없음: export_params.py(5.1절), dump_golden.py(5.3절)
```

- `core`의 출처: `diablo_arch/mcu.py`, `observers/body_attitude.py`, `base_orientation.py`, `lqi8/serial_leg.py`, `lqi8.py`, `law.py`.
- C++로 옮기지 않는 것: `pi3hat_ars.py`(펌웨어 필터 흉내), 시뮬 하네스(`closed_loop.py`, `plan_v5.py`의 모델·노이즈·지연). `host_link.py`는 규칙만 래퍼로 옮긴다(4.4절 2번).

## 4. 실물에 필요한 것

### 4.1 moteus 명령과 질의

1. **토크 명령의 형태.** C++ 기본 `Format`은 토크를 보내지 않고 "위치 0" 명령을 낸다. `Command`는 `position = NaN`, `velocity = 0`, `feedforward_torque = τ`, `kp_scale = 0`, `kd_scale = 0`, `maximum_torque = τ_max`, `watchdog_timeout = 유한한 양수`로 두고, `Format`에서 이 필드를 모두 켠다.
2. **`watchdog_timeout`에 NaN을 넣지 않는다.** NaN은 "시간 초과 없음"이다. 몇 주기쯤의 유한한 값을 넣는다.
3. **질의에 `home_state`를 켠다**(`query_format.home_state = kInt8`). 켜지 않으면 0으로 남아 `arm()`이 계속 거부한다.
4. **영점 전에 위치 모드 명령을 보내지 않는다**(토크 0 명령도 위치 모드다). 전원 직후의 접힌 각은 위치 한계 밖일 수 있고 그러면 fault 39가 난다. 순서: 질의만 → hip·무릎에 `MakeOutputNearest` → `home_state`가 2인지 확인 → 토크 명령.
5. **시간 초과는 토크를 끊지 않는다(결정 필요).** 기본 `servo.timeout_mode`는 12(감쇠, 최대 5 N·m), 0은 풀림, 15는 단락 제동이다. `stop`을 보내야 풀린다. 프로그램이 죽었을 때 다리가 버틸지 풀릴지를 정한다(6절 C3).
6. **위치 한계는 토크만 쓰는 명령에도 걸린다.** 한계를 0.02 회전 넘으면 바깥으로 미는 토크가 0이 된다. 바퀴는 한계를 NaN으로 둔다.
7. **상태 인터페이스.** `mode`, `fault`, 전압, 온도, `home_state`를 제어기까지 올린다.

### 4.2 pi3hat

**보드 펌웨어는 v1.1.0이다**(11-16 사용자 확인). `sudo pi3hat_tool --info`에 나온 해시가 `references/pi3hat`의 커밋 `12ab9af`와 같다. 그래서 보드의 STM32 세 개에서 도는 코드가 `references/pi3hat/fw/`이고, 펌웨어를 올릴 필요가 없다. 응답 없는 CAN 프레임에 프로세서가 멈추는 옛 펌웨어 문제도 없다(`pi3hat.h:290-295`).

1. **root로 돌고 한 번에 한 프로그램만 쓴다.** `ros2_control_node`가 떠 있으면 `tview`, `pi3hat_tool`, `moteus_tool`을 못 쓴다.
2. **넣을 값.** `attitude_rate_hz = 1000`, `mounting_deg = 0, 0, 0`. `opts.default_input.request_attitude_detail = true`를 항상 켠다(11-14 사용자, C++에서만 켤 수 있다).
3. **바이어스는 조용히 0이 된다.** 상세 요청 없이 읽으면 바이어스 칸이 0으로 남고, NaN이 아니라 가드가 잡지 못한다.
4. **표본 시각은 플러그인이 찍는다.** 새 표본(`attitude_present`)일 때만 단조 시계로 찍는다. 새 표본이 없으면 갱신하지 않는다(50 ms 넘게 이어지면 FAULT). `wait_for_attitude`를 쓸지는 정한다.
5. **대기 값은 라이브러리 기본값**(11-14 사용자). 응답이 하나라도 빠진 주기는 1 ms를 넘는다.
6. **응답은 출처 ID로 짝짓는다.** 지난 주기의 늦은 응답이 섞여 온다. 첫 주기는 6대 중 1대의 응답이 빠진다.
7. **응답이 빠진 관절**은 NaN으로 두거나 표본 시각을 갱신하지 않는다. 지난 값을 새 값처럼 내보내지 않는다.
8. **응답할 장치가 없는 버스로 보내지 않는다**(전원이 꺼진 moteus 포함). 시작할 때 `can_unknown_address_safe`를 검사한다.
9. **스레드.** `write()`에서 `Cycle`을 시작하고 다음 `read()`에서 거둔다.

### 4.3 플러그인과 제어기 사이의 규약

| 인터페이스 | 값 | 실물에서 만드는 법 |
|---|---|---|
| 관절 `effort` 명령 | N·m, 로봇 왼쪽에서 보아 반시계 + | moteus 피드포워드 토크 |
| 관절 `position`, `velocity`, `effort` | rad, rad/s, N·m | 회전 → rad |
| 관절 `mode`, `fault`, `home_state`, 전압, 온도 | moteus 값 | 질의 응답(시뮬에는 없음) |
| IMU `orientation` | IMU 축 → 세계(z가 위) | pi3hat 쿼터니언(z가 아래)을 x축 180° 회전 |
| IMU `angular_velocity` | 보드의 `rate`, IMU 축 | `rate_dps` × π/180. **바이어스를 또 빼지 않는다** |
| IMU `linear_acceleration` | 원시 가속도(중력 포함) | `accel_mps2 + 9.81 × IMU에서 본 위 방향` |
| IMU 자이로 바이어스 | 보드의 `bias`, IMU 축 | `bias_dps` × π/180 (시뮬은 0) |
| 표본 시각 | 단조 시계 [s] | 4.2절 4번 (시뮬은 `update()`의 시각) |

- hat이 위인 정지 상태에서 위 방향은 (0, 0, +1)이다.
- IMU는 IMU 축 그대로 내보낸다. 몸통 좌표로 바꾸는 일(`R_base_to_imu` = diag(−1, −1, 1))은 제어기가 한다.
- Jazzy에서는 요청한 상태 인터페이스가 하나라도 없으면 제어기 활성화가 실패한다. 요청 목록을 파라미터로 바꾼다(실물: 바이어스·표본 시각·`home_state` 포함, 시뮬: 표준 값만).
- 실물에서 표본 시각, 가속도, 바이어스는 꼭 있어야 한다. 없으면 가드가 꺼지거나 FAULT다.

### 4.4 제어기 쪽

1. **명령.** DDT의 세 토픽 `command/cmd_twist`, `command/cmd_pose`, `command/cmd_key`. 전진 ±1.0 m/s, 요 ±2.0 rad/s, 높이 `L_stand + 0.06 × up`, 롤·피치 ±0.3 rad로 자른다.
2. **명령이 0.5 s 넘게 끊기면** 전진·요는 0이 되고 균형은 계속한다. 유한하지 않은 값이 든 명령은 버린다.
3. **arm과 상태(정해야 함).** 제안: 거부 사유를 돌려주는 서비스(`std_srvs/Trigger`)와 `cmd_key`. Python의 동작:
   - 실기 진입점은 `armed=False`로 시작한다(하네스는 무장한 채 시작).
   - `arm()`은 측정값 없음, 피치 3° 이상, hip·무릎 `home_state` ≠ 2, 다리 길이 326~485 mm 밖, 다리 각 ±20° 밖이면 거부한다.
   - FAULT가 나도 `armed`는 그대로다. `arm()`이 fault를 지우고 바로 균형으로 돌아간다.
   - 무장 전에도 `L_ref`는 목표(460 mm)로 간다. 다른 길이에서 arm하면 다리 길이 PD에 계단 입력이 든다.
   - C++에서 다르게 하려면 Python을 먼저 바꾸고 단계 시험을 다시 돌린다.
4. **FAULT와 안전 상태.** 피치·롤 34° 초과, NaN 표본 5주기 초과, 오래된 표본 50 ms 초과, 유한하지 않은 균형 법칙 입력 5주기 초과나 토크 명령이면 FAULT이고 모든 관절 토크 0이다.
5. **제어기를 끌 때 토크 0을 쓴다**(`on_deactivate`). ros2_control은 마지막 명령 값을 지우지 않는다. 플러그인은 effort를 잡은 제어기가 없어지면 `stop`을 보낸다.
6. **꺼질 때와 죽었을 때.** Ctrl+C로 끄면 플러그인이 모든 관절에 `stop`을 보낸다. 프로그램이 죽으면 moteus의 시간 초과가 맡는다(4.1절 5번).
7. **전 주기 기록.** 제어에 쓰인 모든 신호를 제어 주기 그대로, 순번을 붙여 실시간이 아닌 스레드에서 기록한다.
8. **root와 DDS.** root 노드와 일반 사용자 노드는 FastDDS 공유 메모리로 통신이 안 된다. 양쪽에 같은 UDP 전용 프로필을 준다.

## 5. 옮길 때 조심할 것

### 5.1 파라미터와 표

- **손으로 옮기지 않는다.** 값의 출처가 여러 곳이다: `description.json`, `plant_params.json`, `PlanV5Config`, `ClosedLoopConfig`, `McuParams` 기본값, LQI8 가중치.
- **`PlanV5`를 만든 직후, 한 스텝도 돌리기 전의 객체에서 모두 내보낸다**(`plan.mcu.p`, `plan.mcu.attitude`, `plan.law`, `plan.law.ctrl`, `plan.leg.g`, `plan.plant`, `plan.link.p`).
- **코드에 박힌 상수는 내보내기로 안 나온다.** 소스에서 옮기고 목록으로 관리한다(요 오프셋 학습의 0.1 s·0.1 rad/s, 기준 제한 0.3 rad, 마찰 보상의 `2.0 × tanh`, 쿼터니언 허용 0.1, `1e-9` 문턱).
- **표.** 다리 길이 격자 0.312~0.497 m, 5 mm, 38점. 게인 K(2 × 8)는 원소마다 선형 보간하고 끝에서 자른다. 게인은 주기마다 다르다.
- **스텝 수는 Python이 계산한 정수를 내보낸다.** 5 ms 창은 500 Hz에서 2.5스텝인데 Python `round`는 2, C++ `std::round`는 3이다. 200 Hz에서는 창이 꺼진다.
- **단계 시험은 커밋된 모델이 아니라 CAD 파일에서 그 자리에서 만든 모델을 쓴다.** 내보낼 때 두 경로의 값이 같은지 비교한다(6절 A1).
- **실물 질량을 잰 뒤에도 yaml을 손으로 고치지 않는다.** `robot_cad_v5/calc/mass_properties.json`을 맞추고 `python -m wbr.models.build_v5` → 내보내기 순서로 다시 만든다.
- **저게인 세트가 필요하다.** 실물의 첫 자립은 낮은 게인으로 하고 단계적으로 올린다.

### 5.2 숨은 상태와 순서

- **C++가 그대로 가져야 할 상태**는 `DiabloMcu.reset()`, `BodyAttitude`, `LQI8`의 멤버 전부다(적분기, 램프, 저역 통과, 위치 이력, 표본 시각, 카운터).
- **순서.** 추정이 명령 적용보다 먼저 돈다. 요 오프셋 학습은 전 주기의 요 기준을, 자세 필터는 전 표본의 hip 가속도를 쓴다.
- **표본 시각 간격**은 자세 필터, 요 오프셋 학습, hip 속도 미분에 쓴다. **위치 적분은 고정 `dt`**다.
- **신호 경로.** 피치·롤 각은 자세 필터에서 나온다. 요 속도는 원시 z 자이로(`rate − bias`)에서 배운 오프셋을 뺀 값이다. 보드의 쿼터니언은 필터 시작값과 매 주기 검사에만 쓴다.
- **`BaseOrientation`의 기본 `R_base_to_imu`(단위 행렬)를 복사하지 않는다.** v5는 diag(−1, −1, 1)이다.

### 5.3 기준 기록과 일치 시험

- **`core`는 Python `obs`와 같은 입력을 받는다(제안).** ROS 규약과의 차이(쿼터니언 순서, x축 180° 회전)는 래퍼가 바꾼다.
- **기준 기록에 넣을 것.** 스텝마다 `mcu.tick`에 들어간 `obs`, 실제로 적용된 명령, 관절 토크 6개, 내부 상태, reset·arm·disarm 스텝 번호.
- **arm 경로와 FAULT가 든 기록을 따로 만든다.** 보통의 기록에는 `arm()`이 나오지 않는다.
- **토크만 비교하지 않는다.** 내부 상태도 스텝마다 비교한다.
- **명령 타임아웃은 `core` 밖(래퍼)에서 시험한다.**
- **기준 기록과 파라미터 파일은 같은 실행에서 만들어 같이 커밋한다.** 날짜, 커밋, 패키지 버전을 적는다.

### 5.4 지연과 시뮬

- **한 주기 지연.** `write()`에서 시작한 `Cycle`의 응답을 다음 `read()`가 쓴다.
- **지연 한계는 11 ms로 본다**(1 kHz, 속도 창 5 ms, 시험은 10 ms). 12 ms라는 기록도 있어 다시 재야 한다.
- **MJCF의 `stand` 키프레임은 다리를 편 자세(500 mm)라 `arm()` 범위 밖이다.**
- **ROS 시뮬은 연결 확인용이다.** 합격 판정은 Python 단계 시험과 C++ 일치 시험으로 한다.

## 6. 작업 순서 (2026-11-13)

단계마다 통과 조건이 있고, 통과하기 전에는 다음으로 가지 않는다. 부호 확인은 시뮬(A3)과 실물(C5)에서 두 번 한다.

### A. 노트북 (실물 없이)

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| A0 | 기준 확인: `python v5_lqi/TEST/run_stages.py --stage 4 --keep-going --out <임시 폴더>` (약 5분) | 넘어짐과 무릎 꺾임이 없다. 판정은 "실패"로 나오는 것이 지금 상태다(아래) |
| A1 | `export_params.py`, `dump_golden.py` | 내보낸 파일만으로 다시 계산한 토크가 Python과 같다. 커밋된 모델과 단계 시험 경로의 값이 같다 |
| A2 | `wbr_description`. `build_v5`가 기립 키프레임(다리 460 mm)과 몸통 고정 변형을 만든다 | `colcon build` 통과. MuJoCo 창에 모델이 뜨고 `joint_states`와 IMU 토픽이 나온다 |
| A3 | ROS 시뮬에서 부호 시험(관절마다 + 토크, 모델을 기울여 IMU 읽기) | 방향이 `bringup_step0.md` 4절·8절 표와 같다 |
| A4 | `core` + `test_equivalence`. serial_leg → gain_table·lqi → estimator → outer_loops → state_machine | 모든 스텝에서 토크와 내부 상태가 Python과 일치(후보: 1e-6 이하) |
| A5 | 래퍼 + `wbr_bringup`, 처음부터 전체 제어기 | MuJoCo 1 kHz에서 정지 60 s 피치 rms ≤ 0.5°, 이동 ≤ 5 cm. 속도 스텝 정상 오차 ≤ 0.05 m/s |
| A6 | 안전 동작 시험(실제 시각) | 명령 0.5 s 끊김, FAULT, 제어기 비활성화, Ctrl+C, 강제 종료에서 정한 대로 된다 |
| A7 | `wbr_msgs`, `wbr_teleop`, 기록 | 키보드로 움직인다. 기록의 순번에 빈칸이 없다 |
| A8 | 계산 시간 벤치(ROS 없이) | 한 틱의 최대와 99 % 값을 적어 둔다 |

- A0의 지금 상태: 시드 0에서 2-3, 2-9, 2-10(다리 토크 rms), 2-12(방향·옆 이동·롤), 2-13(다리 토크 피크·롤)이 기준을 넘는다. 11-15 노트북(MuJoCo 4.0.1)에서는 2-8(롤)도 넘었다. 경계 값은 MuJoCo 버전에 따라 달라진다.
- A5의 시작(제안): launch → 제어기 active → `~/reset_world`로 기립 키프레임 → 바로 arm. 밀기 시험은 Python 단계 시험의 판정을 쓴다.
- 래퍼의 틀은 `ros2_control_demos`, `rl_controller`, `template_ros2_controller`를 참고한다. 이들에는 안전 검사와 명령 타임아웃이 없으므로 4.4절을 반드시 넣는다.

### B. Pi 4 (pi3hat 없이)

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| B1 | Ubuntu 24.04 Server, 실시간 커널, `isolcpus`, Jazzy, 시간 동기 | `cat /sys/kernel/realtime`이 1. 전원 부족 경고가 없다 |
| B2 | `cyclictest`로 1 kHz 타이머 지터 | 최대 지연을 적는다(후보: 200 µs 이하) |
| B3 | Pi에서 빌드. A8 벤치와 가짜 하드웨어 + `wbr_controller` 1 kHz | 한 틱 최대 300 µs 이하(후보). 60 s 주기의 평균·99 %·최대를 적는다 |

- 실시간 커널은 Ubuntu Pro(`sudo pro enable realtime-kernel --variant=raspi`)로 얻는다. 11-14에 Pi 4B에서 이 커널(`6.8.0-2056-raspi-realtime`)과 기본 커널 모두 `pi3hat_tool --info`가 통과했다.
- 모터 통신을 포함한 주기는 C4·C6에서 잰다.

### C. pi3hat과 moteus가 온 뒤

| 번호 | 할 일 | 통과 조건 |
|---|---|---|
| C1 | pi3hat 단독 점검 | `sudo pi3hat_tool --info`에 프로세서 세 줄. `--read-att`의 IMU 부호가 `bringup_step0.md` 8절과 같다 |
| C2 | moteus 1대를 JC1에 연결 | CANH–CANL 약 60 Ω. `tview`에서 값이 갱신된다 |
| C3 | moteus 6대 설정. 순서를 바꾸지 않는다: 펌웨어 → ID → 보정 → 감속비 0.125 → 조립 → 부호 → 위치 한계 → 영점 → 시간 초과 → 덤프 | 부호 기록표, 위치 한계, 영점 값, 덤프가 남는다 |
| C4 | 통신 주기 측정(JC1·JC4, IMU 포함). **여기서 제어 주기를 정한다** | 1 kHz가 안 되면 500 / 200 Hz로 정하고 `WBR_TEST_RATE`로 A0 → A1 → A4를 다시 한다 |
| C5 | `wbr_hardware` + `hardware_check.launch.py` (4.1절 4번의 순서) | 관절·IMU·전압·`home_state`가 올라온다. A3의 부호 시험을 실물로 반복한다 |
| C6 | 전체 루프의 주기와 센서 → 토크 지연 측정 | 지연 8 ms 이하(후보) |
| C7 | 실물 안전 시험(받침 위, 바퀴를 띄우고 낮은 토크 한계) | Ctrl+C, 강제 종료, 제어기 비활성화, CAN 케이블 뽑기, 명령 끊김에서 정한 대로 된다 |
| C8 | 영점 → arm 조건 → 매달기 → 저게인 자립(테더) → 게인 올리기 → 주행 | 1분 이상 자립, 가벼운 밀기에서 복귀, 지속 진동 없음 |

### D. 그 뒤

Gazebo → Isaac Sim → 시작 자세와 스스로 일어서기 → MPC(`v5_mpc`) → 상위 기능(SLAM, 로봇팔).

## 7. 확인하지 못한 것, 정할 것

정할 것:

- moteus 시간 초과 동작(4.1절 5번).
- 상태 기계: fault에서 나가는 방법, arm할 때의 `L_ref`, arm / disarm 수단(4.4절 3번).
- 새 IMU 표본이 없는 주기의 처리(4.2절 4번).
- 전원을 켤 때의 기준 자세 값(`MakeOutputNearest`에 넣을 값).
- CAN ID, moteus 위치 한계, 무릎 완전 신장 보호.
- 6절의 "후보" 수치와 3절의 `[수정]`·`[추가]` 항목.
- 사용자 절차와 Python이 다른 곳: 토크를 끊는 각도(절차 25°, 코드 34°), 절차에만 있는 "토크 램프업"과 "하드웨어 비상정지". 넣는다면 Python에 먼저 넣는다.

확인하지 못한 것:

- 모터 6개 + IMU 상세 읽기로 1 kHz가 나오는지.
- JC1·JC4 배선이 JC1·JC2보다 실제로 나은지.
- 지연 한계가 11 ms인지 12 ms인지.
- 커밋된 `models/wheelbiped_v5`가 지금 CAD 파일에서 만든 것과 같은지(A1).
- `mujoco_ros2_control`에서 물리 0.5 ms와 제어 1 kHz가 맞물리는 방식.
- Isaac Sim에 `wbr_controller` 플러그인이 실리는지.
- Pi의 전원(pi3hat은 5 V 2.5 A까지 준다).
- 11-12에 더한 참조 저장소 다섯 개의 코드 내용.
