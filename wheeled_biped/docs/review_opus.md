# Plan 1·Plan 2 제어 코드와 ROS 2 패키지 적대적 리뷰 (review_opus)

작성일 2026-10-22. 대상: `wheeled_biped/` (Python 패키지 `wbr`, `ros2_ws/src`). 프로젝트 파일은 수정하지 않았다(이 문서만 작성).

## 0. 범위·방법·표기

- **비교 원본**: upkie C++ 관측기·제어기(`references/upkie/upkie/cpp/{observers,controllers}`), upkie Python(`upkie/controllers/mpc_balancer.py`, `upkie/envs/upkie_gyropod.py`, `upkie/envs/backends/spine_backend.py`, `upkie/model/model.py`, `BulletInterface.cpp`), DIABLO ROS 2 SDK(`diablo_ctrl.cpp`, `osdk_movement.cpp`, `osdk_telemetry.cpp`, `onboard_sdk_uart_protocol.h`, `diablo_teleop/teleop.py`, `diablo_interfaces`), 설계보고서 계열(`balancing_robot_design/sim6/plant6.py`, `controllers6.py`, `제어기_비교보고서.md` §8, `설계보고서.md` §4.5).
- **실행한 검증**: `python -m unittest discover -s tests` 13개 통과. 그 밖에 스크래치 스크립트로 아래 수치를 직접 재현했다(바퀴 부호·요 방향, 접지 판정 시계열, LQI6 게인·폐루프 고유값·재설계 시간, 몸체 관성 축, 추정기 대 MuJoCo 진실값, 폐루프 구속 잔차, 링크 단절 aliasing, 모드 패킷, 노이즈 모델, ROS 경로의 요 부호).
- **표기**: `[실측]` 스크립트로 재현함 · `[코드]` 코드 정독으로 확인함 · `[미검증]` 확인하지 못함(ROS 2 런타임 없음 등).
- **심각도**: critical = 그대로 쓰면 로봇이 넘어지거나 위험 / major = 문서화된 기능이 틀리게 동작하거나 결론·수치를 왜곡, 실기에서 사고 가능 / minor = 제한된 상황의 오동작·정확도 손실 / nit = 정리·문서.

critical로 분류한 항목은 없다. 시뮬레이션에서 세 플랜이 서는 것과 Plan 2 제어 법칙·부호가 맞는 것은 확인했다. 문제는 주로 **실기 이식 경로, 시험 하네스의 타당성, ROS 실행 경로, DIABLO 의미론을 바퀴 보조륜 없는 lite 로봇에 그대로 쓴 데서 오는 안전 문제**에 있다.

### 요약 표

| ID | 심각도 | 내용 |
|---|---|---|
| A1 | major(실기) | `BaseOrientation` 포팅에 ARS→world 회전이 없고 IMU 장착 회전 기본값이 I — 실제 Upkie IMU에서는 피치가 틀림 |
| A2 | minor | upkie 접지 판정기가 시뮬에서 첫 판정까지 0.7 s 걸리고, 노이즈 없는 조건에선 지면 위에서 8% 시간 `contact=False` |
| A3 | minor | 모델 기반 피치 영점(`pitch_eq`)은 upkie에 없는 보정, 웅크림 때 갱신 안 됨, ROS로도 그대로 나감 |
| A4 | minor | MPC 변형은 시뮬 진실 접지를 강제 사용 — 실기에서 재현 불가한 수치 |
| A5 | minor | 센서 노이즈의 관절 속도 σ가 12.7°/s(문서는 0.5°/s) — Plan 2 정지 토크 RMS를 2.3배 부풀림 |
| A6–A10 | nit | 접지 파라미터 기본값, 저역필터 예외 처리, gyropod 요 부호 조합, 로그 시간축, MPC 다리 게인 |
| B1 | major | LQI6 설계용 몸체 피치 관성에 **요 축 관성**(0.0380)을 씀. 실제 피치 관성 0.0234 |
| B2 | major | `HostEmulator`가 명령 객체를 공유 참조로 넘겨 "죽은 호스트"의 명령이 MCU에 즉시 전달 — 링크 단절 시험이 의도와 다르게 동작 |
| B3 | major | `mode_mark=True, stand_mode=False` 패킷 하나로 lite 로봇이 주저앉아 넘어짐(DIABLO teleop의 v/b/n/m 키) |
| B4 | major(실기) | 설계보고서 §4.5 안전 규정 미구현: FAULT 뒤에도 다리 힘·피드포워드 유지, 2 s 모터 OFF·arm 조건 없음 |
| B5–B9 | minor | 적분기 windup, 1 kHz 루프 안 온라인 DARE(예외 미처리), 연성 폐루프 구속(1.7°), 롤 P 제어 43% 정상상태 오차, 끊김 프로그램이 속도모드 `up` 유지 |
| B10–B14 | nit | `F_ff·cosθ`, 동체 자이로를 오일러 각속도로 사용, 플랜트 armature 누락, L0 0 나눗셈, leg_split 무시 |
| C1 | major | ROS Plan 1 경로(spine 내장·balancer_node)에서 **요 방향이 반대**(angular.z=+0.5 → −0.425 rad/s) |
| C2 | major | 문서화된 `embedded:=false balancer:=mpc`는 접지 플래그가 안 켜져 1 s 안에 넘어짐 |
| C3 | major | `mcu_node`가 1 ms 안에 연달아 온 패킷을 하나로 합쳐 DIABLO teleop `z`(일어서기)가 유실 |
| C5–C7 | minor | `/cmd_vel` 타임아웃 기본 0(무한 유지), 외부 균형기 시작·리셋 처리, `LegMotors` pos/enc_rev 이중 계산 |
| C8–C12 | nit | 락스텝 시뮬이라 실시간 문제를 드러내지 못함, RobotStatus ctrl_mode, base_pitch 의미, 고정 dt, pip 설치 |

---

## A. Plan 1 (upkie 스택 재구현)

### A-0. 원본과 일치함을 확인한 항목

- `BaseOrientation`의 `compute_pitch_frame_in_parent`는 원본과 줄 단위로 같다(정규화, heading 부호, 클램프). 수평 heading이 0일 때 ±π/2를 돌려주는 가드는 원본(NaN)보다 낫다. 부호는 앞으로 숙이면 +. `[코드]`
- `WheelContact`, `FloorContact`(상체 토크 0.01 s 필터, 임계 10 N·m), `WheelOdometry`(접지 바퀴만 평균, 비접지 시 속도 유지)는 원본 논리와 같다. `[코드]`
- `WheelBalancer`는 게인(1.8/20/0.7/1.6), 적분 한계 10 m/s, 목표 위치 ±1 m, 공중 복귀 1 s 저역통과, 비최소위상 트릭(`-target_ground_velocity`), 넘어짐 처리, 다리 kp/kd 스케일(2 + 2·회전)까지 같다. 추가된 `gain_scale`은 기본 1.0. `[코드]`
- `MPCBalancer`는 `get_target_states`, 비용 가중치, `commanded_accel * dt / 2.0`(원본의 특이한 적분도 그대로), 넘어짐·비접지 시 0.1 s 감쇠까지 같다. ProxQP warm start 대신 qpsolvers DAQP를 쓰는 것만 다르다. `[코드]`
- 서보 모델은 `BulletInterface::compute_joint_torque`(kp 20, kd 1, NaN 위치 = 속도모드, `min(maximum_torque, 모델 한계)` 클램프)와 같다. `[코드]`
- 바퀴 부호: `wheel_forward_signs()` = left +1, right −1. upkie `spine_backend` 기본 `signed_radius` ±0.05와 일치한다. `GyropodMapper`의 left_wheeled 판정도 upkie `Model.left_wheeled`(허브 z축의 y성분 < 0)와 같은 결론이다. `[실측]`
- `Plan1Upkie.step()`의 요 부호 반전(`yaw_sign=-1`)은 맞게 동작한다. +0.5 rad/s 명령에 월드 요레이트 +0.42 rad/s(반시계, REP-103). `[실측]` (ROS 경로에서는 이 반전이 빠진다. C1 참조.)
- 웅크림 IK(`set_height_offset`)는 좌우 대칭이다. 4 cm 웅크림에서 양쪽 축의 (x, z)가 (−0.0033, −0.467)로 같다. `[실측]`

### A1. [major·실기] IMU 프레임 변환 누락 — 실제 Upkie에서 피치가 틀림

- **위치**: `wbr/observers/upkie_observers.py:23-36`, `wbr/models/build_upkie.py:108-116`
- **문제**: upkie는 `R_base_to_world = R_ars_to_world · R_imu_to_ars · R_base_to_imu`를 쓰고, 기본값이 `R_base_to_imu = diag(−1, 1, −1)`, `R_ars_to_world = diag(1, −1, −1)`이다(`BaseOrientation.h` `Parameters::configure`; pi3hat ARS는 +x 앞, +y 오른쪽, +z 아래). 포팅에는 `rotation_ars_to_world`가 아예 없고 `R_base_to_imu` 기본값이 항등이다. 시뮬에서는 IMU 센서를 `imu` 바디(`quat="0 0 1 0"`, y축 180°)가 아니라 `base` 사이트에 붙여 두 회전이 모두 항등이 되게 맞췄기 때문에 동작할 뿐이다.
- **영향**: Plan 1 코드를 실제 Upkie IMU(ARS 쿼터니언)에 연결하면 z축이 뒤집힌 회전으로 피치를 계산한다. 피치 부호·크기가 틀리고, 첫 사이클부터 균형기가 반대 방향으로 바퀴를 돌린다. 시뮬에서 센서를 실기와 같은 `imu` 바디로 옮겨도 피치 부호가 뒤집힌다. 시뮬에서 실기 경로가 한 번도 검증되지 않는다.
- **수정**:
  ```python
  class BaseOrientation:
      def __init__(self, rotation_base_to_imu=None, rotation_ars_to_world=None):
          self.R_base_to_imu = np.diag([-1.0, 1.0, -1.0]) if rotation_base_to_imu is None else rotation_base_to_imu
          self.R_ars_to_world = np.diag([1.0, -1.0, -1.0]) if rotation_ars_to_world is None else rotation_ars_to_world

      def update(self, quat_imu_in_ars, angular_velocity_imu):
          R_imu_to_ars = quat_to_matrix(quat_imu_in_ars)
          self.rotation_base_to_world = self.R_ars_to_world @ R_imu_to_ars @ self.R_base_to_imu
          ...
  ```
  시뮬에서는 센서를 `imu` 사이트로 옮기고, MuJoCo 쿼터니언(world 기준)을 ARS 규약으로 바꿔 넣거나 `rotation_ars_to_world=I`를 명시한다. 그러면 실기와 같은 `R_base_to_imu` 경로를 시뮬에서 시험할 수 있다.

### A2. [minor] 접지 판정기의 시뮬 동작 — 첫 판정 0.7 s, 지면 위 오판

- **위치**: `wbr/observers/upkie_observers.py:57-70`(파라미터 41-45), `wbr/plans/plan1_upkie.py:119-124`
- **현상** `[실측]` (PI 변형, 8 s, 2~5 s 사이 0.3 m/s 주행):
  - 노이즈 없음: 1 cm 낙하 뒤 실제 접지는 약 0.05 s인데 첫 `contact=True`는 0.735 s. 그 뒤에도 8.1% 시간 `contact=False`(겉보기 관성이 liftoff 0.001 근처에서 흔들림).
  - 노이즈 + 2 ms(벤치 조건): 첫 판정 0.72 s. 그 뒤 정지 시나리오에서는 끊김이 없고, 0.5 W 푸시 시나리오에서는 1.3% 끊김.
- **영향**: `contact=False` 동안 `WheelOdometry`가 멈추고, `WheelBalancer`는 "공중" 분기로 들어가 적분이 0으로 새고 목표 위치가 현재 위치로 끌려간다. 벤치 수치 영향은 작다: 10° 복귀 정착 6.96 s(관측기) 대 6.60 s(진실 접지). 원인은 속도모드 바퀴(kd = 1)의 토크가 0.002~0.08 N·m로 작아 겉보기 관성 |τ|/|α|가 문턱 근처에 머무는 것이다. 실기(moteus 전류 기반 토크 추정, 오프셋 있음)에서는 다르게 나올 것이므로 실기에서 다시 보정해야 한다.
- **수정**: 비교보고서에 "관측기 기반 접지, 첫 0.7 s 공중 모드"를 명시한다. 비교의 공정성을 위해 `use_sim_contact=True` 변형도 함께 보고한다. 실기에서는 liftoff/touchdown 관성을 로그로 다시 맞춘다.

### A3. [minor] 모델 기반 피치 영점 `pitch_eq` — upkie에 없는 보정

- **위치**: `wbr/plans/plan1_upkie.py:86, 117-118, 156-176`
- **문제**: 관측 피치에서 "다리를 편 자세에서 COM이 접지점 위에 오는 피치"(1.31°)를 항상 뺀다. upkie는 원시 피치에 `target_pitch = 0`을 쓴다. (1) 설정으로 끌 수 없고, (2) `set_height_offset`이 다리 자세를 바꿔도 다시 계산하지 않으며, (3) 보정된 값이 ROS `SpineObservation.base_pitch`로 그대로 나간다(C10).
- **영향**: 벤치마크의 Plan 1 수치는 "upkie 순정"이 아니라 이 보정이 들어간 값이다(비교보고서 4.4-2는 "넣으면 표류가 준다"고 가정형으로 적었다). 웅크림 중에는 영점이 틀려 PI가 위치 오차로 보상해야 한다. 이는 "2 cm 웅크림도 실패"라는 결론에 일부 기여할 수 있다 `[미검증]`. 실기에서는 모델 대신 측정한 영점이 필요하다.
- **수정**: `Plan1Config.pitch_offset: str = "model" | "none"`를 두고 기본값을 원본과 같은 `"none"`으로 하거나 보고서에 명시한다. `set_height_offset`에서 `self.pitch_eq`를 새 다리 자세로 다시 계산한다.

### A4. [minor] MPC 변형은 진실 접지를 강제 — 실기 재현 불가 수치

- **위치**: `wbr/plans/plan1_upkie.py:72-77`, `wbr/plans/plan3_diablo_upkie.py:31`
- **문제·영향**: MPC 변형은 `use_sim_contact=True`가 강제된다. 관측기만 쓰면 시작부터 접지가 안 켜져 0.98 s 만에 넘어진다 `[실측]`(`min_touchdown_torque`를 원본 0.015로 바꿔도 같다. 정지 상태에서는 `min_touchdown_acceleration` 2 rad/s² 조건이 막는다). 벤치의 MPC 수치(푸시 0.75 W, 지연 80 ms)는 실기에서 같은 관측기로는 얻을 수 없다. 비교보고서 5절에 "실기에서는 착지 충격 또는 소토크 바이어스 필요"라고 적은 것은 맞다. 다만 이 강제가 ROS 경로에는 전달되지 않는다(C2).
- **수정**: 표에 "MPC: 진실 접지 사용"을 각주로 단다. 실기용으로는 "시작 시 접지 가정 + 관측기로 이륙만 판정" 같은 상태기계를 둔다.

### A5. [minor] 센서 노이즈 모델이 문서와 다름 — 관절 속도 σ 12.7°/s

- **위치**: `wbr/plans/common.py:37-40`, `docs/비교보고서.md:26`
- **문제**: 관절 속도 노이즈가 `uniform(−1, 1)·quantum/dt + N(0, 0.5°/s)`이다. 14비트, 1 kHz에서 균등 ±0.38 rad/s, 즉 σ = 0.221 rad/s = **12.7°/s**다. 보고서는 "14-bit 엔코더 양자화 + 0.5°/s 속도 노이즈"(설계보고서 §8.2와 동일)라고 적었다.
- **영향** `[실측]`: Plan 2 기립 시험의 바퀴 토크 RMS가 0.073 N·m(코드 그대로)에서 0.032 N·m(문서대로)로 바뀐다. 비교보고서 4.1의 "바퀴 토크 RMS" 열이 이 노이즈로 결정된다. 이 노이즈는 제어기에만 들어가고 시뮬 서보 PD는 진실 `qd`를 쓴다(`mujoco_sim.py:134-135`). 그래서 토크모드인 Plan 2가 속도모드 서보인 Plan 1/3보다 더 불리하다. (참고: 노이즈를 줄였을 때 Plan 2 피치 RMS가 0.21°에서 0.64°로 오히려 커졌다. 원인은 마찰 + 양자화에 의한 리밋사이클로 추정되며 `[미검증]`.)
- **수정**: 의도가 "미분한 양자화 노이즈"라면 문서를 고친다. 실제 moteus·RMD처럼 필터된 속도를 흉내 내려면 1차 필터(예: 2 ms)를 넣는다. 어느 쪽이든 모든 플랜에 같은 신호 경로로 적용하고 표에 명시한다.

### A6. [nit] 접지 판정 기본값이 원본과 다르고 "추정값"이라는 서술은 틀림

- **위치**: `wbr/observers/upkie_observers.py:44`, `docs/비교보고서.md:161`
- **내용**: `min_touchdown_torque = 0.05`. upkie 기본값은 `upkie/envs/backends/spine_backend.py:93-98`에 있다(`cutoff_period 0.2, liftoff_inertia 0.001, min_touchdown_acceleration 2.0, min_touchdown_torque 0.015, touchdown_inertia 0.004`). 바꿔도 결과는 같았다 `[실측]`. 0.015로 맞추고 보고서 문장을 "upkie spine_backend 기본값"으로 고친다.

### A7. [nit] `low_pass_filter`가 Nyquist 위반을 조용히 통과시킴

- **위치**: `wbr/common/filters.py:15-20`
- **내용**: `cutoff_period <= 2·dt`이면 입력을 그대로 돌려준다(upkie C++는 `FilterError`, Python은 `assert`). 예를 들어 `spine_hz`를 200 Hz 이하로 내리면 `FloorContact`의 상체 토크 0.01 s 필터가 소리 없이 꺼진다. `ValueError`를 던지거나 최소한 경고를 남긴다.

### A8. [nit] `GyropodMapper`의 요 부호 조합 함정

- **위치**: `wbr/controllers/upkie/gyropod.py:57-59`
- **내용**: (yaw_sign = +1, 같은 부호 바퀴 `right_sign = left_sign`) 조합에서는 좌우 바퀴가 같은 방향으로 돌아 요가 0이 된다(추가 인자가 `yaw_sign < 0`일 때만 적용됨). 현재 호출부(Plan 1 미러, Plan 3 −1)는 안전하다. 바퀴 기준 미분 형태로 바꾸면 조합 의존이 없어진다:
  ```python
  dv = self.yaw_sign_ccw * self.contact_radius * yaw_velocity   # +: 오른쪽 바퀴가 앞으로
  left  = self.left_sign  * (ground_velocity - dv) / self.wheel_radius
  right = self.right_sign * (ground_velocity + dv) / self.wheel_radius
  ```

### A9. [nit] 로그 시간축이 지연된 관측 시간

- **위치**: `wbr/plans/plan1_upkie.py:146`, `wbr/plans/plan2_diablo_lqi6.py:142`
- **내용**: `t = o["time"]`은 지연된 관측의 시간이고 `x`, `vx`는 진실값(지연 없음)이다. 정착 시간이 지연만큼(기본 2 ms, 지연 스윕에서 최대 80 ms) 어긋난다. `t = self.sim.time`으로 로그한다.

### A10. [nit] MPC 변형의 다리 게인 스케일 고정 2.0

- **위치**: `wbr/plans/plan1_upkie.py:135`
- **내용**: upkie 예제(`follow_joystick.py`)는 회전 중 `gain_scale + turning_gain_scale`(2 + 2)을 쓴다. PI 분기와 맞추려면 `|w| > 0.1`일 때 4.0을 쓴다.

### A 판정

포팅 자체는 충실하다. 관측기, WheelBalancer, MPC, 서보 모델이 원본과 줄 단위로 맞고, 바퀴·요 부호도 실측으로 확인했다. 결함은 (1) 실기로 옮길 때 깨지는 IMU 프레임 경로(A1), (2) 시뮬 전용 보정(A3 피치 영점, A4 진실 접지)이 "upkie 원본 성능"처럼 비교표에 섞인 점, (3) 노이즈 모델의 문서 불일치(A5)다. Plan 1이 Plan 2보다 낮게 나온 결론 자체는 뒤집히지 않는다. 다만 표에는 이 보정들을 각주로 달아야 한다.

---

## B. Plan 2 (DIABLO 아키텍처 + LQI6)

### B-0. 정확함을 확인한 항목

- **plant6**: `wbr/controllers/lqi6/plant6.py`의 식·심볼·`deriv`·`linearize`는 설계보고서 `sim6/plant6.py`와 같다. 식 (1)~(3)을 직접 유도해 부호도 확인했다: θ = 바퀴→고관절 벡터의 +y축 회전각(고관절이 앞이면 +), φ = 몸체의 −y축 회전(뒤로 젖히면 +, 즉 φ = −pitch_y), T = 바퀴에 +y축으로 거는 토크(전진 +, 다리에 −T 반작용), Tp = 다리에 +y축으로 거는 고관절 토크(몸체 φ 방향 +Tp와 같은 것). `[코드]`
- **LQI6**: `MineLQI6`과 같다(상태 [θ, θ̇, v−v_ref, φ, φ̇, ∫v], Bryson Q, 토크 한계 비례 R, 1 cm 재설계, 1 m/s² 램프, 적분 ±1). `c2d`를 `control.c2d` 대신 블록 행렬 `expm`(ZOH)으로 바꿨는데 수학적으로 같다. `[코드]`
- **게인 부호 패턴**: L0 = 0.198에서 K_T = [−8.99, −1.06, −2.07, +3.37, +0.51, −1.21], K_Tp = [+3.13, +0.20, +0.41, +13.11, +1.07, +0.26]. 설계보고서 §8.7의 HEU 패턴(T행 −−−−++, Tp행 ++++++)과 일치한다. DARE는 L0 0.05~0.30 m 전 범위에서 풀리고 폐루프 이산 고유값 최대 |λ| = 0.9988~0.9993이다. `[실측]`
- **5절 링크**: FK `p = L1·u(θ1) + L2·u(θ0)`, `θ_v = atan2(−p_x, −p_z)`, 야코비안(열 분리: 허벅지는 Motor1, 정강이는 크랭크 = Motor0), `virtual_rates`, IK(무릎 앞쪽 분기), VMC `τ = Jᵀ(F·u + (Tp/L0)·u′)`. 가상일로 확인하면 `(Tp/L0)·u′`의 일률은 `Tp·θ̇_v`이므로 Tp 정의(다리에 +y)와 일치한다. MJCF의 평행사변형(크랭크 0.09 // 정강이 부착점 0.09, 로드 0.14 // 허벅지 0.14)과 관절축(+y, 좌우 동일)도 FK 규약과 맞는다. `[코드]`
- **추정기**: 바퀴 지면속도 `R·(q̇_wheel + q̇_crank + pitch_rate)`는 MuJoCo 바퀴 중심 속도와 RMS 1.1 mm/s, 평균 0.7 mm/s 차이(0.5 m/s 주행 + 0.6 rad/s 선회). θ_world = θ_body + pitch. `[실측]`
- **부호 실측**: 요 +0.8 명령 → +0.78 rad/s. roll_ref ±0.1 → 롤 ±0.059(방향 맞음, 크기는 B8). pitch_ref +0.1 → 피치 +0.100. anti-split(왼쪽 각이 크면 왼쪽 −, 오른쪽 + 토크, 몸체 반작용 상쇄)과 요 PI(왼쪽 −, 오른쪽 +)의 부호도 맞다. `[실측]`/`[코드]`
- **호스트 링크**: 500 ms 뒤 제어권 해제, 180 ms 하트비트, `timeout_act=0`(끊김 프로그램)은 `osdk_movement.cpp`·`diablo_ctrl.cpp`와 같다. `CtrlMode`/`RobotMode` 열거값은 `onboard_sdk_uart_protocol.h`와 같다. `[코드]`

### B1. [major] LQI6 설계에 몸체 **요** 관성을 피치 관성으로 사용

- **위치**: `wbr/plans/plan2_diablo_lqi6.py:52`, `wbr/controllers/lqi6/plant6.py:106`, `docs/비교보고서.md:42`
- **문제** `[실측]`: `Im = body_inertia[base][1]`. MuJoCo `body_inertia`는 주관성 프레임(`body_iquat`) 기준인데, base의 `body_iquat = (0.7071, 0.7071, 0, 0)`(x축 90° 회전)이라 주축 y가 몸체 z(요)다. base 프레임에서 다시 계산하면 Ixx 0.0462, **Iyy(피치) 0.0234**, Izz(요) 0.0380이다. 코드는 0.0380을 피치 관성으로 쓴다(62% 과대). 보고서의 "몸체 피치 관성 0.038 kg·m²"도 같은 오류다.
- **영향**: 지금 모델에서는 LQR이 강건해 성능 차이가 작다 `[실측]`: 10° 정착 0.643 s 대 0.621 s, 2.0 W 푸시는 둘 다 통과, 2.5 W는 둘 다 실패, 지연 30 ms 통과·40 ms 실패로 같다. 그러나 "모델에서 게인을 자동으로 얻는다"는 핵심 주장의 입력이 틀렸다. 몸체 형상이 바뀌어 요·피치 관성 차이가 커지면 조용히 나빠진다.
- **수정**:
  ```python
  R = np.zeros(9); mujoco.mju_quat2Mat(R, m.body_iquat[bid]); R = R.reshape(3, 3)
  I_body = R @ np.diag(m.body_inertia[bid]) @ R.T     # base 프레임 기준, COM 중심
  Im = float(I_body[1, 1])                              # 0.0234
  ```
  `plant6.py:106`의 기본값 주석과 비교보고서 2.1절 표도 고친다.

### B2. [major] 호스트 명령 객체 공유(aliasing) — 링크 단절 시험이 의도와 다름

- **위치**: `wbr/controllers/diablo_arch/host_link.py:86, 125`, `wbr/plans/plan2_diablo_lqi6.py:132`, `wbr/bench/battery.py:168-178`
- **문제**: `HostEmulator.tick()`이 `self.link.receive(self.cmd, t)`로 같은 객체를 넘기고, `HostLink.receive`는 `self.last_cmd = cmd`로 참조를 저장한다. 첫 송신 뒤에는 `host.cmd is link.last_cmd`가 된다. 그래서 (1) `set_command()`의 변경이 50 Hz 송신 시점이 아니라 1 kHz로 즉시 MCU에 보이고(송신 주기 양자화가 없음), (2) `host.enabled = False`(호스트 사망) 뒤에도 `plan.step(v, w)`가 부르는 `set_command`가 MCU 명령을 계속 바꾼다.
- **영향** `[실측]`: 링크 단절 시나리오는 0.5 m/s 주행 중 호스트를 끈 뒤 `plan.run(6.0)`, 즉 `step(0, 0)`을 부른다. 그러면 `forward = 0`이 죽은 호스트에서 바로 MCU로 새어 들어가 끊긴 순간부터 감속한다(끊김 +0.5 s에 0.468 m/s). 깊은 복사로 고치면 타임아웃 전 0.5 s 동안 명령대로 계속 달리다가(+0.5 s에 0.611 m/s) 해제 뒤에 멈춘다. 해제 시각(2.491 s)과 6 s 뒤 속도(0.019 대 0.014)는 같아 최종 결론은 유지된다. 그러나 "500 ms 뒤 자동 정지"를 시험한다는 시나리오는 실제로는 그것을 시험하지 않았다.
- **수정**:
  ```python
  # host_link.py, HostEmulator.tick
  self.link.receive(copy.deepcopy(self.cmd), t)
  # HostLink.poll 끊김 프로그램도 mode를 복사
  safe = MotionCtrl(up=self.last_cmd.up, mode=copy.deepcopy(self.last_cmd.mode))
  ```
  고친 뒤 `scenario_link_loss`를 다시 돌리고, 보고서에 타임아웃 전 0.5 s 동안의 이동 거리도 적는다. (ROS `mcu_node`는 직접 `deepcopy`를 하므로 이 문제가 없다. `[코드]`)

### B3. [major] `stand_mode=False` 모드 패킷 하나로 lite 로봇이 주저앉아 넘어짐

- **위치**: `wbr/controllers/diablo_arch/mcu.py:138-142, 188-190`, `wbr/controllers/diablo_arch/host_link.py:37, 48`, `ros2_ws/src/diablo_lite_ros2/diablo_lite_mcu/diablo_lite_mcu/mcu_node.py:211-219`
- **문제**: DIABLO 의미론 그대로 `mode_mark=True`이고 `stand_mode=False`이면 STAND → TRANSITION_DOWN → 1 s 뒤 CAR(균형 끔)로 간다. DIABLO 실물은 보조 바퀴로 앉지만 DIABLO-lite에는 보조 바퀴가 없다. ROS 메시지 `stand_mode` 기본값은 False이고, DIABLO teleop은 `z`를 한 번도 누르지 않았으면 모든 모드 키(`v` 높이속도모드, `b`, `n`, `m`, `c`, `f`, `g`)에 `stand_mode=False`를 실어 보낸다. 시뮬은 `start_mode=stand`로 시작하므로 사용자는 일어서기 명령을 보낸 적이 없다. 반면 wbr 데이터클래스 기본값은 `stand_mode=True`, `up=1.0`이라 ROS 기본값(False, 0.0)과도 어긋난다.
- **영향** `[실측]`: 첫 패킷으로 DIABLO teleop의 `v`(mode_mark, stand_mode=False, height_ctrl_mode=True)를 보내면 0.001 s에 TRANSITION_DOWN, 1.0 s에 CAR, 1.70 s에 피치 FAULT가 나고 1.78 s에 넘어진다. ROS 경로(`mcu_node`)도 `m.stand_mode = bool(msg.mode.stand_mode)`라서 같다. 실기라면 높이모드를 바꾸려다 로봇이 쓰러진다.
- **수정**: lite에는 CAR 자세가 물리적으로 없으므로 MCU에서 막는다.
  ```python
  @dataclass
  class McuParams:
      ...
      allow_transform_down: bool = False   # DIABLO-lite: 보조 바퀴 없음

  # apply_command
  elif not cmd.mode.stand_mode and self.mode == RobotMode.STAND:
      if self.p.allow_transform_down:
          self._set_mode(RobotMode.TRANSITION_DOWN)
      else:
          self.events.append((self.t, "transform down ignored (no auxiliary wheels)"))
  ```
  CAR가 꼭 필요하면 CAR에서도 L_min으로 균형을 유지한다(`balance=True`). 데이터클래스 기본값은 ROS와 같게(`up=0.0`, `stand_mode=False`) 두고, 시작 모드가 STAND이면 호스트 쪽 상태를 `stand_mode=True`로 초기화한다.

### B4. [major·실기] 설계보고서 §4.5 안전 규정 미구현

- **위치**: `wbr/controllers/diablo_arch/mcu.py:192-196, 221-236`, `wbr/controllers/diablo_arch/host_link.py:59`
- **문제**: §4.5는 "|θ| > 34° → FAULT(토크 0); 명령 0.5 s 없음 → v=0; 2 s → OFF; 드라이버 피드백 50 ms 없음 → FAULT; |θ| < 3°일 때만 arm"이다. 구현은 FAULT 때 T와 Tp만 0으로 하고 **다리 길이 PD, 중력 피드포워드 24.5 N/다리, anti-split, VMC 토크는 그대로 낸다**. `disconnect_brake_s = 2.0`은 정의만 있고 어디서도 쓰지 않는다(2 s OFF 없음). arm 조건과 피드백 타임아웃도 없다. FAULT는 리셋 전까지 래치된다.
- **영향**: 시뮬 결과에는 영향이 없다. 실기에서는 넘어진 뒤에도 양쪽 다리가 바닥이나 몸통을 계속 24.5 N 이상으로 밀고 PD가 버틴다. 사람 손이나 케이블이 끼면 위험하고 모터가 과열된다. 호스트가 죽으면 영원히 제자리 균형을 유지한다(설계는 2 s 뒤 OFF).
- **수정**: FAULT이면 6개 관절 모두 토크 0(또는 바퀴만 저감쇠)으로 둔다. `disconnect_brake_s`를 MCU에서 구현한다(`t − last_rx > 2 s`이면 모든 토크 0 또는 다리 저강성 위치유지). |pitch| < 3°일 때만 arm하는 상태를 추가한다.

### B5. [minor] 적분기 windup — 비균형 구간과 토크 포화

- **위치**: `wbr/controllers/lqi6/lqi6.py:66-72`, `wbr/controllers/diablo_arch/mcu.py:216, 233-236`
- **문제**: `law.compute`는 `balance=False`(CAR, 전이, FAULT)일 때도 호출되어 `z`가 계속 적분된다(v_ref = 0, ±1 m 클램프). 토크 포화(바퀴 5 N·m, 고관절 12 N·m) 때도 back-calculation이 없다. K_∫ = −1.21 N·m/m이라 최대 1.2 N·m 킥이다.
- **영향**: 재기립(TRANSITION_UP)이나 큰 푸시 직후 위치 오버슈트 `[미검증]`.
- **수정**: `if not balance: self.law.reset()`(또는 z 동결). 포화 시에는 `z`를 갱신하지 않는 조건부 적분을 쓴다.

### B6. [minor] 1 kHz 루프 안의 온라인 DARE — 비용과 예외 처리

- **위치**: `wbr/controllers/lqi6/lqi6.py:54, 59-62`, `wbr/controllers/lqi6/law.py:30`
- **문제** `[실측]`: 재설계 1회에 이 PC Python으로 0.6~1.7 ms가 든다(sympy 함수 12회 + expm + DARE). 측정한 L0가 1 cm 바뀔 때마다 1 kHz 주기 하나를 넘긴다(6 cm 웅크림이면 약 6회). `solve_discrete_are`의 `LinAlgError`는 잡지 않는다. ROS `mcu_node`에서는 예외가 `plan.step` 안에서 `sim.step` 전에 나므로, 사이클마다 같은 예외를 내며 시뮬이 멈춘다(K가 None으로 남음). 실기 MCU(Teensy)에서는 온라인 리카티가 현실적이지 않다(설계보고서 §8.6도 다항식 스케줄링을 대안으로 듦).
- **수정**: 시작할 때 L0 그리드(예: 0.10~0.28 m, 1 cm 간격) 게인 표를 미리 계산해 선형보간한다. 스케줄 변수는 측정값이 아니라 `L_ref`(매끄러움)로 한다. 실패하면 마지막 유효 K를 유지한다:
  ```python
  try:
      P = la.solve_discrete_are(Ad, Bd, self.Q, self.R)
  except (np.linalg.LinAlgError, ValueError):
      return self.K   # 마지막 유효 게인 유지
  ```

### B7. [minor] 평행사변형 폐루프 구속이 무름 — 하중에서 1.7° 어긋남

- **위치**: `wbr/models/build_diablo_lite.py:132-135`, `docs/비교보고서.md:43`
- **문제** `[실측]`: `connect` 구속이 기본 `solref = 0.02`라 서 있을 때 루프 잔차 2.7 mm, 정강이와 크랭크의 평행 오차 1.7°이고, 1.5 W 푸시에서는 5 mm, 3.5°까지 간다. 추정기는 완전한 평행사변형을 가정하므로 다리각 추정 오차가 RMS 1.0°, L0 오차가 3.8 mm다. 보고서의 "폐루프 잔차 3×10⁻¹⁰ m"는 무하중 키프레임 값이다.
- **영향**: 실물에 없는 1~3°의 다리 컴플라이언스와 추정 오차가 시뮬에 들어가 있다. 결과가 실기보다 비관적인지 낙관적인지 알 수 없다. θ_eq 표(기구학으로 계산)도 하중 상태 평형과 약 1° 어긋난다.
- **수정**: `<connect ... solref="0.001 1" solimp="0.99 0.999 0.0001"/>`(timeconst ≥ 2·timestep)로 강성을 올리고 잔차를 다시 확인한다. 보고서 표에 "하중 시 잔차"를 적는다.

### B8. [minor] 롤 제어가 P만 있어 정상상태 오차 43%

- **위치**: `wbr/controllers/diablo_arch/mcu.py:206-208`
- **문제** `[실측]`: `dL = 0.5·(roll_ref − roll)`. 롤은 기하학적으로 약 dL/track이므로 폐루프 정적 이득이 (0.5/0.38)/(1 + 0.5/0.38) = 0.57이다. roll_ref 0.1 rad에 0.059 rad만 간다. 옆으로 기운 경사면에서는 경사의 43%만큼 몸체가 기운 채 남는다.
- **수정**: `dL = track·tan(roll_ref) + kp·e + ki·∫e`(피드포워드 + 적분).

### B9. [minor] 끊김 프로그램이 속도모드 입력을 유지

- **위치**: `wbr/controllers/diablo_arch/host_link.py:96`, `wbr/controllers/diablo_arch/mcu.py:145-146`
- **문제**: 끊김 시 `MotionCtrl(up=last.up, mode=last.mode)`를 반환한다. `height_ctrl_mode=True`(수직속도모드)에서 `up ≠ 0`이면 링크가 끊긴 뒤에도 `L_target`이 매 tick 적분되어 다리가 한계까지 계속 늘거나 준다(DIABLO teleop은 `up`을 0으로 되돌리지 않음).
- **수정**: 끊김 프로그램에서는 속도모드인 입력(`up`, `roll`, `pitch`)을 0으로 둔다. 위치모드 입력만 유지한다.

### B10. [nit] 다리 중력 피드포워드 `F_ff·cosθ` → `F_ff/cosθ`

- **위치**: `wbr/controllers/diablo_arch/mcu.py:229`
- **내용**: Tp ≈ 0에서 수직 지지에는 F = W/cosθ가 필요하다. 10°에서 3%(다리당 0.75 N)가 차이 나고, kp 400 N/m이면 1.9 mm 처진다. PD가 메우므로 영향은 작다.

### B11. [nit] 동체 자이로를 오일러 각속도로 사용

- **위치**: `wbr/controllers/diablo_arch/mcu.py:112-114`
- **내용**: `pitch_rate = gyro[1]`, `yaw_rate = gyro[2]`는 동체 각속도다. 롤이 있거나 피치를 준 채 선회하면 오일러율과 cos 계수만큼 다르다. 피치 30°에서 요레이트 13% 과소다. 정확히 하려면 `R`로 월드 각속도로 바꿔 요는 z성분을 쓴다.

### B12. [nit] 플랜트와 MuJoCo 모델의 질량 특성 불일치

- **위치**: `wbr/controllers/lqi6/plant6.py:103-110`
- **내용**: `Iw = 2·½·m_wheel·R²`(0.0021)는 바퀴+모터 0.5 kg 전체를 원판으로 보고, 바퀴 armature 2 × 0.0004(+38%)와 고관절 armature는 빠져 있다. `L_ratio = 0.5`는 5절 링크의 실제 다리 COM과 다르다(θ_eq로 보정). 영향은 강건성으로 흡수되지만 "모델 파라미터만 바꾸면 게인이 따라온다"는 주장에는 이 단순화를 함께 적어야 한다.

### B13. [nit] L0 0 나눗셈·NaN 가드

- **위치**: `wbr/controllers/lqi6/fivebar.py:72`
- **내용**: `virtual_rates`는 L0로 바로 나눈다(`vmc_torques`는 `max(L0, 1e-3)`). 관절 범위가 크랭크−허벅지 각 최대 3.97 rad(> π)를 허용하고 다리 지오메트리에 충돌이 없어서(contype 0) 넘어지는 중에 L0 → 0을 지날 수 있다. 그러면 NaN 토크가 MuJoCo로 간다. 가드를 넣고 출력 토크에 `np.isfinite` 검사를 둔다.

### B14. [nit] `leg_split`·jump·split 무시, 모드 비트 극성 모호

- **위치**: `wbr/controllers/diablo_arch/host_link.py:30-51`, `mcu.py:223`
- **내용**: `value.leg_split`, `jump_mode`, `split_mode`는 파싱만 하고 쓰지 않는다(anti-split 목표는 항상 0). 문서에 "미지원"으로 적는다. DIABLO 헤더 `onboard_sdk_uart_protocol.h:139-154`는 모드 비트 극성을 블록 주석(1 = 각도)과 필드 주석(0 = 각도)에서 서로 반대로 적었다. wbr은 필드 주석(True = 속도)을 따랐다. 실물 DIABLO 펌웨어와 맞는지는 `[미검증]`.

### B 판정

제어 법칙·기구학·부호는 정확하다. plant6과 LQI6은 설계보고서와 같고, 게인 부호 패턴, 폐루프 안정, 추정기(바퀴속도 1 mm/s 오차), 요·롤·피치 부호를 실측으로 확인했다. 결함은 제어 법칙 밖에 있다. (1) 모델 파라미터 1건(B1, 축 오류), (2) 시험 하네스 1건(B2, 링크 단절 시험의 타당성), (3) DIABLO 모드 의미론을 보조 바퀴 없는 로봇에 그대로 적용한 위험(B3), (4) 실기 안전 규정 미구현(B4). Plan 2 추천 결론은 유지되지만 B1·B2를 고친 뒤 벤치마크를 다시 돌려 표를 갱신해야 하고, 실기 전에 B3·B4가 필수다.

---

## C. ROS 2 패키지 (`ros2_ws/src`, 정적 검토, 빌드·실행 `[미검증]`)

### C-0. 정적 검토로 문제없음을 확인한 항목

- `motion_msgs`, `ception_msgs`의 `.msg`는 `references/diablo_ros2/diablo_interfaces`와 공백·개행만 다르고 필드명·타입이 같다. CMake·package.xml(rosidl_default_generators buildtool, `member_of_group rosidl_interface_packages`, `ament_export_dependencies(rosidl_default_runtime ...)`)도 맞다. `wbr_msgs`도 같은 형식이다.
- ament_python 패키지 5개: `resource/<pkg>` 마커, `setup.cfg`(`script_dir`/`install_scripts`), `data_files`(package.xml, launch, config, rviz), `console_scripts` 진입점(`spine_node`, `balancer_node`, `mcu_node`, `teleop_node`, `cmd_vel_bridge`)이 모두 있다. exec_depend 키(`tf2_ros_py`, `std_srvs`, `teleop_twist_keyboard`, `xacro`, `joint_state_publisher_gui`)는 Jazzy rosdep 키로 유효하다.
- `upkie_description` 래퍼: `CMAKE_CURRENT_SOURCE_DIR/../../../../models/upkie_description` = `wheeled_biped/models/upkie_description`로 맞다. xacro는 상대 include를 "include하는 파일의 디렉터리" 기준으로 풀기 때문에 share 루트에 복사한 `upkie.xacro`에서 `xacro/utils/...`가 풀린다(symlink-install에서도 `os.path.dirname`은 링크를 따라가지 않음).
- launch: 인자 `choices`, `IfCondition`/`UnlessCondition`, `PythonExpression`, `ParameterValue(Command([...xacro...]), value_type=str)`, 같은 이름의 robot_state_publisher 두 개의 상호배타 조건이 맞다.
- `wbr` API 호출: `Plan1Config`/`Plan2Config`/`Plan3Config` 필드명, `Plan1Upkie.balancer.step()`·`mapper.servo_commands()` 시그니처, `ServoCommand` 필드, `Plan2DiabloLQI6.step(forward, left)`, `HostLink.receive(cmd, t)`, `link.p.timeout_ms`, `mcu.events`/`link.events`, `est.L0`이 모두 실제 코드와 맞는다. `HostEmulator`를 끄고 패킷을 `deepcopy`하므로 B2의 aliasing은 ROS 경로에 없다.
- 토픽 이름(`diablo/MotionCmd`, `diablo/sensor/{Imu,ImuEuler,Motors,Body_state}`)과 깊이 10 QoS가 DIABLO와 같다. `/upkie/observation`·`/upkie/servo_commands`는 양쪽 다 best-effort라 호환된다. `/joint_states`의 수동 관절(`knee = crank − hip`, `rod = hip − crank`)은 URDF 원점 rpy와 맞는다.
- 스레드 안전: 공유 상태는 락으로 짧게 교환하고, 텔레메트리는 불변 스냅샷 dict를 읽으며, 리셋은 루프 스레드 안에서 한다.

### C1. [major] ROS Plan 1 경로에서 요 방향이 반대

- **위치**: `ros2_ws/src/upkie_ros2/upkie_spine/upkie_spine/spine_node.py:243-244`, `ros2_ws/src/upkie_ros2/upkie_balancer/upkie_balancer/balancer_node.py:107-119`
- **문제**: 두 노드 모두 `/cmd_vel`의 `angular.z`를 그대로 `WheelBalancer.step`과 `GyropodMapper.servo_commands`(yaw_sign = +1, upkie 시계방향 +)에 넘긴다. `Plan1Upkie.step()`이 하는 `cfg.yaw_sign`(−1) 곱하기(`plan1_upkie.py:130`)를 건너뛴다.
- **영향** `[실측]`: spine 내장 경로와 같은 코드로 `angular.z = +0.5`를 주면 월드 요레이트가 **−0.425 rad/s**다(REP-103은 +). teleop_twist_keyboard의 `j`(좌회전)에 로봇이 오른쪽으로 돈다. Nav2 같은 상위 스택은 발산한다.
- **수정**:
  ```python
  # spine_node._cycle (embedded)
  w_upkie = plan.cfg.yaw_sign * w
  out = plan.balancer.step(obs, v, w_upkie)
  cmds = plan.mapper.servo_commands(out["ground_velocity"], w_upkie, out["leg_kp_scale"], out["leg_kd_scale"])
  # balancer_node: self.declare_parameter("yaw_sign", -1.0) 후 w에 곱해 두 호출 모두에 사용
  ```
  더 근본적으로는 `Plan1Upkie`에 `control_step(obs, v, w) -> cmds`를 만들어 두 노드가 같은 함수를 부르게 한다(복제 로직 제거).

### C2. [major] `embedded:=false balancer:=mpc`가 서지 못함

- **위치**: `spine_node.py:106-109`, `upkie_bringup/config/spine.yaml:11`, `upkie_bringup/launch/sim.launch.py:56-73`, `ros2_ws/README.md:84`
- **문제**: `Plan1Upkie`는 MPC일 때 `use_sim_contact = True`를 강제하지만 spine 노드는 `Plan1Config(balancer="pid", use_sim_contact=<spine.yaml: false>)`로 만든다. MPC는 접지 플래그가 없으면 속도를 0으로 감쇠시킨다(`mpc_balancer.py:74-75`).
- **영향** `[실측]`: 같은 객체 구성(관측기 접지 + MPC)으로 돌리면 접지가 한 번도 켜지지 않고 0.98 s에 넘어진다. README 3절의 해당 명령은 동작하지 않는다(외부 노드의 200 Hz, ROS 지연까지 더하면 더 나쁨).
- **수정**: launch에서 MPC 외부 모드일 때 spine에 진실 접지를 켠다.
  ```python
  parameters=[spine_config, {
      'embedded_balancer': ParameterValue(embedded, value_type=bool),
      'use_sim_contact': ParameterValue(PythonExpression(
          ["'", embedded, "' == 'false' and '", balancer, "' == 'mpc'"]), value_type=bool)}]
  ```
  README에 "실기에서는 접지 관측기 문제(A4)를 따로 해결해야 함"을 적는다.

### C3. [major] `mcu_node`가 연속 패킷을 하나로 합쳐 모드 명령을 잃음

- **위치**: `ros2_ws/src/diablo_lite_ros2/diablo_lite_mcu/diablo_lite_mcu/mcu_node.py:228, 268-277`, `diablo_lite_ctrl/teleop_node.py:137-141`
- **문제**: 콜백은 `self._pending = deepcopy(c)`로 **가장 최근 패킷 하나만** 보관하고, 1 kHz 루프가 그것만 `link.receive`한다. MCU는 자신이 보는 패킷에 `mode_mark=True`가 있을 때만 모드를 바꾼다(`mcu.py:138`). DIABLO teleop의 `z`는 원본과 같이 `[mode_mark=True, stand_mode=True]`, `[mode_mark=False, up=1.0]`, 같은 것 한 번 더를 수 µs 간격으로 연달아 발행한다. 세 콜백이 다음 MCU 사이클 전에 모두 처리되면 일어서기 패킷이 덮어써진다. README는 "각 MotionCtrl 메시지는 하나의 시리얼 패킷"이라고 적었지만 실제 동작은 다르다.
- **영향**: `start_mode: car`이거나 앉은 뒤 `z`로 일어서려 할 때 높은 확률로 무시된다. 다른 발행자 조합(cmd_vel_bridge의 기립 명령과 하트비트가 겹칠 때 등)에서도 모드 패킷이 사라질 수 있다. 정적 분석으로 확인했고 런타임 확률은 `[미검증]`(두 콜백 사이에 루프가 끼어들 확률은 콜백 간격/1 ms 정도).
- **수정**: 큐로 순서대로 전달하거나 모드 이벤트를 래치한다.
  ```python
  # 콜백
  self._packets.append(copy.deepcopy(c))          # collections.deque(maxlen=64)
  # 루프
  with self._lock:
      packets = list(self._packets); self._packets.clear()
  for pkt in packets:
      plan.link.receive(pkt, plan.mcu.t)
      if pkt.mode_mark:
          plan.mcu.apply_command(pkt, 0.0)        # 모드 전이는 덮어써지기 전에 반영
  ```
  또는 `HostLink.receive`에서 `mode_mark` 패킷을 `self.pending_mode_event`로 따로 보관하고 `DiabloMcu.tick`이 소비한다.

### C4. [major] DIABLO teleop 호환 시 주저앉아 넘어짐

B3과 같은 결함의 ROS 측면이다. 기본 `start_mode: stand`에서 사용자가 `ros2 run diablo_lite_ctrl teleop_node`로 `v`(높이 속도모드)부터 누르면 로봇이 앉아서 넘어진다. README 3절이 "DIABLO 키맵 호환"을 내세우므로 B3의 MCU 가드가 필요하다.

### C5. [minor] `/cmd_vel` 타임아웃 기본값 0(무한 유지)과 README의 정지 설명

- **위치**: `upkie_bringup/config/spine.yaml:19`, `balancer.yaml:10`, `diablo_lite_ctrl/cmd_vel_bridge.py:31`, `ros2_ws/README.md:153`
- **문제**: 세 곳 모두 `cmd_vel_timeout_ms: 0.0`이다(마지막 명령 무한 유지). teleop_twist_keyboard는 키를 누를 때만 발행하므로, 주행 중 teleop이 죽으면 Plan 1은 계속 달린다. Plan 2/3은 `bridge:=true`일 때 브리지의 180 ms 하트비트가 MCU 링크를 계속 살려 500 ms 타임아웃이 걸리지 않는다. README의 "stop `cmd_vel_bridge` / `teleop_node` and the robot stops"는 브리지 자신이 죽을 때만 맞다. 이것은 DIABLO `diablo_ctrl_node` 하트비트의 알려진 성질을 그대로 옮긴 것이다.
- **수정**: 기본값을 500 ms로 하고 README 문장을 고친다.

### C6. [minor] 외부 균형기의 시작·리셋 처리

- **위치**: `spine_node.py:245-249`, `balancer_node.py:92-99`
- **문제**: (1) 외부 모드에서 spine은 첫 명령이 오기 전에도 시뮬을 진행하면서 6관절에 토크 0(STOP)을 건다. balancer_node가 import(MPC면 qpmpc 등)하는 동안 로봇이 다리째 주저앉는다. (2) `~/reset` 뒤 spine의 오도메트리는 0에서 다시 시작하는데 balancer_node의 적분(`integral_velocity`), `target_ground_position`, 매퍼 다리 위치는 그대로라 리셋 직후 킥이 생긴다(`_initialized`가 True로 남음).
- **수정**: (1) 첫 명령 전에는 시뮬을 진행하지 않거나 다리를 위치유지(kp_scale 1)로 둔다. upkie spine도 에이전트 없이 로봇을 굴리지 않는다. (2) spine 리셋을 토픽·서비스로 알리거나, balancer가 오도메트리 불연속(|Δposition| > 0.1 m)을 감지하면 `reset()`한다.

### C7. [minor] `LegMotors`의 `pos`/`enc_rev` 이중 계산

- **위치**: `diablo_lite_mcu/mcu_node.py:319-321`
- **문제**: DIABLO에서 `*_pos`는 한 바퀴 안 위치(`round_pos/100`)이고, 소비자는 `pos + rev·2π`로 전체 각을 만든다(`osdk_telemetry.cpp:322-355`의 무릎각 계산). wbr은 `pos`에 펼친 전체 각을, `enc_rev`에 `floor(pos/2π)`를 넣는다. DIABLO 도구로 읽으면 바퀴는 회전수가 두 번 더해지고, −0.3 rad의 고관절은 `rev = −1`이라 −0.3 − 2π가 된다.
- **수정**:
  ```python
  rev = math.floor(position / TWO_PI)
  setattr(msg, f"{side}_{motor}_enc_rev", int(rev))
  setattr(msg, f"{side}_{motor}_pos", position - rev * TWO_PI)
  ```

### C8. [nit] 락스텝 시뮬이라 실시간 문제를 드러내지 못함

- **위치**: `spine_node.py:202-226`, `mcu_node.py:239-263`
- **내용**: spine 내장 모드와 `mcu_node`는 제어기와 MuJoCo를 같은 스레드에서 락스텝으로 돌린다. 그래서 GIL 경합, `time.sleep` 지터, B6의 DARE 지연이 제어 품질에 전혀 반영되지 않는다(벽시계와 호스트 명령 페이싱만 영향). ROS 지연이 제어 루프에 들어가는 것은 Plan 1 `embedded:=false`뿐이다. 이 경우 100 ms 미만으로 밀리면 `next_time += period`로 쉬지 않고 몰아서 실행하므로, 같은(낡은) 외부 명령으로 여러 스텝이 진행된다. 비교보고서 8절 3번(ROS 경유 지연 측정)은 이 점을 고려해 해석해야 한다.

### C9. [nit] `RobotStatus.ctrl_mode_msg` 의미 `[미검증]`

- **위치**: `mcu_node.py:333`
- **내용**: wbr은 열거값(0/1/2)을 발행한다. DIABLO `obtain_control`은 `status.ctrl_mode`를 SDK id별 비트마스크로 검사한다(`osdk_movement.cpp:11`). 텔레메트리 필드가 열거인지 비트마스크인지 원본에서 확정할 수 없다.

### C10. [nit] `SpineObservation.base_pitch`는 보정된 피치

- **위치**: `wbr_msgs/msg/SpineObservation.msg:2`, `plan1_upkie.py:117-118`
- **내용**: 메시지 주석은 "world 수직 대비 피치"인데 실제로는 `pitch_eq`(A3)를 뺀 값이다. 필드를 나누거나(`base_pitch`, `base_pitch_offset`) 주석을 고친다.

### C11. [nit] balancer_node의 고정 dt

- **위치**: `balancer_node.py:51, 117`
- **내용**: 적분 dt를 `1/observation_hz`로 고정한다. best-effort 구독에서 메시지가 빠지면 적분이 과소해진다. 헤더 스탬프 차이를 쓰거나 누락 카운트를 경고한다.

### C12. [nit] `pip install -e . --break-system-packages` `[미검증]`

- **위치**: `ros2_ws/README.md` 1절
- **내용**: 시스템 Python에 mujoco/scipy/qpsolvers를 pip로 넣으면 의존성에 따라 apt의 numpy 1.26이 numpy 2.x로 바뀔 수 있다. 그러면 numpy 1.x로 빌드된 ROS 패키지(cv_bridge 등)가 깨진다. README의 (b) `--system-site-packages` venv를 기본 권장으로 올린다.

### C 판정

패키징, 메시지, `wbr` API 호출은 정적 검토상 올바르다(빌드는 우분투에서 확인 필요). 그러나 README에 적힌 실행 경로 중 둘이 틀리게 동작한다. Plan 1 ROS의 요 방향이 반대이고(C1, 실측), `embedded:=false balancer:=mpc`는 넘어진다(C2, 실측). DIABLO 호환 경로에는 패킷 합치기(C3)와 주저앉기(C4/B3) 결함이 있다. 안전 기본값(C5)도 보수적으로 바꿔야 한다. 이 다섯 가지를 고치기 전에는 "DIABLO 도구와 그대로 호환"이라고 말하기 어렵다.

---

## D. 우선순위 수정 목록

1. **C1** ROS Plan 1 요 부호: 두 노드에서 `w *= plan.cfg.yaw_sign`(한 줄씩). 공통 `control_step()`으로 통합하면 더 좋다.
2. **B3/C4** lite MCU에서 TRANSITION_DOWN 차단(`allow_transform_down=False`). 데이터클래스 기본값을 ROS와 맞춤(`up=0.0`, `stand_mode=False`).
3. **C2** launch에서 외부 MPC일 때 `use_sim_contact:=true`. README 갱신.
4. **C3** `mcu_node` 패킷 큐(또는 모드 이벤트 래치).
5. **B2** `HostEmulator`에서 `deepcopy` 후 `scenario_link_loss` 재실행. 표와 "0.5 s 뒤 자동 정지" 서술 갱신.
6. **B1** 피치 관성을 base 프레임으로 회전해 `I[1,1]`(0.0234) 사용. 벤치 재실행, 보고서 2.1절 수정.
7. **B4** 안전 규정 구현(FAULT 시 전 관절 토크 0, 2 s OFF, arm |pitch| < 3°, 피드백 타임아웃). 실기 전 필수.
8. **A1** `BaseOrientation`에 ARS→world·IMU 장착 회전 추가. 시뮬 센서를 `imu` 바디로 옮겨 실기 경로 검증. Plan 1 실기 전 필수.
9. **C5** `cmd_vel_timeout_ms` 기본 500 ms. README 정지 설명 수정.
10. **B5/B6** 적분기 동결·조건부 적분. K(L0) 표 사전계산과 DARE 실패 시 마지막 K 유지(실기 MCU용).
11. **B7** `connect` 구속 강성 상향 후 하중 잔차 재확인.
12. **A3/A4/A5** 비교표 각주(피치 영점 보정, 진실 접지, 실제 속도 노이즈 σ 12.7°/s). 원하면 upkie 순정 설정으로도 재실행.
13. 나머지 minor/nit(B8 롤 피드포워드·적분, B9 끊김 프로그램, C6 시작·리셋, C7 LegMotors, A6~A10, B10~B14, C8~C12).
