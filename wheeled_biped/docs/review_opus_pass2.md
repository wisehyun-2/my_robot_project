# 1차 검토 반영분 2차 검증 (review_opus_pass2)

작성일 2026-10-22. 대상: `docs/review_opus.md`(1차)의 지적 중 `docs/비교보고서.md` 10절이 "반영함"이라고 적은 항목. 프로젝트 파일은 수정하지 않았다(이 문서만 작성).

## 0. 방법

- **코드 정독**: 10절에 적힌 수정 파일 전부(관측기·필터·gyropod·plan1·LQI6 4종·plan2·host_link·mcu·build_diablo_lite·battery, ROS 노드 3종·launch·yaml·msg·bridge·README). upkie(`BaseOrientation.h`, `upkie_gyropod.py`), DIABLO SDK(`diablo_ctrl.cpp`, `teleop.py`, `osdk_telemetry.cpp`, `motor_pos_trans.cpp`)와 대조했다.
- **실측**: 스크래치패드 스크립트(프로젝트 밖)로 확인했다.
  - `check_core.py`: 요 부호, 관성, 폐루프 잔차, 링크 단절, 주저앉기 거부, FAULT 토크, arm, 게인 표, 롤, 끊김 프로그램, settle_time.
  - `check_misc.py`, `check_crouch.py`: A1 왕복, A3 웅크림, 벤치 사례 재현.
  - `check_ros_plan1.py`, `check_ros_mcu.py`: **프로젝트의 ROS 노드 클래스(`SpineNode`, `BalancerNode`, `McuNode`)를 그대로 실행**했다. rclpy·메시지는 동기 버스 스텁(`rosstub.py`)으로 대체했고, 노드의 `time.perf_counter`는 시뮬 시계로 바꿨다.
  - 경로: `C:\Users\User\AppData\Local\Temp\claude\C--Users-User-Desktop-myrobot\f95773f0-4aa7-461d-b7ac-9431e8707288\scratchpad\`
- **단위 테스트**: `python -m unittest discover -s tests -v` → **16개 모두 통과(1.18 s)**.
- **표기**: `[실측]` 스크립트로 재현함, `[코드]` 정독으로 확인함, `[미검증]` ROS 런타임이 없어 확인 못 함.
- **상태 표기**: VERIFIED 수정이 맞고 완결됨, PARTIAL 일부만 됨, NOT FIXED 수정 안 됨, REGRESSION 수정으로 나빠짐.

**요약**: 검증 대상 29건 중 26건 VERIFIED(C1은 동작은 맞지만 10절의 "공용" 서술이 부정확), 3건 PARTIAL(A6, B4, C10). NOT FIXED와 REGRESSION은 없다. 수정 과정에서 생긴 새 결함은 minor 3건(N1~N3), nit 7건(N4~N10)이다. 결론(Plan 2 추천)을 바꾸는 것은 없다.

---

## 1. 검증 표

| ID | 상태 | 근거 |
|---|---|---|
| **A1** IMU 프레임 | **VERIFIED** (+1차 리뷰 정정, 4절) | `upkie_observers.py:33-34` 기본값 diag(−1,1,−1)/diag(1,−1,−1), `:41` 합성 순서, `:43` ω = R_b2iᵀ·ω_imu. upkie `BaseOrientation.h:35,147,161-164`와 같다. 시뮬은 항등 행렬을 명시한다(`plan1_upkie.py:68`). [실측] pi3hat ARS 쿼터니언 왕복에서 피치 +0.2/−0.3 rad와 자이로를 그대로 복원한다. 시뮬 센서는 여전히 `base` 사이트에 있어 실기 IMU 경로는 시뮬에서 시험되지 않는다(제안 사항 미반영). |
| **A3** 피치 영점 | **VERIFIED**, 새 결함 **N3** 동반 | `plan1_upkie.py:46-47` `pitch_offset`, `:90`, `:236-237` 웅크림 때 재계산. [실측] 1.313° → (−4 cm) −4.197° → (복귀) 1.313°. `"none"`이면 0이고 3 s 기립한다. |
| **A4** MPC 진실 접지 | VERIFIED (문서) | 비교보고서 4.1 각주 ①과 10절 각주, `plan1_upkie.py:79-81`, `plan3_diablo_upkie.py:29-31`. |
| **A5** 노이즈 σ | VERIFIED (문서) | 비교보고서 26행이 σ≈12.7°/s와 0.073→0.032 Nm 영향을 명시한다. `common.py:40` 모델은 의도적으로 유지했다. |
| **A6** 접지 파라미터 | **PARTIAL** | 코드는 `upkie_observers.py:53` = 0.015(upkie `spine_backend` 기본값)로 고쳤다. 그러나 **비교보고서 6절 161행**에 "최소 토크 0.05 Nm … 저장소에 없어 추정값" 문장이 그대로 남아 10절 표와 모순된다. |
| **A7** 필터 Nyquist | VERIFIED | `filters.py:17-18` `ValueError`. 현재 호출 경로(1 kHz에서 0.01·0.2·1.0 s, balancer_node 200 Hz에서 0.1·1.0 s)에는 위반이 없다 [코드]. `spine_hz` ≤ 200이면 `Plan1Upkie` 생성 때 바로 예외가 난다. 조용히 꺼지던 이전 동작보다 낫다. |
| **A8** gyropod 요 | VERIFIED | `gyropod.py:56-58` 바퀴별 차동 형태. left_wheeled=True에서는 upkie `upkie_gyropod.py:316-325`와 대수적으로 같다. [실측] 월드 요레이트: +0.5 rad/s 명령에 Plan1 PI +0.426, Plan1 MPC +0.419, Plan3 PI·MPC +0.433. −0.5 명령은 부호만 반대로 대칭이다. 참고: left_wheeled=False이면 upkie 원본과 요 방향이 다르다(원본은 요 항에 left_sign을 곱하지 않음). 새 코드가 물리적으로 일관된 쪽이며, 현재 호출부는 모두 True다. |
| **A9** 로그 시간 | VERIFIED | `plan1_upkie.py:161`, `plan2_diablo_lqi6.py:150` `t=self.sim.time`. |
| **A10** MPC 다리 게인 | VERIFIED | `plan1_upkie.py:143`, `balancer_node.py:131`(2+2). Plan 3 MPC는 여전히 2.0 고정이다(N8). |
| **B1** 피치 관성 | VERIFIED | `plan2_diablo_lqi6.py:52-61`, `plant6.py:106`. [실측] base 프레임 대각 (0.04617, **0.02336**, 0.03796), `plant.Im` = 0.02336. K_T(0.198)의 φ항 2.928(보고서 2.93과 일치). |
| **B2** 명령 aliasing | VERIFIED | `host_link.py:87` deepcopy, `:103` 끊김 프로그램의 mode 복사, 테스트 `test_received_packet_is_a_copy`. [실측] 벤치 조건 링크 단절에서 끊김 뒤 +0.25 s 0.549 m/s, +0.49 s 0.598 m/s로 명령대로 계속 달린다. 2.491 s에 해제되고 6 s 뒤 0.005 m/s다. |
| **B3/C4** 주저앉기 | VERIFIED | `mcu.py:48,165-170`, `host_link.py:38,49`. [실측] DIABLO teleop `v`(mode_mark, stand=False, 높이속도모드)를 받아도 STAND를 유지하고 이벤트를 남긴다. `allow_transform_down=True`이면 1.30 s에 CAR, 2.14 s에 FAULT(옵션의 의미가 맞음). ROS 경로(`McuNode`를 직접 실행)도 STAND를 유지한다. |
| **B4** 안전 규정 | **PARTIAL** | FAULT·비무장 시 6관절 토크 0 [실측: Plan 2·3 모두 `maximum_torque`, kp/kd, ff가 0]. 그러나 (1) arm 게이트가 리셋 직후 첫 사이클에 측정 없이 통과한다(**N1**). (2) "2 s OFF"는 옵션(기본 0, ROS 파라미터 없음)이고, 켜면 노드의 자동 arm과 매 사이클 arm/disarm을 반복한다(**N2**). (3) 드라이버 피드백 50 ms 타임아웃은 구현되지 않았다. |
| **B5** windup | VERIFIED | `law.py:27-29` 비균형 시 reset, `lqi6.py:97-105` 조건부 적분. [실측] FAULT 1 s 뒤 z = 0, v_ramp = 0. 포화 판정은 근사다(N10). |
| **B6** 온라인 DARE | VERIFIED | `lqi6.py:49-59` 표(0.10~0.28 m, 19점), `:77-83` 보간·클립, `:84-89` 온라인 경로 DARE 가드. [실측] 0.0025 m 간격 73점에서 표 대 직접 설계 최대 상대오차 **0.033%**, 표 게인 폐루프 최대 \|λ\| 0.99906. 격자 밖(0.05/0.09/0.30/0.35 m)에서 클립된 게인도 안정하다(\|λ\| ≤ 0.99949). |
| **B7** 폐루프 구속 | VERIFIED | `build_diablo_lite.py:133-134` solref 1 ms(물리 스텝 0.5 ms의 2배). [실측] 기립 잔차 **0.0015 mm**, 1.5 W 푸시 중 최대 0.0024 mm(수정 전 2.7/5 mm). |
| **B8** 롤 | VERIFIED | `mcu.py:246-252` 기하 피드포워드 + PI. [실측] roll_ref ±0.1 → ±0.1055 rad(3 s 뒤). PI 시정수는 (1 + kp/track)/(ki/track) ≈ 2.9 s라 남은 오차가 천천히 준다. |
| **B9** 끊김 프로그램 | VERIFIED | `host_link.py:96-104`. [실측] 높이속도모드 up = 0.02에서 끊긴 뒤 타임아웃 전 0.5 s 동안만 더 늘고(0.208→0.2176 m) 해제 뒤 멈춘다. |
| **B10** F_ff/cosθ | VERIFIED | `mcu.py:273` `F_ff / max(cos θ, 0.5)`. |
| **B11** 월드 요레이트 | VERIFIED | `mcu.py:138` `(R @ gyro)[2]`. [실측] Plan 2 요 ±0.5 명령에 ±0.490 rad/s. |
| **B13** L0 가드 | VERIFIED | `fivebar.py:72`, `mcu.py:280-282` 유한성 검사. |
| **C1** ROS 요 부호 | VERIFIED (10절 서술 일부 부정확) | `spine_node.py:244`가 `control_step`을 쓰고, `balancer_node.py:47,126`에 `yaw_sign` 파라미터가 있다. [실측, 노드 클래스 실행] 월드 요레이트: spine 내장 +0.426, 외부 PI +0.425, 외부 MPC +0.419 rad/s. 단 balancer_node는 `control_step`을 부르지 않고 로직을 복제한다(`balancer_node.py:122-136`). 그래서 "두 노드가 공용"은 spine에만 맞고, 요 부호도 두 곳(`cfg.yaw_sign`, 노드 파라미터)에 있다. |
| **C2** 외부 MPC | VERIFIED (launch 평가는 정적) | `sim.launch.py:62-67`. [실측] spine 외부 + MPC: `use_sim_contact=True`이면 3 s 서 있고 요 +0.419. False이면 0.978 s에 넘어진다. 따라서 launch에서 강제하는 것이 필요하고 충분하다. `PythonExpression` → bool 변환은 기존 `embedded` 인자와 같은 방식이다 [미검증 런타임]. |
| **C3** 패킷 합치기 | VERIFIED | `mcu_node.py:133,236,285-289`. [실측] start_mode car에서 teleop `z`의 3패킷을 한 사이클 안에 넣으면, 큐 덕분에 0.0 s에 TRANSITION_UP, 그 뒤 STAND가 되고 3 s 서 있다. 마지막 패킷만 남기면(수정 전 동작) CAR에 머물다 0.62 s에 FAULT가 난다. |
| **C5** /cmd_vel 타임아웃 | VERIFIED | `spine_node.py:86`·`spine.yaml:20`, `balancer.yaml:10`, `cmd_vel_bridge.py:32` 모두 500 ms. README 157-159행 설명도 고쳤다. [실측] spine 내장에서 /cmd_vel이 1.5 s 끊긴 뒤 요레이트 0.005 rad/s. |
| **C6** 외부 균형기 시작·리셋 | VERIFIED (nit N6) | `spine_node.py:245-248`: [실측] 첫 명령 전 300 사이클 동안 sim.time 0.0을 유지하면서 관측 60건(200 Hz)을 계속 발행한다. 내장 경로는 정상으로 진행한다. `balancer_node.py:98-106`: [실측] 0.8 m/s로 3 s 주행할 때 관측당 이동은 최대 7.6 mm(임계 0.1 m의 1/13)이고 오탐은 0건이다. 주행 뒤 리셋하면 재초기화 1회, 리셋 후 피치 피크 1.39°. |
| **C7** LegMotors | VERIFIED | `mcu_node.py:336-338`. [실측] 6개 모터 모두 pos ∈ [0, 2π), \|pos + rev·2π − q\| ≤ 3·10⁻¹⁶. DIABLO 소비자 식(`osdk_telemetry.cpp:352`)과 일치한다. |
| **C10** base_pitch 의미 | **PARTIAL** | `SpineObservation.msg:2-4` 주석을 고쳤다. 그러나 주석이 가리키는 "spine's pitch_offset setting"이 spine_node 파라미터로 없다(`spine_node.py:106-109`는 기본값 `"model"`로 고정). ROS에서 원시 피치를 받을 방법이 없다. |

---

## 2. 수정으로 생긴 새 결함

### N1. [minor·실기] arm 게이트가 측정 전 기본 추정값으로 통과

- **위치**: `mcu.py:101`(reset이 `Estimate()` 생성, pitch = 0), `mcu.py:116-118`(`arm()`이 `self.est.pitch`만 봄), `mcu_node.py:156-158, 283, 290-292`(disarm 직후 같은 사이클에서 tick 전에 `arm()` 호출)
- **현상** `[실측]`:
  - 10°로 놓은 로봇에서 reset 직후 `est.pitch = 0.0`이고 `arm()`은 True를 돌려준다. tick을 한 번 돈 뒤에는 10.0°가 보이고 False가 나온다.
  - `McuNode`는 시작할 때와 `~/reset` 뒤 모두 첫 사이클(t = 0.0)에 IMU를 한 번도 읽지 않고 arm된다(이벤트 `(0.0,'waiting…'), (0.0,'armed')`).
  - README의 "arms itself once |pitch| < 3 deg"가 지켜지지 않는다.
- **영향**: 시뮬은 항상 직립으로 리셋하므로 무해하다. DiabloMcu는 펌웨어 모델이므로 실기에서 같은 순서를 쓰면 문제가 된다. 3°~34°로 기대 놓은 로봇이 켜지자마자 균형 토크를 낸다. 34° 초과는 첫 tick에서 FAULT가 막는다.
- **수정**: `estimate()`에서 `self.est_valid = True`로 표시하고, `reset()`에서 False로 둔다. `arm()`은 `est_valid and |pitch| < arm_pitch`일 때만 통과시킨다. 또는 노드가 tick 뒤에 arm을 시도한다.

### N2. [minor·잠재] `disconnect_off_s` > 0: 부팅 즉시 OFF, 노드 자동 arm과 1 kHz로 진동

- **위치**: `mcu.py:223-224`, `host_link.py:73`(`last_rx_time = -1e9`), `mcu_node.py:290-292`
- **현상** `[실측]`:
  - `McuNode`에서 `disconnect_off_s = 2.0`으로 두고 패킷을 보내지 않으면, 한 번도 연결된 적이 없는데도 첫 tick에 "host link lost for 2.0 s"로 disarm된다.
  - 다음 사이클에 노드가 다시 arm하고, tick이 다시 disarm한다. 0.3 s 동안 이벤트가 601개 쌓였다. `mcu.events`는 무한히 커지고 status 타이머가 그것을 모두 로그로 찍는다.
  - 토크는 계속 0이라 0.3 s 안에 넘어진다.
  - 벤치 하네스(`armed=True`로 reset)에서는 의도대로 동작한다. 링크 단절 2 s 뒤 OFF 되고 넘어진다(`alive False`, 3.992 s).
- **영향**: 기본값이 0이고 ROS 파라미터도 없어 지금은 잠재 결함이다. 그러나 B4의 "2 s OFF"를 실제로 쓰려는 유일한 경로(ROS/실기)에서 깨진다.
- **수정**:
  - "연결된 적 있음"(`link.ever_connected`)일 때만 카운트하거나, reset 때 `last_rx_time = t`로 둔다.
  - OFF를 래치 상태(`link_off`)로 둔다. 새 패킷이 오고 명시적으로 arm해야만 해제한다.
  - 노드의 자동 arm 조건에 `not link_off`를 넣는다.
  - 이벤트는 상태가 바뀔 때만 남긴다.

### N3. [minor] A3 재계산이 IMU 영점을 계단으로 바꿔 Plan 1 PI가 4 cm 웅크림에서 넘어짐

- **위치**: `plan1_upkie.py:236-237`(`set_height_offset`이 목표 자세 기준 `pitch_eq`를 즉시 대입). 다리는 `GyropodMapper`의 1 s 저역필터로 천천히 움직인다.
- **현상** `[실측]`(노이즈 + 2 ms, 웅크림 2.5 s 구간 생존 여부):

  | 변형 | 2 cm | 4 cm | 6 cm |
  |---|---|---|---|
  | PI, 현재(계단) | 생존, 피크 3.8°, 이동 1.5 m | **전도** | — |
  | PI, 재계산 없음(수정 전) | 생존, 피크 6.2°, 이동 2.0 m | 생존, 피크 6.6°, 이동 4.0 m | — |
  | PI, 영점을 다리 필터와 같이 램프 | 생존, 피크 4.8°, 이동 0.42 m | 생존, 피크 6.5°, 이동 0.46 m | 생존, 피크 7.9°, 이동 0.48 m |
  | MPC, 현재(계단) | 생존, 피크 3.6° | 생존, 피크 5.6° | — |

  −4 cm 웅크림에서 영점은 1.31°에서 −4.20°로 5.5° 바로 뛴다.
- **영향**: 비교보고서의 "Plan 1은 2 cm 웅크림도 전도(구조적 한계)"라는 서술이 일부는 이 하네스 산물이다. Plan 2 추천은 바뀌지 않는다.
- **수정**: `pitch_eq`를 `leg_cutoff_period`와 같은 1차 필터로 목표값에 따라가게 한다. 또는 `mapper.leg_positions`(필터된 다리 목표)로 매 주기 표에서 보간한다.

### N4. [nit·지표] `settle_time` 이동평균의 0 패딩이 "미정착"을 "끝 직전 정착"으로 바꿈

- **위치**: `battery.py:73-76` (`np.convolve(..., mode="same")`)
- **현상** `[실측]`:
  - 피치가 1.2°나 1.5°로 끝까지 일정하면, 1° 안에 한 번도 들어오지 않았는데도 `settle_time`이 7.993 s나 7.996 s를 돌려준다. 끝의 10개 샘플 평균이 0 패딩으로 절반 가까이 깎이기 때문이다.
  - 실제 벤치 결과에서도 `summary.json`의 0.25 W 푸시 6건(plan3_pid 기본·게인 1.5/2/3배, plan1_pid 게인 1.5/2배)이 정착 7.991~7.996 s(9 s 실행의 끝 4~9 ms 전)로 기록되어 있다. 재현(plan3_pid, 0.25 W)해 보면 마지막 0.5 s 피치가 0.77~1.83°이고, 가장자리를 복제 패딩하면 "미정착"이다.
- **영향**: 4.1 표의 0.5 W 열과 순위는 바뀌지 않는다. `pi_gain_sweep` 그림과 4.2절의 "정착시간은 8 s 가까이 그대로"는 실제로는 "창 안에서 미정착"이다.
- **수정**: `np.pad(x, (n//2, n-1-n//2), mode="edge")` 뒤 `mode="valid"`로 계산하거나 후행(causal) 평균을 쓴다. 또는 `idx[-1] >= len(t) - n`이면 None을 돌려준다.

### N5. [nit] `SpineObservation.msg` 주석이 존재하지 않는 spine 설정을 가리킴

C10 PARTIAL 참조. `spine_node`에 `pitch_offset` 파라미터를 선언해 `Plan1Config`로 넘기거나, 메시지에 `base_pitch_raw`를 추가한다(`bo["pitch_raw"]`는 이미 계산되어 있다, `plan1_upkie.py:121`).

### N6. [nit] 오도메트리 점프 리셋 감지의 누락(false negative)

- **위치**: `balancer_node.py:98`
- **현상** `[실측]`: 오도메트리가 0.069 m일 때 spine을 리셋하면 재초기화가 일어나지 않는다. 적분 −0.046 m/s가 그대로 남고, 리셋 뒤 피치 피크가 3.54°다(재초기화된 경우 1.39°). 넘어지지는 않는다. 오탐은 0.8 m/s, 200 Hz에서 없다(1절 C6).
- **수정**: spine이 리셋 카운터(`uint32 reset_count`)를 `SpineObservation`에 실어 보내거나, 리셋 때 토픽을 하나 발행한다.

### N7. [nit] 문서·주석이 수정 내용과 어긋남

- 비교보고서 6절 161행: 접지 파라미터 "0.05 Nm … 추정값"(A6).
- 비교보고서 6절 160행: "LQI6는 온라인 리카티"(지금은 시작 때 격자 표를 계산한다).
- 비교보고서 2.2절 60행과 9절 202행: "13개 통과"·"13 tests"(지금은 16개).
- `lqi6.py:1-5` 모듈 docstring: "re-designed whenever the leg length changes by more than 1 cm"(기본은 표 보간).
- `mcu_node.py:197-198` 주석: "host_link's up = 1.0 default"(지금 기본값은 0.0).

### N8. [nit] Plan 3 MPC의 다리 게인 스케일은 여전히 2.0 고정

`upkie_law.py:92`. A10을 Plan 1에만 반영했다. 회전 중(\|yaw_ref\| > 0.1)에는 4.0으로 맞춘다.

### N9. [nit] 주저앉기 거부 이벤트가 프로세스 수명 동안 한 번만 기록됨

`mcu.py:168-170`. `_down_refused`를 `reset()`에서 지우지 않는다. 리셋 뒤 다시 거부해도 로그가 남지 않는다. `reset()`에서 False로 둔다.

### N10. [nit] 조건부 적분의 포화 판정이 근사

`lqi6.py:102`. `|T| > 2·τ_wheel`은 요 차동 토크(±0.5·τ_yaw)를 무시한다. `|Tp| > 2·τ_hip`은 VMC 사상(다리 힘 F와 Jᵀ)을 무시한다. 그래서 실제 모터 포화와 시점이 다르다. 영향은 작다. 엄밀하게 하려면 MCU가 클램프 여부를 법칙에 돌려준다.

---

## 3. 이번 수정 범위 밖에서 확인한 남은 문제(새 결함 아님)

- **`start_mode: car`는 lite에서 쓸 수 없다** `[실측]`: CAR에서는 균형을 끄므로 명령 없이 0.62 s에 FAULT가 난다. `cmd_vel_bridge`의 기립 명령(`stand_delay_s` 1.0 s)보다 먼저다. FAULT가 래치되어 다시 서지 못한다. 상태기계는 FAULT 중에도 전이하므로 `robot_mode_msg = 3`(STAND)이면서 `error_msg` bit 0이 켜진 상태로 누워 있다(`mcu.py:203-218`이 FAULT 검사보다 앞에 있음). `mcu.yaml`과 README는 `car`를 선택지로 제시한다. 선택지에서 빼거나, 1차 리뷰 B3 제안대로 CAR에서도 L_min으로 균형을 잡는다.
- 수정 대상이 아니었던 항목: A2, B12, B14, C8, C9, C11, C12(`ros2_ws/README.md:44`의 `--break-system-packages` 그대로). 10절에 "미반영"으로 정직하게 적혀 있다.

## 4. 1차 리뷰 정정 (A1 심각도)

`[실측]` 임의 자세 2000개에서 upkie 기본 회전을 적용한 경우와 항등 행렬로 ARS 쿼터니언을 그대로 읽은 경우를 비교했다. **피치 차이와 피치율(ω_y) 차이는 모두 0.0이었다.** ω_x와 ω_z는 부호가 뒤집혀 최대 7.9 rad/s 차이가 났다.

이유는 다음과 같다. diag(1,−1,−1)은 x축 180° 회전이고 diag(−1,1,−1)은 y축 180° 회전이다. 둘을 앞뒤로 곱하면 프레임 축의 월드 x성분만 뒤집힌다. "자기 heading 방향으로 기운 각"인 upkie 피치는 이 변환에 불변이다.

따라서 1차 리뷰 A1의 "실기에서 피치 부호·크기가 틀려 첫 사이클부터 반대로 구동한다"는 틀렸다. WheelBalancer는 피치만, MPC는 피치와 ω[1]만 쓰므로 영향이 없다. 실제 영향은 `SpineObservation.base_angular_velocity`의 x·z 성분과 롤·요 회전에 한정되며 minor다. 그래도 이번 수정은 upkie와 같게 맞춘 것이라 옳다.

## 5. 단위 테스트

`python -m unittest discover -s tests -v` → **Ran 16 tests, OK (1.18 s)**. 추가를 권하는 테스트:

1. reset 직후 기울어진 로봇에서 `arm()`이 False인지(N1).
2. `disconnect_off_s` > 0일 때 부팅 직후 disarm되지 않고 이벤트가 상태 변화 때만 쌓이는지(N2).
3. `settle_time`이 끝까지 1°를 넘는 신호에 None을 돌려주는지(N4).
4. Plan 1 −4 cm 웅크림 생존(N3).
5. 게인 표의 격자 중점 오차(현재 0.205 한 점만 시험함).

## 6. 판정

반영 작업은 대체로 정확하고 완결적이다.

- 실측으로 다시 확인한 것: 요 부호(세 플랜, ROS 세 경로), 피치 관성(0.0234), 구속 잔차(0.0015 mm), 링크 단절 시험의 타당성, 주저앉기 거부, FAULT 전 관절 토크 0, 게인 표(오차 0.033%), 패킷 큐(teleop `z` 보존), 오도메트리 점프 감지(오탐 없음), LegMotors 규약.
- REGRESSION은 없다. 16개 테스트가 통과한다.
- 남은 일:
  1. **실기 전 필수**: B4 안전 경로의 두 구멍. N1(arm이 측정 전 통과)과 N2(2 s OFF 옵션이 노드에서 진동)를 고치고, 드라이버 피드백 타임아웃을 추가한다.
  2. **벤치 해석**: N3(Plan 1 웅크림 결론 일부가 하네스 산물)과 N4(0.25 W 푸시 "정착 ~8 s"는 미정착)를 고친 뒤 재실행해 표와 그림을 갱신한다.
  3. **문서 정리**: A6 문장, 테스트 개수, 온라인 리카티 서술, C10 파라미터(N5·N7).

Plan 2 추천 결론은 그대로 유지된다.
