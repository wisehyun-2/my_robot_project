# my_robot_project

6모터 휠 이족 로봇 v5의 CAD, 제어기, 시뮬레이션, ROS 2 패키지(만드는 중)를 담은 저장소다.
각 폴더의 README가 그 폴더의 기준 문서다. Claude Code가 지킬 규칙은 `CLAUDE.md`에 있다.

## 지금 하는 일 (2026-11-15)

- **v5 로봇의 ROS 2 Jazzy 패키지를 만든다.** 기준 문서는 `wheeled_biped/ros2_ws/README.md`다.
  - 0절: 두 컴퓨터 작업법
  - 2절: 정해진 것
  - 3절: 패키지 구조 안
  - 4절: 실물에 필요한 것
  - 5절: 옮길 때 조심할 것
  - 6절: 작업 순서와 통과 조건
  - 7절: 정할 것·확인 못 한 것
  - 패키지는 아직 없다(`ros2_ws/src` 없음).
- 실물의 설정 순서, 부호·영점·IMU 시험: `wheeled_biped/docs/bringup_step0.md`.
- 미뤄 둔 것(11-15 사용자)
  - 단계 시험 보고서를 다시 만든다(`wheeled_biped/v5_lqi/TEST/README.md` 6절).
  - 시뮬레이션의 IMU 위치를 실물 칩 ICM-42688-P 자리로 옮긴다(`wheeled_biped/docs/robot_v5_model.md` 9절).
- v5 로봇: 6모터 휠 이족, AKE60-8 × 6, Raspberry Pi 4B + pi3hat, 링크 25 cm, 바퀴 반경 0.101 m, 제어기 모델 질량 11.597 kg(`wheeled_biped/models/wheelbiped_v5/plant_params.json`).
- 균형 제어기는 LQI8(`wheeled_biped/v5_lqi/`)이다. MPC(`wheeled_biped/v5_mpc/`)는 나중에 같은 자리에 끼운다. 실물의 MCU 프로그램은 C++ `wbr_controller`로 만든다(아직 없음).
- 방향 규약
  - 로봇의 앞 = CAD −x(무릎 반대쪽, 셸 조각 `shell_front` 쪽)
  - `_L` 부품 = 로봇 왼쪽 = CAD −y
  - 관절 부호는 모두 +1, `KNEE_SIGN` +1, `R_base_to_imu` = diag(−1, −1, 1)
  - 로커 스위치는 허브 버전 18(11-09)부터 윗면(CAD x +78, 중심보다 뒤)에 있다. 앞뒤를 스위치로 판단하지 않는다. "스위치 쪽"이라는 옛 표현은 11-15에 문서·코드에서 없앴다.

## 저장소 구성

로봇 폴더는 `robot_cad_v5/`와 `wheeled_biped/`만 둔다(2026-11-14 사용자).

### `wheeled_biped/` — 제어기·모델·MuJoCo 검증

ROS에 의존하지 않는 Python 패키지 `wbr`이 중심이다. `README.md`에 다음이 있다: 폐루프 구성, 설치·실행 명령, 시험, 폴더·문서·결과 구성, 지운 것의 이력.

- **v5 로봇만 있다.** v5가 쓰지 않는 코드(upkie 모델, 이전 제어 스택, 기능을 끄는 스위치)는 두지 않는다.
- MCU의 v5 기능은 늘 켜져 있고 끄는 스위치가 없다: 순응 롤, 무릎 결합 보상, 서스펜션, 바퀴 속도 차 요, 고르지 않은 바닥 감속, MCU 자세 필터, 정적 다리 힘.
- `v5_lqi/`, `v5_mpc/` — 균형 제어기 폴더 두 개(LQI8, MCU 안의 MPC).
  - 바깥 루프·모델·IMU 경로는 같이 쓴다.
  - 균형 제어기를 바꾸는 자리는 `ClosedLoop._make_law`다. v5 폐루프는 공용 런타임 `wbr/plans/closed_loop.py` 위의 `plan_v5.py`다.
- 단계 시험
  - `wheeled_biped/`에서 `python v5_lqi/TEST/run_stages.py --stage 4 --keep-going`으로 돌린다(MPC는 `v5_mpc/TEST/run_stages.py`).
  - 전체 제어기(`--stage 4`)만 쓴다.
  - 지금 상태와 기준은 `v5_lqi/TEST/README.md`에 있다. 넘어짐·무릎 꺾임은 없다.
- `docs/robot_v5_model.md` — v5 모델, IMU 경로, 바깥 루프의 기록·설계 근거와 남은 일. `docs/bringup_step0.md` — 실물 설정 절차.
- 합격 기준: 시나리오·지표는 `wbr/bench/acceptance.py`, 기준 숫자는 `wbr/bench/stage_tests.py`와 `v5_lqi/TEST/README.md` 3절에 있다.
- 기록
  - 11-15에 GitHub에서 `docs/`의 옛 문서(`합격기준.md`, `HANDOFF.md` 등)와 `results/`의 `bench`·`figures`·`videos`·`torque_sizing`을 지웠다. 내용은 `git show f76f3b0:<경로>`로 본다.
  - 같이 지워졌던 `wbr/sim/`(MuJoCo 백엔드)과 `docs/bringup_step0.md`는 쓰는 것이라 11-15에 되살렸다.
  - 11-15 정리(사용자): Plan 4에서 온 이름을 바꿨다(`closed_loop.py`, `ClosedLoop`, `verify_plant.py`). 단계 시험은 전체 제어기 하나만 남기고 옛 보고서를 지웠다(다시 만드는 일이 남았다). 금속 가공 부품은 쓰지 않는다. 자세한 것은 각 폴더 README의 이력에 있다.

### `robot_cad_v5/` — 지금 로봇의 Fusion 모델

- 허브 `WheelBiped_v5` 버전 18 = `export/`.
- 모든 부품을 PLA·PETG로 출력한다(11-09 사용자, 강도는 보지 않음). 금속 가공 부품은 쓰지 않는다(11-15 사용자). 모델 질량 11.610 kg과 제어기 모델의 11.597 kg은 아직 금속 밀도로 계산된 값이라 실제보다 약 0.89 kg 무겁다(`robot_cad_v5/README.md` 5절, Fusion에서 다시 뽑아야 한다).
- 제어기 모델은 `calc/*.json`과 `export/links_stl`에서 `python -m wbr.models.build_v5`로 만든다.
- `fusion_base/`는 몸통·셸·구매품·바퀴를 만드는 바탕 코드다(v4에서 물려받음, 11-15에 `v4/fusion`에서 이름을 바꿈). `fusion/v5run.py`가 불러서 v5 코드로 덮어쓴다.
  - 11-14에 `robot_cad_v4/fusion`에서 옮긴 뒤 같은 날 Fusion에서 확인했다.
    - 허브 `WheelBiped_v5` 버전 18이 열린다: v5 태그 있음, 부품 536개, 몸체 801개 중 숨김 0, as-built 회전 조인트 6개(hip·knee·wheel × 좌우), 외부 참조 0.
    - `v5run.load()`가 바탕 모듈을 모두 바탕 코드 폴더에서, v5 모듈을 `robot_cad_v5/fusion`에서 불러온다.
    - `v5run.report()`가 돈다(다리 몸체 16개).
  - `build()`와 허브 문서를 바꾸는 단계는 돌려 보지 않았다. 11-15의 폴더 이름 변경 뒤에는 Fusion에서 아직 아무것도 돌려 보지 않았다(Fusion 밖에서 `load()`가 도는 것만 확인).
- 지난 판(v2~v4) README는 11-15에 지웠다. 아직 맞는 내용(엔코더 정렬, 포트 이름 규칙, 몸통 하중 경로)은 `robot_cad_v5/README.md` 2절에 있다.
- CAN 포트 배정은 `wheeled_biped/ros2_ws/README.md` 2절이 기준이다.
- 11-14에 옛 CAD 폴더를 지울 때 쓰는 것만 옮겼다.
  - v4 코드 → 지금의 `robot_cad_v5/fusion_base/`(v4 계산 파일은 11-15에 지움)
  - v3의 `hlr_gmsh.py`와 치수 참고용 v2 도면·STEP → `robot_cad_v5/manufacturing/`
  - 세 README → `robot_cad_v5/docs/`(11-15에 지움, `git show e16135d:robot_cad_v5/docs/v2/README.md`)
  - 지운 나머지는 git 태그 `before-cleanup-2026-11-14`에 있다.

### `references/` — 참조 저장소의 git 서브모듈

목록·커밋·받는 법은 `references/README.md`에, 참조 코드를 쓸 때의 라이선스 규칙은 `wheeled_biped/ros2_ws/README.md` 1절 (d)에 있다.

- 지금 쓰는 것(2026-11-14 사용자 정리)
  - mjbots: `pi3hat`, `moteus`, `power_dist`, `fdcanusb`, `moteus_ros2`
  - `upkie`
  - DDT·TITA: `ddt_ros2_control`, `TITA_ROS2_Control_Sim`, `TITA-Development-Manual`
  - DIABLO: `diablo_ros2`, `diablo_sdk`, `diablo-sdk-docs`
  - README 1절 (d)의 11-12에 더한 다섯 개: `tita_hardware_ros2_control`, `pi3hat_hw_bartlomiej`, `pi3hat_robotic_systems`, `pi3hat_hw_glevine`, `ros2_control_demos`
- 11-14에 지운 것
  - `upkie_parts`, `upkie_description`: upkie 모델을 쓰지 않는다.
  - `mjlab_upkie`, `Col_DIABLO_Issac_RL`: 강화학습, 범위 밖.
  - `diablo-sim-env`, `DiabloSTLSDF`: 삭제된 Plan 2용.
  - `TITA-SDK-ROS2`, `TITA_CAN_Inerface`: TITA 보드 CAN 규약. README 1절에서 "쓰지 않는 것"으로 정했다.

## 컴퓨터

Windows PC, Ubuntu 노트북, Pi 4가 브랜치 `ros2-v5` 하나를 같이 쓰고, 동시에 쓰지 않는다. 작업은 `ros2-v5`에서만 하고, 프로젝트가 끝나면 `main`에 합친다(11-15 사용자).

- Windows PC
  - ROS 2와 C++ 컴파일러가 없다.
  - venv(`C:\Users\User\Desktop\myrobot\.venv`)에 mujoco 3.16.0이 있지만, Smart App Control(켜짐)이 그 `.pyd`를 막아 import가 실패한다(2026-11-14 확인).
  - MuJoCo 없이 도는 시험은 `tests/test_core.py`, `tests/test_leg_plant.py`뿐이다.
- C++ 빌드, ROS 실행, MuJoCo 시험은 노트북과 Pi에서 한다.
