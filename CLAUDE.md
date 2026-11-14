# my_robot_project — Claude Code 프로젝트 지침


## 답변 규칙
- 모든 답변·진행 상황·질문은 **한국어**로 쓴다. 코드·경로·명령어는 그대로 둔다.
- **ROS 2 작업(`wheeled_biped/ros2_ws`, `wbr_hardware`·pi3hat·moteus 등)에서는 일반 지식으로 답하지 않는다.**
  - 먼저 `wheeled_biped/ros2_ws/README.md`의 해당 절과 `wheeled_biped/docs/bringup_step0.md`를 읽는다. 날짜를 붙여 정한 것은 확정으로 보고, 다른 안을 다시 꺼내지 않는다.
  - 라이브러리·펌웨어에 대한 말은 `references/`(이 저장소의 서브모듈, `references/README.md`)의 원본 소스를 직접 열어 보고 `파일:줄`을 붙인다. 줄 번호는 서브모듈에 고정된 커밋 기준이다. 서브모듈을 받지 않았거나 커밋이 다르면 그렇다고 밝힌다.
  - README와 소스 어디에도 없는 것은 "확인하지 못함"이라고 쓴다. 기억이나 일반 지식으로 메우지 않는다.

## 지금 하는 일 (2026-11-14)
- **v5 로봇의 ROS 2 Jazzy 패키지를 만든다.** 기준 문서는 `wheeled_biped/ros2_ws/README.md`다: 두 컴퓨터 작업법(0절), 정해진 것(2절), 패키지 구조 안(3절), 실물에 필요한 것(4절), 옮길 때 조심할 것(5절), 작업 순서와 통과 조건(6절), 정할 것·확인 못 한 것(7절). 패키지는 아직 없다(`ros2_ws/src` 없음).
- 실물의 설정 순서, 부호·영점·IMU 시험: `wheeled_biped/docs/bringup_step0.md`.
- v5 로봇: 6모터 휠 이족, AKE60-8 × 6, Raspberry Pi 4B + pi3hat, 링크 25 cm, 바퀴 반경 0.101 m, 제어기 모델 질량 11.597 kg(`wheeled_biped/models/wheelbiped_v5/plant_params.json`).
- 균형 제어기는 LQI8(`wheeled_biped/v5_lqi/`)이다. MPC(`wheeled_biped/v5_mpc/`)는 나중에 같은 자리에 끼운다. 실물의 MCU 프로그램은 C++ `wbr_controller`로 만든다(아직 없음).
- 방향 규약: 로봇의 앞 = CAD −x(무릎 반대쪽, 셸 조각 `shell_front` 쪽), `_L` 부품 = 로봇 왼쪽 = CAD −y. 관절 부호는 모두 +1, `KNEE_SIGN` +1, `R_base_to_imu` = diag(−1, −1, 1).
  - **문서에 남은 "스위치 쪽"은 앞쪽(옛 앞면 `shell_front`)을 뜻한다.** 로커 스위치는 허브 버전 18(11-09)에 윗면(CAD x +78, 중심보다 뒤)으로 옮겨져 지금은 앞면에 없다(`robot_cad_v5/fusion/switch_v5.py`). `bringup_step0.md`는 11-14에 "앞쪽"으로 고쳤고, `docs/robot_v5_model.md`, `v5_lqi/TEST/README.md`, `robot_cad_v5/README.md`에는 옛 표현이 남아 있다.

## 저장소 구성 (각 폴더의 README를 먼저 읽는다)
- `wheeled_biped/` — 제어기·모델·MuJoCo 검증. ROS에 의존하지 않는 Python 패키지 `wbr`이 중심이다. `README.md`에 설치·실행 명령·폴더 구성이 있다.
  - `v5_lqi/`, `v5_mpc/` — v5의 균형 제어기 폴더 두 개(LQI8, MCU 안의 MPC). 바깥 루프·모델·IMU 경로는 같이 쓴다. 균형 제어기를 바꾸는 자리는 `Plan4LQI8._make_law`다.
  - 단계 시험은 `wheeled_biped/`에서 `python v5_lqi/TEST/run_stages.py --stage 4 --keep-going`으로 돌린다(MPC는 `v5_mpc/TEST/run_stages.py`). 전체 제어기(`--stage 4`)만 쓴다. 지금 상태는 27개 중 7개가 일부 기준을 넘는 것이다(넘어짐·무릎 꺾임 없음, 기준 숫자는 바꾸지 않음, `v5_lqi/TEST/README.md`).
  - `docs/robot_v5_model.md` — v5 모델, IMU 경로, 바깥 루프의 기록과 남은 일.
  - Plan 4 — upkie 하드웨어 모델(`models/upkie_plan4`) 위의 하네스(`wbr/plans/plan4_lqi8.py`)와 합격 판정(`docs/합격기준.md`). v5와 LQI8 코드를 같이 쓴다. **합격 기준 수치·모터 한계·노이즈는 사용자 결정 없이 바꾸지 않는다.**
  - 기록으로만 남은 것: Plan 1·2·3은 2026-11-11에 삭제했다(코드는 git 이력, 비교는 `docs/비교보고서.md`와 `results/bench`). 균형 제어기 이름은 11-10에 LQI6 → LQI8로 바꿨고, 날짜 붙은 기록과 `results/`의 키·파일 이름(`plan4_lqi6` 등)은 옛 이름 그대로다. `docs/HANDOFF.md`는 10-22~11-11의 인수인계 기록이라 브랜치·컴퓨터·확정안이 그때 기준이다.
- `robot_cad_v5/` — 지금 로봇의 Fusion 모델. 허브 `WheelBiped_v5` 버전 18 = `export/`. 모든 부품을 PLA·PETG로 출력한다(11-09 사용자, 강도는 보지 않음). 모델 질량 11.610 kg은 금속 부품 기준이다. 제어기 모델은 `calc/*.json`과 `export/links_stl`에서 `python -m wbr.models.build_v5`로 만든다.
  - `v4/fusion`은 v5가 불러 쓰는 v4 코드다(`fusion/v5run.py`가 불러서 덮어쓴다). v4 파일은 고치지 않는다.
  - 몸통·셸·엔코더·배선의 바탕 설명은 지난 판 README에 있다: `docs/v4/README.md` → `docs/v3/README.md` → `docs/v2/README.md`. CAN 포트 배정은 `ros2_ws/README.md` 2절이 기준이다.
- `references/` — 참조 저장소의 git 서브모듈(아래 작업 규칙, `references/README.md`).
- **2026-11-14 사용자: 로봇 폴더는 `robot_cad_v5/`와 `wheeled_biped/`만 둔다.** `robot_cad`(v1)·`robot_cad_v2`·`v3`·`v4`·`balancing_robot_design`은 지웠고, 쓰는 것만 옮겼다: v4 코드·계산 → `robot_cad_v5/v4/`, v3의 `hlr_gmsh.py`와 v2 가공 도면·STEP → `robot_cad_v5/manufacturing/`, 세 README → `robot_cad_v5/docs/`, upkie 다리 STEP → `wheeled_biped/models/upkie_step/`, 2륜 보고서 두 편 → `wheeled_biped/docs/balancing_robot_design/`. 지운 나머지는 git 태그 `before-cleanup-2026-11-14`에 있다. `wbr.bench.ake_cad`의 기본 CAD는 v5다(`results/ake_cad`·`ake_cad_v3`·`ake_cad_v4`는 지운 판의 기록).
- `wheeled_biped/docs/balancing_robot_design/` — 10-21의 2륜 자가균형 로봇 `설계보고서.md`와 `제어기_비교보고서.md`. 그때의 "내가 만든 제어기"는 `MineLQI6`다(제어기_비교보고서 8절, 코드는 위 태그의 `balancing_robot_design/sim6/controllers6.py`).

## 작업 규칙
- 브랜치는 `ros2-v5` 하나다. Windows PC, Ubuntu 노트북, Pi 4가 같이 쓰고 동시에 쓰지 않는다. 작업을 시작하기 전에 `git pull`로 다른 컴퓨터의 변경을 받는다.
- **커밋·푸시는 사용자가 요청할 때만 한다.** 올리지 않은 변경을 남긴 채 작업을 끝낼 때는 그렇다고 알린다(다른 컴퓨터로 넘어가기 전에 올려야 한다). 커밋 메시지 끝에 `Co-Authored-By: Claude <실제로 작업한 모델 이름> <noreply@anthropic.com>`를 붙인다(예: `Claude Opus 5.5`).
- Claude Code 메모는 컴퓨터마다 따로다. 다른 컴퓨터가 알아야 할 결정은 해당 README에 날짜와 함께 적는다. 이 파일에는 한 줄 요약과 문서 위치만 적는다.
- `wbr` 코어는 ROS에 의존하지 않게 유지한다. 제어기·모델을 바꾸면 `wheeled_biped/`에서 `python -m unittest discover -s tests`와 단계 시험을 다시 돌리고 보고서(`v5_lqi/TEST/results/stage_report.md`)를 갱신한다.
- Windows PC에는 ROS 2와 C++ 컴파일러가 없다(`wheeled_biped/docs/HANDOFF.md` 5절). C++ 빌드와 ROS 실행은 노트북과 Pi에서 한다.
- 참조 저장소는 **이 저장소의 `references/` 아래 git 서브모듈**로 둔다(2026-11-14 사용자: 여러 컴퓨터가 같은 커밋을 보게). 커밋이 고정되어 있고, 받는 법·목록·라이선스 규칙은 `references/README.md`에 있다. 받을 때는 `git submodule update --init`(전부) 또는 경로를 지정해 일부만 받는다. 서브모듈 안의 파일은 고치지 않는다. 예전 위치 `../references/`(저장소 밖)는 쓰지 않는다.
  - 지금 쓰는 것(2026-11-14 사용자 정리): mjbots `pi3hat`·`moteus`·`power_dist`·`fdcanusb`·`moteus_ros2`, `upkie`, `ddt_ros2_control`·`TITA_ROS2_Control_Sim`·`TITA-Development-Manual`, `diablo_ros2`·`diablo_sdk`·`diablo-sdk-docs`, 그리고 README 1절 (d)의 11-12에 더한 다섯 개(`tita_hardware_ros2_control`, `pi3hat_hw_bartlomiej`, `pi3hat_robotic_systems`, `pi3hat_hw_glevine`, `ros2_control_demos`).
  - 11-14에 지운 것(다시 받지 않는다): `upkie_parts`(v1 CAD의 STEP은 `wheeled_biped/models/upkie_step`에 있음), `upkie_description`(저장소 안 `wheeled_biped/models/upkie_description`을 씀), `mjlab_upkie`·`Col_DIABLO_Issac_RL`(강화학습, 범위 밖), `diablo-sim-env`·`DiabloSTLSDF`(삭제된 Plan 2용), `TITA-SDK-ROS2`·`TITA_CAN_Inerface`(TITA 보드 CAN 규약, README 1절에서 "쓰지 않는 것").
- 이 저장소의 사본은 과거 파일 내용을 필요할 때만 받는 부분 클론일 수 있다(`git config remote.origin.partialclonefilter`가 `blob:none`). 그런 사본에서는 이력 전체의 내용을 훑는 명령(`git log -S`, `git log -p`, `git rev-list --objects --all` 등)을 쓰지 않는다. 한꺼번에 내려받아 `.git`이 수백 MB 커진다.
