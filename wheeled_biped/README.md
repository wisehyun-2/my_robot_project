# wheeled_biped — v5 휠 이족 로봇의 제어기·모델·MuJoCo 검증

ROS에 의존하지 않는 Python 패키지 `wbr`이 중심이고, MuJoCo로 검증한다. v5 로봇만 있다.
실물의 ROS 2 쪽은 Jazzy 위에서 TITA `ros2_control` 구조를 따른다(`ros2_ws/README.md`, 패키지는 아직 없다).

| 항목 | 내용 |
|---|---|
| 모델 | Fusion에서 뽑은 `models/wheelbiped_v5`(AKE60-8 × 6, 링크 25 cm, 바퀴 반경 0.101 m), 실물과 같은 IMU 경로 |
| 제어 | DIABLO 방식 MCU(1 kHz) + 균형 제어기 LQI8(`v5_lqi/`, 지금 쓰는 것) 또는 MCU 안의 MPC(`v5_mpc/`) |
| 진입점 | `wbr.plans.plan_v5.PlanV5`(공용 런타임 `wbr.plans.closed_loop.ClosedLoop` 위) |
| 단계 시험 | `v5_lqi/TEST`, `v5_mpc/TEST` |

## 폐루프

아래 기능은 늘 켜져 있고 끄는 스위치가 없다. 게인은 로봇의 플랜트에서 계산해(`wbr.bench.ake_cad.v5_gains`) `models/wheelbiped_v5/description.json`에 둔다.

- **IMU**(`wbr/plans/plan_v5.py`): pi3hat 칩 위치의 MuJoCo 사이트 → 센서 노이즈 → pi3hat 펌웨어 필터 포팅(`wbr/observers/pi3hat_ars.py`) → 표본 지연 → MCU.
- **MCU 추정**(`wbr/controllers/diablo_arch/mcu.py`): 보드 축 변환(`base_orientation.py`, `R_base_to_imu` = diag(−1, −1, 1)), IMU 위치를 넣은 자세 필터(`body_attitude.py`), 바퀴로 오프셋을 배우는 요 속도, 5 ms 위치 차 관절 속도, NaN·오래된 표본 가드.
- **균형 제어기**: LQI8(`wbr/controllers/lqi8/`: 상태 8개, 다리 길이별 게인표, 2 m/s² 램프 + 0.3 s 필터) 또는 MPC(`v5_mpc/mpc_law.py`). 끼우는 자리는 `ClosedLoop._make_law`다.
- **바깥 루프**: 다리 길이(긴 쪽 다리 PD = 서스펜션, 정적 다리 힘, 다리별 댐퍼, 범위 끝의 무른 벽), 순응 롤, 요(자이로 PI + 바퀴 속도 차), 벌어짐 PD, 무릎의 바퀴 반작용 보상, 고르지 않은 바닥 감속, 마찰 보상.
- **안전**: 피치·롤 34°, NaN·오래된 표본, 유한하지 않은 토크면 FAULT(모든 관절 토크 0). `arm()`은 측정값이 없거나 기울었거나 다리가 범위 밖이면 거부한다. 호스트가 0.5 s 끊기면 주행·회전만 멈추고 균형은 계속한다.

## 설치

```bash
python -m venv .venv && source .venv/bin/activate     # Windows: .venv/Scripts/activate
pip install -e .
```

`python -m wbr.models.build_v5`는 Fusion 내보내기에서 `models/wheelbiped_v5`를 **덮어써서** 다시 만든다. CAD가 바뀌었을 때만 돌린다.

## 실행 (`wheeled_biped/`에서)

```bash
python -m unittest discover -s tests                      # 단위 시험
python v5_lqi/TEST/run_stages.py --stage 4 --keep-going   # 단계 시험, LQI8 (보고서 v5_lqi/TEST/results)
python v5_mpc/TEST/run_stages.py --stage 4 --keep-going   # 단계 시험, MPC
python -m wbr.viewer.course_v5                            # 한쪽 바퀴 과속방지턱을 넘는 실시간 창
python -m wbr.models.verify_plant                         # 플랜트와 MuJoCo 대조 → results/wheelbiped_v5_model
python -m wbr.bench.ake_cad                               # AKE 모터 점검(짐 없이 / 5 kg) → results/ake_cad_v5
```

- MuJoCo 없이 도는 단위 시험: `tests/test_core.py`, `tests/test_leg_plant.py`. 나머지(`test_robot_v5.py`, `test_runtime_v5.py`, `test_v5_mpc.py`)는 MuJoCo가 필요하다.
- Windows PC는 Smart App Control이 mujoco DLL을 막는다(11-14). MuJoCo 시험은 Ubuntu 노트북에서 돌린다.

## 폴더

```
wbr/common        필터, 회전, moteus 방식 서보 모델(upkie 코드 포팅, Apache-2.0)
wbr/sim           MuJoCo 백엔드(서보 PD, IMU 센서, 밀기, 접촉)
wbr/models        v5 모델 빌더(build_v5), 플랜트 파라미터(plant_params), 플랜트 대조(verify_plant)
wbr/observers     pi3hat 펌웨어 필터 포팅, MCU 자세 필터, IMU 축 변환
wbr/controllers   lqi8(직렬 다리 플랜트, LQI8, VMC), diablo_arch(호스트 링크, MCU 프로그램)
wbr/plans         closed_loop(폐루프 런타임), plan_v5(그 위의 v5 로봇), common(지연, 노이즈, 기록)
wbr/bench         단계 시험의 시나리오·지표(acceptance, stage_tests), 모터 점검(ake_cad, ake_sizing)
wbr/viewer        v5 코스 뷰어
v5_lqi/, v5_mpc/  균형 제어기 두 개와 단계 시험
tests/            단위 시험
models/           로봇 기술(wheelbiped_v5)
ros2_ws/          ROS 2 패키지 계획(README만)
docs/             robot_v5_model.md(모델·IMU 경로·바깥 루프의 기록과 설계 근거), bringup_step0.md(실물 설정 순서·부호·영점·IMU 시험)
results/          wheelbiped_v5_model(플랜트 대조), ake_cad_v5(모터 점검)
```

## 이력

- 11-11, 11-14: v5 이전의 제어 스택과 upkie 모델, v5 기능을 끄는 스위치를 지웠다(사용자: v5가 쓰지 않는 것은 두지 않는다).
- 11-15
  - GitHub에서 `docs/`의 옛 문서와 `results/`의 `bench`·`figures`·`videos`·`torque_sizing`을 지웠다. `wbr/sim`과 `docs/bringup_step0.md`는 쓰는 것이라 되살렸다.
  - 이름을 바꿨다: `wbr/plans/plan4_lqi8.py` → `closed_loop.py`(`ClosedLoop`, `ClosedLoopConfig`), `wbr/models/verify_plan4_plant.py` → `verify_plant.py`.
  - 단계 시험은 전체 제어기 하나만 남겼다(단계 1~3의 시험 파일과 단계 정의를 지움). 11-14 이전에 만든 저장 보고서는 지웠고, 다시 만드는 일이 남았다(`v5_lqi/TEST/README.md` 6절).
  - 안 쓰이는 함수와 의존성을 지웠고, 기본값을 v5 로봇 값으로 맞췄다. `fast-simplification`을 의존성에 넣었다(`build_v5`의 메시 축소).
  - 지운 것의 전문은 git 이력에 있다(11-15 정리 직전: `git show 39e448f:<경로>`, 문서 삭제 직전: `f76f3b0`).
