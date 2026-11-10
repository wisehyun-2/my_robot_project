# 단계별 통과 시험 (MPC)

`v5_lqi/TEST`와 **같은 시나리오, 같은 지표, 같은 기준**으로 MPC를 시험한다. 공통 코드는 `wbr/bench/stage_tests.py`이고,
이 폴더의 `stage_common.py`는 MPC 단계(`v5_mpc/controller.py`의 `STAGES`)를 등록만 한다. 기준표, 시나리오 설명,
합격기준.md와 다른 점은 `v5_lqi/TEST/README.md` 3·4절을 그대로 따른다.

## 1. 단계

| 단계 | 균형 제어기 | `mpc_kwargs` | 이 단계에서 확인하는 것 | 파일 |
|---|---|---|---|---|
| 1 | MPC, 예측 상태 6개 `[θ, θ̇, v − v_r, φ, φ̇, x − x_r]` | `q_att=None, q_pos=None` | MPC만으로 서고, 기울기·밀기에서 복귀하고, 속도 스텝을 따라간다 | `test_stage1_mpc.py` |
| 2 | 단계 1과 같음 | 같음 | 바깥 루프 네 개(높이, 요, 롤, 벌어짐)가 MPC와 같이 제 일을 한다 | `test_stage2_outer_loops.py` |
| 3 | + ∫위치 상태 (z3) | `q_att=None, q_pos=0.5` | 일정한 외란 아래에서 출발 위치로 돌아온다. 단계 1·2 재통과 | `test_stage3_int_position.py` |
| 4 | + ∫기울기 상태 (z2) = 전체 MPC | `q_att=0.2, q_pos=0.5` | 일정한 외란 아래에서 실제 몸통이 수평으로 돌아온다. 단계 1~3 재통과 | `test_stage4_int_tilt.py` |

LQI 폴더와 단계 구성이 같고, 적분 상태가 LQR 대신 MPC의 예측 모델에 들어간다는 점만 다르다.

## 2. 실행 (`wheeled_biped/`에서)

```
python v5_mpc/TEST/run_stages.py --stage 4 --keep-going    # 전체 MPC. 보고서: v5_mpc/TEST/results/stage_report.md
python v5_mpc/TEST/run_stages.py                    # 단계 1 → 4, 실패한 단계에서 멈춤
python -m unittest discover -s v5_mpc/TEST -p "test_stage1*.py" -v
```

환경 변수(`WBR_TEST_SEEDS`, `WBR_TEST_LATENCY_MS`, `WBR_TEST_RATE`, `WBR_TEST_LONG`, `WBR_TEST_CAD`)는 LQI 폴더와 같다.

## 3. 결과

아직 MuJoCo 폐루프로 돌리지 못했다(2026-11-10, 이 PC의 응용 프로그램 제어 정책이 `mujoco` DLL을 막았다). 돌리면 여기에 LQI8과 나란히 적는다.
