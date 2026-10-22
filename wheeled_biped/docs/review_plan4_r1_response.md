# Plan 4 검토 1라운드 응답 (반영 / 반박 / 보류)

작성 2026-10-23. 대상: `docs/review_plan4_r1.md`(1단계, major 5·minor 13·nit 8), `docs/review_plan4_mpc_r1.md`(2단계, major 5·minor 11·nit 7). 모든 수정 뒤 단위 테스트 73개 통과(`tests/test_plan4.py` 38 + `tests/test_plan4_safety.py` 15 + `tests/test_core.py` 20), 두 단계 전체 매트릭스 재실행(캐시 태그 `r2_s1`·`r2_s2`; 결과 수치는 `docs/plan4_lqi6.md`·`docs/plan4_mpc_host.md` 최종 상태 절과 `results/acceptance/*.md`). 기준 수치·등급·노이즈·지연·모터 한계는 바꾸지 않았다(`docs/합격기준.md` 6.2에 측정 정의의 변경만 기록).

## 1단계 (review_plan4_r1)

| # | 판정 | 조치 | 위치 |
|---|---|---|---|
| M1 지연 여유 과대 보고 | **반영(부분)** | (1) 15번에 강건 여유 기록 지표 추가(노이즈·2 mm 낙하·0.5 W 밀기·20 s, 마지막 5 s 피치 rms < 0.5°; 첫 실패에서 중단). (2) 마찰 보상에 **피치 속도 게이트** 1/(1 + (φ̇/0.1)²)를 넣어 정지 보조로만 작동하게 함(`McuParams.friction_comp_rate_gate`, Plan 4 기본 0.1 rad/s). 측정(노이즈·2 mm 낙하·0.5 W 밀기, μ 0.75): 게이트 없음 1 kHz 12 ms 한계 사이클(피치 rms 2.1°)·13 ms 전복 → 게이트 있음 12 ms 깨끗한 정착(2.3 s, 꼬리 0.14°)·13 ms 한계 사이클(전복 없음)·15 ms 전복. **반박 부분**: 검토가 지목한 마찰 보상은 주원인이 아니었다 — 보상을 둘 다 꺼도(변형 E) 1 kHz 13 ms에서 전복하고 200 Hz 20 ms에서 전복한다. 한계는 10 ms로 설계된 LQI6 루프 자체의 비선형 여유(설계 지연 +2~3 ms)이며, 10 ms 예측기(지연 허용 2배, 밀기 피치 2배)는 사용자 선택지로 문서에 둔다. 15번 기준값(3° 무노이즈 ≥ 25 ms)은 그대로이고 강건 여유(1 kHz 12 ms·200 Hz 15 ms)는 기록·HANDOFF에 명시. 속도 창은 5 ms 유지(바퀴 떨림 억제가 사용자 요구) | `mcu.py` 마찰 보상, `plan4_upkie_lqi6.py`, `acceptance.py scenario_latency_sweep` |
| M2 NaN 샘플 → 적분기 오염 | **반영** | 추정기 입구에서 쿼터니언(유한·노름)·자이로·관절 위치/속도 유한성 검사; 무효 표본은 추정값 유지(위치 차분 창은 시간축을 유지), `max_bad_samples`(5) 초과 시 FAULT + 이벤트. 시간 스탬프가 멈추면(`stale_fault_s` 50 ms) FAULT. LQI6.step은 비유한 입력이면 직전 출력 유지. 출력 토크 비유한 → 0으로 바꾸되 FAULT + 이벤트. 시뮬 지연선의 준비 구간은 과거 시각으로 스탬프해(`ObservationDelay(dt=…)`) 정지 감시가 시작 시 추정을 붙잡지 않게 했다(붙잡으면 첫 표본의 1스텝 미분 속도 잡음이 10 ms 동안 유지되어 1 kHz 2·3번 기울기 복귀가 깨졌다). 단위 테스트: 자이로/엔코더 NaN 1표본(유지, 전복 없음)·8표본(FAULT), 정지 시간 스탬프, 시간축 유지, 시작 지연선 | `mcu.py _sample_valid/estimate/tick`, `lqi6.py step`, `tests/test_plan4_safety.py` |
| M3 IMU 관례 어댑터 없음 | **반영** | MCU에 upkie `BaseOrientation`과 같은 변환(`imu_rotation_base_to_imu`, `imu_rotation_ars_to_world`) 추가. 시뮬 플랜은 IMU 표본을 스파인 방식(ARS 좌표 z 아래, IMU y축 180° 장착)으로 내보내(`Plan4Config.real_imu_convention`, 기본 켬) 어댑터가 모든 판정에서 실행됨. 단위 테스트: 피치 0.1 rad·요 0.4 rad·각속도 (0.2, 0.3, 1.0)에서 변환 경유 추정 = 세계 좌표 추정(1e−9), 표본 자체는 다른 좌표계임을 확인. 롤 FAULT(34°) 추가. HANDOFF에 실기 첫 매달기 시험의 롤·요 부호 확인 절차 | `mcu.py`, `plan4_upkie_lqi6.py _observe`, `HANDOFF.md` |
| M4 6번 실패 건수 오기 | **반영** | 문서 네 곳 수정. 재실행 뒤의 실제 건수는 `plan4_lqi6.md` 4절(결과 파일에서 직접 집계) | `plan4_lqi6.md`, `비교보고서.md`, `HANDOFF.md` |
| M5 모델 대조 수치 재현 불가 | **반영** | 현재 모델로 네 L0 재실행, 표·요약·`linearization.json`(네 L0)·`linearization.md`(표 자동 생성) 갱신. m9(아마추어 상대 회전)·m10(강성 접촉, 성분별 B 오차, 선형 구간 rms) 반영 뒤 극 오차 ≤ 0.06 %, B ≤ 0.34 %, 정적 토크 1 % 안 | `plan4_model.md` 0·2·3·5절, `verify_plan4_plant.py` |
| m1 생존 항목 시드 0만 | 반영 | 필수-생존(4·7·23) 시드 0~4 실행. 4번 μ0.5 전복은 보고서 참고로 기록 | `acceptance.py build_tasks` |
| m2 끝점 변위 | 반영 | 2·3·5 이동, 8·13·19 정지 거리 = 최대 이탈; 끝점·되돌아감 기록. 속도 지표가 base 속도임을 명시 | `acceptance.py` |
| m3 정지형 alive | 반영 | STAND_CHECKS·12번에 alive | `acceptance.py` |
| m4 정착 유지 시간 | 반영 | 마지막 위반 뒤 1 s 유지(`SETTLE_HOLD_S`) | `acceptance.py settle_time_true` |
| m5 11번 끊는 시점 | 반영 | 3 s 순항 뒤 끊음 | `acceptance.py scenario_link_loss` |
| m6 모터별 rms | 반영 | 다리 rms 모터별 최댓값, 바퀴 포화 비율 바퀴별 최댓값 | `acceptance.py torque_metrics` |
| m7 "포화 허용" 해석 | **보류(사용자 결정)** | 판정 코드는 보수적 해석 유지(피크 상한만 제외). `합격기준.md` 6.2에 결정 항목으로 명시 | 문서 |
| m8 캐시 해시 범위 | 반영 | `wbr/` 전체 소스 + 모델 XML/JSON | `acceptance.py code_hash` |
| m9 아마추어 절대 회전 | 반영 | ½J_w(ẋ/R − θ̇)², 바퀴 감쇠도 관절 속도, I_w는 강체만, `plan4_params.json` 재생성(`J_w`) | `plant_serial.py`, `upkie_params.py` |
| m10 대조 절차 | 반영 | 강성 접촉(`--soft-contact`로 끌 수 있음), 성분별 B 오차, |Δθ| < 6° 구간 rms | `verify_plan4_plant.py` |
| m11 타이어 condim | 반영 | Plan 4 모델 타원체 + condim 4로 재생성(`--tire-condim`), 원기둥은 상한 참고. 전체 매트릭스 재실행 | `build_upkie.py`, `models/upkie_plan4` |
| m12 실기 안전 공백 | 반영(부분) | 시간 스탬프 정지 감시(FAULT), 롤 FAULT 추가. 무장 해제 시작·들림 감지·관절 한계 FAULT는 실기 진입점 작업으로 HANDOFF에 명시(시뮬 하네스는 armed=True 유지) | `mcu.py`, `HANDOFF.md` |
| m13 하드코딩·참값 경로 | 반영(부분) | MCU 관측 dict에서 `sim` 키 제거(테스트로 보장). 마찰 보상 크기는 실기에서 0부터 올리는 절차로 HANDOFF에. 바퀴 부호 상수는 URDF 기반 테스트로 확인된 값이라 유지 | `plan4_upkie_lqi6.py _obs_to_mcu`, `HANDOFF.md` |
| n1 독스트링 | 반영 | q_att·q_pos 사용으로 수정 | `lqi6.py` |
| n2 요 적분 한계 주석 | 반영 | 원기둥 시절 값임을 명시 | `plan4_upkie_lqi6.py` |
| n3 테스트 독스트링 | 반영 | 제거 | `tests/test_plan4.py` |
| n4 문서 간 불일치 | 반영 | 지연 허용·옛 수치 정정(재실행 값으로) | 문서 |
| n5 fallen 추정 피치 | 반영 | 실제 피치 사용 | `plan4_upkie_lqi6.py` |
| n6 비단조 스윕 / verify 기본 출력 | 반영(부분) | 스윕은 첫 실패에서 중단. `--out` 기본은 저장소 결과 디렉터리 유지(도움말에 명시) | `acceptance.py`, `verify_plan4_plant.py` |
| n7 15번 마찰 | 반영 | 공칭 μ 0.75 전달 | `acceptance.py` |
| n8 12번 포화 검사 | 반영 | 추가 | `acceptance.py` |

## 2단계 (review_plan4_mpc_r1)

| # | 판정 | 조치 | 위치 |
|---|---|---|---|
| M1 텔레메트리 보정이 LQI6와 싸움 | **반영** | (1) 목표 데드밴드 0.02 m/s(1e−4 m/s 조이스틱 잔류는 정확히 0으로). (2) 혁신 게이팅 0.1 m/s(그 이상의 예측 오차 = 외란, 보정 안 함, `gated_corrections` 집계). (3) 관측기 과정 잡음 σ_z 0.002 → 0.0005(적분기 없는 모델에서는 가장 느린 모드 방향; 관측기 극 0.80 → 0.91/20 ms). (4) 내부 루프: 호스트 성형 기준이 가속하는 동안 **모든 적분기 동결**(`freeze_int_on_accel_plain`) — 보정과 적분기가 같은 오차를 동시에 적분하는 구조 자체를 없앴다. (5) 배터리에 24번(0.3 m/s 순항 중 0.75 W 밀기, 기록) 추가해 두 호스트를 비교: m×1.15에서 MPC 정착 2.4 s(램프 2.3 s), 후진 최대 0.02 m/s(검토 때 −0.67 m/s) | `reference_mpc.py MpcConfig/_target/_correct`, `lqi6.py`, `acceptance.py` |
| M2 일시 QP 실패 뒤 복귀 불가 | **반영** | 복귀 조건을 "내부 램프 |a| < 0.05 m/s² 그리고 |측정 속도 − 목표| < 0.1 m/s"로, 복귀 시 MCU 텔레메트리의 자체 기준 `est.v_ref`로 재동기(무충격). 폴백 중 MCU 성형으로의 넘김은 적분기 동결(위 (4)) 덕분에 혼합 과도가 없음. 23번에 가속(0.8 s)·목표 근처(1.2 s)·제동(3.04 s) 중 0.1 s 실패 변형 추가: 세 경우 모두 8번 통과·MPC 복귀(공칭 확인: 1.96 s / 2.24 s에 복귀, 오버슈트 6~11 %) | `reference_mpc.py _update/_resync`, `acceptance_mpc.py run_fallback` |
| M3 정지 거리 끝점 정의 | **반영** | `stop_disp` = 정지 명령 뒤 최대 이탈, `stop_back`(되돌아감) 기록. 적분기 동결로 되돌아감 0.21~0.23 m → 0.04~0.07 m. 그러나 최대 이탈로 재면 a_max 0.93 m/s²의 대칭 제약으로는 0.31~0.35 m(기준 0.3 초과)가 되어, **제동 한계를 가속 한계의 1.3배로 분리**(`decel_factor`, QP 단계별 비대칭 상하한 `stage_bounds`, 모드 전환 시 저크 도달 가능 한계로 가능성 보장). "제동"은 기준 가속도가 **로봇의 예측 속도**와 반대인 것으로 정의(기준 부호로 정하면 기준이 0을 지나는 순간 제동을 풀어 정지가 0.02 m 길어짐), 저크 상승 시간 0.15 → 0.10 s. 측정(0.5 m/s 스텝, m×0.85/1.0/1.15): 대칭 한계 정지 0.32~0.35 m(실패)·피치 ≤ 1.53°, 채택안 정지 0.297~0.300 m·제동 피치 ≤ 1.95°, 1.5배 정지 0.27 m·피치 2.09~2.17°(실패). 즉 19번(0.3 m 그리고 2°)은 질량 ±15 %에서 여유 0~3 %의 물리적 한계에 있다(내부 루프 지연 0.26 s × 0.5 m/s = 0.13 m는 호스트가 줄일 수 없음; `plan4_mpc_host.md` 3.4절). **보류(사용자 확인)**: 정지 거리 해석(최대 이탈 vs 끝점) | `acceptance.py _velocity_profile`, `reference_mpc.py ReferenceQP.stage_bounds/derive_limits` |
| M4 예측 모델에 z_att 게이팅 없음 | **반영** | 내부 루프를 "기동 중 적분기 전부 동결"로 바꾸고 예측 모델도 **적분기 없는 상태 되먹임 루프**(`lqi6_options`: use_int=False, z_att·z_pos 제외, `select_gains`로 K의 5열; `identify`는 `host_shaped = True`로 실제 MPC 모드 내부 루프를 돈다)로 맞췄다. a_max는 이 모델에서 다시 유도하되 측정/모델 피치 비 1.16~1.21(`identify` 스텝 피크 2.06~2.18° vs 1.71~1.80°)을 `pitch_model_factor` 1.3으로 반영 → 가속 한계 0.889/0.884 m/s²(피치-스텝 결정, `pitch_step_binds` 기본 참), 제동 1.155/1.150, 저크 8.9 m/s³. `at_a_max` 예측과 실측을 문서에 병기 | `model.py`, `reference_mpc.py derive_limits` |
| M5 재현되지 않는 수치 | **반영** | `identify`·`acceptance_mpc`·전체 매트릭스를 최종 코드로 재실행, 문서 수치는 결과 파일에서 다시 적음. 코드 해시를 각 결과 표에 기록 | `plan4_mpc_host.md`, `results/plan4_mpc/` |
| m1 a_mcu 램프 값 | 반영 | `LQI6._plain`(4 m/s²) 사용 | `reference_mpc.py derive_limits` |
| m2 다리 길이 제약 | 반영(문서) | QP 밖 통과값 클립임을 문서에 명시(L0 스케줄 모델은 보류) | `plan4_mpc_host.md` |
| m3 몸체 속도 > v_max | 보류 | 기준 r에만 제약(사양 "속도 한계"를 기준 속도로 해석). 반전 기동의 2~6 % 초과를 문서에 명시 | 문서 |
| m4 위반 계수 자기 검사 | 반영 | 송신 패킷 가로채기 기반 `packet_violations` 추가(패킷마다 그 시점의 로봇 속도를 함께 기록; 가속도가 로봇 속도와 반대면 a_dec, 아니면 a_max; 저크 j_max T), 21번 검사 | `acceptance_mpc.py` |
| m5 폴백 이벤트 문구 | 반영 | 실제 동작대로 수정 | `reference_mpc.py` |
| m6 23번 정지 10 s | 반영 | 60 s | `acceptance_mpc.py` |
| m7 20번 배터리 편입 | 반영 | `regression_row`: 1~17 통과 유지 + 가장 빠듯한 지표 여유율 악화 ≤ 10 %를 2단계 표에 자동 계산(종합 판정 포함) | `acceptance.py` |
| m8 OSQP 시간 제한 | 반영 | `time_limit` = 호스트 주기의 절반; 초과 시 실패 → 폴백 | `reference_mpc.py` |
| m9 HostEmulator 게시 격자 | 반영 | 고정 격자(정확히 20 ms) | `host_link.py` |
| m10 재동기 불일치 | 반영 | MCU 텔레메트리에 `v_ref`(자체 성형 기준) 추가, 재동기·복귀가 그 값을 씀 | `mcu.py Estimate`, `reference_mpc.py _resync` |
| m11 50 Hz 텔레메트리 | 보류 | 실기 텔레메트리 형식 확정 후 관측기 재조정(HANDOFF) | 문서 |
| n1 낡은 수치 | 반영 | 재실행 값으로 | `plan4_mpc_host.md` |
| n2 풀이 시간 범위 | 반영(문서) | `solve_problem`만 재는 것임을 명시, 호스트 주기 전체 p99 참고값 병기 | 문서 |
| n3 "두 호스트 수치 동일" | 반영 | 삭제(10번 1 kHz 차이의 원인 m9 해결) | 문서 |
| n4 범위 검사 | 반영 | 주기 50~100 Hz, 구간 1~2 s, MCU 주기 정수배 아니면 `ValueError` | `reference_mpc.py __init__` |
| n5 roll·leg_split 통과 | 반영 | 0으로 고정 송신 | `reference_mpc.py _send` |
| n6 설정 지연을 측정 지연으로 | 보류(문서) | 실기에서는 측정값으로 바꿀 것을 HANDOFF에 | 문서 |
| n7 shaping_on 리셋 | 반영 | `reset()`이 성형 복원 | `lqi6.py` |
