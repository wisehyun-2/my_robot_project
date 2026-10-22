# Plan 4 합격 판정 — 200 Hz, 호스트 mpc, 매트릭스 full, 시드 5개

**종합: 불합격**  (노이즈 ON, 지연 10 ms, 판정 피치 = 시뮬 실제 피치; 종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시; 코드 해시 1cbc282b92)

| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |
|---|---|---|---|---|---|---|
| 1 | 정지 60 s | 필수-정착 | 통과 | 45 | pitch_ma_max: 0.686 <= 1.000 (×0.69) | m×1.15 μ0.75 s4 |
| 2 | 초기 기울기 10° | 필수-정착 | 통과 | 45 | wheel_peak: 1.288 <= 1.360 (×0.95) | m×0.85 μ0.50 s0 |
| 3 | 초기 기울기 15° | 필수-정착 | 통과 | 45 | wheel_rms10: 0.118 <= 0.200 (×0.59) | m×1.15 μ0.50 s3 |
| 4 | 초기 기울기 20° | 필수-생존 | 통과 | 5 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 5 | 밀기 0.5 W | 필수-정착 | 통과 | 45 | settle_s: 1.940 <= 3.000 (×0.65) | m×0.85 μ0.50 s3 |
| 6 | 밀기 0.75 W | 필수-정착 | 통과 | 45 | wheel_rms10: 0.112 <= 0.200 (×0.56) | m×1.15 μ0.50 s2 |
| 7 | 밀기 1.0 W | 필수-생존 | 통과 | 5 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 1.085 <= 1.500 (×0.72) | m×0.85 μ0.50 s4 |
| 9 | 요 1 rad/s | 필수-정착 | 통과 | 45 | wheel_rms10: 0.109 <= 0.200 (×0.54) | m×1.15 μ1.00 s1 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 45 | leg_rms: 1.977 <= 4.000 (×0.49) | m×1.15 μ1.00 s4 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 45 | stop_time: 2.360 <= 3.000 (×0.79) | m×0.85 μ0.50 s4 |
| 12 | 경사 5° 정지 | 필수-정착 | 통과 | 45 | wheel_rms10: 0.159 <= 0.200 (×0.79) | m×1.15 μ1.00 s2 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | 통과 | 45 | rise_s: 1.470 <= 1.500 (×0.98) | m×1.15 μ0.50 s2 |
| 15 | 지연 스윕(3° 기울기, 무노이즈; 강건 여유도 함께 기록) | 기록 | 통과 | 1 | max_latency_ms: 25 >= 25.000 (×1.00) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 45 | drift_max: 0.091 <= 0.050 (×1.81) | m×0.85 μ0.75 s4 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 45 | drift_max: 0.110 <= 0.050 (×2.19) | m×0.85 μ0.50 s1 |
| 19 | MPC 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | stop_disp: 0.297 <= 0.300 (×0.99) | m×0.85 μ0.75 s4 |
| 20 | 회귀 없음(1~17: 1단계 통과 유지, 가장 빠듯한 지표 여유율 악화 ≤ 10 %) | 필수-정착 | 통과 | 15 | regression_flips: 0 <= 0 (×0.00) | - |
| 21 | MPC 제약 준수(8·9·13·19) | 필수-정착 | 통과 | 45 | wheel_peak: 0.422 <= 1.360 (×0.31) | m×1.15 μ0.50 s1 |
| 22 | MPC 풀이 시간 | 기록 | 통과 | 1 | solve_p99_ms: 0.255 <= 5.000 (×0.05) | m×1.00 μ0.75 s0 |
| 23 | MPC 폴백(솔버 강제 실패) | 필수-생존 | 통과 | 5 | alive: 1 >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 24 | 주행 중 밀기(0.3 m/s 순항, 0.75 W; 기록, 검토 r1) | 기록 | 통과 | 1 | settle_s: 2.025 <= 4.000 (×0.51) | m×1.00 μ0.75 s0 |

## 실패 목록

| # | 항목 | 조합 | 지표 | 측정 | 기준 |
|---|---|---|---|---|---|
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 2 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 3 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 4 | drift_max | 0.090 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 0 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 1 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 3 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 4 | drift_max | 0.091 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 0 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 1 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 3 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 4 | drift_max | 0.091 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 0 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 1 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 2 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 3 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 4 | drift_max | 0.090 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 2 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 3 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 4 | drift_max | 0.090 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 0 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 1 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 2 | drift_max | 0.089 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 3 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 4 | drift_max | 0.090 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 0 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 1 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 2 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 4 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 0 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 1 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 2 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 4 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 2 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 4 | drift_max | 0.088 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | drift_max | 0.102 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | drift_max | 0.110 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 2 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 3 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 4 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 0 | drift_max | 0.102 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 1 | drift_max | 0.109 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 2 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 3 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 4 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 0 | drift_max | 0.102 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 1 | drift_max | 0.109 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 2 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 3 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 4 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 0 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 1 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 2 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 3 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 4 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 2 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 3 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 4 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 0 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 1 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 2 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 3 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 4 | drift_max | 0.095 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 0 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 1 | drift_max | 0.087 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 2 | drift_max | 0.085 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 3 | drift_max | 0.084 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 4 | drift_max | 0.084 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 0 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 1 | drift_max | 0.087 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 2 | drift_max | 0.085 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 3 | drift_max | 0.085 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 4 | drift_max | 0.084 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | drift_max | 0.087 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 2 | drift_max | 0.085 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 3 | drift_max | 0.084 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 4 | drift_max | 0.084 | <= 0.050 |

## 항목별 전체 지표(최악값)

- **1 정지 60 s**: alive O, pitch_ma_max 0.686, pitch_rms 0.299, drift_max 0.025, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.138, leg_peak 1.902, leg_rms 1.637, wheel_speed_peak 1.469, leg_speed_peak 0.280
- **2 초기 기울기 10°**: settled 1, settle_s 1.400, disp 0.135, wheel_peak 1.288, wheel_rms10 0.084, wheel_sat_duty 0.000, leg_peak 1.972, leg_rms 1.634, wheel_speed_peak 15.609, leg_speed_peak 1.240
- **3 초기 기울기 15°**: settled 1, settle_s 1.745, disp 0.191, wheel_sat_s 0.010, wheel_rms10 0.118, wheel_sat_duty 0.001, leg_peak 2.115, leg_rms 1.650, wheel_speed_peak 29.445, leg_speed_peak 2.014
- **4 초기 기울기 20°**: alive O
- **5 밀기 0.5 W**: settled 1, settle_s 1.940, pitch_peak 4.080, disp 0.151, wheel_peak 0.385, wheel_rms10 0.080, wheel_sat_duty 0.000, leg_peak 2.838, leg_rms 1.670, wheel_speed_peak 18.661, leg_speed_peak 3.015
- **6 밀기 0.75 W**: settled 1, settle_s 2.185, pitch_peak 6.052, wheel_rms10 0.112, wheel_sat_duty 0.000, leg_peak 3.892, leg_rms 1.683, wheel_speed_peak 27.730, leg_speed_peak 4.360
- **7 밀기 1.0 W**: alive O
- **8 속도 스텝 0.5 m/s**: rise_s 1.085, overshoot_pct 4.253, ss_err 0.004, pitch_peak 1.978, stop_disp 0.297, settled 1, wheel_peak 0.285, wheel_rms10 0.084, wheel_sat_duty 0.000, leg_peak 1.916, leg_rms 1.634, wheel_speed_peak 11.057, leg_speed_peak 0.545
- **9 요 1 rad/s**: yaw_err_pct 0.434, pitch_peak 0.385, alive O, wheel_peak 0.301, wheel_rms10 0.109, wheel_sat_duty 0.000, leg_peak 2.470, leg_rms 1.966, wheel_speed_peak 3.656, leg_speed_peak 0.585
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.798, settled 1, wheel_peak 0.156, wheel_rms10 0.048, wheel_sat_duty 0.000, leg_peak 3.499, leg_rms 1.977, wheel_speed_peak 4.090, leg_speed_peak 3.722
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.360, motors_on 1, settled 1, wheel_peak 0.310, wheel_rms10 0.071, wheel_sat_duty 0.000, leg_peak 1.914, leg_rms 1.645, wheel_speed_peak 12.434, leg_speed_peak 0.763
- **12 경사 5° 정지**: alive O, disp_max 0.062, pitch_dev_ma_max 0.446, pitch_dev_rms 0.174, wheel_rms10 0.159, wheel_sat_duty 0.000, wheel_peak 0.295, leg_peak 1.832, leg_rms 1.561, wheel_speed_peak 2.986, leg_speed_peak 0.928
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 1.470, overshoot_pct 10.405, ss_err 0.026, pitch_peak 2.577, stop_disp 0.297, settled 1, wheel_peak 0.422, wheel_rms10 0.175, wheel_sat_duty 0.000, leg_peak 1.852, leg_rms 1.557, wheel_speed_peak 11.465, leg_speed_peak 0.591
- **15 지연 스윕(3° 기울기, 무노이즈; 강건 여유도 함께 기록)**: max_latency_ms 25, max_latency_robust_ms 15
- **16 IMU 피치 바이어스 +1°**: alive O, pitch_ma_max 0.703, pitch_rms 0.308, drift_max 0.091 ✗, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.141, leg_peak 1.910, leg_rms 1.637, wheel_speed_peak 1.728, leg_speed_peak 0.283
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: alive O, pitch_ma_max 0.792, pitch_rms 0.431, drift_max 0.110 ✗, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.135, leg_peak 1.947, leg_rms 1.671, wheel_speed_peak 2.015, leg_speed_peak 0.241
- **19 MPC 속도 스텝 0.5 m/s**: pitch_peak 1.978, overshoot_pct 4.253, rise_s 1.085, stop_disp 0.297, settled 1, wheel_peak 0.285, wheel_rms10 0.084, wheel_sat_duty 0.000, leg_peak 1.916, leg_rms 1.634, wheel_speed_peak 11.057, leg_speed_peak 0.545
- **20 회귀 없음(1~17: 1단계 통과 유지, 가장 빠듯한 지표 여유율 악화 ≤ 10 %)**: regression_flips 0, margin_worse_10pct 0, metric_worse_10pct_record 2
- **21 MPC 제약 준수(8·9·13·19)**: constraint_violations 0, packet_violations 0, wheel_peak 0.422, alive 1
- **22 MPC 풀이 시간**: solve_p99_ms 0.255, qp_failures 0
- **23 MPC 폴백(솔버 강제 실패)**: alive 1, fallback_used 1, passes_1_8_11 1, passes_8_transient 1, mpc_resumed 1
- **24 주행 중 밀기(0.3 m/s 순항, 0.75 W; 기록, 검토 r1)**: alive O, settled 1, settle_s 2.025, v_back 0.006
