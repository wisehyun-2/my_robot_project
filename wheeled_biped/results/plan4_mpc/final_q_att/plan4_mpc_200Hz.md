# Plan 4 합격 판정 — 200 Hz, 호스트 mpc, 매트릭스 reduced, 시드 2개

**종합: 불합격**  (노이즈 ON, 지연 10 ms, 판정 피치 = 시뮬 실제 피치; 종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시)

| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |
|---|---|---|---|---|---|---|
| 1 | 정지 60 s | 필수-정착 | 통과 | 6 | drift_max: 0.035 <= 0.050 (×0.71) | m×1.00 μ0.75 s1 |
| 2 | 초기 기울기 10° | 필수-정착 | 통과 | 6 | settle_s: 1.695 <= 2.000 (×0.85) | m×1.15 μ1.00 s1 |
| 3 | 초기 기울기 15° | 필수-정착 | 통과 | 6 | settle_s: 2.250 <= 3.000 (×0.75) | m×0.85 μ0.50 s1 |
| 4 | 초기 기울기 20° | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 5 | 밀기 0.5 W | 필수-정착 | 통과 | 6 | settle_s: 2.565 <= 3.000 (×0.86) | m×0.85 μ0.50 s1 |
| 6 | 밀기 0.75 W | 필수-정착 | 통과 | 6 | settle_s: 2.715 <= 4.000 (×0.68) | m×0.85 μ0.50 s0 |
| 7 | 밀기 1.0 W | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 6 | rise_s: 1.065 <= 1.500 (×0.71) | m×0.85 μ0.50 s0 |
| 9 | 요 1 rad/s | 필수-정착 | **실패** | 6 | wheel_rms10: 0.248 <= 0.200 (×1.24) | m×1.15 μ1.00 s1 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 6 | leg_rms: 1.403 <= 4.000 (×0.35) | m×1.15 μ1.00 s1 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 6 | stop_time: 2.915 <= 3.000 (×0.97) | m×0.85 μ0.50 s1 |
| 12 | 경사 5° 정지 | 필수-정착 | **실패** | 6 | disp_max: 0.176 <= 0.100 (×1.76) | m×1.15 μ1.00 s1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | 통과 | 6 | wheel_rms10: 0.170 <= 0.200 (×0.85) | m×1.15 μ1.00 s0 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | 기록 | **실패** | 1 | max_latency_ms: 15 >= 25.000 (×1.67) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 6 | drift_max: 0.154 <= 0.050 (×3.09) | m×0.85 μ0.50 s0 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 6 | drift_max: 0.193 <= 0.050 (×3.86) | m×0.85 μ0.50 s1 |
| 19 | MPC 속도 스텝 0.5 m/s | 필수-정착 | **실패** | 6 | pitch_peak: 2.360 <= 2.000 (×1.18) | m×1.15 μ1.00 s1 |
| 21 | MPC 제약 준수(8·9·13·19) | 필수-정착 | 통과 | 6 | wheel_rms10: 0.084 <= 0.200 (×0.42) | m×1.15 μ1.00 s1 |
| 22 | MPC 풀이 시간 | 기록 | 통과 | 1 | solve_p99_ms: 0.231 <= 5.000 (×0.05) | m×1.00 μ0.75 s0 |
| 23 | MPC 폴백(솔버 강제 실패) | 필수-생존 | 통과 | 1 | alive: 1 >= 1 (×0.00) | m×1.00 μ0.75 s0 |

## 실패 목록

| # | 항목 | 조합 | 지표 | 측정 | 기준 |
|---|---|---|---|---|---|
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 0 | wheel_rms10 | 0.247 | <= 0.200 |
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 1 | wheel_rms10 | 0.248 | <= 0.200 |
| 12 | 경사 5° 정지 | m×1.00 μ0.75 seed 0 | disp_max | 0.144 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.00 μ0.75 seed 1 | disp_max | 0.144 | <= 0.100 |
| 12 | 경사 5° 정지 | m×0.85 μ0.50 seed 0 | disp_max | 0.110 | <= 0.100 |
| 12 | 경사 5° 정지 | m×0.85 μ0.50 seed 1 | disp_max | 0.110 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.15 μ1.00 seed 0 | disp_max | 0.176 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.15 μ1.00 seed 1 | disp_max | 0.176 | <= 0.100 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | m×1.00 μ0.75 seed 0 | max_latency_ms | 15 | >= 25.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | drift_max | 0.153 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | drift_max | 0.151 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | drift_max | 0.154 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | drift_max | 0.152 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | drift_max | 0.152 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | drift_max | 0.150 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | drift_max | 0.165 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | drift_max | 0.168 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | drift_max | 0.190 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | drift_max | 0.193 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | drift_max | 0.146 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | drift_max | 0.148 | <= 0.050 |
| 19 | MPC 속도 스텝 0.5 m/s | m×1.00 μ0.75 seed 0 | pitch_peak | 2.319 | <= 2.000 |
| 19 | MPC 속도 스텝 0.5 m/s | m×1.00 μ0.75 seed 1 | pitch_peak | 2.329 | <= 2.000 |
| 19 | MPC 속도 스텝 0.5 m/s | m×0.85 μ0.50 seed 0 | pitch_peak | 2.318 | <= 2.000 |
| 19 | MPC 속도 스텝 0.5 m/s | m×0.85 μ0.50 seed 1 | pitch_peak | 2.304 | <= 2.000 |
| 19 | MPC 속도 스텝 0.5 m/s | m×1.15 μ1.00 seed 0 | pitch_peak | 2.347 | <= 2.000 |
| 19 | MPC 속도 스텝 0.5 m/s | m×1.15 μ1.00 seed 1 | pitch_peak | 2.360 | <= 2.000 |

## 항목별 전체 지표(최악값)

- **1 정지 60 s**: pitch_ma_max 0.697, pitch_rms 0.310, drift_max 0.035, wheel_rms10 0.040, wheel_sat_duty 0.000, wheel_peak 0.119, leg_peak 1.900, leg_rms 1.162, wheel_speed_peak 1.569, leg_speed_peak 0.218
- **2 초기 기울기 10°**: settled 1, settle_s 1.695, disp 0.111, wheel_peak 0.959, wheel_rms10 0.070, wheel_sat_duty 0.000, leg_peak 1.874, leg_rms 1.157, wheel_speed_peak 12.132, leg_speed_peak 1.721
- **3 초기 기울기 15°**: settled 1, settle_s 2.250, disp 0.157, wheel_sat_s 0.000, wheel_rms10 0.100, wheel_sat_duty 0.000, leg_peak 1.918, leg_rms 1.174, wheel_speed_peak 19.982, leg_speed_peak 2.550
- **4 초기 기울기 20°**: alive O
- **5 밀기 0.5 W**: settled 1, settle_s 2.565, pitch_peak 4.452, disp 0.027, wheel_peak 0.261, wheel_rms10 0.064, wheel_sat_duty 0.000, leg_peak 2.777, leg_rms 1.190, wheel_speed_peak 15.356, leg_speed_peak 2.401
- **6 밀기 0.75 W**: settled 1, settle_s 2.715, pitch_peak 6.973, wheel_rms10 0.088, wheel_sat_duty 0.000, leg_peak 3.752, leg_rms 1.220, wheel_speed_peak 23.045, leg_speed_peak 3.643
- **7 밀기 1.0 W**: alive O
- **8 속도 스텝 0.5 m/s**: rise_s 1.065, overshoot_pct 5.823, ss_err 0.011, pitch_peak 2.360, stop_disp 0.120, settled 1, wheel_peak 0.283, wheel_rms10 0.084, wheel_sat_duty 0.000, leg_peak 1.882, leg_rms 1.157, wheel_speed_peak 12.157, leg_speed_peak 0.548
- **9 요 1 rad/s**: yaw_err_pct 1.578, pitch_peak 0.372, alive O, wheel_peak 0.371, wheel_rms10 0.248 ✗, wheel_sat_duty 0.000, leg_peak 2.933, leg_rms 1.731, wheel_speed_peak 3.599, leg_speed_peak 0.465
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.655, settled 1, wheel_peak 0.125, wheel_rms10 0.041, wheel_sat_duty 0.000, leg_peak 3.466, leg_rms 1.403, wheel_speed_peak 3.926, leg_speed_peak 3.639
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.915, motors_on 1, settled 1, wheel_peak 0.236, wheel_rms10 0.062, wheel_sat_duty 0.000, leg_peak 1.884, leg_rms 1.171, wheel_speed_peak 13.823, leg_speed_peak 0.728
- **12 경사 5° 정지**: disp_max 0.176 ✗, pitch_dev_ma_max 0.492, pitch_dev_rms 0.189, wheel_rms10 0.155, wheel_peak 0.262, wheel_sat_duty 0.000, leg_peak 1.827, leg_rms 1.111, wheel_speed_peak 2.904, leg_speed_peak 0.867
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 1.160, overshoot_pct 5.516, ss_err 0.008, pitch_peak 3.075, stop_disp 0.132, settled 1, wheel_peak 0.375, wheel_rms10 0.170, wheel_sat_duty 0.000, leg_peak 1.841, leg_rms 1.124, wheel_speed_peak 11.507, leg_speed_peak 0.578
- **15 지연 스윕(3° 기울기, 무노이즈)**: max_latency_ms 15 ✗
- **16 IMU 피치 바이어스 +1°**: pitch_ma_max 0.687, pitch_rms 0.311, drift_max 0.154 ✗, wheel_rms10 0.040, wheel_sat_duty 0.000, wheel_peak 0.121, leg_peak 1.899, leg_rms 1.163, wheel_speed_peak 1.590, leg_speed_peak 0.196
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: pitch_ma_max 0.835, pitch_rms 0.430, drift_max 0.193 ✗, wheel_rms10 0.040, wheel_sat_duty 0.000, wheel_peak 0.122, leg_peak 1.927, leg_rms 1.179, wheel_speed_peak 1.558, leg_speed_peak 0.190
- **19 MPC 속도 스텝 0.5 m/s**: pitch_peak 2.360 ✗, overshoot_pct 5.823, rise_s 1.065, stop_disp 0.120, settled 1, wheel_peak 0.283, wheel_rms10 0.084, wheel_sat_duty 0.000, leg_peak 1.882, leg_rms 1.157, wheel_speed_peak 12.157, leg_speed_peak 0.548
- **21 MPC 제약 준수(8·9·13·19)**: constraint_violations 0, wheel_peak 0.283, wheel_rms10 0.084, wheel_sat_duty 0.000, leg_peak 1.882, leg_rms 1.157, wheel_speed_peak 12.157, leg_speed_peak 0.548
- **22 MPC 풀이 시간**: solve_p99_ms 0.231, qp_failures 0
- **23 MPC 폴백(솔버 강제 실패)**: alive 1, fallback_used 1
