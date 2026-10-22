# Plan 4 합격 판정 — 1000 Hz, 호스트 ramp, 매트릭스 reduced, 시드 2개

**종합: 불합격**  (노이즈 ON, 지연 10 ms, 판정 피치 = 시뮬 실제 피치; 종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시)

| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |
|---|---|---|---|---|---|---|
| 1 | 정지 60 s | 필수-정착 | 통과 | 6 | wheel_rms10: 0.098 <= 0.200 (×0.49) | m×0.85 μ0.50 s1 |
| 2 | 초기 기울기 10° | 필수-정착 | 통과 | 6 | settle_s: 1.915 <= 2.000 (×0.96) | m×1.00 μ0.75 s0 |
| 3 | 초기 기울기 15° | 필수-정착 | 통과 | 6 | settle_s: 2.189 <= 3.000 (×0.73) | m×0.85 μ0.50 s0 |
| 4 | 초기 기울기 20° | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 5 | 밀기 0.5 W | 필수-정착 | 통과 | 6 | settle_s: 2.435 <= 3.000 (×0.81) | m×0.85 μ0.50 s1 |
| 6 | 밀기 0.75 W | 필수-정착 | 통과 | 6 | settle_s: 2.653 <= 4.000 (×0.66) | m×0.85 μ0.50 s1 |
| 7 | 밀기 1.0 W | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 6 | rise_s: 1.380 <= 1.500 (×0.92) | m×0.85 μ0.50 s1 |
| 9 | 요 1 rad/s | 필수-정착 | **실패** | 6 | yaw_err_pct: 15.970 <= 10.000 (×1.60) | m×1.15 μ1.00 s0 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 6 | wheel_rms10: 0.096 <= 0.200 (×0.48) | m×0.85 μ0.50 s1 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 6 | stop_time: 2.861 <= 3.000 (×0.95) | m×0.85 μ0.50 s0 |
| 12 | 경사 5° 정지 | 필수-정착 | **실패** | 6 | disp_max: 0.137 <= 0.100 (×1.37) | m×1.15 μ1.00 s0 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | **실패** | 6 | settled: 0 >= 1 (×inf) | m×1.00 μ0.75 s0 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | 기록 | **실패** | 1 | max_latency_ms: 15 >= 25.000 (×1.67) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 6 | pitch_rms: 1.459 <= 0.500 (×2.92) | m×1.15 μ1.00 s1 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 6 | drift_max: 0.180 <= 0.050 (×3.60) | m×0.85 μ0.50 s0 |

## 실패 목록

| # | 항목 | 조합 | 지표 | 측정 | 기준 |
|---|---|---|---|---|---|
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 0 | yaw_err_pct | 15.970 | <= 10.000 |
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 0 | wheel_rms10 | 0.263 | <= 0.200 |
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 1 | yaw_err_pct | 15.906 | <= 10.000 |
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 1 | wheel_rms10 | 0.261 | <= 0.200 |
| 12 | 경사 5° 정지 | m×1.00 μ0.75 seed 0 | disp_max | 0.106 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.00 μ0.75 seed 1 | disp_max | 0.103 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.15 μ1.00 seed 0 | disp_max | 0.137 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.15 μ1.00 seed 1 | disp_max | 0.134 | <= 0.100 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | rise_s | 1.961 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | overshoot_pct | 24.213 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | ss_err | 0.091 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | rise_s | 2.044 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | overshoot_pct | 24.334 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | ss_err | 0.092 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | rise_s | 1.708 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | ss_err | 0.058 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | rise_s | 1.780 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | ss_err | 0.058 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | rise_s | 2.268 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | overshoot_pct | 30.404 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | ss_err | 0.123 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | rise_s | 2.334 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | overshoot_pct | 29.835 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | ss_err | 0.123 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | settled | 0 | >= 1 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | m×1.00 μ0.75 seed 0 | max_latency_ms | 15 | >= 25.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | pitch_ma_max | 1.611 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | pitch_rms | 1.376 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | drift_max | 0.133 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | pitch_ma_max | 1.628 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | pitch_rms | 1.379 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | drift_max | 0.137 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | pitch_ma_max | 1.519 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | pitch_rms | 1.296 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | drift_max | 0.132 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | pitch_ma_max | 1.562 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | pitch_rms | 1.299 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | drift_max | 0.136 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | pitch_ma_max | 1.723 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | pitch_rms | 1.455 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | drift_max | 0.133 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | pitch_ma_max | 1.714 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | pitch_rms | 1.459 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | drift_max | 0.137 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | pitch_ma_max | 1.210 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | pitch_rms | 0.957 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | drift_max | 0.155 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | pitch_ma_max | 1.225 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | pitch_rms | 0.955 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | drift_max | 0.150 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | pitch_ma_max | 1.383 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | pitch_rms | 1.087 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | drift_max | 0.180 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | pitch_ma_max | 1.330 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | pitch_rms | 1.086 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | drift_max | 0.176 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | pitch_ma_max | 1.125 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | pitch_rms | 0.842 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | drift_max | 0.136 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | pitch_ma_max | 1.119 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | pitch_rms | 0.840 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | drift_max | 0.132 | <= 0.050 |

## 항목별 전체 지표(최악값)

- **1 정지 60 s**: pitch_ma_max 0.331, pitch_rms 0.104, drift_max 0.022, wheel_rms10 0.098, wheel_sat_duty 0.000, wheel_peak 0.308, leg_peak 2.751, leg_rms 1.259, wheel_speed_peak 2.018, leg_speed_peak 0.801
- **2 초기 기울기 10°**: settled 1, settle_s 1.915, disp 0.095, wheel_peak 1.134, wheel_rms10 0.113, wheel_sat_duty 0.000, leg_peak 2.646, leg_rms 1.256, wheel_speed_peak 13.644, leg_speed_peak 2.329
- **3 초기 기울기 15°**: settled 1, settle_s 2.189, disp 0.140, wheel_sat_s 0.000, wheel_rms10 0.133, wheel_sat_duty 0.000, leg_peak 3.469, leg_rms 1.259, wheel_speed_peak 19.482, leg_speed_peak 2.958
- **4 초기 기울기 20°**: alive O
- **5 밀기 0.5 W**: settled 1, settle_s 2.435, pitch_peak 2.694, disp 0.012, wheel_peak 0.433, wheel_rms10 0.108, wheel_sat_duty 0.000, leg_peak 3.284, leg_rms 1.275, wheel_speed_peak 15.602, leg_speed_peak 2.914
- **6 밀기 0.75 W**: settled 1, settle_s 2.653, pitch_peak 4.156, wheel_rms10 0.123, wheel_sat_duty 0.000, leg_peak 4.413, leg_rms 1.291, wheel_speed_peak 23.189, leg_speed_peak 4.133
- **7 밀기 1.0 W**: alive O
- **8 속도 스텝 0.5 m/s**: rise_s 1.380, overshoot_pct 9.604, ss_err 0.017, pitch_peak 1.094, stop_disp 0.116, settled 1, wheel_peak 0.384, wheel_rms10 0.118, wheel_sat_duty 0.000, leg_peak 2.633, leg_rms 1.263, wheel_speed_peak 13.243, leg_speed_peak 1.250
- **9 요 1 rad/s**: yaw_err_pct 15.970 ✗, pitch_peak 0.486, alive O, wheel_peak 0.526, wheel_rms10 0.263 ✗, wheel_sat_duty 0.000, leg_peak 3.834, leg_rms 1.824, wheel_speed_peak 4.548, leg_speed_peak 0.740
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.426, settled 1, wheel_peak 0.293, wheel_rms10 0.096, wheel_sat_duty 0.000, leg_peak 4.065, leg_rms 1.494, wheel_speed_peak 4.294, leg_speed_peak 3.844
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.861, motors_on 1, settled 1, wheel_peak 0.369, wheel_rms10 0.108, wheel_sat_duty 0.000, leg_peak 2.637, leg_rms 1.260, wheel_speed_peak 12.396, leg_speed_peak 1.108
- **12 경사 5° 정지**: disp_max 0.137 ✗, pitch_dev_ma_max 0.361, pitch_dev_rms 0.109, wheel_rms10 0.174, wheel_peak 0.436, wheel_sat_duty 0.000, leg_peak 2.636, leg_rms 1.201, wheel_speed_peak 3.819, leg_speed_peak 1.378
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 2.334 ✗, overshoot_pct 30.404 ✗, ss_err 0.123 ✗, pitch_peak 2.518, stop_disp 0.179, settled 0 ✗, wheel_peak 0.540, wheel_rms10 0.193, wheel_sat_duty 0.000, leg_peak 2.619, leg_rms 1.206, wheel_speed_peak 14.531, leg_speed_peak 1.256
- **15 지연 스윕(3° 기울기, 무노이즈)**: max_latency_ms 15 ✗
- **16 IMU 피치 바이어스 +1°**: pitch_ma_max 1.723 ✗, pitch_rms 1.459 ✗, drift_max 0.137 ✗, wheel_rms10 0.098, wheel_sat_duty 0.000, wheel_peak 0.307, leg_peak 2.762, leg_rms 1.268, wheel_speed_peak 2.009, leg_speed_peak 0.796
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: pitch_ma_max 1.383 ✗, pitch_rms 1.087 ✗, drift_max 0.180 ✗, wheel_rms10 0.098, wheel_sat_duty 0.000, wheel_peak 0.311, leg_peak 2.800, leg_rms 1.268, wheel_speed_peak 2.070, leg_speed_peak 0.752
