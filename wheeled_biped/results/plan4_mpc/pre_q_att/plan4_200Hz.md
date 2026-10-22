# Plan 4 합격 판정 — 200 Hz, 호스트 ramp, 매트릭스 reduced, 시드 2개

**종합: 불합격**  (노이즈 ON, 지연 10 ms, 판정 피치 = 시뮬 실제 피치; 종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시)

| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |
|---|---|---|---|---|---|---|
| 1 | 정지 60 s | 필수-정착 | 통과 | 6 | drift_max: 0.029 <= 0.050 (×0.59) | m×1.15 μ1.00 s0 |
| 2 | 초기 기울기 10° | 필수-정착 | 통과 | 6 | settle_s: 1.665 <= 2.000 (×0.83) | m×1.15 μ1.00 s1 |
| 3 | 초기 기울기 15° | 필수-정착 | 통과 | 6 | settle_s: 2.180 <= 3.000 (×0.73) | m×0.85 μ0.50 s0 |
| 4 | 초기 기울기 20° | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 5 | 밀기 0.5 W | 필수-정착 | 통과 | 6 | settle_s: 2.450 <= 3.000 (×0.82) | m×0.85 μ0.50 s0 |
| 6 | 밀기 0.75 W | 필수-정착 | 통과 | 6 | settle_s: 2.710 <= 4.000 (×0.68) | m×0.85 μ0.50 s0 |
| 7 | 밀기 1.0 W | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 6 | rise_s: 1.495 <= 1.500 (×1.00) | m×0.85 μ0.50 s0 |
| 9 | 요 1 rad/s | 필수-정착 | **실패** | 6 | yaw_err_pct: 16.925 <= 10.000 (×1.69) | m×1.15 μ1.00 s1 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 6 | leg_rms: 1.405 <= 4.000 (×0.35) | m×1.15 μ1.00 s1 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 6 | stop_time: 2.845 <= 3.000 (×0.95) | m×0.85 μ0.50 s0 |
| 12 | 경사 5° 정지 | 필수-정착 | **실패** | 6 | disp_max: 0.152 <= 0.100 (×1.52) | m×1.15 μ1.00 s0 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | **실패** | 6 | rise_s: - <= 1.500 (×inf) | m×1.15 μ1.00 s1 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | 기록 | **실패** | 1 | max_latency_ms: 20 >= 25.000 (×1.25) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 6 | drift_max: 0.146 <= 0.050 (×2.92) | m×0.85 μ0.50 s0 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 6 | drift_max: 0.185 <= 0.050 (×3.70) | m×0.85 μ0.50 s1 |

## 실패 목록

| # | 항목 | 조합 | 지표 | 측정 | 기준 |
|---|---|---|---|---|---|
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 0 | yaw_err_pct | 16.848 | <= 10.000 |
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 0 | wheel_rms10 | 0.251 | <= 0.200 |
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 1 | yaw_err_pct | 16.925 | <= 10.000 |
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 1 | wheel_rms10 | 0.251 | <= 0.200 |
| 12 | 경사 5° 정지 | m×1.00 μ0.75 seed 0 | disp_max | 0.119 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.00 μ0.75 seed 1 | disp_max | 0.119 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.15 μ1.00 seed 0 | disp_max | 0.152 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.15 μ1.00 seed 1 | disp_max | 0.151 | <= 0.100 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | rise_s | 1.905 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | overshoot_pct | 23.668 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | ss_err | 0.100 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | rise_s | 1.965 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | overshoot_pct | 24.951 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | ss_err | 0.101 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | rise_s | 1.710 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | ss_err | 0.063 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | rise_s | 1.705 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | ss_err | 0.064 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | rise_s | 2.450 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | overshoot_pct | 30.231 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | ss_err | 0.134 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | settled | 0 | >= 1 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | rise_s | - | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | overshoot_pct | 31.158 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | ss_err | 0.135 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | settled | 0 | >= 1 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | m×1.00 μ0.75 seed 0 | max_latency_ms | 20 | >= 25.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | pitch_ma_max | 1.712 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | pitch_rms | 1.381 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | drift_max | 0.145 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | pitch_ma_max | 1.734 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | pitch_rms | 1.382 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | drift_max | 0.143 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | pitch_ma_max | 1.644 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | pitch_rms | 1.308 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | drift_max | 0.146 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | pitch_ma_max | 1.627 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | pitch_rms | 1.303 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | drift_max | 0.144 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | pitch_ma_max | 1.781 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | pitch_rms | 1.457 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | drift_max | 0.144 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | pitch_ma_max | 1.824 | <= 1.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | pitch_rms | 1.456 | <= 0.500 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | drift_max | 0.142 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | pitch_ma_max | 1.166 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | pitch_rms | 0.886 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | drift_max | 0.159 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | pitch_ma_max | 1.216 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | pitch_rms | 0.890 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | drift_max | 0.160 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | pitch_ma_max | 1.363 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | pitch_rms | 1.023 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | drift_max | 0.183 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | pitch_ma_max | 1.414 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | pitch_rms | 1.029 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | drift_max | 0.185 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | pitch_ma_max | 1.077 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | pitch_rms | 0.777 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | drift_max | 0.141 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | pitch_ma_max | 1.115 | <= 1.000 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | pitch_rms | 0.774 | <= 0.500 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | drift_max | 0.140 | <= 0.050 |

## 항목별 전체 지표(최악값)

- **1 정지 60 s**: pitch_ma_max 0.396, pitch_rms 0.173, drift_max 0.029, wheel_rms10 0.039, wheel_sat_duty 0.000, wheel_peak 0.117, leg_peak 1.892, leg_rms 1.166, wheel_speed_peak 1.378, leg_speed_peak 0.265
- **2 초기 기울기 10°**: settled 1, settle_s 1.665, disp 0.112, wheel_peak 0.923, wheel_rms10 0.069, wheel_sat_duty 0.000, leg_peak 1.871, leg_rms 1.159, wheel_speed_peak 11.898, leg_speed_peak 1.487
- **3 초기 기울기 15°**: settled 1, settle_s 2.180, disp 0.157, wheel_sat_s 0.000, wheel_rms10 0.098, wheel_sat_duty 0.000, leg_peak 1.935, leg_rms 1.172, wheel_speed_peak 18.279, leg_speed_peak 2.177
- **4 초기 기울기 20°**: alive O
- **5 밀기 0.5 W**: settled 1, settle_s 2.450, pitch_peak 3.434, disp 0.029, wheel_peak 0.260, wheel_rms10 0.063, wheel_sat_duty 0.000, leg_peak 2.822, leg_rms 1.189, wheel_speed_peak 15.542, leg_speed_peak 2.956
- **6 밀기 0.75 W**: settled 1, settle_s 2.710, pitch_peak 5.220, wheel_rms10 0.087, wheel_sat_duty 0.000, leg_peak 3.906, leg_rms 1.218, wheel_speed_peak 23.238, leg_speed_peak 4.347
- **7 밀기 1.0 W**: alive O
- **8 속도 스텝 0.5 m/s**: rise_s 1.495, overshoot_pct 7.110, ss_err 0.010, pitch_peak 0.972, stop_disp 0.122, settled 1, wheel_peak 0.219, wheel_rms10 0.075, wheel_sat_duty 0.000, leg_peak 1.907, leg_rms 1.161, wheel_speed_peak 12.248, leg_speed_peak 0.768
- **9 요 1 rad/s**: yaw_err_pct 16.925 ✗, pitch_peak 0.313, alive O, wheel_peak 0.370, wheel_rms10 0.251 ✗, wheel_sat_duty 0.000, leg_peak 2.938, leg_rms 1.755, wheel_speed_peak 3.464, leg_speed_peak 0.467
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.444, settled 1, wheel_peak 0.123, wheel_rms10 0.040, wheel_sat_duty 0.000, leg_peak 3.473, leg_rms 1.405, wheel_speed_peak 3.809, leg_speed_peak 3.642
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.845, motors_on 1, settled 1, wheel_peak 0.190, wheel_rms10 0.060, wheel_sat_duty 0.000, leg_peak 1.897, leg_rms 1.170, wheel_speed_peak 11.818, leg_speed_peak 0.825
- **12 경사 5° 정지**: disp_max 0.152 ✗, pitch_dev_ma_max 0.480, pitch_dev_rms 0.250, wheel_rms10 0.153, wheel_peak 0.258, wheel_sat_duty 0.000, leg_peak 1.815, leg_rms 1.098, wheel_speed_peak 2.938, leg_speed_peak 0.873
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s - ✗, overshoot_pct 31.158 ✗, ss_err 0.135 ✗, pitch_peak 2.411, stop_disp 0.174, settled 0 ✗, wheel_peak 0.389, wheel_rms10 0.173, wheel_sat_duty 0.000, leg_peak 1.833, leg_rms 1.104, wheel_speed_peak 13.980, leg_speed_peak 0.818
- **15 지연 스윕(3° 기울기, 무노이즈)**: max_latency_ms 20 ✗
- **16 IMU 피치 바이어스 +1°**: pitch_ma_max 1.824 ✗, pitch_rms 1.457 ✗, drift_max 0.146 ✗, wheel_rms10 0.039, wheel_sat_duty 0.000, wheel_peak 0.114, leg_peak 1.894, leg_rms 1.178, wheel_speed_peak 1.551, leg_speed_peak 0.287
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: pitch_ma_max 1.414 ✗, pitch_rms 1.029 ✗, drift_max 0.185 ✗, wheel_rms10 0.039, wheel_sat_duty 0.000, wheel_peak 0.120, leg_peak 1.939, leg_rms 1.172, wheel_speed_peak 1.439, leg_speed_peak 0.311
