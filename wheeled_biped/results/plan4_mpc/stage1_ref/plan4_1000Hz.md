# Plan 4 합격 판정 — 1000 Hz, 호스트 ramp, 매트릭스 reduced, 시드 2개

**종합: 불합격**  (노이즈 ON, 지연 10 ms, 판정 피치 = 시뮬 실제 피치; 종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시)

| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |
|---|---|---|---|---|---|---|
| 1 | 정지 60 s | 필수-정착 | 통과 | 6 | pitch_ma_max: 0.534 <= 1.000 (×0.53) | m×1.15 μ1.00 s0 |
| 2 | 초기 기울기 10° | 필수-정착 | 통과 | 6 | settle_s: 1.912 <= 2.000 (×0.96) | m×1.00 μ0.75 s0 |
| 3 | 초기 기울기 15° | 필수-정착 | 통과 | 6 | settle_s: 2.187 <= 3.000 (×0.73) | m×0.85 μ0.50 s0 |
| 4 | 초기 기울기 20° | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 5 | 밀기 0.5 W | 필수-정착 | 통과 | 6 | settle_s: 2.433 <= 3.000 (×0.81) | m×0.85 μ0.50 s1 |
| 6 | 밀기 0.75 W | 필수-정착 | 통과 | 6 | settle_s: 2.649 <= 4.000 (×0.66) | m×0.85 μ0.50 s1 |
| 7 | 밀기 1.0 W | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 6 | rise_s: 1.377 <= 1.500 (×0.92) | m×0.85 μ0.50 s1 |
| 9 | 요 1 rad/s | 필수-정착 | **실패** | 6 | wheel_rms10: 0.261 <= 0.200 (×1.30) | m×1.15 μ1.00 s0 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 6 | wheel_rms10: 0.098 <= 0.200 (×0.49) | m×1.15 μ1.00 s1 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 6 | stop_time: 2.991 <= 3.000 (×1.00) | m×0.85 μ0.50 s0 |
| 12 | 경사 5° 정지 | 필수-정착 | **실패** | 6 | disp_max: 0.155 <= 0.100 (×1.55) | m×1.15 μ1.00 s0 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | **실패** | 6 | rise_s: - <= 1.500 (×inf) | m×1.15 μ1.00 s0 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | 기록 | **실패** | 1 | max_latency_ms: 15 >= 25.000 (×1.67) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 6 | drift_max: 0.141 <= 0.050 (×2.81) | m×1.00 μ0.75 s1 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 6 | drift_max: 0.182 <= 0.050 (×3.65) | m×0.85 μ0.50 s0 |

## 실패 목록

| # | 항목 | 조합 | 지표 | 측정 | 기준 |
|---|---|---|---|---|---|
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 0 | wheel_rms10 | 0.261 | <= 0.200 |
| 9 | 요 1 rad/s | m×1.15 μ1.00 seed 1 | wheel_rms10 | 0.259 | <= 0.200 |
| 12 | 경사 5° 정지 | m×1.00 μ0.75 seed 0 | disp_max | 0.123 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.00 μ0.75 seed 1 | disp_max | 0.120 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.15 μ1.00 seed 0 | disp_max | 0.155 | <= 0.100 |
| 12 | 경사 5° 정지 | m×1.15 μ1.00 seed 1 | disp_max | 0.152 | <= 0.100 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | rise_s | 2.111 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | overshoot_pct | 26.273 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 0 | ss_err | 0.109 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | rise_s | 2.149 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | overshoot_pct | 27.365 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.00 μ0.75 seed 1 | ss_err | 0.110 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | rise_s | 1.871 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | overshoot_pct | 20.202 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 0 | ss_err | 0.074 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | rise_s | 1.913 | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | overshoot_pct | 20.957 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×0.85 μ0.50 seed 1 | ss_err | 0.074 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | rise_s | - | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | overshoot_pct | 33.164 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 0 | ss_err | 0.143 | <= 0.050 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | rise_s | - | <= 1.500 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | overshoot_pct | 33.596 | <= 20.000 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | m×1.15 μ1.00 seed 1 | ss_err | 0.144 | <= 0.050 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | m×1.00 μ0.75 seed 0 | max_latency_ms | 15 | >= 25.000 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | drift_max | 0.137 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | drift_max | 0.141 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | drift_max | 0.136 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | drift_max | 0.140 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | drift_max | 0.137 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | drift_max | 0.140 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | drift_max | 0.157 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | drift_max | 0.153 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | drift_max | 0.182 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | drift_max | 0.178 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | drift_max | 0.138 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | drift_max | 0.135 | <= 0.050 |

## 항목별 전체 지표(최악값)

- **1 정지 60 s**: pitch_ma_max 0.534, pitch_rms 0.259, drift_max 0.022, wheel_rms10 0.099, wheel_sat_duty 0.000, wheel_peak 0.315, leg_peak 2.706, leg_rms 1.249, wheel_speed_peak 1.961, leg_speed_peak 0.674
- **2 초기 기울기 10°**: settled 1, settle_s 1.912, disp 0.097, wheel_peak 1.179, wheel_rms10 0.115, wheel_sat_duty 0.000, leg_peak 2.618, leg_rms 1.247, wheel_speed_peak 13.731, leg_speed_peak 2.563
- **3 초기 기울기 15°**: settled 1, settle_s 2.187, disp 0.150, wheel_sat_s 0.001, wheel_rms10 0.135, wheel_sat_duty 0.000, leg_peak 2.640, leg_rms 1.250, wheel_speed_peak 30.763, leg_speed_peak 3.415
- **4 초기 기울기 20°**: alive O
- **5 밀기 0.5 W**: settled 1, settle_s 2.433, pitch_peak 3.854, disp 0.014, wheel_peak 0.439, wheel_rms10 0.110, wheel_sat_duty 0.000, leg_peak 3.187, leg_rms 1.266, wheel_speed_peak 15.393, leg_speed_peak 2.330
- **6 밀기 0.75 W**: settled 1, settle_s 2.649, pitch_peak 6.012, wheel_rms10 0.125, wheel_sat_duty 0.000, leg_peak 4.215, leg_rms 1.284, wheel_speed_peak 22.850, leg_speed_peak 3.353
- **7 밀기 1.0 W**: alive O
- **8 속도 스텝 0.5 m/s**: rise_s 1.377, overshoot_pct 10.423, ss_err 0.027, pitch_peak 2.309, stop_disp 0.110, settled 1, wheel_peak 0.390, wheel_rms10 0.121, wheel_sat_duty 0.000, leg_peak 2.620, leg_rms 1.255, wheel_speed_peak 13.359, leg_speed_peak 1.035
- **9 요 1 rad/s**: yaw_err_pct 1.574, pitch_peak 0.388, alive O, wheel_peak 0.531, wheel_rms10 0.261 ✗, wheel_sat_duty 0.000, leg_peak 3.721, leg_rms 1.799, wheel_speed_peak 4.549, leg_speed_peak 0.691
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.636, settled 1, wheel_peak 0.298, wheel_rms10 0.098, wheel_sat_duty 0.000, leg_peak 4.031, leg_rms 1.486, wheel_speed_peak 4.315, leg_speed_peak 3.853
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.991, motors_on 1, settled 1, wheel_peak 0.379, wheel_rms10 0.111, wheel_sat_duty 0.000, leg_peak 2.615, leg_rms 1.252, wheel_speed_peak 12.669, leg_speed_peak 0.864
- **12 경사 5° 정지**: disp_max 0.155 ✗, pitch_dev_ma_max 0.341, pitch_dev_rms 0.094, wheel_rms10 0.179, wheel_peak 0.458, wheel_sat_duty 0.000, leg_peak 2.610, leg_rms 1.201, wheel_speed_peak 3.849, leg_speed_peak 1.332
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s - ✗, overshoot_pct 33.596 ✗, ss_err 0.144 ✗, pitch_peak 2.460, stop_disp 0.195, settled 1, wheel_peak 0.536, wheel_rms10 0.198, wheel_sat_duty 0.000, leg_peak 2.597, leg_rms 1.203, wheel_speed_peak 14.911, leg_speed_peak 1.034
- **15 지연 스윕(3° 기울기, 무노이즈)**: max_latency_ms 15 ✗
- **16 IMU 피치 바이어스 +1°**: pitch_ma_max 0.533, pitch_rms 0.261, drift_max 0.141 ✗, wheel_rms10 0.099, wheel_sat_duty 0.000, wheel_peak 0.315, leg_peak 2.707, leg_rms 1.249, wheel_speed_peak 2.087, leg_speed_peak 0.673
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: pitch_ma_max 0.651, pitch_rms 0.398, drift_max 0.182 ✗, wheel_rms10 0.099, wheel_sat_duty 0.000, wheel_peak 0.316, leg_peak 2.744, leg_rms 1.264, wheel_speed_peak 2.010, leg_speed_peak 0.658
