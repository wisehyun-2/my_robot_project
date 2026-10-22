# Plan 4 합격 판정 — 1000 Hz, 호스트 ramp, 매트릭스 full, 시드 5개

**종합: 불합격**  (노이즈 ON, 지연 10 ms, 판정 피치 = 시뮬 실제 피치; 종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시)

| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |
|---|---|---|---|---|---|---|
| 1 | 정지 60 s | 필수-정착 | 통과 | 45 | pitch_rms: 0.290 <= 0.500 (×0.58) | m×1.15 μ1.00 s3 |
| 2 | 초기 기울기 10° | 필수-정착 | 통과 | 45 | wheel_peak: 1.293 <= 1.360 (×0.95) | m×0.85 μ0.50 s3 |
| 3 | 초기 기울기 15° | 필수-정착 | 통과 | 45 | wheel_rms10: 0.124 <= 0.200 (×0.62) | m×0.85 μ0.50 s4 |
| 4 | 초기 기울기 20° | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 5 | 밀기 0.5 W | 필수-정착 | 통과 | 45 | settle_s: 1.882 <= 3.000 (×0.63) | m×0.85 μ0.75 s3 |
| 6 | 밀기 0.75 W | 필수-정착 | **실패** | 45 | wheel_rms10: 0.227 <= 0.200 (×1.13) | m×0.85 μ0.75 s2 |
| 7 | 밀기 1.0 W | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 1.248 <= 1.500 (×0.83) | m×0.85 μ0.50 s1 |
| 9 | 요 1 rad/s | 필수-정착 | 통과 | 45 | wheel_rms10: 0.069 <= 0.200 (×0.34) | m×1.15 μ1.00 s2 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 45 | leg_rms: 1.407 <= 4.000 (×0.35) | m×1.15 μ0.50 s1 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 45 | stop_time: 2.286 <= 3.000 (×0.76) | m×0.85 μ0.50 s2 |
| 12 | 경사 5° 정지 | 필수-정착 | 통과 | 45 | wheel_rms10: 0.155 <= 0.200 (×0.77) | m×1.15 μ0.50 s2 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | 통과 | 45 | rise_s: 1.390 <= 1.500 (×0.93) | m×1.15 μ0.50 s2 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | 기록 | **실패** | 1 | max_latency_ms: 15 >= 25.000 (×1.67) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 45 | drift_max: 0.087 <= 0.050 (×1.74) | m×0.85 μ0.50 s2 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 45 | drift_max: 0.105 <= 0.050 (×2.10) | m×0.85 μ0.50 s1 |

## 실패 목록

| # | 항목 | 조합 | 지표 | 측정 | 기준 |
|---|---|---|---|---|---|
| 6 | 밀기 0.75 W | m×0.85 μ0.75 seed 0 | wheel_rms10 | 0.222 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ0.75 seed 1 | wheel_rms10 | 0.223 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ0.75 seed 2 | wheel_rms10 | 0.227 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ0.75 seed 3 | wheel_rms10 | 0.223 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ0.75 seed 4 | wheel_rms10 | 0.224 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ1.00 seed 0 | wheel_rms10 | 0.222 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ1.00 seed 1 | wheel_rms10 | 0.225 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ1.00 seed 2 | wheel_rms10 | 0.226 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ1.00 seed 3 | wheel_rms10 | 0.223 | <= 0.200 |
| 6 | 밀기 0.75 W | m×0.85 μ1.00 seed 4 | wheel_rms10 | 0.222 | <= 0.200 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | m×1.00 μ0.75 seed 0 | max_latency_ms | 15 | >= 25.000 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 3 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 4 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 0 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 1 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 2 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 3 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 4 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 0 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 1 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 2 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 3 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 4 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 0 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 2 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 3 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 4 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 2 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 3 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 4 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 0 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 2 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 3 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 4 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 0 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 2 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 4 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 0 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 1 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 2 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 4 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 2 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 4 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 2 | drift_max | 0.103 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 3 | drift_max | 0.103 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 4 | drift_max | 0.103 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 0 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 1 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 2 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 3 | drift_max | 0.103 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 4 | drift_max | 0.103 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 0 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 1 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 2 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 3 | drift_max | 0.103 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 4 | drift_max | 0.103 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 0 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 1 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 2 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 3 | drift_max | 0.090 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 4 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 2 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 3 | drift_max | 0.090 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 4 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 0 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 1 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 2 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 3 | drift_max | 0.090 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 4 | drift_max | 0.091 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 0 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 1 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 2 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 3 | drift_max | 0.080 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 4 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 0 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 1 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 2 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 3 | drift_max | 0.080 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 4 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | drift_max | 0.082 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 2 | drift_max | 0.081 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 3 | drift_max | 0.080 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 4 | drift_max | 0.081 | <= 0.050 |

## 항목별 전체 지표(최악값)

- **1 정지 60 s**: pitch_ma_max 0.573, pitch_rms 0.290, drift_max 0.025, wheel_rms10 0.038, wheel_sat_duty 0.000, wheel_peak 0.140, leg_peak 1.882, leg_rms 1.159, wheel_speed_peak 1.221, leg_speed_peak 0.488
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, pitch_ma_max 2.063, pitch_rms 1.071, drift_max 0.334, drift_max_from_t0 0.192, drift_end 0.035, pitch_mean 1.040, pitch_ma_max_incl_transient 5.261, wheel_peak 0.410, wheel_rms10 0.103, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 0.179, leg_rms 0.122, wheel_speed_peak 4.750, leg_speed_peak 0.054
- **2 초기 기울기 10°**: settled 1, settle_s 1.401, disp 0.087, wheel_peak 1.293, wheel_rms10 0.086, wheel_sat_duty 0.000, leg_peak 2.158, leg_rms 1.177, wheel_speed_peak 16.787, leg_speed_peak 2.490
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.033, pitch_peak 10.000, wheel_peak 1.700, wheel_rms10 0.115, wheel_sat_duty 0.000, wheel_sat_s 0.001, leg_peak 0.334, leg_rms 0.125, wheel_speed_peak 10.078, leg_speed_peak 0.149
- **3 초기 기울기 15°**: settled 1, settle_s 1.684, disp 0.132, wheel_sat_s 0.007, wheel_rms10 0.124, wheel_sat_duty 0.001, leg_peak 3.210, leg_rms 1.188, wheel_speed_peak 27.793, leg_speed_peak 3.798
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.122, pitch_peak 15.000, wheel_peak 1.700, wheel_rms10 0.143, wheel_sat_duty 0.002, wheel_sat_s 0.013, leg_peak 1.170, leg_rms 0.135, wheel_speed_peak 16.952, leg_speed_peak 0.490
- **4 초기 기울기 20°**: alive O
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.194, pitch_peak 20.000, wheel_peak 1.700, wheel_rms10 0.172, wheel_sat_duty 0.004, wheel_sat_s 0.031, leg_peak 1.488, leg_rms 0.147, wheel_speed_peak 24.356, leg_speed_peak 0.517
- **5 밀기 0.5 W**: settled 1, settle_s 1.882, pitch_peak 3.407, disp 0.007, wheel_peak 0.587, wheel_rms10 0.117, wheel_sat_duty 0.000, leg_peak 3.151, leg_rms 1.208, wheel_speed_peak 20.664, leg_speed_peak 3.738
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, force_N 26.189, pitch_peak 16.141, disp 0.275, v_peak 0.977, wheel_peak 0.670, wheel_rms10 0.158, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 0.526, leg_rms 0.141, wheel_speed_peak 34.011, leg_speed_peak 0.163
- **6 밀기 0.75 W**: settled 1, settle_s 2.278, pitch_peak 6.148, wheel_rms10 0.227 ✗, wheel_sat_duty 0.000, leg_peak 5.917, leg_rms 1.377, wheel_speed_peak 31.700, leg_speed_peak 6.721
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive X, settled 0, force_N 39.283, pitch_peak 62.021, disp 1.926, v_peak 3.177, wheel_peak 1.037, wheel_rms10 0.382, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 1.749, leg_rms 0.278, wheel_speed_peak 39.964, leg_speed_peak 0.857
- **7 밀기 1.0 W**: alive O
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive X, settled 0, force_N 52.378, pitch_peak 61.382, disp 0.736, v_peak 2.241, wheel_peak 1.700, wheel_rms10 0.958, wheel_sat_duty 0.105, wheel_sat_s 0.092, leg_peak 2.219, leg_rms 0.747, wheel_speed_peak 39.968, leg_speed_peak 1.106
- **8 속도 스텝 0.5 m/s**: rise_s 1.248, overshoot_pct 10.858, ss_err 0.039, pitch_peak 1.933, stop_disp 0.087, settled 1, wheel_peak 0.320, wheel_rms10 0.086, wheel_sat_duty 0.000, leg_peak 1.953, leg_rms 1.174, wheel_speed_peak 13.248, leg_speed_peak 1.283
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, rise_s 0.591, overshoot_pct 41.707, ss_err 0.184, pitch_peak 11.656, stop_disp 0.081, settled 0, wheel_peak 1.700, wheel_rms10 0.171, wheel_sat_duty 0.003, wheel_sat_s 0.016, leg_peak 1.944, leg_rms 0.160, wheel_speed_peak 23.472, leg_speed_peak 0.849
- **9 요 1 rad/s**: yaw_err_pct 3.018, pitch_peak 0.419, alive O, wheel_peak 0.339, wheel_rms10 0.069, wheel_sat_duty 0.000, leg_peak 2.513, leg_rms 1.176, wheel_speed_peak 3.752, leg_speed_peak 0.752
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, yaw_rate_mean 0.927, yaw_err_pct 7.347, pitch_peak 3.581, wheel_peak 1.700, wheel_rms10 0.215, wheel_sat_duty 0.000, wheel_sat_s 0.001, leg_peak 2.210, leg_rms 1.135, wheel_speed_peak 7.496, leg_speed_peak 0.311
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.926, settled 1, wheel_peak 0.228, wheel_rms10 0.050, wheel_sat_duty 0.000, leg_peak 3.540, leg_rms 1.407, wheel_speed_peak 4.615, leg_speed_peak 3.923
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, pitch_peak 6.211, L0_min 0.307, settled 0, wheel_peak 0.421, wheel_rms10 0.111, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 1.888, leg_rms 0.669, wheel_speed_peak 8.608, leg_speed_peak 0.926
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.286, motors_on 1, settled 1, wheel_peak 0.316, wheel_rms10 0.070, wheel_sat_duty 0.000, leg_peak 1.948, leg_rms 1.177, wheel_speed_peak 13.891, leg_speed_peak 1.420
- **12 경사 5° 정지**: disp_max 0.055, pitch_dev_ma_max 0.448, pitch_dev_rms 0.153, wheel_rms10 0.155, wheel_peak 0.292, wheel_sat_duty 0.000, leg_peak 1.969, leg_rms 1.104, wheel_speed_peak 3.734, leg_speed_peak 1.267
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 1.390, overshoot_pct 11.168, ss_err 0.042, pitch_peak 2.358, stop_disp 0.119, settled 1, wheel_peak 0.484, wheel_rms10 0.176, wheel_sat_duty 0.000, leg_peak 1.875, leg_rms 1.115, wheel_speed_peak 13.172, leg_speed_peak 1.271
- **15 지연 스윕(3° 기울기, 무노이즈)**: max_latency_ms 15 ✗
- **16 IMU 피치 바이어스 +1°**: pitch_ma_max 0.591, pitch_rms 0.297, drift_max 0.087 ✗, wheel_rms10 0.038, wheel_sat_duty 0.000, wheel_peak 0.146, leg_peak 1.879, leg_rms 1.160, wheel_speed_peak 1.519, leg_speed_peak 0.486
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: pitch_ma_max 0.711, pitch_rms 0.423, drift_max 0.105 ✗, wheel_rms10 0.038, wheel_sat_duty 0.000, wheel_peak 0.155, leg_peak 1.919, leg_rms 1.176, wheel_speed_peak 1.767, leg_speed_peak 0.481
