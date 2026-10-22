# Plan 4 합격 판정 — 200 Hz, 호스트 ramp, 매트릭스 full, 시드 5개

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
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 1.245 <= 1.500 (×0.83) | m×0.85 μ0.50 s3 |
| 9 | 요 1 rad/s | 필수-정착 | 통과 | 45 | wheel_rms10: 0.109 <= 0.200 (×0.54) | m×1.15 μ1.00 s1 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 45 | leg_rms: 1.977 <= 4.000 (×0.49) | m×1.15 μ1.00 s4 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 45 | stop_time: 2.390 <= 3.000 (×0.80) | m×0.85 μ0.50 s1 |
| 12 | 경사 5° 정지 | 필수-정착 | 통과 | 45 | wheel_rms10: 0.159 <= 0.200 (×0.79) | m×1.15 μ1.00 s2 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | 통과 | 45 | rise_s: 1.405 <= 1.500 (×0.94) | m×1.15 μ0.50 s0 |
| 15 | 지연 스윕(3° 기울기, 무노이즈; 강건 여유도 함께 기록) | 기록 | 통과 | 1 | max_latency_ms: 25 >= 25.000 (×1.00) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 45 | drift_max: 0.091 <= 0.050 (×1.81) | m×0.85 μ0.75 s4 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 45 | drift_max: 0.110 <= 0.050 (×2.19) | m×0.85 μ0.50 s1 |
| 24 | 주행 중 밀기(0.3 m/s 순항, 0.75 W; 기록, 검토 r1) | 기록 | 통과 | 1 | settle_s: 1.990 <= 4.000 (×0.50) | m×1.00 μ0.75 s0 |

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
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, pitch_ma_max 2.063, pitch_rms 1.071, drift_max 0.334, drift_max_from_t0 0.192, drift_end 0.035, pitch_mean 1.040, pitch_ma_max_incl_transient 5.261, wheel_peak 0.410, wheel_rms10 0.103, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 0.179, leg_rms 0.146, wheel_speed_peak 4.750, leg_speed_peak 0.054
- **2 초기 기울기 10°**: settled 1, settle_s 1.400, disp 0.135, wheel_peak 1.288, wheel_rms10 0.084, wheel_sat_duty 0.000, leg_peak 1.972, leg_rms 1.634, wheel_speed_peak 15.609, leg_speed_peak 1.240
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.069, disp_end 0.033, pitch_peak 10.000, wheel_peak 1.700, wheel_rms10 0.115, wheel_sat_duty 0.000, wheel_sat_s 0.001, leg_peak 0.334, leg_rms 0.146, wheel_speed_peak 10.078, leg_speed_peak 0.149
- **3 초기 기울기 15°**: settled 1, settle_s 1.745, disp 0.191, wheel_sat_s 0.010, wheel_rms10 0.118, wheel_sat_duty 0.001, leg_peak 2.115, leg_rms 1.650, wheel_speed_peak 29.445, leg_speed_peak 2.014
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.184, disp_end 0.122, pitch_peak 15.000, wheel_peak 1.700, wheel_rms10 0.143, wheel_sat_duty 0.002, wheel_sat_s 0.013, leg_peak 1.170, leg_rms 0.148, wheel_speed_peak 16.952, leg_speed_peak 0.490
- **4 초기 기울기 20°**: alive O
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.295, disp_end 0.194, pitch_peak 20.000, wheel_peak 1.700, wheel_rms10 0.172, wheel_sat_duty 0.004, wheel_sat_s 0.031, leg_peak 1.488, leg_rms 0.150, wheel_speed_peak 24.356, leg_speed_peak 0.517
- **5 밀기 0.5 W**: settled 1, settle_s 1.940, pitch_peak 4.080, disp 0.151, wheel_peak 0.385, wheel_rms10 0.080, wheel_sat_duty 0.000, leg_peak 2.838, leg_rms 1.670, wheel_speed_peak 18.661, leg_speed_peak 3.015
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, force_N 26.189, pitch_peak 16.141, disp 0.894, disp_end 0.275, v_peak 0.977, wheel_peak 0.670, wheel_rms10 0.158, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 0.526, leg_rms 0.148, wheel_speed_peak 34.011, leg_speed_peak 0.163
- **6 밀기 0.75 W**: settled 1, settle_s 2.185, pitch_peak 6.052, wheel_rms10 0.112, wheel_sat_duty 0.000, leg_peak 3.892, leg_rms 1.683, wheel_speed_peak 27.730, leg_speed_peak 4.360
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive X, settled 0, force_N 39.283, pitch_peak 62.021, disp 1.926, disp_end 1.926, v_peak 3.177, wheel_peak 1.037, wheel_rms10 0.382, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 1.749, leg_rms 0.370, wheel_speed_peak 39.964, leg_speed_peak 0.857
- **7 밀기 1.0 W**: alive O
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive X, settled 0, force_N 52.378, pitch_peak 61.382, disp 0.736, disp_end 0.736, v_peak 2.241, wheel_peak 1.700, wheel_rms10 0.958, wheel_sat_duty 0.199, wheel_sat_s 0.092, leg_peak 2.219, leg_rms 1.081, wheel_speed_peak 39.968, leg_speed_peak 1.106
- **8 속도 스텝 0.5 m/s**: rise_s 1.245, overshoot_pct 12.589, ss_err 0.039, pitch_peak 2.201, stop_disp 0.236, settled 1, wheel_peak 0.306, wheel_rms10 0.089, wheel_sat_duty 0.000, leg_peak 1.912, leg_rms 1.639, wheel_speed_peak 12.851, leg_speed_peak 0.764
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, rise_s 0.591, overshoot_pct 41.707, ss_err 0.184, pitch_peak 11.656, stop_disp 0.186, stop_end 0.081, stop_back 0.105, settled 0, wheel_peak 1.700, wheel_rms10 0.171, wheel_sat_duty 0.003, wheel_sat_s 0.016, leg_peak 1.944, leg_rms 0.163, wheel_speed_peak 23.472, leg_speed_peak 0.849
- **9 요 1 rad/s**: yaw_err_pct 0.434, pitch_peak 0.385, alive O, wheel_peak 0.301, wheel_rms10 0.109, wheel_sat_duty 0.000, leg_peak 2.470, leg_rms 1.966, wheel_speed_peak 3.656, leg_speed_peak 0.585
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, yaw_rate_mean 0.927, yaw_err_pct 7.347, pitch_peak 3.581, wheel_peak 1.700, wheel_rms10 0.215, wheel_sat_duty 0.000, wheel_sat_s 0.001, leg_peak 2.210, leg_rms 1.510, wheel_speed_peak 7.496, leg_speed_peak 0.311
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.798, settled 1, wheel_peak 0.156, wheel_rms10 0.048, wheel_sat_duty 0.000, leg_peak 3.499, leg_rms 1.977, wheel_speed_peak 4.090, leg_speed_peak 3.722
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, pitch_peak 6.211, L0_min 0.307, settled 0, wheel_peak 0.421, wheel_rms10 0.111, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 1.888, leg_rms 0.923, wheel_speed_peak 8.608, leg_speed_peak 0.926
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.390, motors_on 1, settled 1, wheel_peak 0.315, wheel_rms10 0.071, wheel_sat_duty 0.000, leg_peak 1.918, leg_rms 1.645, wheel_speed_peak 12.515, leg_speed_peak 0.758
- **12 경사 5° 정지**: alive O, disp_max 0.062, pitch_dev_ma_max 0.446, pitch_dev_rms 0.174, wheel_rms10 0.159, wheel_sat_duty 0.000, wheel_peak 0.295, leg_peak 1.832, leg_rms 1.561, wheel_speed_peak 2.986, leg_speed_peak 0.928
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 1.405, overshoot_pct 12.448, ss_err 0.043, pitch_peak 2.527, stop_disp 0.233, settled 1, wheel_peak 0.485, wheel_rms10 0.178, wheel_sat_duty 0.000, leg_peak 1.862, leg_rms 1.561, wheel_speed_peak 12.778, leg_speed_peak 0.759
- **15 지연 스윕(3° 기울기, 무노이즈; 강건 여유도 함께 기록)**: max_latency_ms 25, max_latency_robust_ms 15
- **16 IMU 피치 바이어스 +1°**: alive O, pitch_ma_max 0.703, pitch_rms 0.308, drift_max 0.091 ✗, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.141, leg_peak 1.910, leg_rms 1.637, wheel_speed_peak 1.728, leg_speed_peak 0.283
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: alive O, pitch_ma_max 0.792, pitch_rms 0.431, drift_max 0.110 ✗, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.135, leg_peak 1.947, leg_rms 1.671, wheel_speed_peak 2.015, leg_speed_peak 0.241
- **24 주행 중 밀기(0.3 m/s 순항, 0.75 W; 기록, 검토 r1)**: alive O, settled 1, settle_s 1.990, v_back 0.004
