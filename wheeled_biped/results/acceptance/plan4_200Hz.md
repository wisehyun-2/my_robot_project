# Plan 4 합격 판정 — 200 Hz, 호스트 ramp, 매트릭스 full, 시드 5개

**종합: 불합격**  (노이즈 ON, 지연 10 ms, 판정 피치 = 시뮬 실제 피치; 종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시)

| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |
|---|---|---|---|---|---|---|
| 1 | 정지 60 s | 필수-정착 | 통과 | 45 | pitch_ma_max: 0.687 <= 1.000 (×0.69) | m×1.15 μ0.50 s4 |
| 2 | 초기 기울기 10° | 필수-정착 | 통과 | 45 | wheel_peak: 1.251 <= 1.360 (×0.92) | m×1.00 μ0.50 s0 |
| 3 | 초기 기울기 15° | 필수-정착 | 통과 | 45 | wheel_rms10: 0.117 <= 0.200 (×0.59) | m×1.15 μ0.50 s3 |
| 4 | 초기 기울기 20° | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 5 | 밀기 0.5 W | 필수-정착 | 통과 | 45 | settle_s: 1.945 <= 3.000 (×0.65) | m×0.85 μ0.50 s1 |
| 6 | 밀기 0.75 W | 필수-정착 | 통과 | 45 | settle_s: 2.280 <= 4.000 (×0.57) | m×1.15 μ0.50 s1 |
| 7 | 밀기 1.0 W | 필수-생존 | 통과 | 1 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 1.245 <= 1.500 (×0.83) | m×0.85 μ0.50 s3 |
| 9 | 요 1 rad/s | 필수-정착 | 통과 | 45 | wheel_rms10: 0.073 <= 0.200 (×0.37) | m×1.15 μ1.00 s2 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 45 | leg_rms: 1.409 <= 4.000 (×0.35) | m×1.15 μ0.50 s4 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 45 | stop_time: 2.365 <= 3.000 (×0.79) | m×0.85 μ0.50 s1 |
| 12 | 경사 5° 정지 | 필수-정착 | 통과 | 45 | wheel_rms10: 0.156 <= 0.200 (×0.78) | m×1.15 μ0.75 s0 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | 통과 | 45 | rise_s: 1.410 <= 1.500 (×0.94) | m×1.15 μ0.50 s0 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | 기록 | 통과 | 1 | max_latency_ms: 25 >= 25.000 (×1.00) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 45 | drift_max: 0.088 <= 0.050 (×1.76) | m×0.85 μ0.50 s4 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 45 | drift_max: 0.108 <= 0.050 (×2.15) | m×0.85 μ0.75 s1 |

## 실패 목록

| # | 항목 | 조합 | 지표 | 측정 | 기준 |
|---|---|---|---|---|---|
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 4 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 0 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 1 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 2 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 4 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 0 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 1 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 4 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 0 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 4 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 4 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 0 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 3 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 4 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 0 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 3 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 4 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 0 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 3 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 4 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 3 | drift_max | 0.083 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 4 | drift_max | 0.087 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | drift_max | 0.099 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | drift_max | 0.107 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 2 | drift_max | 0.102 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 3 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 4 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 0 | drift_max | 0.099 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 1 | drift_max | 0.108 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 2 | drift_max | 0.103 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 3 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 4 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 0 | drift_max | 0.099 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 1 | drift_max | 0.107 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 2 | drift_max | 0.102 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 3 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 4 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 0 | drift_max | 0.090 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 1 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 2 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 3 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 4 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | drift_max | 0.090 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 2 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 3 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 4 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 0 | drift_max | 0.090 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 1 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 2 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 3 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 4 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 0 | drift_max | 0.080 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 1 | drift_max | 0.085 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 2 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 3 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 4 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 0 | drift_max | 0.079 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 1 | drift_max | 0.085 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 2 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 3 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 4 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | drift_max | 0.079 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | drift_max | 0.085 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 2 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 3 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 4 | drift_max | 0.083 | <= 0.050 |

## 항목별 전체 지표(최악값)

- **1 정지 60 s**: pitch_ma_max 0.687, pitch_rms 0.301, drift_max 0.024, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.134, leg_peak 1.943, leg_rms 1.165, wheel_speed_peak 1.601, leg_speed_peak 0.353
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, pitch_ma_max 2.063, pitch_rms 1.071, drift_max 0.334, drift_max_from_t0 0.192, drift_end 0.035, pitch_mean 1.040, pitch_ma_max_incl_transient 5.261, wheel_peak 0.410, wheel_rms10 0.103, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 0.179, leg_rms 0.122, wheel_speed_peak 4.750, leg_speed_peak 0.054
- **2 초기 기울기 10°**: settled 1, settle_s 1.440, disp 0.087, wheel_peak 1.251, wheel_rms10 0.083, wheel_sat_duty 0.000, leg_peak 2.053, leg_rms 1.169, wheel_speed_peak 15.572, leg_speed_peak 1.248
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.033, pitch_peak 10.000, wheel_peak 1.700, wheel_rms10 0.115, wheel_sat_duty 0.000, wheel_sat_s 0.001, leg_peak 0.334, leg_rms 0.125, wheel_speed_peak 10.078, leg_speed_peak 0.149
- **3 초기 기울기 15°**: settled 1, settle_s 1.745, disp 0.129, wheel_sat_s 0.010, wheel_rms10 0.117, wheel_sat_duty 0.001, leg_peak 2.185, leg_rms 1.182, wheel_speed_peak 24.251, leg_speed_peak 1.929
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.122, pitch_peak 15.000, wheel_peak 1.700, wheel_rms10 0.143, wheel_sat_duty 0.002, wheel_sat_s 0.013, leg_peak 1.170, leg_rms 0.135, wheel_speed_peak 16.952, leg_speed_peak 0.490
- **4 초기 기울기 20°**: alive O
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.194, pitch_peak 20.000, wheel_peak 1.700, wheel_rms10 0.172, wheel_sat_duty 0.004, wheel_sat_s 0.031, leg_peak 1.488, leg_rms 0.147, wheel_speed_peak 24.356, leg_speed_peak 0.517
- **5 밀기 0.5 W**: settled 1, settle_s 1.945, pitch_peak 3.850, disp 0.012, wheel_peak 0.370, wheel_rms10 0.080, wheel_sat_duty 0.000, leg_peak 2.869, leg_rms 1.202, wheel_speed_peak 18.847, leg_speed_peak 3.102
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, force_N 26.189, pitch_peak 16.141, disp 0.275, v_peak 0.977, wheel_peak 0.670, wheel_rms10 0.158, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 0.526, leg_rms 0.141, wheel_speed_peak 34.011, leg_speed_peak 0.163
- **6 밀기 0.75 W**: settled 1, settle_s 2.280, pitch_peak 5.768, wheel_rms10 0.112, wheel_sat_duty 0.000, leg_peak 3.945, leg_rms 1.223, wheel_speed_peak 27.989, leg_speed_peak 4.471
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive X, settled 0, force_N 39.283, pitch_peak 62.021, disp 1.926, v_peak 3.177, wheel_peak 1.037, wheel_rms10 0.382, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 1.749, leg_rms 0.278, wheel_speed_peak 39.964, leg_speed_peak 0.857
- **7 밀기 1.0 W**: alive O
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive X, settled 0, force_N 52.378, pitch_peak 61.382, disp 0.736, v_peak 2.241, wheel_peak 1.700, wheel_rms10 0.958, wheel_sat_duty 0.105, wheel_sat_s 0.092, leg_peak 2.219, leg_rms 0.747, wheel_speed_peak 39.968, leg_speed_peak 1.106
- **8 속도 스텝 0.5 m/s**: rise_s 1.245, overshoot_pct 13.290, ss_err 0.040, pitch_peak 2.017, stop_disp 0.088, settled 1, wheel_peak 0.295, wheel_rms10 0.089, wheel_sat_duty 0.000, leg_peak 1.981, leg_rms 1.174, wheel_speed_peak 13.047, leg_speed_peak 0.873
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, rise_s 0.591, overshoot_pct 41.707, ss_err 0.184, pitch_peak 11.656, stop_disp 0.081, settled 0, wheel_peak 1.700, wheel_rms10 0.171, wheel_sat_duty 0.003, wheel_sat_s 0.016, leg_peak 1.944, leg_rms 0.160, wheel_speed_peak 23.472, leg_speed_peak 0.849
- **9 요 1 rad/s**: yaw_err_pct 2.864, pitch_peak 0.381, alive O, wheel_peak 0.315, wheel_rms10 0.073, wheel_sat_duty 0.000, leg_peak 2.460, leg_rms 1.184, wheel_speed_peak 3.843, leg_speed_peak 0.580
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, yaw_rate_mean 0.927, yaw_err_pct 7.347, pitch_peak 3.581, wheel_peak 1.700, wheel_rms10 0.215, wheel_sat_duty 0.000, wheel_sat_s 0.001, leg_peak 2.210, leg_rms 1.135, wheel_speed_peak 7.496, leg_speed_peak 0.311
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.893, settled 1, wheel_peak 0.153, wheel_rms10 0.048, wheel_sat_duty 0.000, leg_peak 3.583, leg_rms 1.409, wheel_speed_peak 4.074, leg_speed_peak 3.716
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, pitch_peak 6.211, L0_min 0.307, settled 0, wheel_peak 0.421, wheel_rms10 0.111, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 1.888, leg_rms 0.669, wheel_speed_peak 8.608, leg_speed_peak 0.926
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.365, motors_on 1, settled 1, wheel_peak 0.286, wheel_rms10 0.074, wheel_sat_duty 0.000, leg_peak 1.965, leg_rms 1.176, wheel_speed_peak 13.420, leg_speed_peak 0.910
- **12 경사 5° 정지**: disp_max 0.062, pitch_dev_ma_max 0.456, pitch_dev_rms 0.178, wheel_rms10 0.156, wheel_peak 0.294, wheel_sat_duty 0.000, leg_peak 1.866, leg_rms 1.113, wheel_speed_peak 3.048, leg_speed_peak 0.929
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 1.410, overshoot_pct 13.024, ss_err 0.044, pitch_peak 2.500, stop_disp 0.119, settled 1, wheel_peak 0.480, wheel_rms10 0.178, wheel_sat_duty 0.000, leg_peak 1.898, leg_rms 1.116, wheel_speed_peak 12.966, leg_speed_peak 0.841
- **15 지연 스윕(3° 기울기, 무노이즈)**: max_latency_ms 25
- **16 IMU 피치 바이어스 +1°**: pitch_ma_max 0.715, pitch_rms 0.311, drift_max 0.088 ✗, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.138, leg_peak 1.969, leg_rms 1.165, wheel_speed_peak 1.743, leg_speed_peak 0.351
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: pitch_ma_max 0.805, pitch_rms 0.429, drift_max 0.108 ✗, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.133, leg_peak 1.966, leg_rms 1.181, wheel_speed_peak 2.028, leg_speed_peak 0.311
