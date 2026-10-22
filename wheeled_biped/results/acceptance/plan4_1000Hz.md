# Plan 4 합격 판정 — 1000 Hz, 호스트 ramp, 매트릭스 full, 시드 5개

**종합: 불합격**  (노이즈 ON, 지연 10 ms, 판정 피치 = 시뮬 실제 피치; 종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시; 코드 해시 1cbc282b92)

| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |
|---|---|---|---|---|---|---|
| 1 | 정지 60 s | 필수-정착 | 통과 | 45 | pitch_rms: 0.288 <= 0.500 (×0.58) | m×1.15 μ1.00 s0 |
| 2 | 초기 기울기 10° | 필수-정착 | 통과 | 45 | wheel_peak: 1.323 <= 1.360 (×0.97) | m×0.85 μ0.50 s3 |
| 3 | 초기 기울기 15° | 필수-정착 | 통과 | 45 | wheel_rms10: 0.125 <= 0.200 (×0.63) | m×0.85 μ0.50 s0 |
| 4 | 초기 기울기 20° | 필수-생존 | 통과 | 5 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 5 | 밀기 0.5 W | 필수-정착 | 통과 | 45 | settle_s: 1.899 <= 3.000 (×0.63) | m×0.85 μ0.75 s1 |
| 6 | 밀기 0.75 W | 필수-정착 | 통과 | 45 | wheel_rms10: 0.122 <= 0.200 (×0.61) | m×0.85 μ1.00 s3 |
| 7 | 밀기 1.0 W | 필수-생존 | 통과 | 5 | alive: O >= 1 (×0.00) | m×1.00 μ0.75 s0 |
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 1.250 <= 1.500 (×0.83) | m×0.85 μ1.00 s1 |
| 9 | 요 1 rad/s | 필수-정착 | 통과 | 45 | wheel_rms10: 0.107 <= 0.200 (×0.53) | m×1.15 μ1.00 s1 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 45 | leg_rms: 1.975 <= 4.000 (×0.49) | m×1.15 μ0.50 s2 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 45 | stop_time: 2.358 <= 3.000 (×0.79) | m×0.85 μ1.00 s2 |
| 12 | 경사 5° 정지 | 필수-정착 | 통과 | 45 | wheel_rms10: 0.156 <= 0.200 (×0.78) | m×1.15 μ0.75 s2 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | 통과 | 45 | rise_s: 1.384 <= 1.500 (×0.92) | m×1.15 μ0.50 s0 |
| 15 | 지연 스윕(3° 기울기, 무노이즈; 강건 여유도 함께 기록) | 기록 | **실패** | 1 | max_latency_ms: 20 >= 25.000 (×1.25) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 45 | drift_max: 0.088 <= 0.050 (×1.75) | m×0.85 μ1.00 s3 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 45 | drift_max: 0.107 <= 0.050 (×2.14) | m×0.85 μ0.75 s1 |
| 24 | 주행 중 밀기(0.3 m/s 순항, 0.75 W; 기록, 검토 r1) | 기록 | 통과 | 1 | settle_s: 1.897 <= 4.000 (×0.47) | m×1.00 μ0.75 s0 |

## 실패 목록

| # | 항목 | 조합 | 지표 | 측정 | 기준 |
|---|---|---|---|---|---|
| 15 | 지연 스윕(3° 기울기, 무노이즈; 강건 여유도 함께 기록) | m×1.00 μ0.75 seed 0 | max_latency_ms | 20 | >= 25.000 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 0 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 1 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 3 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.50 seed 4 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 0 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 1 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 3 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ0.75 seed 4 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 0 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 1 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 3 | drift_max | 0.088 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×0.85 μ1.00 seed 4 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 0 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 1 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 2 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 3 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.50 seed 4 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 0 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 1 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 2 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 3 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ0.75 seed 4 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 0 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 1 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 2 | drift_max | 0.086 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 3 | drift_max | 0.087 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.00 μ1.00 seed 4 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 0 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 2 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 3 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.50 seed 4 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 0 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 2 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 3 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ0.75 seed 4 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 0 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 1 | drift_max | 0.084 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 2 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 3 | drift_max | 0.085 | <= 0.050 |
| 16 | IMU 피치 바이어스 +1° | m×1.15 μ1.00 seed 4 | drift_max | 0.084 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 0 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 1 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 2 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 3 | drift_max | 0.104 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.50 seed 4 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 0 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 1 | drift_max | 0.107 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 2 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 3 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ0.75 seed 4 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 0 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 1 | drift_max | 0.107 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 2 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 3 | drift_max | 0.105 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×0.85 μ1.00 seed 4 | drift_max | 0.106 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 0 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 1 | drift_max | 0.094 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 2 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 3 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.50 seed 4 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 0 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 1 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 2 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 3 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ0.75 seed 4 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 0 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 1 | drift_max | 0.094 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 2 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 3 | drift_max | 0.092 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.00 μ1.00 seed 4 | drift_max | 0.093 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 0 | drift_max | 0.082 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 1 | drift_max | 0.084 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 2 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 3 | drift_max | 0.082 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.50 seed 4 | drift_max | 0.082 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 0 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 1 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 2 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 3 | drift_max | 0.082 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ0.75 seed 4 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 0 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 1 | drift_max | 0.084 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 2 | drift_max | 0.083 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 3 | drift_max | 0.082 | <= 0.050 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | m×1.15 μ1.00 seed 4 | drift_max | 0.082 | <= 0.050 |

## 항목별 전체 지표(최악값)

- **1 정지 60 s**: alive O, pitch_ma_max 0.547, pitch_rms 0.288, drift_max 0.025, wheel_rms10 0.038, wheel_sat_duty 0.000, wheel_peak 0.146, leg_peak 1.882, leg_rms 1.629, wheel_speed_peak 1.186, leg_speed_peak 0.483
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, pitch_ma_max 2.063, pitch_rms 1.071, drift_max 0.334, drift_max_from_t0 0.192, drift_end 0.035, pitch_mean 1.040, pitch_ma_max_incl_transient 5.261, wheel_peak 0.410, wheel_rms10 0.103, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 0.179, leg_rms 0.146, wheel_speed_peak 4.750, leg_speed_peak 0.054
- **2 초기 기울기 10°**: settled 1, settle_s 1.387, disp 0.134, wheel_peak 1.323, wheel_rms10 0.081, wheel_sat_duty 0.000, leg_peak 1.951, leg_rms 1.639, wheel_speed_peak 16.606, leg_speed_peak 2.059
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.069, disp_end 0.033, pitch_peak 10.000, wheel_peak 1.700, wheel_rms10 0.115, wheel_sat_duty 0.000, wheel_sat_s 0.001, leg_peak 0.334, leg_rms 0.146, wheel_speed_peak 10.078, leg_speed_peak 0.149
- **3 초기 기울기 15°**: settled 1, settle_s 1.723, disp 0.197, wheel_sat_s 0.008, wheel_rms10 0.125, wheel_sat_duty 0.001, leg_peak 2.716, leg_rms 1.657, wheel_speed_peak 36.824, leg_speed_peak 3.061
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.184, disp_end 0.122, pitch_peak 15.000, wheel_peak 1.700, wheel_rms10 0.143, wheel_sat_duty 0.002, wheel_sat_s 0.013, leg_peak 1.170, leg_rms 0.148, wheel_speed_peak 16.952, leg_speed_peak 0.490
- **4 초기 기울기 20°**: alive O
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, disp 0.295, disp_end 0.194, pitch_peak 20.000, wheel_peak 1.700, wheel_rms10 0.172, wheel_sat_duty 0.004, wheel_sat_s 0.031, leg_peak 1.488, leg_rms 0.150, wheel_speed_peak 24.356, leg_speed_peak 0.517
- **5 밀기 0.5 W**: settled 1, settle_s 1.899, pitch_peak 3.472, disp 0.148, wheel_peak 0.472, wheel_rms10 0.080, wheel_sat_duty 0.000, leg_peak 3.234, leg_rms 1.678, wheel_speed_peak 20.230, leg_speed_peak 3.537
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, settled 0, force_N 26.189, pitch_peak 16.141, disp 0.894, disp_end 0.275, v_peak 0.977, wheel_peak 0.670, wheel_rms10 0.158, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 0.526, leg_rms 0.148, wheel_speed_peak 34.011, leg_speed_peak 0.163
- **6 밀기 0.75 W**: settled 1, settle_s 2.305, pitch_peak 5.172, wheel_rms10 0.122, wheel_sat_duty 0.000, leg_peak 4.671, leg_rms 1.693, wheel_speed_peak 31.114, leg_speed_peak 5.372
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive X, settled 0, force_N 39.283, pitch_peak 62.021, disp 1.926, disp_end 1.926, v_peak 3.177, wheel_peak 1.037, wheel_rms10 0.382, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 1.749, leg_rms 0.370, wheel_speed_peak 39.964, leg_speed_peak 0.857
- **7 밀기 1.0 W**: alive O
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive X, settled 0, force_N 52.378, pitch_peak 61.382, disp 0.736, disp_end 0.736, v_peak 2.241, wheel_peak 1.700, wheel_rms10 0.958, wheel_sat_duty 0.199, wheel_sat_s 0.092, leg_peak 2.219, leg_rms 1.081, wheel_speed_peak 39.968, leg_speed_peak 1.106
- **8 속도 스텝 0.5 m/s**: rise_s 1.250, overshoot_pct 10.627, ss_err 0.039, pitch_peak 2.142, stop_disp 0.235, settled 1, wheel_peak 0.330, wheel_rms10 0.086, wheel_sat_duty 0.000, leg_peak 1.905, leg_rms 1.641, wheel_speed_peak 13.171, leg_speed_peak 1.190
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, rise_s 0.591, overshoot_pct 41.707, ss_err 0.184, pitch_peak 11.656, stop_disp 0.186, stop_end 0.081, stop_back 0.105, settled 0, wheel_peak 1.700, wheel_rms10 0.171, wheel_sat_duty 0.003, wheel_sat_s 0.016, leg_peak 1.944, leg_rms 0.163, wheel_speed_peak 23.472, leg_speed_peak 0.849
- **9 요 1 rad/s**: yaw_err_pct 0.494, pitch_peak 0.340, alive O, wheel_peak 0.327, wheel_rms10 0.107, wheel_sat_duty 0.000, leg_peak 2.514, leg_rms 1.967, wheel_speed_peak 3.571, leg_speed_peak 0.784
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, yaw_rate_mean 0.927, yaw_err_pct 7.347, pitch_peak 3.581, wheel_peak 1.700, wheel_rms10 0.215, wheel_sat_duty 0.000, wheel_sat_s 0.001, leg_peak 2.210, leg_rms 1.510, wheel_speed_peak 7.496, leg_speed_peak 0.311
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.696, settled 1, wheel_peak 0.158, wheel_rms10 0.042, wheel_sat_duty 0.000, leg_peak 3.490, leg_rms 1.975, wheel_speed_peak 3.929, leg_speed_peak 3.898
  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): alive O, pitch_peak 6.211, L0_min 0.307, settled 0, wheel_peak 0.421, wheel_rms10 0.111, wheel_sat_duty 0.000, wheel_sat_s 0.000, leg_peak 1.888, leg_rms 0.923, wheel_speed_peak 8.608, leg_speed_peak 0.926
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.358, motors_on 1, settled 1, wheel_peak 0.316, wheel_rms10 0.068, wheel_sat_duty 0.000, leg_peak 1.923, leg_rms 1.647, wheel_speed_peak 12.565, leg_speed_peak 1.187
- **12 경사 5° 정지**: alive O, disp_max 0.055, pitch_dev_ma_max 0.437, pitch_dev_rms 0.143, wheel_rms10 0.156, wheel_sat_duty 0.000, wheel_peak 0.280, leg_peak 1.947, leg_rms 1.549, wheel_speed_peak 3.642, leg_speed_peak 1.266
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 1.384, overshoot_pct 11.008, ss_err 0.041, pitch_peak 2.385, stop_disp 0.234, settled 1, wheel_peak 0.481, wheel_rms10 0.176, wheel_sat_duty 0.000, leg_peak 1.815, leg_rms 1.556, wheel_speed_peak 13.071, leg_speed_peak 1.205
- **15 지연 스윕(3° 기울기, 무노이즈; 강건 여유도 함께 기록)**: max_latency_ms 20 ✗, max_latency_robust_ms 12
- **16 IMU 피치 바이어스 +1°**: alive O, pitch_ma_max 0.569, pitch_rms 0.293, drift_max 0.088 ✗, wheel_rms10 0.038, wheel_sat_duty 0.000, wheel_peak 0.158, leg_peak 1.862, leg_rms 1.631, wheel_speed_peak 1.559, leg_speed_peak 0.484
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: alive O, pitch_ma_max 0.708, pitch_rms 0.419, drift_max 0.107 ✗, wheel_rms10 0.038, wheel_sat_duty 0.000, wheel_peak 0.156, leg_peak 1.916, leg_rms 1.664, wheel_speed_peak 1.822, leg_speed_peak 0.481
- **24 주행 중 밀기(0.3 m/s 순항, 0.75 W; 기록, 검토 r1)**: alive O, settled 1, settle_s 1.897, v_back 0.003
