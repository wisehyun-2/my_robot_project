# Plan 4 합격 판정 — 200 Hz, 호스트 mpc, 매트릭스 full, 시드 5개

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
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 0.990 <= 1.500 (×0.66) | m×0.85 μ0.50 s0 |
| 9 | 요 1 rad/s | 필수-정착 | 통과 | 45 | wheel_rms10: 0.073 <= 0.200 (×0.37) | m×1.15 μ1.00 s2 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 45 | leg_rms: 1.409 <= 4.000 (×0.35) | m×1.15 μ0.50 s4 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 45 | stop_time: 2.360 <= 3.000 (×0.79) | m×0.85 μ0.50 s1 |
| 12 | 경사 5° 정지 | 필수-정착 | 통과 | 45 | wheel_rms10: 0.156 <= 0.200 (×0.78) | m×1.15 μ0.75 s0 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | 통과 | 45 | rise_s: 1.455 <= 1.500 (×0.97) | m×1.15 μ0.50 s1 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | 기록 | 통과 | 1 | max_latency_ms: 25 >= 25.000 (×1.00) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 45 | drift_max: 0.088 <= 0.050 (×1.76) | m×0.85 μ0.50 s4 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 45 | drift_max: 0.108 <= 0.050 (×2.15) | m×0.85 μ0.75 s1 |
| 19 | MPC 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 0.990 <= 1.500 (×0.66) | m×0.85 μ0.50 s0 |
| 21 | MPC 제약 준수(8·9·13·19) | 필수-정착 | 통과 | 45 | wheel_peak: 0.423 <= 1.360 (×0.31) | m×1.15 μ1.00 s1 |
| 22 | MPC 풀이 시간 | 기록 | 통과 | 1 | solve_p99_ms: 0.331 <= 5.000 (×0.07) | m×1.00 μ0.75 s0 |
| 23 | MPC 폴백(솔버 강제 실패) | 필수-생존 | 통과 | 1 | passes_1_8_11: 1 >= 1 (×1.00) | m×1.00 μ0.75 s0 |

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
- **2 초기 기울기 10°**: settled 1, settle_s 1.440, disp 0.087, wheel_peak 1.251, wheel_rms10 0.083, wheel_sat_duty 0.000, leg_peak 2.053, leg_rms 1.169, wheel_speed_peak 15.572, leg_speed_peak 1.248
- **3 초기 기울기 15°**: settled 1, settle_s 1.745, disp 0.129, wheel_sat_s 0.010, wheel_rms10 0.117, wheel_sat_duty 0.001, leg_peak 2.185, leg_rms 1.182, wheel_speed_peak 24.251, leg_speed_peak 1.929
- **4 초기 기울기 20°**: alive O
- **5 밀기 0.5 W**: settled 1, settle_s 1.945, pitch_peak 3.850, disp 0.012, wheel_peak 0.370, wheel_rms10 0.080, wheel_sat_duty 0.000, leg_peak 2.869, leg_rms 1.202, wheel_speed_peak 18.847, leg_speed_peak 3.102
- **6 밀기 0.75 W**: settled 1, settle_s 2.280, pitch_peak 5.768, wheel_rms10 0.112, wheel_sat_duty 0.000, leg_peak 3.945, leg_rms 1.223, wheel_speed_peak 27.989, leg_speed_peak 4.471
- **7 밀기 1.0 W**: alive O
- **8 속도 스텝 0.5 m/s**: rise_s 0.990, overshoot_pct 3.043, ss_err 0.013, pitch_peak 1.292, stop_disp 0.075, settled 1, wheel_peak 0.335, wheel_rms10 0.094, wheel_sat_duty 0.000, leg_peak 1.979, leg_rms 1.168, wheel_speed_peak 12.234, leg_speed_peak 0.713
- **9 요 1 rad/s**: yaw_err_pct 2.864, pitch_peak 0.381, alive O, wheel_peak 0.315, wheel_rms10 0.073, wheel_sat_duty 0.000, leg_peak 2.460, leg_rms 1.184, wheel_speed_peak 3.843, leg_speed_peak 0.580
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.893, settled 1, wheel_peak 0.153, wheel_rms10 0.048, wheel_sat_duty 0.000, leg_peak 3.583, leg_rms 1.409, wheel_speed_peak 4.074, leg_speed_peak 3.716
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.360, motors_on 1, settled 1, wheel_peak 0.248, wheel_rms10 0.060, wheel_sat_duty 0.000, leg_peak 1.947, leg_rms 1.170, wheel_speed_peak 11.811, leg_speed_peak 0.838
- **12 경사 5° 정지**: disp_max 0.062, pitch_dev_ma_max 0.456, pitch_dev_rms 0.178, wheel_rms10 0.156, wheel_peak 0.294, wheel_sat_duty 0.000, leg_peak 1.866, leg_rms 1.113, wheel_speed_peak 3.048, leg_speed_peak 0.929
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 1.455, overshoot_pct 3.168, ss_err 0.018, pitch_peak 2.416, stop_disp 0.107, settled 1, wheel_peak 0.423, wheel_rms10 0.177, wheel_sat_duty 0.000, leg_peak 1.890, leg_rms 1.119, wheel_speed_peak 12.246, leg_speed_peak 0.733
- **15 지연 스윕(3° 기울기, 무노이즈)**: max_latency_ms 25
- **16 IMU 피치 바이어스 +1°**: pitch_ma_max 0.715, pitch_rms 0.311, drift_max 0.088 ✗, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.138, leg_peak 1.969, leg_rms 1.165, wheel_speed_peak 1.743, leg_speed_peak 0.351
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: pitch_ma_max 0.805, pitch_rms 0.429, drift_max 0.108 ✗, wheel_rms10 0.045, wheel_sat_duty 0.000, wheel_peak 0.133, leg_peak 1.966, leg_rms 1.181, wheel_speed_peak 2.028, leg_speed_peak 0.311
- **19 MPC 속도 스텝 0.5 m/s**: pitch_peak 1.292, overshoot_pct 3.043, rise_s 0.990, stop_disp 0.075, settled 1, wheel_peak 0.335, wheel_rms10 0.094, wheel_sat_duty 0.000, leg_peak 1.979, leg_rms 1.168, wheel_speed_peak 12.234, leg_speed_peak 0.713
- **21 MPC 제약 준수(8·9·13·19)**: constraint_violations 0, wheel_peak 0.423, alive 1
- **22 MPC 풀이 시간**: solve_p99_ms 0.331, qp_failures 0
- **23 MPC 폴백(솔버 강제 실패)**: alive 1, fallback_used 1, passes_1_8_11 1
