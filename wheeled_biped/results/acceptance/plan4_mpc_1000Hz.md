# Plan 4 합격 판정 — 1000 Hz, 호스트 mpc, 매트릭스 full, 시드 5개

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
| 8 | 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 0.958 <= 1.500 (×0.64) | m×0.85 μ0.50 s4 |
| 9 | 요 1 rad/s | 필수-정착 | 통과 | 45 | wheel_rms10: 0.069 <= 0.200 (×0.34) | m×1.15 μ1.00 s2 |
| 10 | 4 cm 웅크림 후 복귀 | 필수-정착 | 통과 | 45 | leg_rms: 1.406 <= 4.000 (×0.35) | m×1.15 μ0.50 s1 |
| 11 | 링크 끊김(0.5 m/s 주행 중) | 필수-정착 | 통과 | 45 | stop_time: 2.153 <= 3.000 (×0.72) | m×0.85 μ1.00 s0 |
| 12 | 경사 5° 정지 | 필수-정착 | 통과 | 45 | wheel_rms10: 0.155 <= 0.200 (×0.77) | m×1.15 μ0.50 s2 |
| 13 | 경사 5° 주행 0.5 m/s(오르기·내리기) | 필수-정착 | 통과 | 45 | rise_s: 1.400 <= 1.500 (×0.93) | m×1.15 μ0.50 s4 |
| 15 | 지연 스윕(3° 기울기, 무노이즈) | 기록 | **실패** | 1 | max_latency_ms: 15 >= 25.000 (×1.67) | m×1.00 μ0.75 s0 |
| 16 | IMU 피치 바이어스 +1° | 필수-정착 | **실패** | 45 | drift_max: 0.087 <= 0.050 (×1.74) | m×0.85 μ0.50 s2 |
| 17 | 무게중심 전방 오프셋(0.3 kg, +10 cm) | 필수-정착 | **실패** | 45 | drift_max: 0.105 <= 0.050 (×2.10) | m×0.85 μ0.50 s1 |
| 19 | MPC 속도 스텝 0.5 m/s | 필수-정착 | 통과 | 45 | rise_s: 0.958 <= 1.500 (×0.64) | m×0.85 μ0.50 s4 |
| 21 | MPC 제약 준수(8·9·13·19) | 필수-정착 | 통과 | 45 | wheel_peak: 0.438 <= 1.360 (×0.32) | m×1.15 μ0.75 s3 |
| 22 | MPC 풀이 시간 | 기록 | 통과 | 1 | solve_p99_ms: 0.376 <= 5.000 (×0.08) | m×1.00 μ0.75 s0 |
| 23 | MPC 폴백(솔버 강제 실패) | 필수-생존 | 통과 | 1 | passes_1_8_11: 1 >= 1 (×1.00) | m×1.00 μ0.75 s0 |

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
- **2 초기 기울기 10°**: settled 1, settle_s 1.401, disp 0.087, wheel_peak 1.293, wheel_rms10 0.086, wheel_sat_duty 0.000, leg_peak 2.158, leg_rms 1.177, wheel_speed_peak 16.787, leg_speed_peak 2.490
- **3 초기 기울기 15°**: settled 1, settle_s 1.684, disp 0.132, wheel_sat_s 0.007, wheel_rms10 0.124, wheel_sat_duty 0.001, leg_peak 3.210, leg_rms 1.188, wheel_speed_peak 27.793, leg_speed_peak 3.798
- **4 초기 기울기 20°**: alive O
- **5 밀기 0.5 W**: settled 1, settle_s 1.882, pitch_peak 3.407, disp 0.007, wheel_peak 0.587, wheel_rms10 0.117, wheel_sat_duty 0.000, leg_peak 3.151, leg_rms 1.208, wheel_speed_peak 20.664, leg_speed_peak 3.738
- **6 밀기 0.75 W**: settled 1, settle_s 2.278, pitch_peak 6.148, wheel_rms10 0.227 ✗, wheel_sat_duty 0.000, leg_peak 5.917, leg_rms 1.377, wheel_speed_peak 31.700, leg_speed_peak 6.721
- **7 밀기 1.0 W**: alive O
- **8 속도 스텝 0.5 m/s**: rise_s 0.958, overshoot_pct 0.572, ss_err 0.013, pitch_peak 1.172, stop_disp 0.071, settled 1, wheel_peak 0.311, wheel_rms10 0.091, wheel_sat_duty 0.000, leg_peak 1.944, leg_rms 1.165, wheel_speed_peak 12.106, leg_speed_peak 1.024
- **9 요 1 rad/s**: yaw_err_pct 3.018, pitch_peak 0.419, alive O, wheel_peak 0.339, wheel_rms10 0.069, wheel_sat_duty 0.000, leg_peak 2.513, leg_rms 1.176, wheel_speed_peak 3.752, leg_speed_peak 0.752
- **10 4 cm 웅크림 후 복귀**: alive O, pitch_peak 0.932, settled 1, wheel_peak 0.221, wheel_rms10 0.050, wheel_sat_duty 0.000, leg_peak 3.576, leg_rms 1.406, wheel_speed_peak 4.592, leg_speed_peak 3.934
- **11 링크 끊김(0.5 m/s 주행 중)**: alive O, stop_time 2.153, motors_on 1, settled 1, wheel_peak 0.260, wheel_rms10 0.056, wheel_sat_duty 0.000, leg_peak 1.906, leg_rms 1.168, wheel_speed_peak 11.938, leg_speed_peak 1.261
- **12 경사 5° 정지**: disp_max 0.055, pitch_dev_ma_max 0.448, pitch_dev_rms 0.153, wheel_rms10 0.155, wheel_peak 0.292, wheel_sat_duty 0.000, leg_peak 1.969, leg_rms 1.104, wheel_speed_peak 3.734, leg_speed_peak 1.267
- **13 경사 5° 주행 0.5 m/s(오르기·내리기)**: rise_s 1.400, overshoot_pct 1.266, ss_err 0.017, pitch_peak 2.254, stop_disp 0.093, settled 1, wheel_peak 0.438, wheel_rms10 0.176, wheel_sat_duty 0.000, leg_peak 1.856, leg_rms 1.115, wheel_speed_peak 12.015, leg_speed_peak 1.071
- **15 지연 스윕(3° 기울기, 무노이즈)**: max_latency_ms 15 ✗
- **16 IMU 피치 바이어스 +1°**: pitch_ma_max 0.591, pitch_rms 0.297, drift_max 0.087 ✗, wheel_rms10 0.038, wheel_sat_duty 0.000, wheel_peak 0.146, leg_peak 1.879, leg_rms 1.160, wheel_speed_peak 1.519, leg_speed_peak 0.486
- **17 무게중심 전방 오프셋(0.3 kg, +10 cm)**: pitch_ma_max 0.711, pitch_rms 0.423, drift_max 0.105 ✗, wheel_rms10 0.038, wheel_sat_duty 0.000, wheel_peak 0.155, leg_peak 1.919, leg_rms 1.176, wheel_speed_peak 1.767, leg_speed_peak 0.481
- **19 MPC 속도 스텝 0.5 m/s**: pitch_peak 1.172, overshoot_pct 0.572, rise_s 0.958, stop_disp 0.071, settled 1, wheel_peak 0.311, wheel_rms10 0.091, wheel_sat_duty 0.000, leg_peak 1.944, leg_rms 1.165, wheel_speed_peak 12.106, leg_speed_peak 1.024
- **21 MPC 제약 준수(8·9·13·19)**: constraint_violations 0, wheel_peak 0.438, alive 1
- **22 MPC 풀이 시간**: solve_p99_ms 0.376, qp_failures 0
- **23 MPC 폴백(솔버 강제 실패)**: alive 1, fallback_used 1, passes_1_8_11 1
