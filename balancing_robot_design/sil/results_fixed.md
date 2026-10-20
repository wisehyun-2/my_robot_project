**5 kg급 — 고정 시나리오** (정착시간(1°) / 최대 속도 / 최종 변위 / 최대 토크·휠)

| 제어기 | 10° 기울임 복귀 | 40 N × 0.1 s 푸시 |
|---|---|---|
| **내 제어기: LQI 1 kHz (온라인 리카티)** | 0.83 s / 0.34 m/s / 0.00 m / 1.72 Nm | 1.73 s / 1.44 m/s / 0.04 m / 1.08 Nm |
| A1 fern: 3상태 LQR 500 Hz (그들의 Q/R) | 4.79 s / 0.33 m/s / 0.02 m / 0.37 Nm | 실패 |
| A1 fern + 중력 FF2 0.5 | 1.19 s / 0.34 m/s / 0.01 m / 0.59 Nm | 실패 |
| A1 fern 구조 + 내 Q/R (속도 PI 0.3/0.6) | 0.54 s / 0.35 m/s / 0.00 m / 1.47 Nm | 2.33 s / 1.36 m/s / 0.09 m / 0.95 Nm |
| A1 fern 구조 + 내 Q/R (속도 PI 0.15/0.3) | 0.77 s / 0.35 m/s / 0.00 m / 1.47 Nm | 3.14 s / 1.36 m/s / 0.01 m / 0.96 Nm |
| A2 LZU_ASCENTO: 4상태 LQR 1 kHz | 1.58 s / 0.28 m/s / 0.04 m / 0.47 Nm | 2.57 s / 1.30 m/s / 0.28 m / 0.70 Nm |
| A3 zephyr_ascento: 4상태 LQR 1 kHz (코드 부호) | 0.76 s / 0.39 m/s / 0.00 m / 1.17 Nm | 2.27 s / 1.76 m/s / 0.00 m / 1.43 Nm |
| A4 Pow_WBR: 4상태 LQR 125 Hz | 0.66 s / 0.33 m/s / 0.14 m / 0.53 Nm | 2.09 s / 1.41 m/s / 0.98 m / 0.83 Nm |
| A5 MuShibo: 4항 P + 소토크 PI ~1 kHz | 실패 | 실패 |
| A8 SonicRobot: PID→속도모드 100 Hz | 3.68 s / 0.88 m/s / 0.01 m / 1.97 Nm | 실패 |
| A9 Ryung: 3중 PID→속도모드, 실제 배치(100 Hz, 30 ms) | 실패 | 실패 |
| A9 Ryung: 같은 코드, MCU 가정(100 Hz, 2 ms) | 0.19 s / 0.49 m/s / 0.70 m / 3.99 Nm | 실패 |
| U1 upkie WheelBalancer(스파인 1 kHz, 속도 PI→moteus 속도모드) | 0.52 s / 0.36 m/s / 0.02 m / 4.92 Nm | 실패 |
| U1 upkie WheelBalancer, 무른 휠 속도 루프(Kp 0.2) | 1.18 s / 0.41 m/s / 0.02 m / 1.00 Nm | 실패 |
| U2 upkie pd_balancing.py (1 kHz, 속도모드 PD) | 0.05 s / 0.98 m/s / 0.20 m / 5.00 Nm | 실패 |
| U3 upkie MPCBalancer (qpmpc+ProxQP, 200 Hz, 실제 Python 코드) | 4.81 s / 0.36 m/s / 0.16 m / 0.34 Nm | 실패 |
| B10 WheelLeg_LQR(1772631792): 4상태 카트-폴 dLQR 1 kHz, 그들의 Q/R을 이 로봇에 설계 | 1.45 s / 0.27 m/s / 0.02 m / 1.02 Nm | 실패 |
| B10 WheelLeg_LQR: 그들의 로봇(1+5 kg, COM 0.25 m) K 그대로 | 1.12 s / 0.29 m/s / 0.02 m / 1.04 Nm | 실패 |
| A1 fern 구조 + 내 Q/R (속도 PI 0.15/0.3) | 0.77 s / 0.35 m/s / 0.00 m / 1.47 Nm | 3.14 s / 1.36 m/s / 0.01 m / 0.96 Nm |

**10 kg급 — 고정 시나리오** (정착시간(1°) / 최대 속도 / 최종 변위 / 최대 토크·휠)

| 제어기 | 10° 기울임 복귀 | 40 N × 0.1 s 푸시 |
|---|---|---|
| **내 제어기: LQI 1 kHz (온라인 리카티)** | 0.97 s / 0.43 m/s / 0.00 m / 2.28 Nm | 1.46 s / 0.74 m/s / 0.02 m / 0.80 Nm |
| A1 fern: 3상태 LQR 500 Hz (그들의 Q/R) | 0.67 s / 0.48 m/s / 0.00 m / 1.97 Nm | 2.05 s / 0.79 m/s / 0.00 m / 0.97 Nm |
| A1 fern + 중력 FF2 0.5 | 0.77 s / 0.54 m/s / 0.00 m / 2.71 Nm | 2.69 s / 0.76 m/s / 0.01 m / 0.84 Nm |
| A1 fern 구조 + 내 Q/R (속도 PI 0.3/0.6) | 실패 | 실패 |
| A1 fern 구조 + 내 Q/R (속도 PI 0.15/0.3) | 0.67 s / 0.48 m/s / 0.00 m / 1.97 Nm | 2.05 s / 0.79 m/s / 0.00 m / 0.97 Nm |
| A2 LZU_ASCENTO: 4상태 LQR 1 kHz | 1.41 s / 0.36 m/s / 0.05 m / 1.66 Nm | 2.26 s / 0.64 m/s / 0.16 m / 0.63 Nm |
| A3 zephyr_ascento: 4상태 LQR 1 kHz (코드 부호) | 1.68 s / 0.51 m/s / 0.00 m / 1.27 Nm | 2.35 s / 1.01 m/s / 0.00 m / 1.45 Nm |
| A4 Pow_WBR: 4상태 LQR 125 Hz | 0.79 s / 0.44 m/s / 0.16 m / 1.03 Nm | 1.31 s / 0.74 m/s / 0.38 m / 0.65 Nm |
| A5 MuShibo: 4항 P + 소토크 PI ~1 kHz | 1.94 s / 1.31 m/s / 0.09 m / 5.00 Nm | 실패 |
| A8 SonicRobot: PID→속도모드 100 Hz | 실패 | 실패 |
| A9 Ryung: 3중 PID→속도모드, 실제 배치(100 Hz, 30 ms) | 실패 | 실패 |
| A9 Ryung: 같은 코드, MCU 가정(100 Hz, 2 ms) | 0.19 s / 0.50 m/s / 0.85 m / 4.02 Nm | 실패 |
| U1 upkie WheelBalancer(스파인 1 kHz, 속도 PI→moteus 속도모드) | 1.10 s / 0.44 m/s / 0.00 m / 3.89 Nm | 실패 |
| U2 upkie pd_balancing.py (1 kHz, 속도모드 PD) | 5.00 s / 0.96 m/s / 0.42 m / 5.00 Nm | 실패 |
| U3 upkie MPCBalancer (qpmpc+ProxQP, 200 Hz, 실제 Python 코드) | 2.70 s / 0.50 m/s / 0.20 m / 0.73 Nm | 3.52 s / 1.12 m/s / 0.59 m / 0.93 Nm |
| U4 upkie torque_balancing.py (200 Hz, 토크 P만) | 실패 | 실패 |
| B10 WheelLeg_LQR(1772631792): 4상태 카트-폴 dLQR 1 kHz, 그들의 Q/R을 이 로봇에 설계 | 1.27 s / 0.40 m/s / 0.02 m / 1.33 Nm | 1.85 s / 0.69 m/s / 0.03 m / 0.63 Nm |
| B10 WheelLeg_LQR: 그들의 로봇(1+5 kg, COM 0.25 m) K 그대로 | 2.54 s / 0.44 m/s / 0.00 m / 0.75 Nm | 2.67 s / 0.71 m/s / 0.00 m / 0.56 Nm |
| A1 fern 구조 + 내 Q/R (속도 PI 0.15/0.3) | 0.67 s / 0.48 m/s / 0.00 m / 1.97 Nm | 2.05 s / 0.79 m/s / 0.00 m / 0.97 Nm |
