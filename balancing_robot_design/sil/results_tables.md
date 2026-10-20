**5 kg급 로봇 (r 65 mm, l 0.16 m, 5 Nm/휠)**

| 제어기 | 최대 복귀 틸트 [°] | 최대 푸시 [N·0.1 s] | 0.8 m/s 스텝: 상승 s / 오버슈트 % / 최대 기울기 ° | 지연 허용 [ms] | 다리 1.4× 변화: 안정 / 최대 기울기 ° | 정지 토크 RMS [Nm] / 기울기 RMS [°] | 출력 스케일 |
|---|---|---|---|---|---|---|---|
| **내 제어기: LQI 1 kHz (온라인 리카티)** | 40.0 | 60 | 1.24 / 23 / 9.2 | 30 | O / 0.1 | 0.098 / 0.022 | — |
| A1 fern: 3상태 LQR 500 Hz (그들의 Q/R) | 10.0 | 10 | 실패 | 80 | O / 0.2 | 0.016 / 0.067 | — |
| A1 fern + 중력 FF2 0.5 | 30.0 | 20 | 실패 | 50 | O / 0.1 | 0.025 / 0.045 | — |
| A1 fern 구조 + 내 Q/R (속도 PI 0.3/0.6) | 40.0 | 160 | 0.83 / 39 / 17.8 | 30 | O / 0.5 | 0.074 / 0.102 | — |
| A1 fern 구조 + 내 Q/R (속도 PI 0.15/0.3) | 40.0 | 50 | 0.96 / 17 / 11.7 | 40 | O / 0.1 | 0.071 / 0.050 | — |
| A2 LZU_ASCENTO: 4상태 LQR 1 kHz | 37.5 | 40 | 1.93 / 8 / 7.1 | 60 | O / 0.1 | 0.020 / 0.021 | 1 |
| A3 zephyr_ascento: 4상태 LQR 1 kHz (코드 부호) | 40.0 | 85 | 0.55 / 75 / 20.0 | 30 | O / 0.1 | 0.052 / 0.033 | 1.78 |
| A3 zephyr_ascento (v·x 부호 반전) | 균형 불가 (no output scale makes it balance on this robot) | | | | | | |
| A4 Pow_WBR: 4상태 LQR 125 Hz | 40.0 | 125 | 1.73 / 17 / 9.4 | 50 | O / 0.1 | 0.022 / 0.054 | 1.78 |
| A5 MuShibo: 4항 P + 소토크 PI ~1 kHz | 7.5 | 10 | 실패 | 5 | O / 5.9 | 0.150 / 0.067 | 0.562 |
| A6 WL1: 캐스케이드 PID 100/20 Hz | 균형 불가 (no output scale makes it balance on this robot) | | | | | | |
| A7 Venus-1: 캐스케이드 PID 1 kHz | 균형 불가 (no output scale makes it balance on this robot) | | | | | | |
| A8 SonicRobot: PID→속도모드 100 Hz | 20.0 | 5 | 실패 | 5 | O / 0.6 | 0.267 / 0.192 | 0.562 |
| A9 Ryung: 3중 PID→속도모드, 실제 배치(100 Hz, 30 ms) | 0.0 | 0 | 실패 | 5 | X / 12.0 | — / — | — |
| A9 Ryung: 같은 코드, MCU 가정(100 Hz, 2 ms) | 12.5 | 5 | 실패 | 5 | O / 0.1 | 0.225 / 0.044 | — |
| U1 upkie WheelBalancer(스파인 1 kHz, 속도 PI→moteus 속도모드) | 40.0 | 10 | 실패 | 60 | O / 0.1 | 0.199 / 0.029 | — |
| U1 upkie WheelBalancer, 무른 휠 속도 루프(Kp 0.2) | 40.0 | 10 | 실패 | 30 | O / 0.1 | 0.039 / 0.034 | — |
| U2 upkie pd_balancing.py (1 kHz, 속도모드 PD) | 15.0 | 5 | 실패 | 25 | O / 0.1 | 1.112 / 0.043 | — |
| U3 upkie MPCBalancer (qpmpc+ProxQP, 200 Hz, 실제 Python 코드) | 37.5 | 20 | 0.81 / 12 / 12.3 | 30 | O / 1.0 | 0.004 / 0.072 | — |
| U3 upkie MPCBalancer, 무른 휠 속도 루프(Kp 0.2) | 균형 불가 (does not balance with either sign on this robot) | | | | | | |
| U4 upkie torque_balancing.py (200 Hz, 토크 P만) | 균형 불가 (no output scale makes it balance on this robot) | | | | | | |

**10 kg급 로봇 (r 82.5 mm, l 0.25 m, 5 Nm/휠)**

| 제어기 | 최대 복귀 틸트 [°] | 최대 푸시 [N·0.1 s] | 0.8 m/s 스텝: 상승 s / 오버슈트 % / 최대 기울기 ° | 지연 허용 [ms] | 다리 1.4× 변화: 안정 / 최대 기울기 ° | 정지 토크 RMS [Nm] / 기울기 RMS [°] | 출력 스케일 |
|---|---|---|---|---|---|---|---|
| **내 제어기: LQI 1 kHz (온라인 리카티)** | 40.0 | 120 | 1.21 / 25 / 8.1 | 40 | O / 0.1 | 0.128 / 0.023 | — |
| A1 fern: 3상태 LQR 500 Hz (그들의 Q/R) | 15.0 | 20 | 실패 | 60 | O / 0.2 | 0.042 / 0.083 | — |
| A1 fern + 중력 FF2 0.5 | 30.0 | 40 | 실패 | 40 | O / 0.1 | 0.071 / 0.054 | — |
| A1 fern 구조 + 내 Q/R (속도 PI 0.3/0.6) | 0.0 | 0 | 실패 | — | X / 16.7 | 0.185 / 0.584 | — |
| A1 fern 구조 + 내 Q/R (속도 PI 0.15/0.3) | 40.0 | 160 | 0.68 / 20 / 13.5 | 50 | O / 0.2 | 0.095 / 0.067 | — |
| A2 LZU_ASCENTO: 4상태 LQR 1 kHz | 35.0 | 85 | 2.40 / 8 / 5.0 | 50 | O / 0.1 | 0.069 / 0.020 | 3.16 |
| A3 zephyr_ascento: 4상태 LQR 1 kHz (코드 부호) | 37.5 | 80 | 0.32 / 90 / 28.2 | 50 | O / 0.7 | 0.057 / 0.072 | 1.78 |
| A3 zephyr_ascento (v·x 부호 반전) | 균형 불가 (no output scale makes it balance on this robot) | | | | | | |
| A4 Pow_WBR: 4상태 LQR 125 Hz | 40.0 | 160 | 1.16 / 29 / 10.3 | 50 | O / 0.2 | 0.043 / 0.062 | 3.16 |
| A5 MuShibo: 4항 P + 소토크 PI ~1 kHz | 10.0 | 15 | 실패 | 0 | X / 12.2 | 0.259 / 0.079 | 1 |
| A6 WL1: 캐스케이드 PID 100/20 Hz | 균형 불가 (no output scale makes it balance on this robot) | | | | | | |
| A7 Venus-1: 캐스케이드 PID 1 kHz | 균형 불가 (no output scale makes it balance on this robot) | | | | | | |
| A8 SonicRobot: PID→속도모드 100 Hz | 0.0 | 5 | 실패 | 0 | X / 4.5 | 0.902 / 0.293 | 1.78 |
| A9 Ryung: 3중 PID→속도모드, 실제 배치(100 Hz, 30 ms) | 0.0 | 0 | 실패 | 20 | X / 4.6 | — / — | — |
| A9 Ryung: 같은 코드, MCU 가정(100 Hz, 2 ms) | 12.5 | 10 | 실패 | 20 | O / 0.1 | 0.185 / 0.044 | — |
| U1 upkie WheelBalancer(스파인 1 kHz, 속도 PI→moteus 속도모드) | 40.0 | 20 | 0.32 / 117 / 17.9 | 50 | O / 0.1 | 0.156 / 0.033 | — |
| U1 upkie WheelBalancer, 무른 휠 속도 루프(Kp 0.2) | 균형 불가 (does not balance with either sign on this robot) | | | | | | |
| U2 upkie pd_balancing.py (1 kHz, 속도모드 PD) | 10.0 | 10 | 실패 | 30 | O / 0.1 | 0.867 / 0.037 | — |
| U3 upkie MPCBalancer (qpmpc+ProxQP, 200 Hz, 실제 Python 코드) | 27.5 | 40 | 0.74 / 23 / 12.0 | 50 | O / 0.2 | 0.010 / 0.049 | — |
| U3 upkie MPCBalancer, 무른 휠 속도 루프(Kp 0.2) | 균형 불가 (does not balance with either sign on this robot) | | | | | | |
| U4 upkie torque_balancing.py (200 Hz, 토크 P만) | 2.5 | 5 | — / -96 / 0.5 | 0 | O / 0.4 | 0.028 / 0.159 | 0.316 |
