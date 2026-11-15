# v5_lqi — LQI8 하나만 쓰는 v5 로봇

v5 로봇(Fusion에서 뽑은 `models/wheelbiped_v5`, AKE60-8 × 6, 링크 25 cm)의 균형 제어기를 **LQI8 하나**로 두는 폴더다.
MPC는 이 폴더 어디에도 없다. 호스트는 MotionCtrl 목표값만 보낸다.
MPC로 같은 일을 하는 폴더는 `../v5_mpc/`다.

```
호스트(50 Hz): 목표 속도·요 속도·높이 (HostEmulator)
      │
MCU(1 kHz): 상태 추정 ─▶ LQI8 (T, Tp) ─▶ 높이·요·롤·벌어짐 루프 ─▶ 직렬 다리 VMC ─▶ 관절 토크
```

## 파일

| 경로 | 내용 |
|---|---|
| `controller.py` | 이 폴더의 제어기: 전체 제어기 정의(`STAGES`, `FULL_STAGE` = LQR + ∫위치 + ∫기울기), `make_plan()` |
| `TEST/` | 통과 시험. 안내는 `TEST/README.md` |
| `../wbr/controllers/lqi8/` | LQI8 코드(`lqi8.py` 게인표·기준 성형·적분, `law.py` MCU 연결) |
| `../wbr/plans/plan_v5.py` | v5 폐루프(URDF, IMU 경로, 게인). 균형 제어기는 `ClosedLoop._make_law`가 만드는 LQI8 |

로봇 모델, 시뮬레이터, IMU 경로, MCU와 바깥 루프 네 개는 MPC 폴더와 같은 코드다. 두 폴더는 MCU 안의 균형 제어기만 다르다.

## 실행 (`wheeled_biped/`에서)

```
python v5_lqi/TEST/run_stages.py --stage 4 --keep-going     # 지금 쓰는 전체 제어기의 단계 시험
python -c "from v5_lqi.controller import make_plan; p = make_plan(); p.reset(); print(p.run(2.0))"
```
