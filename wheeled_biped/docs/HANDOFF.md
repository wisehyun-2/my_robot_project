# 인수인계: Windows 세션 → 우분투(듀얼부팅)에서 이어가기

작성 2026-10-22, 갱신 2026-10-23(Plan 4). 이 문서는 Windows PC의 Claude Code 세션에서 한 일과, 우분투에서 Claude Code로 이어갈 때 필요한 것을 정리한 것이다. 대화 기록 요약본은 `docs/session_log_2026-10-21.md`, 기술 결론은 `docs/비교보고서.md`(Plan 4는 11절), 독립 검토는 `docs/review_opus*.md`·`docs/review_plan4_r*.md`에 있다.

## 0. 최종 확정안 (2026-10-22)

**Plan 4 = upkie 하드웨어(직렬 무릎 다리, qdd100 ×4 + mj5208 ×2, 112 mm 타이어) + LQI6 균형기 + DIABLO식 호스트/MCU 분리.** 2단계로 호스트 MPC 기준 생성기를 LQI6 위에 둔다. 라이다·카메라·SLAM·로봇팔 2개는 균형 제어가 실기에서 안정된 뒤에 붙인다(그때 팔 관절 질량 피드포워드, 롤 보상, Jetson 호스트, 3D 라이다 관성 오도메트리가 필요).

핵심 문서: `docs/합격기준.md`(사용자가 정한 시뮬 합격 기준), `docs/plan4_model.md`(플랜트 유도·MuJoCo 대조), `docs/plan4_lqi6.md`(제어기·합격 판정 결과), `docs/plan4_mpc_host.md`(2단계).

## 1. 한 일 (상태)

| 항목 | 상태 | 위치 |
|---|---|---|
| ROS 비의존 코어 `wbr`(MuJoCo 백엔드, upkie 관측기·PI·MPC 포팅, LQI6+5절 링크 VMC, DIABLO MCU/호스트 링크 에뮬레이션, 벤치마크, 라이브 뷰어, 영상) | 완료, Windows에서 검증 | `wbr/` |
| 로봇 모델: upkie(URDF→MJCF, 타이어 0.05), **upkie_plan4(타이어 0.056 = BOM 112 mm)**, DIABLO-lite 6 kg | 완료 | `models/`, `wbr/models/` |
| Plan 1/2/3 조립과 벤치마크 | 완료(비교보고서 0~10절) | `results/bench` |
| **Plan 4**: 직렬 다리 기구학(`serial_leg.py`), 3체 플랜트 라그랑지안 유도·선형화(`plant_serial.py`), 파라미터 표(`upkie_params.py`), MuJoCo 대조(`verify_plan4_plant.py`), 조립(`plans/plan4_upkie_lqi6.py`), 합격 판정 배터리(`bench/acceptance.py`) | 완료, Windows에서 검증. 모델은 `models/upkie_plan4`(112 mm 타원체 타이어) | `wbr/`, `docs/plan4_*.md`, `results/acceptance/` |
| 2단계 호스트 MPC(`wbr/controllers/mpc_host/`) | 완료: 19·21·22·23 통과, 회귀 없음(전체 매트릭스) | `docs/plan4_mpc_host.md` |
| Plan 4 합격 판정(전체 매트릭스) | 1 kHz: 6(3/45회)·16·17 실패, 200 Hz: 16·17 실패 — 사용자 결정 항목(`docs/plan4_lqi6.md` 4절) | `results/acceptance/` |
| 단위 테스트 | 58개 통과(`tests/test_core.py` 20 + `tests/test_plan4.py` 38) | `tests/` |
| ROS 2 Jazzy 패키지(메시지 3종, upkie 4패키지, diablo_lite 4패키지) | **작성만 됨, 빌드·실행 미검증**. Plan 4(lqi6 균형기) 옵션은 아직 없음 | `ros2_ws/src`, `ros2_ws/README.md` |

## 2. 우분투에서 할 일 (순서대로)

1. 환경: Ubuntu 24.04 + ROS 2 Jazzy desktop, `git clone https://github.com/wisehyun-2/my_robot_project`, NVIDIA 드라이버(RTX 5080은 570 이상, 오픈 커널 모듈 권장).
2. 파이썬: `ros2_ws/README.md` 1절의 (a) 방식 — `python3 -m venv --system-site-packages ~/wbr_venv && source ~/wbr_venv/bin/activate && pip install -e wheeled_biped`. (`qpsolvers`, `daqp`, `osqp`, `qpmpc`, `sympy`, `imageio[ffmpeg]`도 필요.)
3. 코어 확인(ROS 없이): `cd wheeled_biped && python -m wbr.models.build_upkie && python -m wbr.models.build_upkie --out models/upkie_plan4 --tire-radius 0.056 && python -m wbr.models.upkie_params && python -m unittest discover -s tests && python -m wbr.viewer.live --plan 4 --noise --latency-ms 10`. 합격 판정 재현: `python -m wbr.bench.acceptance --rate both --matrix full --seeds 5 --reference`(약 20~40분, 코어 수에 따라).
4. ROS 2 빌드: `cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install && source install/setup.bash`. 빌드 오류는 거의 확실히 몇 개 있을 것(Windows에서 한 번도 빌드하지 못함). 예상 지점: `upkie_description` 래퍼 CMake의 상대 경로, xacro 처리, `setup.py`의 data_files, rosidl 의존성.
5. **Plan 4용 ROS 2 작업**: `upkie_balancer`에 `balancer:=lqi6` 옵션을 추가해 `Plan4UpkieLQI6`의 MCU(`DiabloMcu` + `LQI6Law` + `SerialLeg`)를 균형 노드로 쓰고, 호스트 쪽은 `diablo_lite_ctrl`의 텔레옵/`cmd_vel` 브리지를 재사용한다(MotionCtrl 의미론 동일). 실기에서는 균형 루프를 ROS 밖(upkie 스파인 에이전트, 200 Hz)에서 돌리고 ROS 2는 호스트 계층(50 Hz)에만 둔다(비교보고서 11절, 합격기준 18번 항목이 200 Hz 통과를 요구하는 이유).
6. 실행 검증 순서: `ros2 launch upkie_bringup sim.launch.py balancer:=lqi6`(Plan 4) → `ros2 topic hz`, 텔레옵, rviz2 → `embedded:=false`로 ROS 토픽 경유 시 지연·지터가 지연 허용(Plan 4: 1 kHz 15 ms·200 Hz 20 ms, 합격 판정 15번 항목 참조)의 몇 %인지 측정. Plan 1~3 런치는 기존대로.
7. (선택) Isaac Sim 5.x로 `models/upkie_plan4/upkie.xml`(또는 upkie URDF) 교차 검증.
8. 실기 전: 설계보고서 §4.6 절차(파라미터 측정 → `wbr/models/upkie_params.py` 재생성 또는 `models/upkie_plan4/plan4_params.json` 교체 → 게인 자동 생성 → 매달기 테스트 → 저게인 착지). 실기에서 확정할 것: qdd100 연속 토크(합격기준 1절), 관절 건마찰 크기(마찰 보상 파라미터 `friction_comp_Nm`), 112 mm 타이어 사용 여부(100 mm면 발목 하우징이 7°에서 바닥에 닿음), 드라이버 피드백 타임아웃.

## 3. 우분투에서 Claude Code에 줄 첫 메시지(예시)

```
이 저장소의 CLAUDE.md와 wheeled_biped/docs/HANDOFF.md, docs/비교보고서.md(11절), docs/plan4_lqi6.md, docs/합격기준.md를 읽고,
HANDOFF.md 2절의 순서대로 우분투에서 ROS 2 Jazzy 빌드·실행 검증을 진행해줘. Plan 4(upkie + LQI6)부터.
```

## 4. 이 기록이 있는 곳

- GitHub: https://github.com/wisehyun-2/my_robot_project (브랜치 `main`).
- 대화 기록 요약본: `wheeled_biped/docs/session_log_2026-10-21.md` (사용자 발화 + Claude 서술만).
- 전체 원본(도구 출력 포함, 약 18 MB zip): Windows PC `C:\Users\User\Downloads\session-export-*.zip` — 필요하면 USB 등으로 옮겨 Claude에게 파일을 읽히면 된다(저장소에는 넣지 않았다).
- Windows의 Claude Code 메모리(`C:\Users\User\.claude\projects\C--Users-User-Desktop-myrobot\memory\`)의 요점은 저장소 루트 `CLAUDE.md`에 옮겨 적었다. 우분투에서는 경로가 달라 자동으로 로드되지 않는다.

## 5. Windows 쪽에서 알아둘 것

- Windows에는 ROS 2·WSL·Docker·MSVC가 없고, MuJoCo는 `C:\Users\User\Desktop\myrobot\.venv`(Python 3.14)에서 동작한다(Smart App Control이 새 DLL을 한 번 막았다가 허용함; `scipy.optimize`는 차단되어 쓰지 않음).
- 참조 저장소 클론은 `C:\Users\User\Desktop\myrobot\references\`에 있다(저장소 밖).
- 합격 판정 결과 캐시는 `results/acceptance/cache/<tag>/`에 있고 제어기 소스 해시로 키가 매겨진다(코드를 바꾸면 자동 무효화).
