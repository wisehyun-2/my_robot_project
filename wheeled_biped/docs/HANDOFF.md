# 인수인계: Windows 세션 → 우분투(듀얼부팅)에서 이어가기

작성 2026-10-22. 이 문서는 Windows PC의 Claude Code 세션에서 한 일과, 우분투에서 Claude Code로 이어갈 때 필요한 것을 정리한 것이다. 대화 기록 요약본은 `docs/session_log_2026-10-21.md`, 기술 결론은 `docs/비교보고서.md`, 독립 검토는 `docs/review_opus.md`에 있다.

## 1. 한 일 (상태: 완료, 커밋 `30015d1` 이후)

| 항목 | 상태 | 위치 |
|---|---|---|
| ROS 비의존 코어 `wbr`(MuJoCo 백엔드, upkie 관측기·PI·MPC 포팅, LQI6+5절 링크 VMC, DIABLO MCU/호스트 링크 에뮬레이션, 벤치마크, 라이브 뷰어, 영상) | 완료, Windows에서 검증 | `wbr/` |
| 로봇 모델: upkie(URDF→MJCF), DIABLO-lite 6 kg(MJCF+URDF 생성기) | 완료 | `models/`, `wbr/models/` |
| Plan 1/2/3 조립과 벤치마크(노이즈·지연 포함) | 완료, 추천 = Plan 2 | `results/bench/summary.md`, `results/figures`, `results/videos` |
| 단위 테스트 16개 | 통과 | `tests/test_core.py` |
| ROS 2 Jazzy 패키지(메시지 3종, upkie 4패키지, diablo_lite 4패키지, README) | **작성만 됨, 빌드·실행 미검증** | `ros2_ws/src`, `ros2_ws/README.md` |
| Opus xhigh 검토 반영(major 7건 등) | 완료 | `docs/review_opus.md`, 비교보고서 10절 |

## 2. 우분투에서 할 일 (순서대로)

1. 환경: Ubuntu 24.04 + ROS 2 Jazzy desktop, `git clone https://github.com/wisehyun-2/my_robot_project`, NVIDIA 드라이버(RTX 5080은 570 이상, 오픈 커널 모듈 권장).
2. 파이썬: `ros2_ws/README.md` 1절의 (a) 방식 — `python3 -m venv --system-site-packages ~/wbr_venv && source ~/wbr_venv/bin/activate && pip install -e wheeled_biped`.
3. 코어 확인(ROS 없이): `cd wheeled_biped && python -m wbr.models.build_upkie && python -m wbr.models.build_diablo_lite && python -m unittest discover -s tests && python -m wbr.viewer.live --plan 2`.
4. ROS 2 빌드: `cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install && source install/setup.bash`. 빌드 오류는 거의 확실히 몇 개 있을 것(Windows에서 한 번도 빌드하지 못함). 예상 지점: `upkie_description` 래퍼 CMake의 상대 경로, xacro 처리, `setup.py`의 data_files, rosidl 의존성.
5. 실행 검증 순서: `ros2 launch diablo_lite_bringup sim.launch.py balancer:=lqi6`(Plan 2) → `ros2 topic hz /diablo/sensor/Imu`, `ros2 run diablo_lite_ctrl teleop_node`(DIABLO 키맵) / `/cmd_vel` 브리지, rviz2. 그다음 `ros2 launch upkie_bringup sim.launch.py`(Plan 1 내장 균형기) → `embedded:=false balancer:=pid|mpc`로 ROS 토픽 경유 시 지연·지터가 지연 허용(PI 40 ms, LQI6 30 ms)의 몇 %인지 측정(비교보고서 8절 3번, 검토 C8 참고). Plan 3은 `balancer:=upkie_pid|upkie_mpc`.
6. (선택) Isaac Sim 5.x로 `models/diablo_lite/diablo_lite.urdf` 교차 검증. 5절 링크 폐루프는 Isaac에서 별도 처리 필요.
7. 실기 전: 설계보고서 §4.6 절차(파라미터 측정 → `wbr/controllers/lqi6/plant6.py`의 `diablo_lite_params` 교체 → 게인 자동 생성 → 매달기 테스트 → 저게인 착지). 검토 미반영 항목(B12 플랜트 단순화, B14 leg_split/jump 미지원, C9·C11·C12)도 그때 처리.

## 3. 우분투에서 Claude Code에 줄 첫 메시지(예시)

```
이 저장소의 CLAUDE.md와 wheeled_biped/docs/HANDOFF.md, docs/비교보고서.md, docs/review_opus.md를 읽고,
HANDOFF.md 2절의 순서대로 우분투에서 ROS 2 Jazzy 빌드·실행 검증을 진행해줘. Plan 2(diablo_lite lqi6)부터.
```

## 4. 이 기록이 있는 곳

- GitHub: https://github.com/wisehyun-2/my_robot_project (브랜치 `main`).
- 대화 기록 요약본: `wheeled_biped/docs/session_log_2026-10-21.md` (사용자 발화 + Claude 서술만).
- 전체 원본(도구 출력 포함, 약 18 MB zip): Windows PC `C:\Users\User\Downloads\session-export-*.zip` — 필요하면 USB 등으로 옮겨 Claude에게 파일을 읽히면 된다(저장소에는 넣지 않았다).
- Windows의 Claude Code 메모리(`C:\Users\User\.claude\projects\C--Users-User-Desktop-myrobot\memory\`)의 요점은 저장소 루트 `CLAUDE.md`에 옮겨 적었다. 우분투에서는 경로가 달라 자동으로 로드되지 않는다.

## 5. Windows 쪽에서 알아둘 것

- Windows에는 ROS 2·WSL·Docker·MSVC가 없고, MuJoCo는 `C:\Users\User\Desktop\myrobot\.venv`(Python 3.14)에서 동작한다(Smart App Control이 새 DLL을 한 번 막았다가 허용함).
- 참조 저장소 클론은 `C:\Users\User\Desktop\myrobot\references\`에 있다(저장소 밖).
