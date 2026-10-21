# my_robot_project — Claude Code 프로젝트 지침

(이 파일은 Claude Code가 이 저장소에서 자동으로 읽는다. 다른 PC·OS에서 작업을 이어갈 때의 맥락이다.)

## 답변 규칙
- 모든 답변·진행 상황·질문은 **한국어**로 쓴다. 코드·경로·명령어는 그대로 둔다.

## 저장소 구성
- `balancing_robot_design/` — 2륜 자가균형 로봇 설계보고서(`설계보고서.md`), 제어기 전수 비교(`제어기_비교보고서.md`), SIL 하네스(`sil/`, `sim/`, `sim6/`). **"내가 만든 제어기" = `sim6/controllers6.py`의 `MineLQI6`(LQI6, 보고서 §8).**
- `wheeled_biped/` — 6모터 휠 이족 로봇 3개 플랜의 구현·MuJoCo 검증·ROS 2 패키지. 자세한 인수인계는 **`wheeled_biped/docs/HANDOFF.md`** 를 먼저 읽을 것.
  - `wbr/` ROS 비의존 코어(시뮬·제어기·관측기·벤치마크·뷰어), `models/`(upkie_description, diablo_lite), `ros2_ws/src/`(Jazzy 패키지, 우분투에서 빌드), `docs/`(비교보고서, Opus 검토, 세션 기록), `results/`(벤치마크 표·그림·영상), `tests/`.

## 지금까지의 결정(2026-10-21~22)
- 플랜: 1 = upkie 스택을 ROS 2 구조로 재작성, 2 = DIABLO 아키텍처 + LQI6(6 kg DIABLO-lite 5절 링크), 3 = DIABLO 아키텍처 + upkie 제어 코드. **최종 추천 = Plan 2**(근거: `wheeled_biped/docs/비교보고서.md` 0·4·7절).
- 로봇 질량 목표 약 6 kg, 바퀴 반경 0.065 m, 고관절 모터 12 Nm·바퀴 5 Nm 가정(설계보고서 RMD-L 계열).
- 시뮬레이터는 MuJoCo(Isaac Sim은 선택 사항). Windows에서는 ROS 2가 없어 ROS 2 패키지는 작성만 했고 **우분투 24.04 + ROS 2 Jazzy에서 빌드·실행 검증이 다음 단계**다.
- Opus 5.5(xhigh) 독립 검토(`wheeled_biped/docs/review_opus.md`)의 major 7건을 모두 반영했다(비교보고서 10절).

## 작업 규칙
- `wbr` 코어는 ROS에 의존하지 않게 유지한다(ROS 2 노드는 얇은 래퍼). 제어기·모델을 바꾸면 `python -m unittest discover -s tests`와 `python -m wbr.bench.run_bench`를 다시 돌려 `results/`와 보고서 표를 갱신한다.
- 커밋은 사용자가 요청할 때만 한다. 커밋 메시지 끝에 `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` 를 붙인다.
- 참조 저장소(upkie, upkie_description, mjlab_upkie, diablo_ros2, diablo-sim-env, DiabloSTLSDF, Col_DIABLO_Issac_RL, diablo_sdk)는 저장소 밖(`../references/`)에 클론해 쓴다. 없으면 다시 클론한다.
