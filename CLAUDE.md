# my_robot_project — Claude Code 규칙

모든 세션을 시작하기 전에 반드시 claude.md를 읽는다.

프로젝트 개요, 지금 하는 일, 저장소 구성, 컴퓨터별 사정은 루트 `README.md`에 있다. 작업의 맥락이 필요할 때, 그것과 해당 폴더의 README를 먼저 읽는다.

## instructions
- 모든 답변·진행 상황·질문은 **한국어**로 쓴다. 코드·경로·명령어는 그대로 둔다.
- **my_robot_project에서 ROS2 개발을 포함한 모든 상황에서 답변은 기억이나 일반 지식으로 답하지 않는다. 반드시 소스코드나 객관적인 자료를 바탕으로 답변한다.**
- 사실을 판정할 수 없을 경우나 모호한 경우에는 사용자에게 얘기한다.
- 라이브러리·펌웨어·소스코드에 대한 언급이 나올 경우 `references/`의 원본 소스를 찾아본 후 답변한다. 

## git
- 작업을 시작하기 전에 `git pull`로 다른 컴퓨터의 변경을 받는다. 브랜치는 `ros2-v5` 하나다.
- **커밋·푸시는 사용자가 요청할 때만 한다.**
  - 올리지 않은 변경을 남긴 채 끝낼 때는 그렇다고 알린다. 다른 컴퓨터로 넘어가기 전에 올려야 한다.
  - 커밋 메시지 끝에 `Co-Authored-By: Claude <실제로 작업한 모델 이름> <noreply@anthropic.com>`를 붙인다(예: `Claude Opus 5.5`).
- 부분 클론(`git config remote.origin.partialclonefilter`가 `blob:none`)에서는 이력 전체의 내용을 훑는 명령을 쓰지 않는다. `.git`이 수백 MB 커진다.
  - 쓰지 않는 명령: `git log -S`, `git log -p`, `git rev-list --objects --all` 등

## 기록
- Claude Code 메모는 컴퓨터마다 따로다. 다른 컴퓨터가 알아야 할 결정은 해당 README에 날짜와 함께 적는다.
- 이 파일에는 규칙만 적는다. 사실과 진행 상황은 README에 둔다.

## 코드와 시험
- `wbr` 코어는 ROS에 의존하지 않게 유지한다.
- 제어기·모델을 바꾸면 다음을 한다(`wheeled_biped/`에서).
  - `python -m unittest discover -s tests`를 돌린다.
  - 단계 시험을 돌린다.
  - 보고서(`v5_lqi/TEST/results/stage_report.md`)를 갱신한다.
- **사용자 결정 없이 바꾸지 않는다**
  - 합격 기준 수치, 모터 한계, 노이즈(`docs/합격기준.md`, 단계 시험 기준)
- **v5가 쓰지 않는 코드를 다시 들이지 않는다**(2026-11-14 사용자). 다음은 모두 지웠다.
  - upkie 모델
  - v5 기능을 끄는 스위치
  - Plan 1~4 전용 코드
- 로봇 폴더는 `robot_cad_v5/`와 `wheeled_biped/`만 둔다(2026-11-14 사용자).
- `robot_cad_v5/v4/` 아래 v4 파일은 고치지 않는다(v5가 불러 쓴다).

## 참조 저장소
- `references/` 아래 git 서브모듈로 둔다. 목록·커밋·받는 법·라이선스 규칙은 `references/README.md`에 있다.
- 받을 때는 `git submodule update --init`(전부) 또는 경로를 지정해 일부만 받는다.
- 서브모듈 안의 파일은 고치지 않는다.
