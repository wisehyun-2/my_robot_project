# 관절·몸통 조립 그림과 영상 (바퀴 · 무릎 · hip · 몸통)

`WheelBiped_v2`(Fusion)에서 부품을 새 문서로 복사하고, 모델에 BOM으로만 있는 나사·와셔·너트·평행핀·인서트를
BOM 규격대로 만들어 분해 → 조립 단계별로 캡처한 그림과 영상이다. 바퀴는 `wheel/`(스크립트·설명은 `wheel/README.md`).
설명용 그림이며 제작 도면이 아니다. 다리는 왼쪽만(오른쪽은 대칭).

| 폴더 | 단계 | 영상 |
| --- | --- | --- |
| `wheel/` | 1 모터 → 정강이 포켓, 2 SUS 링 + ① M2.5 × 8 × 6, 3 인서트 → 허브 플랜지, 4 ③ 핀 + 허브 플랜지 + 와셔 + ② M3 × 14 × 6, 5 타이어 → 한 조각 림(작업대), 6 림 + 타이어 → 허브 플랜지(④ M3 × 14 + 와셔 × 6) — 10-31 개정 3 | `wheel_assembly.mp4`(10-31 개정 3) |
| `knee/` | 1 인서트 M2.5 × 3 → 허벅지, 2 모터 → 무릎 포켓, 3 SUS 링 + ① M2.5 × 8 × 6, 4 ③ 평행핀 + 정강이, 5 SUS 출력 링 + 와셔 + ② M3 × 16 × 6, 6 moteus → 컵(④ M2.5 × 6 × 4), 7 컵 → 허벅지(⑤ M2.5 × 18 × 3) | `knee_assembly.mp4` |
| `hip/` | 1 모터 → hip 판 포켓, 2 ① M2.5 × 6 (12.9) × 6, 3 moteus → 컵(④), 4 컵 → hip 판 Ø80 보스(⑤ M3 × 40 × 3), 5 ③ 평행핀 + 허벅지, 6 SUS 출력 링 + 와셔 + ② M3 × 16 × 6 | `hip_assembly.mp4` |
| `torso/` | 1 전장 데크(작업대), 2 척추 봉·레일 → 왼쪽 hip 유닛(① M5 × 12, ② M4 × 12 접시), 3 데크 → hip 판(③ M3 × 8), 4 오른쪽 hip 유닛, 5 트레이 + 배터리, 6 윗판(④ M4 × 25 + 나일론 너트), 7 셸 hip 부분(⑤ M3 × 6 버튼 × 12), 8 셸 앞·뒤 + 이음판(⑥ M3 × 8 + 너트 × 24) | `torso_assembly.mp4` |

각 폴더: `00_overview.png`(분해도), `NN_sK_a_before.png` / `NN_sK_b_after.png`(단계마다 조립 전·후, 빨간 화살표 =
이동 방향, 초록 라벨 = 방금 결합된 부품), `NN_done.png`(완성), 단면 2장, `all_steps.png`(한눈에 보기), 영상(1920 × 1080, 30 fps).
색은 부품 구분용(허벅지 파랑, 정강이 초록, 컵 회색, 데크 파랑, 트레이 주황 — 실제는 검정·회색 PETG).
배선·커넥터·벨크로 스트랩·pi3hat 고정 나사는 생략.
몸통 그림은 10-31 mjpower-ss를 mjbots r2 STEP으로 바꾼 뒤, 다시 셸·장식 정리(안장 모따기 없음, 드럼과 붙이는 장식 전부
제거 — 장식판 단계가 없어져 8단계) 뒤에 찍었고, 무릎·hip 그림도 허벅지 안쪽 면 1.5 mm 들임 뒤 다시 찍었다. 영상 3개(`*_assembly.mp4`)는 그 전 모델로
찍은 그대로다.

## 다시 만들기

Fusion이 켜져 있고 `WheelBiped_v2`가 열려 있을 때(문서는 바꾸지 않음, 저장하지 않는 새 문서를 연다), `scripts/`에서
`<job>` = `knee` · `hip` · `torso`:

```
python fusion_rpc.py f_ax_build.py 900 <job>     # 새 문서 + 부품 복사 + 체결 부품
python fusion_rpc.py f_ax_stills.py 1200 <job>   # 스틸 -> %TEMP%\wbv2_assembly\<job>\raw
python -X utf8 annotate.py %TEMP%\wbv2_assembly\<job>\raw ..\<job>
python fusion_rpc.py f_ax_video.py 2400 <job>    # 영상 프레임 -> %TEMP%\wbv2_assembly\<job>\frames
python -X utf8 compose_video.py %TEMP%\wbv2_assembly\<job> ..\<job> ..\<job>\<job>_assembly.mp4
```

`axlib.py` = 공통 엔진(부품 복사, BOM 체결 부품 형상, 분해 위치·회전, 카메라, 캡처). `job_<job>.py` = 부품 목록,
체결 부품 위치(모델의 `params` / `layout` / `legs` / `torso` / `shell` 구멍 좌표에서 계산), 분해 위치(`home`), 단계,
카메라, 단면, 라벨 문구. `compose_video.py`는 ffmpeg 경로(`FFMPEG`)를 쓴다.
