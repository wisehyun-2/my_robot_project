# v3 바퀴 출력부 조립 그림·영상 (왼쪽 다리)

`WheelBiped_v3`(Fusion)에서 바퀴 관절 부품을 새 문서로 복사하고, 모델에 BOM으로만 있는 나사·와셔·평행핀·인서트를
BOM 규격대로 만들어 분해 → 조립 단계별로 캡처한 그림과 영상이다. 설명용 그림이며 제작 도면이 아니다.
양식과 스크립트는 `../../../../../robot_cad_v2/manufacturing/explain/assembly/wheel/`(v2)와 같다.

| 파일 | 내용 |
| --- | --- |
| `00_overview.png` | 모든 부품이 떨어진 상태 (분해도) |
| `01_s1_*` … `06_s5_*` | 1–6단계, 단계마다 `a_before`(조립 전, 빨간 화살표 = 이동 방향) / `b_after`(조립 후) |
| `07_done.png` | 조립 완료 |
| `08_cut1.png` | 단면: ② M3 × 14(-5°) · ④ M3 × 14(90°) 자리, 림 돌기 ↔ 타이어 구멍, A/B/C 중심 맞춤 |
| `09_cut2.png` | 단면: ③ 평행핀 · ① M2.5 × 8 (25°, 145°) 자리 — 머리가 6061 판의 카운터보어에 바로 앉음 |
| `all_steps.png` | 위 그림을 한 장에 (`scripts/make_all_steps.py`) |
| `wheel_assembly.mp4` | 체결 영상 1920 × 1080, 30 fps, 50 초 (부품이 축을 따라 들어가고 나사는 돌면서 조여짐) |

조립 순서: 1 모터 → 정강이 하우징(Ø42 H7 포켓 깊이 1.5), 2 ① M2.5 × 8 저두 × 6 → Ø4.8 × 1.7 카운터보어 → 고정자 탭(물림 3.2),
3 인서트 → 허브 플랜지(작업대), 4 ③ 핀 + 허브 플랜지 + 와셔 + ② M3 × 14 (12.9) × 6 → 출력부, 5 타이어 → 한 조각 림(작업대),
6 림 + 타이어 → 허브 플랜지(Ø11 등록 턱 → 림 Ø11.2 홈), 와셔 + ④ M3 × 14 (8.8) × 6(와셔 자리 9 mm, 0.5 N·m).

**v2와 다른 점**: 정강이가 6061 셸(8 mm 모터 자리 판 + r 41.5 ~ 44 하우징 벽 + 3 mm 덮개 상자)이라 v2 2단계의 SUS304 클램프 링이
없다 — M2.5 머리 면압(검토 100 MPa)이 6061 허용치(약 260 MPa) 안이다(`calc/strength.md` 1b · 2절). 나사 길이·머리 위치·물림은
v2와 같다. 허브 플랜지·림·타이어·체결 부품 ②③④는 v2 부품 그대로다.

그림·영상은 허브 플랜지 스피곳 1.4 mm 때 모델로 만들었다. 10-31에 1.3 mm(출력부 홈 바닥 틈 0.2)로 바꿨지만 0.1 mm 차이라
그림에서는 보이지 않아 다시 만들지 않았다 — 실제 비율 단면도 `../../wheel_output_stack.png`는 새 값이다.

색은 부품 구분용이다(허브 플랜지는 검정 PETG, 정강이는 6061 아노다이징 권장). 정강이는 바퀴 쪽 아래(z ≤ 132 mm)만 — 그 높이에서
잘린 상자 단면(바깥 판 + 덮개 사이 빈 공간)이 보인다. moteus 드라이버 컵은 생략.

## 다시 만들기

Fusion이 켜져 있고 `WheelBiped_v3`가 활성 문서일 때(문서는 바꾸지 않음, 저장하지 않는 새 문서를 하나 연다). v3 문서에서
림·타이어 등이 숨겨져 있어도 된다 — 빌드는 몸체 전구만 본다.

```
cd scripts
python fusion_rpc.py f_wx_build.py 600      # 새 문서 + 부품 복사 + 체결 부품
python fusion_rpc.py f_wx_stills.py 900     # 스틸 캡처 -> %TEMP%\wbv3_wheel_assembly\raw
python -X utf8 annotate.py %TEMP%\wbv3_wheel_assembly\raw ..
python -X utf8 make_all_steps.py ..
python fusion_rpc.py f_wx_video.py 1800     # 영상 프레임 -> %TEMP%\wbv3_wheel_assembly\frames
python -X utf8 compose_video.py %TEMP%\wbv3_wheel_assembly .. ..\wheel_assembly.mp4
```

분해 위치·단계·카메라 방향은 `scripts/wxlib.py`(`HOME`, `STEPS`, `CUTS`), 나사 치수는 `f_wx_build.py`.
`compose_video.py`는 ffmpeg 경로(`FFMPEG`)를 쓴다. 영상 캡처 중에는 Fusion 창 위에 마우스를 두지 않는다.

단면도(실제 비율, 나사·핀·하중)는 `../../wheel_output_stack.png` — `python wheel_output_stack.py`(matplotlib, 저장소 `.venv`).
