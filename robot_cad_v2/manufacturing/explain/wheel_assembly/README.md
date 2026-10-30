# 바퀴 출력부 조립 그림·영상 (왼쪽 다리)

`WheelBiped_v2`(Fusion)에서 바퀴 관절 부품을 새 문서로 복사하고, 모델에 BOM으로만 있는 나사·와셔·평행핀·인서트를
BOM 규격대로 만들어 분해 → 조립 단계별로 캡처한 그림과 영상이다. 설명용 그림이며 제작 도면이 아니다.

| 파일 | 내용 |
| --- | --- |
| `00_overview.png` | 모든 부품이 떨어진 상태 (분해도) |
| `01_s1_*` … `07_s6_*` | 1–7단계, 단계마다 `a_before`(조립 전, 빨간 화살표 = 이동 방향) / `b_after`(조립 후) |
| `08_done.png` | 조립 완료 |
| `09_cut1.png` | 단면: ② M3 × 14(-5°) · ④ M3 × 8(90°) 자리, ⑤ M5 볼트, A/B/C 중심 맞춤 |
| `10_cut2.png` | 단면: ③ 평행핀 · ① M2.5 × 8 + SUS 링 (25°, 145°) 자리 |
| `wheel_assembly.mp4` | 체결 영상 1920 × 1080, 30 fps (부품이 축을 따라 들어가고 나사는 돌면서 조여짐) — **10-30 첫 판 그대로**(아래) |

조립 순서: 1 모터 → 정강이 포켓, 2 SUS 링 + ① M2.5 × 8 × 6, 3 인서트 → 허브 플랜지(작업대),
4 ③ 핀 + 허브 플랜지 + 와셔 + ② M3 × 14 × 6 → 출력부, 5 ⑤ M5 × 50 → 육각 허브(작업대),
6 바퀴 + ⑥ 너트(작업대), 7 육각 허브 + 바퀴 → 허브 플랜지, ④ M3 × 8 × 6.

**10-30 수정**: 바퀴 출력 나사를 M3 × 16 → **M3 × 14 (12.9)**로 바꾸고 허브 플랜지의 와셔 자리를 출력면에서 11.2 → 9.5 mm로 옮겼다
(M3 × 16 머리가 육각 허브의 Ø20 등록 턱과 1.2 mm 겹쳤다; 지금은 0.5 mm 떨어지고 출력 탭 물림 4.0 mm). 그림은 이 형상으로 다시
만들었고, 영상은 요청에 따라 첫 판(M3 × 16)을 그대로 두었다.

색은 부품 구분용이다(실제 허브·정강이는 검정 PETG). 정강이는 바퀴 쪽 아래(z ≤ 132 mm)만, moteus 드라이버 컵은 생략.

## 다시 만들기

Fusion이 켜져 있고 `WheelBiped_v2`가 활성 문서일 때(문서는 바꾸지 않음, 저장하지 않는 새 문서를 하나 연다):

```
cd scripts
python fusion_rpc.py f_wx_build.py 600      # 새 문서 + 부품 복사 + 체결 부품
python fusion_rpc.py f_wx_stills.py 900     # 스틸 캡처 -> %TEMP%\wbv2_wheel_assembly\raw
python -X utf8 annotate.py %TEMP%\wbv2_wheel_assembly\raw ..
python fusion_rpc.py f_wx_video.py 1800     # 영상 프레임 -> %TEMP%\wbv2_wheel_assembly\frames
python -X utf8 compose_video.py %TEMP%\wbv2_wheel_assembly .. ..\wheel_assembly.mp4
```

분해 위치·단계·카메라 방향은 `scripts/wxlib.py`(`HOME`, `STEPS`, `CUTS`), 나사 치수는 `f_wx_build.py`.
`compose_video.py`는 ffmpeg 경로(`FFMPEG`)를 쓴다.
