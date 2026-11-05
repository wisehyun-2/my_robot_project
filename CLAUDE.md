# my_robot_project — Claude Code 프로젝트 지침

(이 파일은 Claude Code가 이 저장소에서 자동으로 읽는다. 다른 PC·OS에서 작업을 이어갈 때의 맥락이다.)

## 답변 규칙
- 모든 답변·진행 상황·질문은 **한국어**로 쓴다. 코드·경로·명령어는 그대로 둔다.

## 저장소 구성
- `balancing_robot_design/` — 2륜 자가균형 로봇 설계보고서(`설계보고서.md`), 제어기 전수 비교(`제어기_비교보고서.md`), SIL 하네스(`sil/`, `sim/`, `sim6/`). **"내가 만든 제어기" = `sim6/controllers6.py`의 `MineLQI6`(LQI6, 보고서 §8).**
- `wheeled_biped/` — 6모터 휠 이족 로봇 3개 플랜의 구현·MuJoCo 검증·ROS 2 패키지. 자세한 인수인계는 **`wheeled_biped/docs/HANDOFF.md`** 를 먼저 읽을 것.
  - `wbr/` ROS 비의존 코어(시뮬·제어기·관측기·벤치마크·뷰어), `models/`(upkie_description, diablo_lite), `ros2_ws/src/`(Jazzy 패키지, 우분투에서 빌드), `docs/`(비교보고서, Opus 검토, 세션 기록), `results/`(벤치마크 표·그림·영상), `tests/`.
- `robot_cad/` — 실제 로봇 Fusion 모델(upkie 2.0 다리 + AKE50 hip·바퀴 / AKE60 무릎, Ø150 바퀴, 24 V 15 Ah 배터리, RPi 4B + pi3hat). **`robot_cad/README.md`를 먼저 읽을 것.** 모델은 `robot_cad/fusion/build_all.py`로 처음부터 다시 만들고 치수는 `fusion/params.py`에 있다. 내보낸 파일은 `export/`, 그림은 `images/`.
- `robot_cad_v3/` — v2(git `987a914`)에서 허벅지·정강이만 6061-T6 가공 셸 + 3 mm 덮개(닫힌 상자, 폭 30 mm)로 바꾼 모델(10.39 kg, 폭 510 mm). **`robot_cad_v3/README.md`를 먼저 읽을 것** — 두께 근거는 `calc/strength.md` 1·1a·1b절, 빌드는 `fusion/v3run.py`의 `build()`(빈 문서에서), 모터 재검증은 `python -m wbr.bench.ake_cad --cad ../robot_cad_v3/calc/mass_properties.json --out results/ake_cad_v3`.
- `robot_cad_v4/` — v3(git `5a550c1`)에서 허벅지·정강이만 **SLA 유사ABS 레진 한 덩어리**(닫힌 상자 벽 3.0 + 안쪽 와렌 트러스 리브 2.0 × 3, v2처럼 바깥으로 넓힘, v2 출력 관절부: SUS 클램프 링·접착 인서트)로 바꾼 모델. 2026-11-02 사용자: 링크의 리브 배선 채널·배선 창·배수 구멍을 모두 없애고(닫힌 칸 4개, 레진은 안 갇힌다고 가정) 모델 배선도 뺌 → 9.90 kg, 폭 521 mm, 허브 `WheelBiped_v4` 버전 2. **`robot_cad_v4/README.md`를 먼저 읽을 것** — 근거는 `calc/strength_v4.md`(`python calc/strength_v4.py`), 빌드는 `fusion/v4run.py`의 `build()`(**빈 새 문서에서**), 모터 재검증은 `--cad ../robot_cad_v4/calc/mass_properties.json --out results/ake_cad_v4`.
- `robot_cad_v5/` — v4(git `681ad00`)에서 **바퀴를 바깥쪽으로 돌리고** 허벅지·정강이를 **PETG 출력 갈래 다리**(시안 A 모놀리스, 2026-11-01 사용자 선택: 갈래 허벅지 + 61807 무릎 양쪽 지지, 접선 외형의 평평한 면, 구멍 없는 와렌 리브, 링크 배선 구멍 없음)로 바꾼 모델(배선 없음 — 사용자 11-01, 배선 경로는 git `5a01c18`). 11-01 안 A(정강이를 9.5 mm 띄워 넣는 조립). **11-02 사용자**: hip 캡·칸별 배출 구멍 없앰, 무릎 틈 1.5 → 4.0(바퀴 묶음 2.5 mm 바깥으로 — 타이어 중심 y 270, 폭 594, 정강이 평평한 30 mm), 링크 **PETG FDM**(알루미늄 CNC 안은 취소), hub 인서트를 6061 받침 링으로 끼워 고정(열압입 인서트 ⑥ 없앰), 무릎 캡 관 꼭 맞춤, 셸 뒤 세로 모서리 R30 라운드(v5만, `reshell()`) — 허브 `WheelBiped_v5` **버전 7**, 10.93 kg. **`robot_cad_v5/README.md`를 먼저 읽을 것** — v4 코드를 그대로 쓰고 `fusion/v5run.py`가 링크만 바꾼다(`build()` 또는 허브 파일에서 `relegs()` 뒤 **따로, 마지막에** `turn()`). 강도: `calc/strength_v5.md`(⑥·캡 통과; 정강이 1 mm 기준·hip ② 검토 하중 초과는 README 5절 결정 사항). 실제 단면도(Fusion에서 자른 단면 + 겹침·틈 검사): `manufacturing/explain/real_sections.py`. 모터 재검증(11-05): `v5run.save_mass_json()`(읽기 전용, 바깥쪽 타이어 중심 y 270) → `calc/mass_properties.json` → `--cad ../robot_cad_v5/calc/mass_properties.json --out results/ake_cad_v5`. `ake_cad`는 이때부터 좌우 치수(링크 무게중심 y·타이어 중심·MCU 트랙)도 CAD 값을 쓰고(`--lateral upkie`가 예전 방식), **요 PI·다리 길이 PD를 로봇 크기로 환산해** 쓴다(11-05 사용자 결정, `ake_cad.outer_loop_gains`; v5 1.60·3.21 / 1504·75, 짐 5 kg 1.70·3.40 / 2936·147; `--outer-gains upkie`가 예전 값, Plan 4 기본값은 그대로). upkie 게인에서는 짐 5 kg이 15° 내리막 정지에서 무릎이 뒤로 꺾였고(v4의 11-02 "회복" 판정도 같은 꺾임을 놓친 것), 환산 게인에서는 두 경우 28개 모두 회복한다 — README 6절. 무릎 꺾임은 이제 판정에 들어간다(`ake_sizing.knee_flipped`). 남은 결정: 무릎 완전 신장 멈춤 / MCU 관절 한계 보호, 실기 MCU 설정에 환산 게인 넣기.
- `robot_cad_v2/` — 재설계 Fusion 모델(AKE60-8 × 6, moteus r4.11 컵, PETG로 넓힌 링크, TITA형 몸통, 통짜 척추 봉·레일, 체결 부품 438개 형상 포함, 사용자 타이어 Xiaomi M365 8.5 × 2 + 한 조각 PETG 림, 10.54 kg). **`robot_cad_v2/README.md`를 먼저 읽을 것.** `fusion/v2run.py`의 `build()`로 처음부터 다시 만들고(구매품 STEP 경로는 `fusion/vendor.py`) 치수는 `fusion/params.py`, 모터 재검증은 `wheeled_biped/wbr/bench/ake_cad.py`.

## 지금까지의 결정(2026-10-21~22)
- 플랜: 1 = upkie 스택을 ROS 2 구조로 재작성, 2 = DIABLO 아키텍처 + LQI6(6 kg DIABLO-lite 5절 링크), 3 = DIABLO 아키텍처 + upkie 제어 코드. **최종 추천 = Plan 2**(근거: `wheeled_biped/docs/비교보고서.md` 0·4·7절).
- 로봇 질량 목표 약 6 kg, 바퀴 반경 0.065 m, 고관절 모터 12 Nm·바퀴 5 Nm 가정(설계보고서 RMD-L 계열).
- 시뮬레이터는 MuJoCo(Isaac Sim은 선택 사항). Windows에서는 ROS 2가 없어 ROS 2 패키지는 작성만 했고 **우분투 24.04 + ROS 2 Jazzy에서 빌드·실행 검증이 다음 단계**다.
- Opus 5.5(xhigh) 독립 검토(`wheeled_biped/docs/review_opus.md`)의 major 7건을 모두 반영했다(비교보고서 10절).
- **2026-10-22 최종 확정: Plan 4 = upkie 하드웨어(직렬 무릎 다리, 112 mm 타이어) + LQI6(직렬 다리용으로 플랜트 재유도) + DIABLO식 호스트/MCU 분리, 2단계로 호스트 MPC 기준 생성기.** 근거·결과는 비교보고서 11절, `docs/plan4_model.md`, `docs/plan4_lqi6.md`, `docs/plan4_mpc_host.md`. 합격 기준은 사용자가 정한 `docs/합격기준.md`이며 기준 수치·모터 한계·노이즈는 사용자 결정 없이 바꾸지 않는다.
- Plan 4 모델은 `models/upkie_plan4`(타이어 0.056 m). upkie_description의 0.05 m 타이어로는 발목 모터 하우징이 7° 기울기에서 바닥에 닿는다(실기도 112 mm 타이어 필수).
- 라이다·카메라·SLAM·로봇팔 2개는 실기 균형이 안정된 뒤 추가(팔 질량 피드포워드, 롤 보상, Jetson 호스트 필요).
- **2026-10-23 검토 1라운드 반영**(`wheeled_biped/docs/review_plan4_r1_response.md`): 바퀴 로터 아마추어·감쇠를 관절 속도에(플랜트 대조 오차 0.3 % 이하), MCU 센서 가드(NaN·정지 → FAULT)·IMU 관례 어댑터(시뮬이 pi3hat 좌표계를 흉내냄)·마찰 보상 피치 속도 게이트, 2단계는 기동 중 적분기 전부 동결 + 적분기 없는 예측 모델 + 비대칭 제동 한계(로봇 속도 기준) + 혁신 게이트. 판정 정의 변경은 `docs/합격기준.md` 6.2. 19번(정지 0.3 m·피치 2°)은 질량 ±15 %에서 여유가 몇 %뿐이다(2라운드 검토 뒤 넘김 시간 1.0 s·제동 계수 1.17로 통과). 2라운드 반영: 호스트 패킷·VMC 입력 가드, 20번 비교 단위는 사용자 결정. **3라운드(2026-10-23) 뒤 최종 판정 두 단계·두 주기 모두 합격**(캐시 `r5_s1`·`r5_s2`): 16·17은 사용자가 위임한 결정으로 드리프트 기준점 x(0)·한계 10 cm(합격기준 6.3, 12번과 같은 일정 외란 부류); 2단계는 명령 지연을 기준 입력 순수 지연으로, 정지 넘김은 텔레메트리 속도 조건으로. 해석 항목 네 가지(20번 비교 단위, 정지 거리 해석·측정점, 3·6번 포화 허용 해석, 15번 강건 여유의 기준 편입)는 2026-10-23 사용자가 "전부 현행 유지"로 확정(합격기준 6.4) — 남은 사용자 결정 항목 없음.

## 작업 규칙
- `wbr` 코어는 ROS에 의존하지 않게 유지한다(ROS 2 노드는 얇은 래퍼). 제어기·모델을 바꾸면 `python -m unittest discover -s tests`와 `python -m wbr.bench.run_bench`를 다시 돌려 `results/`와 보고서 표를 갱신한다.
- 커밋은 사용자가 요청할 때만 한다. 커밋 메시지 끝에 `Co-Authored-By: Claude <실제로 작업한 모델 이름> <noreply@anthropic.com>` 를 붙인다(예: `Claude Opus 5.5`). 모델 이름은 그 세션에서 실제로 작업한 모델을 쓴다.
- 참조 저장소(upkie, upkie_description, mjlab_upkie, diablo_ros2, diablo-sim-env, DiabloSTLSDF, Col_DIABLO_Issac_RL, diablo_sdk)는 저장소 밖(`../references/`)에 클론해 쓴다. 없으면 다시 클론한다.
