# references — 참조 저장소 (git 서브모듈)

2026-11-14 사용자: 여러 컴퓨터(Windows PC, Ubuntu 노트북, Pi 4)가 **같은 커밋의 원본 소스**를 보도록, 참조 저장소를 이 폴더 아래 git 서브모듈로 둔다. 예전에는 저장소 밖 `../references/`에 각자 클론했지만 지금은 쓰지 않는다.

- 서브모듈은 아래 표의 커밋에 **고정**되어 있다. 문서에 적힌 `파일:줄`은 이 커밋 기준이다.
- 서브모듈 안의 파일은 **고치지 않는다**(읽기 전용).
- 라이선스 규칙(코드를 가져다 쓸 때)은 맨 아래와 `wheeled_biped/ros2_ws/README.md` 1절 (d)를 따른다.

## 받는 법

새로 클론한 저장소나 `git pull` 뒤에 서브모듈은 비어 있다. 필요한 것만 받는다(`.gitmodules`에 `shallow = true`라서 최신 커밋 하나만 받는다).

```bash
# 저장소 맨 위에서. 전부(약 300 MB)
git submodule update --init

# 일부만(예: Pi에서 플러그인 작업에 필요한 것만)
git submodule update --init references/pi3hat references/moteus
```

Windows에서는 줄바꿈 변환 때문에 바이너리 파일이 "수정됨"으로 보일 수 있으므로 이렇게 받는다.

```bash
git -c core.autocrlf=false submodule update --init
```

커밋을 바꿀 때(다른 커밋의 소스를 기준으로 삼을 때):

```bash
git -C references/<이름> fetch --depth 1 origin <커밋>
git -C references/<이름> checkout <커밋>
git add references/<이름>        # 그다음 이 표와, 그 저장소의 줄 번호를 인용한 문서를 함께 고쳐 커밋
```

## 목록

크기는 받은 뒤의 파일 크기(얕은 클론)다. 라이선스는 각 저장소 **최상위 LICENSE 파일** 기준이다.

| 폴더 | 원격 | 고정 커밋 | 라이선스 | 쓰는 곳 |
|---|---|---|---|---|
| `pi3hat` | `github.com/mjbots/pi3hat` | `12ab9af` (2026-09-09, v1.1.0) | Apache-2.0 | 하드웨어 플러그인(`lib/cpp/mjbots/pi3hat`), `pi3hat_tool`, 펌웨어(`fw/`), 보드 문서(`docs/reference.md`) |
| `moteus` | `github.com/mjbots/moteus` | `aec0333` (2026-11-03) | Apache-2.0 | moteus 프로토콜과 통신 계층(`lib/cpp/mjbots/moteus`: `Transport`, `Fdcanusb`, `Socketcan`) |
| `power_dist` | `github.com/mjbots/power_dist` | `5936cba` (2025-12-16) | Apache-2.0 | power dist r4.5b(JC3) 문서·펌웨어 |
| `fdcanusb` | `github.com/mjbots/fdcanusb` | `91c2b7f` (2026-10-06) | Apache-2.0 | mjcanfd-usb-1x 문서, udev 규칙 `70-fdcanusb.rules` |
| `moteus_ros2` | `github.com/mjbots/moteus_ros2` | `3d0f85b` (2024-11-29) | Apache-2.0 | 토픽 방식 moteus 노드. 참고·저속 시험용(그 README: 100 Hz 넘게는 라이브러리를 직접 쓰라) |
| `upkie` | `github.com/upkie/upkie` | `ffa9e4d` (2026-10-11) | Apache-2.0 | pi3hat IMU 변환, 스파인 구조 참고 |
| `ddt_ros2_control` | `github.com/DDTRobot/ddt_ros2_control` | `beb05df` (2026-10-30) | 최상위 파일 없음 | 제어기 골격(`rl_controller`), 키보드, 토픽 이름 |
| `TITA_ROS2_Control_Sim` | `github.com/DDTRobot/TITA_ROS2_Control_Sim` | `3c782e4` (2024-10-31) | 최상위 파일 없음 | 시뮬 bringup, `template_ros2_controller` |
| `TITA-Development-Manual` | `github.com/DDTRobot/TITA-Development-Manual` | `aba3583` (2025-12-15) | MIT(README 배지는 Apache-2.0) | TITA 개발 매뉴얼(TITA 구조 조사) |
| `tita_hardware_ros2_control` | `github.com/DDTRobot/tita_hardware_ros2_control` | `8c7256c` (2024-11-06) | 최상위 파일 없음 | 실기 launch·`controllers.yaml` 틀, `read()`·`write()`에서 통신을 부르는 자리 |
| `diablo_ros2` | `github.com/DDTRobot/diablo_ros2` | `ed0de02` (2024-06-29) | Apache-2.0 | 우리 MCU 구조(`wbr/controllers/diablo_arch`)가 흉내 낸 DIABLO 원본 |
| `diablo_sdk` | `github.com/DDTRobot/diablo_sdk` | `cf2c2a9` (2024-01-12) | Apache-2.0 | DIABLO 호스트 링크(OSDK) 원본(`host_link.py`) |
| `diablo-sdk-docs` | `github.com/DDTRobot/diablo-sdk-docs` | `6b73c5b` (2023-01-07) | **GPL-3.0** | DIABLO SDK 문서 |
| `pi3hat_hw_bartlomiej` | `github.com/BartlomiejK2/pi3hat_hardware_interface` | `04a3a29` (2026-06-10) | **GPL-3.0** | 우리 플러그인과 가장 가까운 예(moteus 전용). **읽기만** |
| `pi3hat_robotic_systems` | `github.com/CentroEPiaggio/pi3hat_robotic_systems` | `168a9b4` (2026-03-31) | 최상위 파일 없음 | moteus + power dist를 읽는 예. **읽기만** |
| `pi3hat_hw_glevine` | `github.com/G-Levine/pi3hat_hardware_interface` | `da07dcb` (2024-02-21) | MIT | root에서 FastDDS 공유 메모리 문제와 UDP 프로필 |
| `ros2_control_demos` | `github.com/ros-controls/ros2_control_demos` (브랜치 `jazzy`) | `5b8777c` (2026-11-10) | Apache-2.0 | Jazzy 방식 하드웨어·제어기 공식 예제 |

`pi3hat` ~ `diablo-sdk-docs`의 커밋은 11-14 이전부터 Windows PC에서 읽던 사본과 같다. 아래 다섯 개(`tita_hardware_ros2_control`, `pi3hat_hw_*`, `pi3hat_robotic_systems`, `ros2_control_demos`)는 11-14에 그때의 최신 커밋으로 처음 고정했다. 노트북의 예전 사본(`~/references`)과 커밋이 다를 수 있다.

## 라이선스 규칙

`wheeled_biped/ros2_ws/README.md` 1절 (d)의 규칙과 같다.

- **GPL-3.0과 최상위 라이선스 파일이 없는 저장소는 읽고 구조만 참고한다. 코드를 복사하지 않는다.**
- 복사해도 되는 것은 MIT와 Apache-2.0이다. 이때 출처를 남긴다.
- DDT 저장소는 그 파일의 머리말과 그 패키지의 `package.xml`이 둘 다 Apache-2.0일 때만 복사한다.
- 서브모듈은 원격 주소와 커밋만 기록하므로, 이 저장소가 남의 코드를 다시 배포하지는 않는다.
