# 02. 로봇 모델과 Gazebo (저장소 Section4)

> 참조: `Section4_Locomotion/bumperbot_ws/src/bumperbot_description/` (참조 저장소 최상위 기준 경로)

## 이 장에서 하는 일

- 코드가 없는 "자료 패키지" `bumperbot_description`을 만든다. 로봇의 모양(URDF/xacro), 메시, RViz 설정, launch 파일이 들어 있다.
- 로봇 모델을 링크와 조인트로 한 부분씩 써서 `xacro`와 `check_urdf`로 확인한다.
- RViz로 모델을 보는 `display.launch.py`와 Gazebo에 로봇을 스폰하는 `gazebo.launch.py`를 쓴다.
- ros2_control 전체에서의 자리: 제어기와 하드웨어가 다루는 대상(바퀴 조인트 두 개)이 여기서 정해진다. 다음 장(03)에서 이 URDF에 `<ros2_control>` 태그를 더한다. 이 장의 Gazebo 로봇은 아직 움직이지 않는다.

## 이번 장의 파일

```text
~/bumperbot_ws/src/bumperbot_description/
├── CMakeLists.txt                  [새]
├── LICENSE                         [새] (ros2 pkg create가 만든다)
├── package.xml                     [새]
├── meshes/                         [새] 참조 저장소에서 복사 (STL 6개)
├── urdf/
│   ├── bumperbot.urdf.xacro        [새] 로봇 모델 본체
│   └── bumperbot_gazebo.xacro      [새] Gazebo 전용 속성(마찰 등)
├── launch/
│   ├── display.launch.py           [새] RViz로 모델 보기
│   └── gazebo.launch.py            [새] Gazebo에 스폰
└── rviz/
    └── display.rviz                [새] RViz 설정 (참조 파일 복사)
```

이 장은 `bumperbot_description` 패키지 하나만 다룬다. `bumperbot_controller`는 03장에서 생긴다. 그래서 이 장 시점의 xacro에는 그 패키지를 가리키는 `$(find bumperbot_controller)`가 없다.

## 단계 1. `bumperbot_description` 패키지 뼈대

### 왜 필요한가

colcon은 `package.xml`이 있는 폴더를 패키지로 인식하고, `package.xml`의 `<build_type>`을 보고 빌드 도구를 고른다. 이 패키지는 C++ 코드가 없으므로 `ament_cmake`가 하는 일은 하나다: 파일들을 `install/<패키지>/share/<패키지>/` 아래로 옮겨(설치해) 두는 것이다.

설치가 필요한 이유는 ROS 도구들이 파일을 설치 위치에서 찾기 때문이다.

- `get_package_share_directory("bumperbot_description")`(launch 파일에서 쓴다)은 `install/bumperbot_description/share/bumperbot_description`을 돌려준다.
- `xacro`의 `$(find bumperbot_description)`도 같은 곳을 본다.
- URDF의 `package://bumperbot_description/meshes/...` 주소는 이 `share` 폴더를 기준으로 풀린다.

소스 폴더에 파일이 있어도 `share`에 설치되지 않으면 위 세 가지가 모두 실패한다.

### 코드

`~/bumperbot_ws/src`는 01장에서 만든 작업 공간이다. 패키지 뼈대를 만든다. 옵션의 뜻은 01장과 같다(`Your Name`, `you@example.com`은 본인 것으로 바꿔도 된다).

```bash
cd ~/bumperbot_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 \
  --description "Bumperbot Description package" \
  --maintainer-name "Your Name" --maintainer-email "you@example.com" \
  bumperbot_description
cd bumperbot_description
rm -r include src
mkdir -p launch urdf rviz meshes
```

`ros2 pkg create`는 `package.xml`, `CMakeLists.txt`, `LICENSE`(Apache-2.0 전문)와 빈 `include/bumperbot_description/`, `src/` 폴더를 만든다. 이 패키지는 C++ 소스가 없으므로 `include`와 `src`는 지우고(`rm -r`), 설치할 폴더 네 개(`launch`, `urdf`, `rviz`, `meshes`)를 만든다(`mkdir -p`는 폴더가 이미 있어도 오류를 내지 않는다). 이 폴더들이 없으면 아래 `CMakeLists.txt`의 `install(DIRECTORY ...)`가 빌드 중에 실패한다("자주 나는 오류" 참고).

`CMakeLists.txt`는 아래 내용으로 바꾼다. `package.xml`은 위 옵션으로 만든 것과 같다(옵션을 다르게 줬다면 아래 내용으로 바꾼다).

> 파일: `bumperbot_description/CMakeLists.txt` — 전체 (단계 1 시점)
```cmake
cmake_minimum_required(VERSION 3.8)
project(bumperbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

> 파일: `bumperbot_description/package.xml` — 전체 (단계 1 시점)
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bumperbot_description</name>
  <version>0.0.0</version>
  <description>Bumperbot Description package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 코드 설명

`CMakeLists.txt`:

| 줄 | 뜻 |
|---|---|
| `cmake_minimum_required(VERSION 3.8)` | 이 파일이 요구하는 CMake 최소 버전. `ros2 pkg create`가 만든 값 그대로다(참조는 `3.5`). |
| `project(bumperbot_description)` | CMake 프로젝트 이름. `package.xml`의 `<name>`과 같아야 한다. 아래 `${PROJECT_NAME}`이 이 값이다. |
| `find_package(ament_cmake REQUIRED)` | ament의 CMake 함수(`ament_package`, ament용 install 도우미)를 불러온다. 없으면 빌드가 멈춘다. |
| `install(DIRECTORY launch meshes urdf rviz DESTINATION share/${PROJECT_NAME})` | 네 폴더를 통째로 `install/bumperbot_description/share/bumperbot_description/` 아래로 설치한다. 이 패키지의 핵심 줄이다. |
| `if(BUILD_TESTING) ... endif()` | `BUILD_TESTING` 옵션이 켜져 있을 때만 린터(코드 스타일 검사)를 불러온다. 이 옵션의 기본값은 **켜짐**이라 보통의 `colcon build`에서도 이 블록이 실행된다(01장에서 `CMakeCache.txt`로 확인했다). 이 블록은 린터 테스트를 **등록**할 뿐 린터는 `colcon test`에서 돌아간다(일반 지식). |
| `ament_package()` | 이 패키지를 ament 인덱스에 등록하고 설치 파일을 마무리한다. 반드시 `CMakeLists.txt`의 마지막 줄이어야 한다. |

`ros2 pkg create`가 만든 `CMakeLists.txt`에 있는 `add_compile_options(-Wall -Wextra -Wpedantic)` 블록과 안내 주석은 이 패키지에 C++ 코드가 없으므로 뺐다. 참조에 있는 `find_package(urdf REQUIRED)`도 뺐다. 이 패키지의 파일이 `urdf` 패키지의 코드를 쓰는 곳이 없고, 뺀 채로 빌드와 시험이 통과했다.

`package.xml`:

| 태그 | 뜻 |
|---|---|
| `<name>`, `<version>`, `<description>` | 패키지 이름·버전·설명. 이름은 폴더 이름, `project()`와 같아야 한다. |
| `<maintainer>` | 관리자. 본인 이름과 이메일로 바꿔도 된다. |
| `<license>` | 라이선스 표기(`Apache-2.0`). 같은 이름의 `LICENSE` 파일이 `ros2 pkg create`로 함께 만들어진다. |
| `<buildtool_depend>ament_cmake</buildtool_depend>` | 이 패키지를 빌드하는 도구가 `ament_cmake`임을 밝힌다. |
| `<test_depend>` | `colcon test`용 린트 패키지. |
| `<export><build_type>ament_cmake</build_type></export>` | colcon이 이 줄을 읽고 `cmake` 빌드를 호출한다. 없으면 colcon이 패키지 종류를 알지 못한다. |

`<exec_depend>`(실행할 때 필요한 패키지)는 단계 9에서 다룬다.

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix bumperbot_description
ls install/bumperbot_description/share/bumperbot_description
```

기대 결과(이 패키지의 줄만):

```text
Starting >>> bumperbot_description
Finished <<< bumperbot_description [0.59s]
```

작업 공간에 앞 장의 패키지도 있으면 그 패키지의 `Finished` 줄도 함께 나오고 `Summary`의 패키지 수는 사용자마다 다르다. `Failed`가 없고 경고가 없으면 된다.

```text
/home/<사용자>/bumperbot_ws/install/bumperbot_description
cmake  environment  hook  launch  local_setup.bash  ...  meshes  package.bash  ...  package.xml  ...  rviz  urdf
```

- 통과 조건: 경고 없이 빌드되고, `ros2 pkg prefix`가 `install/bumperbot_description`을 출력하고, `share/bumperbot_description` 아래에 `launch`, `meshes`, `rviz`, `urdf` 폴더가 있다.
- `--symlink-install`을 쓰면 설치된 파일이 소스 파일을 가리키는 심볼릭 링크가 된다. 파일 내용을 고치면 다시 빌드하지 않아도 반영된다. 단, **새 파일을 만들었을 때는 다시 `colcon build`를 해야** 링크가 생긴다.
- 이 장에서 새 터미널을 열 때마다 `source ~/bumperbot_ws/install/setup.bash`를 먼저 친다.

## 단계 2. 메시 복사

### 왜 필요한가

URDF의 `<mesh filename="package://bumperbot_description/meshes/base_link.STL"/>`는 CAD에서 내보낸 3D 모양 파일을 가리킨다. STL은 바이너리 파일이라 손으로 칠 수 없다. 참조 저장소에서 복사한다. 이 파일들이 없으면 RViz와 Gazebo에서 로봇의 몸체가 보이지 않는다(Gazebo는 충돌 모양도 이 파일에서 읽는다).

### 코드

```bash
cp -r ~/my_robot_project/references/bumperbot_odometry_control/Section4_Locomotion/bumperbot_ws/src/bumperbot_description/meshes/. ~/bumperbot_ws/src/bumperbot_description/meshes/
ls -la ~/bumperbot_ws/src/bumperbot_description/meshes
```

- 끝의 `meshes/.`는 폴더 안의 내용물만 복사하라는 뜻이다. `meshes/`로 쓰면 `meshes/meshes/`가 생기는 경우가 있다.

### 코드 설명

복사되는 파일은 여섯 개다.

| 파일 | 크기 | 쓰이는 곳 |
|---|---|---|
| `base_link.STL` | 약 13.6 MB | 몸체 (visual과 collision) |
| `wheel_left_link.STL`, `wheel_right_link.STL` | 각 약 1.0 MB | 바퀴 (visual만. collision은 구) |
| `caster_front_link.STL`, `caster_rear_link.STL` | 각 약 0.35 MB | 캐스터 (visual만) |
| `imu_link.STL` | 약 0.2 MB | 09장(센서 융합)에서 IMU 링크를 더할 때 쓴다. 이 장의 URDF에서는 쓰지 않는다. |

참조 저장소의 Section4부터 Section12까지 `meshes/` 폴더는 같다(`diff -rq`로 확인했다). `imu_link.STL`도 처음부터 들어 있으므로 복사는 이번 한 번이면 된다.

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ls -l install/bumperbot_description/share/bumperbot_description/meshes
```

- 통과 조건: `meshes/` 아래에 STL 여섯 개가 있고, 설치 위치의 각 파일이 소스의 STL로 향하는 심볼릭 링크(`->`)다.
- STL 파일을 나중에 추가했다면 이 빌드를 다시 해야 링크가 생긴다.

## 단계 3. URDF 시작: `base_footprint`와 `base_link`

### 왜 필요한가

URDF는 로봇을 **링크(link, 강체)와 조인트(joint, 링크 사이의 관계)의 트리**로 쓴 XML이다. 읽는 곳은 셋이다.

- `robot_state_publisher`: 조인트 값과 URDF의 기하 관계로 TF(`/tf`, `/tf_static`)를 만든다.
- RViz: `visual`로 모델을 그린다.
- Gazebo: `collision`과 `inertial`로 물리 시뮬레이션을 한다.

`xacro`는 URDF에 변수·매크로·`include`를 쓸 수 있게 하는 전처리기다. 파일 이름이 `.urdf.xacro`이고, 실행하면 일반 URDF XML이 나온다. 이 장에서는 `include`만 쓴다(단계 7).

로봇은 `base_footprint`(바닥 위 기준점)와 `base_link`(몸체) 두 링크로 시작한다.

### 코드

> 파일: `bumperbot_description/urdf/bumperbot.urdf.xacro` — 전체 (단계 3 시점)
```xml
<?xml version="1.0"?>

<robot name="bumperbot" xmlns:xacro="http://ros.org/wiki/xacro">

  <link name="base_footprint"/>

  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.04" rpy="0.0 0.25 0.3" />
      <mass value="8.2573504e-01"/>
      <inertia ixx="2.2124416e-02" ixy="-1.2294101e-04" ixz="3.4938785e-04"
               iyy="2.1193702e-02" iyz="-5.0120904e-05"
               izz="2.0064271e-02" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/base_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link" />
    <origin xyz="0 0 0.033" rpy="0 0 0"/>
  </joint>

</robot>
```

### 코드 설명

- `<?xml version="1.0"?>`: XML 선언.
- `<robot name="bumperbot" xmlns:xacro="http://ros.org/wiki/xacro">`: 로봇 이름과 xacro 이름공간 선언. `xmlns:xacro`가 없으면 `<xacro:include>` 같은 태그를 xacro가 알아보지 못해 오류가 난다("자주 나는 오류" 참고).
- `<link name="base_footprint"/>`: 내용이 없는 링크. 바닥 위의 한 점으로, 나중에 오도메트리(07장)가 `odom → base_footprint` 변환의 자식 프레임으로 쓴다. 모양도 질량도 없으므로 `visual`/`inertial`이 없다. (Gazebo는 SDF로 바꿀 때 이 링크에 몸체를 합친다. 단계 7 참고.)
- `<link name="base_link">`: 몸체. 세 부분으로 되어 있다.
  - `<inertial>`: 질량 `mass`(kg), 관성 모멘트 `inertia`(`ixx ixy ixz iyy iyz izz`, kg·m²), 무게중심의 위치·방향 `origin`. Gazebo가 동역학 계산에 쓴다. 참조 값 그대로 옮긴다. `origin`의 `rpy="0.0 0.25 0.3"`가 어떤 의도인지는 확인하지 못했다.
  - `<visual>`: 화면에 그릴 모양. `<geometry><mesh filename="package://...">`이 STL 파일을 가리킨다. `package://<패키지 이름>/<경로>`는 그 패키지의 `share` 폴더 기준 주소다. `origin`은 링크 좌표계에서 모양의 위치·방향이다.
  - `<collision>`: 충돌 계산용 모양. 여기서는 visual과 같은 메시를 쓴다. 단순한 모양(구 등)을 쓰면 시뮬레이션이 빠르다. 바퀴가 그렇게 되어 있다(단계 4).
- `<joint name="base_joint" type="fixed">`: 두 링크를 고정으로 잇는다.
  - `<parent link="base_footprint"/>`, `<child link="base_link"/>`: 트리의 방향. 부모 → 자식.
  - `<origin xyz="0 0 0.033" rpy="0 0 0"/>`: 부모 좌표계에서 자식 좌표계의 위치(m)와 방향(roll pitch yaw, rad). `base_link`는 `base_footprint`보다 0.033 m 위에 있다. 이 값이 바퀴 반지름과 같다. 바퀴 중심 높이가 `base_link`의 원점이다.
  - `type` 종류: `fixed`(고정), `continuous`(무한 회전), `revolute`(각도 제한 회전), `prismatic`(직선 이동) 등.

### 빌드와 시험

```bash
cd ~/bumperbot_ws
xacro src/bumperbot_description/urdf/bumperbot.urdf.xacro | check_urdf /dev/stdin
```

기대 결과:

```text
robot name is: bumperbot
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
```

- 통과 조건: `Successfully Parsed XML`이 나오고 트리가 `base_footprint → base_link`다.
- `xacro 파일`은 xacro를 풀어 URDF를 표준 출력으로 낸다. `check_urdf`는 URDF 문법과 트리 구조(루트 하나, 끊어진 링크 없음)를 검사한다. 메시 파일이 실제로 있는지는 검사하지 않는다.

## 단계 4. 바퀴 두 개

### 왜 필요한가

바퀴는 회전하는 조인트다. 03장에서 ros2_control이 이 조인트의 속도를 명령하고 위치·속도를 읽는다. 그래서 조인트 이름(`wheel_left_joint`, `wheel_right_joint`)이 03장의 `<ros2_control>` 태그와 제어기 설정에 그대로 쓰인다. 이름을 바꾸면 그곳도 같이 바꿔야 한다.

### 코드

`base_joint` 바로 아래, `</robot>` 앞에 넣는다.

> 파일: `bumperbot_description/urdf/bumperbot.urdf.xacro` — 수정 (단계 4 시점)
```xml
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link" />
    <origin xyz="0 0 0.033" rpy="0 0 0"/>
  </joint>
<!-- ... -->
  <link name="wheel_right_link">
    <inertial>
      <origin xyz="0 -0.014 0" rpy="0 0 0" />
      <mass value="0.0530086090966721" />
      <inertia ixx="1.88176298336665E-05" ixy="-4.8444933029987E-10" ixz="1.88163708851143E-08"
               iyy="3.11510738647764E-05" iyz="-6.93560663069607E-11"
               izz="1.8801969558182E-05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/wheel_right_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 -0.015 0" rpy="1.57 0 0" />
      <geometry>
        <sphere radius="0.033"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_right_joint" type="continuous">
    <origin xyz="0 -0.0701101849418637 0" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="wheel_right_link" />
    <axis xyz="0 1 0" />
  </joint>

  <link name="wheel_left_link">
    <inertial>
      <origin xyz="0 0.014 0" rpy="0 0 0" />
      <mass value="0.0530086043217644" />
      <inertia ixx="1.8792911368909E-05"  ixy="3.31256392204584E-10" ixz="-1.14082001662767E-08"
               iyy="3.11510738759073E-05" iyz="-3.60499245082192E-10"
               izz="1.88266886146847E-05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="-1.57 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/wheel_left_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0.015 0" rpy="-1.57 0 0" />
      <geometry>
        <sphere radius="0.033"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <origin xyz="0 0.0701101849418642 0" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="wheel_left_link" />
    <axis xyz="0 1 0" />
  </joint>
```

블록 가운데의 `<!-- ... -->` 줄은 "그 사이에 원래 있던 줄을 그대로 둔다"는 뜻이다. 파일에 치지 않는다. 이 가이드의 "수정" 블록에서 `...`만 있는 줄(`# ...`, `// ...`)은 모두 같은 뜻이다.

### 코드 설명

- 링크 `wheel_right_link`와 `wheel_left_link`는 같은 구조다(좌우 부호만 반대).
  - `<inertial>`: 바퀴 질량 약 0.053 kg. 무게중심 `origin`이 바퀴 중심에서 y축으로 ∓0.014 m 어긋나 있다.
  - `<visual>`: 바퀴 메시. `rpy="1.57 0 0"`(오른쪽)과 `rpy="-1.57 0 0"`(왼쪽)은 메시를 x축 둘레로 약 90도 돌려 올바른 방향으로 세운다. 메시의 원래 방향 때문으로 보이지만 이유는 확인하지 못했다.
  - `<collision>`: **구**(`<sphere radius="0.033"/>`). 반지름 0.033 m는 바퀴 반지름이고, 05장의 `wheel_radius`가 이 값이다. 구는 바닥과 한 점에서 닿아 시뮬레이션이 안정적이고 빠르다. 구의 중심이 y축으로 ±0.015 m 바깥에 있다.
- 조인트 `wheel_right_joint`, `wheel_left_joint`: `type="continuous"`. 각도 제한이 없는 회전 조인트다.
  - `<origin xyz="0 -0.0701... 0">`(오른쪽), `<origin xyz="0 0.0701... 0">`(왼쪽): `base_link` 기준으로 y축 ∓0.0701 m. 바퀴는 몸체의 좌우에 있다. ROS 좌표계는 x가 앞, y가 왼쪽, z가 위이므로 `+y`가 왼쪽이다.
  - `<axis xyz="0 1 0"/>`: 회전축이 y축(몸체의 좌우 방향)이다. 이 축 둘레의 양(+)의 회전은 바퀴가 앞으로 굴러가는 방향이다.
  - `<parent link="base_link"/>`, `<child link="wheel_..._link"/>`.
- 참고: 바퀴의 구 충돌 중심이 y축으로 ±(0.0701 + 0.015) = ±0.0851 m에 있으므로, 실제로 바닥과 닿는 두 점의 간격은 약 0.1702 m다. 05장에서 쓰는 `wheel_separation` 0.17은 이 값이다. 조인트 원점의 간격(0.1402 m)이 아니다.

### 빌드와 시험

```bash
cd ~/bumperbot_ws
xacro src/bumperbot_description/urdf/bumperbot.urdf.xacro | check_urdf /dev/stdin
```

기대 결과:

```text
robot name is: bumperbot
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  wheel_left_link
        child(2):  wheel_right_link
```

- 통과 조건: `base_link` 아래에 `wheel_left_link`와 `wheel_right_link`가 있다.

## 단계 5. 캐스터 두 개

### 왜 필요한가

바퀴 두 개만으로는 로봇이 앞뒤로 쓰러진다. 앞뒤에 구슬 모양의 보조 바퀴(캐스터)를 달아 세 점 이상으로 받친다. 이 캐스터는 회전하지 않는 고정 조인트이고, 마찰이 작은 구로 바닥과 닿는다(마찰값은 단계 7에서 정한다).

### 코드

`</robot>` 바로 앞에 넣는다.

> 파일: `bumperbot_description/urdf/bumperbot.urdf.xacro` — 수정 (단계 5 시점)
```xml
  <link name="caster_front_link">
    <inertial>
      <origin xyz="3.89968128948481E-07 2.40073004698083E-07 0.00219173397061506" rpy="0 0 0" />
      <mass value="1.44379265037903E-06" />
      <inertia ixx="7.23075276010246E-11" ixy="-6.10956883467859E-16" ixz="2.22849693525031E-15"
               iyy="3.8713793085917E-11"  iyz="2.93909858781952E-15"
               izz="1.00967486676306E-10" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/caster_front_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.005"/>
      </geometry>
    </collision>
  </link>

  <joint name="caster_front_joint" type="fixed">
    <origin xyz="0.04755 0 -0.0275" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="caster_front_link" />
    <axis xyz="0 0 0" />
  </joint>

  <link name="caster_rear_link">
    <inertial>
      <origin xyz="3.8996812895542E-07 2.40073004698083E-07 0.00219173397061509" rpy="0 0 0" />
      <mass value="1.44379265037902E-06" />
      <inertia ixx="7.23075276010246E-11" ixy="-6.10956872363424E-16" ixz="2.22849693527207E-15"
               iyy="3.87137930859167E-11" iyz="2.939098587861E-15"
               izz="1.00967486676306E-10" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/caster_rear_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.005"/>
      </geometry>
    </collision>
  </link>

  <joint name="caster_rear_joint" type="fixed">
    <origin xyz="-0.04755 0 -0.0275" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="caster_rear_link" />
    <axis xyz="0 0 0" />
  </joint>
```

단계 5가 끝난 시점의 파일 전체는 아래와 같다. 단계 4까지 쓴 것과 이어 붙인 모습을 확인한다.

> 파일: `bumperbot_description/urdf/bumperbot.urdf.xacro` — 전체 (단계 5 시점)
```xml
<?xml version="1.0"?>

<robot name="bumperbot" xmlns:xacro="http://ros.org/wiki/xacro">

  <link name="base_footprint"/>

  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.04" rpy="0.0 0.25 0.3" />
      <mass value="8.2573504e-01"/>
      <inertia ixx="2.2124416e-02" ixy="-1.2294101e-04" ixz="3.4938785e-04"
               iyy="2.1193702e-02" iyz="-5.0120904e-05"
               izz="2.0064271e-02" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/base_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link" />
    <origin xyz="0 0 0.033" rpy="0 0 0"/>
  </joint>

  <link name="wheel_right_link">
    <inertial>
      <origin xyz="0 -0.014 0" rpy="0 0 0" />
      <mass value="0.0530086090966721" />
      <inertia ixx="1.88176298336665E-05" ixy="-4.8444933029987E-10" ixz="1.88163708851143E-08"
               iyy="3.11510738647764E-05" iyz="-6.93560663069607E-11"
               izz="1.8801969558182E-05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/wheel_right_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 -0.015 0" rpy="1.57 0 0" />
      <geometry>
        <sphere radius="0.033"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_right_joint" type="continuous">
    <origin xyz="0 -0.0701101849418637 0" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="wheel_right_link" />
    <axis xyz="0 1 0" />
  </joint>

  <link name="wheel_left_link">
    <inertial>
      <origin xyz="0 0.014 0" rpy="0 0 0" />
      <mass value="0.0530086043217644" />
      <inertia ixx="1.8792911368909E-05"  ixy="3.31256392204584E-10" ixz="-1.14082001662767E-08"
               iyy="3.11510738759073E-05" iyz="-3.60499245082192E-10"
               izz="1.88266886146847E-05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="-1.57 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/wheel_left_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0.015 0" rpy="-1.57 0 0" />
      <geometry>
        <sphere radius="0.033"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <origin xyz="0 0.0701101849418642 0" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="wheel_left_link" />
    <axis xyz="0 1 0" />
  </joint>

  <link name="caster_front_link">
    <inertial>
      <origin xyz="3.89968128948481E-07 2.40073004698083E-07 0.00219173397061506" rpy="0 0 0" />
      <mass value="1.44379265037903E-06" />
      <inertia ixx="7.23075276010246E-11" ixy="-6.10956883467859E-16" ixz="2.22849693525031E-15"
               iyy="3.8713793085917E-11"  iyz="2.93909858781952E-15"
               izz="1.00967486676306E-10" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/caster_front_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.005"/>
      </geometry>
    </collision>
  </link>

  <joint name="caster_front_joint" type="fixed">
    <origin xyz="0.04755 0 -0.0275" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="caster_front_link" />
    <axis xyz="0 0 0" />
  </joint>

  <link name="caster_rear_link">
    <inertial>
      <origin xyz="3.8996812895542E-07 2.40073004698083E-07 0.00219173397061509" rpy="0 0 0" />
      <mass value="1.44379265037902E-06" />
      <inertia ixx="7.23075276010246E-11" ixy="-6.10956872363424E-16" ixz="2.22849693527207E-15"
               iyy="3.87137930859167E-11" iyz="2.939098587861E-15"
               izz="1.00967486676306E-10" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bumperbot_description/meshes/caster_rear_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.005"/>
      </geometry>
    </collision>
  </link>

  <joint name="caster_rear_joint" type="fixed">
    <origin xyz="-0.04755 0 -0.0275" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="caster_rear_link" />
    <axis xyz="0 0 0" />
  </joint>

</robot>
```

### 코드 설명

- `caster_front_link`와 `caster_rear_link`: 질량이 약 1.4×10⁻⁶ kg으로 거의 0인 작은 링크다. `visual`은 메시, `collision`은 **반지름 0.005 m의 구**다.
- `caster_front_joint`, `caster_rear_joint`: `type="fixed"`. `base_link`에 붙은 위치는 앞 `x=+0.04755`, 뒤 `x=-0.04755`, 둘 다 `z=-0.0275`이다. `base_link`가 바닥에서 0.033 m 위이므로 캐스터 중심은 바닥 위 0.0055 m이고, 구의 아래 끝은 바닥에서 0.0005 m 떨어져 있다. 그래서 평소에는 바퀴 두 개가 몸체를 받치고, 로봇이 앞뒤로 약 0.6도(0.0105 rad) 기울면 캐스터가 바닥에 닿는다. Gazebo에서 로봇을 스폰한 뒤 자세를 읽으면 이 값이 나온다(단계 8).
- `<axis xyz="0 0 0"/>`: 고정 조인트에서는 쓰이지 않는 줄이다. 참조가 넣어 둔 것을 그대로 옮긴다.

### 빌드와 시험

```bash
cd ~/bumperbot_ws
xacro src/bumperbot_description/urdf/bumperbot.urdf.xacro | check_urdf /dev/stdin
```

기대 결과:

```text
robot name is: bumperbot
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  caster_front_link
        child(2):  caster_rear_link
        child(3):  wheel_left_link
        child(4):  wheel_right_link
```

- 통과 조건: `base_link` 아래에 링크 네 개(캐스터 둘, 바퀴 둘)가 있다.
- 자식 목록의 순서는 이름순으로 나온다. 파일에 쓴 순서와 다를 수 있다.

## 단계 6. `display.launch.py`와 RViz

### 왜 필요한가

로봇 모델이 맞게 쓰였는지 눈으로 보려면 RViz에 올려야 한다. 그러려면 세 가지가 필요하다.

1. URDF 문자열을 `robot_description` 파라미터로 가진 `robot_state_publisher`. 이 노드는 URDF를 `/robot_description` 토픽(RViz가 모델을 그리는 데 쓴다)으로 내보내고, 링크 사이의 변환을 `/tf_static`(고정 조인트)과 `/tf`(움직이는 조인트)로 내보낸다.
2. 움직이는 조인트의 값(`/joint_states`). 바퀴는 `continuous` 조인트라 값이 없으면 `wheel_*_link` 프레임이 TF에 나오지 않는다. 이 장에서는 값을 내는 노드가 없으므로 슬라이더 창 `joint_state_publisher_gui`가 그 역할을 한다. **03장에서 `joint_state_broadcaster`(제어기)가 `/joint_states`를 내게 되면 이 GUI는 필요 없어진다.**
3. RViz 자신(`rviz2`)과 설정 파일.

이 셋을 한 번에 띄우는 것이 launch 파일이다.

### 코드

> 파일: `bumperbot_description/launch/display.launch.py` — 전체
```python
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bumperbot_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
```

### 코드 설명

- `generate_launch_description()`: launch 파일의 약속된 진입점. `ros2 launch`가 이 함수를 호출해 `LaunchDescription`(실행할 일의 목록)을 받는다.
- `get_package_share_directory("bumperbot_description")`: 설치된 `share/bumperbot_description`의 절대 경로. 단계 1에서 `install(DIRECTORY ...)`로 설치했기 때문에 찾을 수 있다.
- `DeclareLaunchArgument(name="model", default_value=..., description=...)`: 명령줄에서 `model:=<경로>`로 바꿀 수 있는 launch 인자를 선언한다. 기본값은 설치된 `bumperbot.urdf.xacro`다. `ros2 launch bumperbot_description display.launch.py --show-args`로 인자 목록을 볼 수 있다.
- `LaunchConfiguration("model")`: 그 인자의 값을 가리키는 자리표시자다. 값은 launch가 실행될 때 정해진다. 파이썬 문자열이 아니다.
- `Command(["xacro ", LaunchConfiguration("model")])`: 실행 시점에 `xacro <model 경로>` 명령을 돌리고 그 출력(URDF 문자열)을 값으로 쓴다. `"xacro "` 끝의 공백이 필요하다. 없으면 `xacro/path...`로 붙어 버린다.
- `ParameterValue(..., value_type=str)`: 이 값을 **문자열 그대로** 파라미터로 넘기라고 알린다. 없으면 launch가 값을 YAML 해석기에 통과시킨다. 이 URDF는 통과하지만 줄바꿈이 공백으로 바뀌어 한 줄로 들어간다(확인했다). URDF에 `: `가 든 주석이 있으면 YAML 해석이 실패한다(일반 지식, 파이썬 `yaml`로 `<!-- note: a -->`가 오류가 되는 것을 확인했다). 그래서 항상 이 래퍼를 쓴다.
- `Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": robot_description}])`: 노드 하나를 실행한다. `parameters`의 딕셔너리가 그 노드의 파라미터가 된다.
- `joint_state_publisher_gui`: 슬라이더 창을 띄워 `/joint_states`를 낸다. `robot_state_publisher`가 이를 받아 `wheel_*_link`의 TF를 만든다.
- `rviz2` 노드: `arguments=["-d", ".../rviz/display.rviz"]`로 설정 파일을 연다. `output="screen"`은 노드의 로그를 터미널에 보인다.
- `LaunchDescription([...])`: 위에서 만든 인자·노드를 담아 돌려준다. 인자 선언(`model_arg`)이 맨 앞에 있어야 한다.

### RViz 설정 파일 `rviz/display.rviz`

이 파일은 RViz가 저장하는 텍스트라 손으로 칠 수 없다. `display.launch.py`가 `-d` 인자로 이 파일을 열므로 **launch를 실행하기 전에** 있어야 한다. 두 방법 중 하나로 만든다.

**방법 1(권장): 참조 파일을 복사한다.**

```bash
cp ~/my_robot_project/references/bumperbot_odometry_control/Section4_Locomotion/bumperbot_ws/src/bumperbot_description/rviz/display.rviz ~/bumperbot_ws/src/bumperbot_description/rviz/display.rviz
```

참조의 Section4부터 Section12까지 이 파일은 같다. 이 파일의 핵심 설정은 `Fixed Frame: base_footprint`, `RobotModel`(`Description Topic: /robot_description`), `TF` 표시, `Grid`다. 참조 파일에는 `camera_link`, `laser_link`, `imu_link`가 들어 있다. 이 로봇에는 아직 없는 프레임이지만 RViz는 없는 프레임을 무시한다고 보았다(RViz 화면은 이 가이드를 쓸 때 확인하지 못했다).

**방법 2: launch 파일 없이 노드를 따로 띄워 RViz에서 직접 만든다.** 아래 RViz 조작은 이 가이드를 쓸 때 화면이 없어 따라 해 보지 못했다. 노드를 띄우는 명령은 확인했다(`robot_state_publisher`에 `-p robot_description:="$(xacro ...)"`로 URDF를 주면 `/robot_description`과 `/tf_static`이 나온다).

먼저 빌드해서 `$(find bumperbot_description)`가 풀리게 한다.

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
```

터미널 A에서 `robot_state_publisher`를 띄운다. 끝나지 않는 명령이다.

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro)"
```

터미널 B에서 슬라이더 창을 띄운다. 끝나지 않는 명령이다.

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

터미널 C에서 설정 파일 없이 RViz를 띄운다. 끝나지 않는 명령이다.

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 run rviz2 rviz2
```

RViz 안에서:

1. 왼쪽 위 `Global Options`의 `Fixed Frame`을 `base_footprint`로 바꾼다.
2. 왼쪽 아래 `Add`를 눌러 `RobotModel`을 추가하고 `Description Topic`을 `/robot_description`으로 정한다.
3. `Add`로 `TF`를 추가한다.
4. 메뉴 `File → Save Config As`로 `~/bumperbot_ws/src/bumperbot_description/rviz/display.rviz`에 저장한다.

저장한 뒤 세 터미널을 모두 `Ctrl+C`로 끝낸다. 새 파일이 생겼으므로 아래 "빌드와 시험"의 `colcon build`가 필요하다.

### 빌드와 시험

터미널 A에서 빌드하고 launch를 실행한다. launch는 끝나지 않는다. 시험을 마치면 이 터미널에서 `Ctrl+C`로 끝낸다.

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bumperbot_description display.launch.py
```

창이 뜨면 RViz에 `RobotModel`이 보이고 `joint_state_publisher_gui`의 슬라이더 두 개(`wheel_left_joint`, `wheel_right_joint`)로 바퀴가 도는 것을 볼 수 있어야 한다. 이 두 창은 이 가이드를 쓸 때 화면 없이 돌렸으므로 직접 보지는 못했다. 아래 명령으로 확인되는 것이 통과 조건이다.

터미널 B에서:

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /tf_static --once
```

이 가이드를 쓸 때는 창을 띄우지 않고, launch에서 GUI 두 노드를 뺀 사본으로 `robot_state_publisher`만 띄워 같은 항목을 확인했다. 아래 출력은 그 결과다. **사용자가 GUI가 뜨는 원래 launch를 돌리면 `/joint_state_publisher_gui`, `/rviz2` 노드와 RViz가 쓰는 토픽이 더 보인다.**

```text
/robot_state_publisher
```

```text
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

`/tf_static`의 내용(줄여서):

```text
frame_id: base_footprint   child_frame_id: base_link
frame_id: base_link        child_frame_id: caster_front_link
frame_id: base_link        child_frame_id: caster_rear_link
```

- 통과 조건 1: `/robot_description`과 `/tf_static`이 있다.
- 통과 조건 2: `/tf_static`에는 **고정 조인트**(`base_joint`, 캐스터 둘)만 있고 `wheel_*_link`는 없다. 바퀴는 `continuous` 조인트라 `/joint_states` 값이 와야 TF가 나온다. (`joint_state_publisher_gui`가 떠 있으면 슬라이더 값이 `/joint_states`로 나가므로 바퀴 프레임도 `/tf`에 나온다.)

**`joint_state_publisher_gui`를 띄우지 않았을 때**(GUI가 없는 컴퓨터, 또는 launch에서 그 노드를 뺀 경우)는 슬라이더가 하는 일을 손으로 대신해 볼 수 있다. `joint_state_publisher_gui`가 떠 있는 동안에는 두 발행자가 같은 토픽에 겹치므로 이 시험을 하지 않는다. 아래 명령은 `/joint_states` 메시지를 1초에 10번 계속 보낸다. `stamp: now`가 필요하다. 시각이 0이면 `robot_state_publisher`가 TF를 내지 않는다(이 가이드를 쓸 때 확인했다).

터미널 B에서(끝나지 않는 명령. 확인이 끝나면 `Ctrl+C`):

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{header: {stamp: now}, name: [wheel_left_joint, wheel_right_joint], position: [0.5, 1.0]}"
```

터미널 C에서:

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 run tf2_ros tf2_echo base_link wheel_left_link
```

기대 결과(`At time` 값은 실행할 때마다 다르고, 변환은 계속 출력된다. 확인이 끝나면 `Ctrl+C`):

```text
At time 1795239077.986811033
- Translation: [0.000, 0.070, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.247, 0.000, 0.969]
- Rotation: in RPY (radian) [0.000, 0.500, 0.000]
```

- `Translation y = 0.070`: 조인트 `origin`의 y(0.0701). `RPY pitch = 0.500`: 보낸 `position`(0.5 rad)만큼 y축 둘레로 돌았다. 슬라이더가 하는 일이 바로 이 메시지를 보내는 것이다.
- `tf2_echo`를 시작한 직후에는 `Invalid frame ID ... frame does not exist`가 한두 번 나올 수 있다. TF를 받기 전의 대기다.
- 주의: `tf2_echo`의 출력을 파이프(`|`)로 넘기면 버퍼링되어 보이지 않을 수 있다. 터미널에 그대로 띄운다.

## 단계 7. Gazebo 전용 속성: `bumperbot_gazebo.xacro`

### 왜 필요한가

URDF에는 마찰 계수 같은 시뮬레이터용 속성을 쓰는 표준 자리가 없다. `<gazebo reference="링크 이름">` 태그가 그 자리다. Gazebo가 URDF를 읽어 SDF(Gazebo 자신의 로봇 파일 형식)로 바꿀 때 이 태그 안의 값들을 해당 링크의 충돌 속성으로 옮긴다. RViz나 `robot_state_publisher`는 이 태그를 무시한다.

속성들을 `bumperbot.urdf.xacro`에 섞어 쓰면 길어지므로 별도 파일로 나누고 `<xacro:include>`로 불러온다. 03장에서 이 파일에 ros2_control 플러그인이 더해진다.

### 코드

> 파일: `bumperbot_description/urdf/bumperbot_gazebo.xacro` — 전체
```xml
<?xml version="1.0"?>

<robot name="bumperbot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Wheels -->
  <gazebo reference="wheel_left_link">
    <mu1>1000000000000000.0</mu1>
    <mu2>1000000000000000.0</mu2>
    <kp>1000000000000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
  </gazebo>

  <gazebo reference="wheel_right_link">
    <mu1>1000000000000000.0</mu1>
    <mu2>1000000000000000.0</mu2>
    <kp>1000000000000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
  </gazebo>

  <!-- Caster Wheels -->
  <gazebo reference="caster_rear_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>1.0</maxVel>
  </gazebo>

  <gazebo reference="caster_front_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>1.0</maxVel>
  </gazebo>

</robot>
```

그리고 `bumperbot.urdf.xacro`의 맨 위 `<robot ...>` 줄 바로 아래, `<link name="base_footprint"/>` 앞에 `include`를 넣는다.

> 파일: `bumperbot_description/urdf/bumperbot.urdf.xacro` — 수정
```xml
<robot name="bumperbot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Include Gazebo Parameters -->
  <xacro:include filename="$(find bumperbot_description)/urdf/bumperbot_gazebo.xacro" />

  <link name="base_footprint"/>
```

### 코드 설명

`bumperbot_gazebo.xacro`:

- `<robot name="bumperbot" xmlns:xacro=...>`: include되는 파일도 `<robot>`을 루트로 쓰는 것이 관례다(루트가 `<robot>`이어야 한다고 확인하지는 않았다). xacro는 include한 파일의 루트 태그는 버리고 안의 내용만 가져온다.
- `<gazebo reference="wheel_left_link">`: 이 링크에 적용할 Gazebo 속성. 안의 태그들은 충돌 모양의 접촉 속성이 된다.

| 태그 | 뜻 | SDF에서 |
|---|---|---|
| `mu1`, `mu2` | 마찰 계수(두 방향) | `<friction><ode><mu>`, `<mu2>` |
| `kp`, `kd` | 접촉의 강성·감쇠 | `<contact><ode><kp>`, `<kd>` |
| `minDepth` | 접촉이 시작되는 침투 깊이 | `<min_depth>` |
| `maxVel` | 접촉에서 튕겨 나오는 최대 속도 | `<max_vel>` |
| `fdir1` | 마찰 방향 1의 축 | `<fdir1>` |

- 바퀴는 `mu1`·`mu2`가 매우 크다(10¹⁵). 바닥에서 미끄러지지 않게 한다. 캐스터는 0.1로 작다. 마찰이 작아야 캐스터가 제자리에서 끌리지 않고 로봇이 회전한다.
- 위 대응표는 `gz sdf -p <urdf 파일>`로 URDF를 SDF로 바꾸어 직접 확인했다(바퀴 collision 안에 `<mu>1000000000000000</mu>`, `<fdir1>1 0 0</fdir1>`이 들어 있다). 이 값들 중 어느 것을 Gazebo의 물리 엔진이 실제로 쓰는지는 확인하지 못했다. 저장소 코드에는 없지만 일반 지식에 따르면 `kp`, `kd`, `minDepth`, `maxVel`은 ODE 엔진용 값이고 Gazebo Harmonic의 기본 엔진은 ODE가 아니다.
- 같은 변환에서 `base_footprint`, `base_link`, 캐스터 두 개가 **하나의 링크로 합쳐진** 것도 확인했다(fixed 조인트로 이어진 링크는 SDF에서 합쳐진다). 바퀴 두 개만 별도의 링크와 회전 조인트로 남는다. 03장에서 제어하는 대상이 바퀴 조인트 둘뿐인 것과 맞는다.
- 변환 결과에서 메시 주소가 `package://bumperbot_description/meshes/...`에서 `model://bumperbot_description/meshes/...`로 바뀐다. 이것이 단계 8의 `GZ_SIM_RESOURCE_PATH`가 필요한 이유다.

`bumperbot.urdf.xacro`:

- `<xacro:include filename="$(find bumperbot_description)/urdf/bumperbot_gazebo.xacro" />`: 그 자리에 파일 내용을 끼워 넣는다. `$(find 패키지)`는 xacro가 해당 패키지의 `share` 경로로 바꾼다. 그래서 패키지가 설치되어 있고 `source install/setup.bash`가 되어 있어야 풀린다. 경로가 아니라 이름으로 적기 때문에 작업 공간 위치가 달라도 동작한다.

### 빌드와 시험

새 터미널이라면 `source ~/bumperbot_ws/install/setup.bash`를 먼저 친다.

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
xacro src/bumperbot_description/urdf/bumperbot.urdf.xacro > /tmp/bumperbot.urdf
check_urdf /tmp/bumperbot.urdf | head -3
grep -c "<gazebo" /tmp/bumperbot.urdf
gz sdf -p /tmp/bumperbot.urdf | grep -E "<link name|<joint name|<uri>"
```

기대 결과:

```text
robot name is: bumperbot
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
```

```text
4
```

- 통과 조건: `check_urdf`가 통과하고, 풀린 URDF에 `<gazebo` 태그가 4개(바퀴 2, 캐스터 2) 있다. `gz sdf -p`의 출력에는 링크 `base_footprint`, `wheel_left_link`, `wheel_right_link`와 `model://bumperbot_description/meshes/...` 주소가 나온다.
- `error: <class 'ament_index_python.packages.PackageNotFoundError'>: "package 'bumperbot_description' not found, searching: ['/opt/ros/jazzy']"`: `source install/setup.bash`를 하지 않았거나 패키지를 빌드하지 않은 경우다(`$(find ...)`가 풀리지 않는다).

## 단계 8. `gazebo.launch.py`: Gazebo에 로봇 스폰

### 왜 필요한가

Gazebo(gz sim)는 ROS와 별개의 프로그램이다. 로봇을 올리려면 (1) Gazebo를 띄우고, (2) 로봇 모델 파일을 넘겨 월드에 생성하고, (3) 시뮬레이터의 시간을 ROS로 넘겨야 한다. launch 파일 하나가 이를 모두 한다.

### 코드

> 파일: `bumperbot_description/launch/gazebo.launch.py` — 전체
```python
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bumperbot_description, "urdf", "bumperbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(bumperbot_description).parent.resolve())
            ]
        )

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4", " -r", " empty.sdf"]
                    )
                ]
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
```

### 코드 설명

- `model_arg`: 단계 6과 같은 `model` 인자. 기본값이 설치된 `bumperbot.urdf.xacro`다.
- `SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=[...])`: 이 launch가 띄우는 프로세스들의 환경변수를 정한다. 터미널에 이미 `GZ_SIM_RESOURCE_PATH`가 설정되어 있어도 이 launch 안에서는 이 값으로 **덮어쓴다**(기존 값에 이어 붙이지 않는다). 값은 `install/bumperbot_description/share`(`bumperbot_description`의 **부모** 폴더)다. 이유: 단계 7에서 본 것처럼 Gazebo는 메시 주소를 `model://bumperbot_description/meshes/base_link.STL`로 받는다. Gazebo는 `GZ_SIM_RESOURCE_PATH`에 적힌 폴더들 아래에서 `bumperbot_description/meshes/base_link.STL`을 찾는다. 그래서 `share` 폴더를 적어야 한다. `Path(...).parent.resolve()`는 `share/bumperbot_description`의 부모를 절대 경로로 만든다.
  - 이 변수가 없을 때의 증상은 "자주 나는 오류"에 있다(모델은 생성되지만 메시를 못 읽는다).
- `robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)`: 단계 6과 같다.
- `robot_state_publisher_node`: URDF를 `/robot_description` 토픽으로 낸다. 이번에는 `"use_sim_time": True`를 준다. 이 노드가 시간을 시계(`/clock`) 토픽에서 읽으라는 뜻이다. 이 노드와 Gazebo의 시간이 같아야 TF의 시각이 맞는다.
- `gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(.../gz_sim.launch.py), launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])])`: `ros_gz_sim` 패키지가 제공하는 launch 파일을 불러온다. `gz_args`는 `gz sim`에 그대로 넘어간다.
  - `-v 4`: 로그 상세 수준.
  - `-r`: 시작하자마자 시뮬레이션을 실행한다(없으면 일시정지 상태로 시작한다).
  - `empty.sdf`: 월드 파일. 바닥(`ground_plane`)과 조명만 있는 빈 월드다.
  - 창(GUI)과 물리 서버가 모두 뜬다. 서버만 띄우려면 `-s`를 더한다(이 가이드를 쓸 때는 창 없이 확인하려고 이 방식을 썼다).
  - 이 launch 파일은 내부에서 `ruby gz sim ...` 명령을 실행하고, 환경변수 `GZ_SIM_SYSTEM_PLUGIN_PATH`에 `LD_LIBRARY_PATH`를 더한다. 03장의 gz 플러그인을 찾는 데 쓰이는 경로다.
- `gz_spawn_entity = Node(package="ros_gz_sim", executable="create", arguments=["-topic", "robot_description", "-name", "bumperbot"])`: 월드에 모델을 생성한다. `-topic robot_description`은 `robot_description` 토픽에서 URDF 문자열을 받아 쓰라는 뜻이다. 토픽이 올 때까지 기다린다. `-name bumperbot`은 Gazebo 안에서의 모델 이름이다. 생성이 끝나면 이 프로세스는 종료한다(`process has finished cleanly`).
- `gz_ros2_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])`: Gazebo 내부 통신(gz transport)과 ROS 2 토픽 사이를 잇는 다리다. 인자의 형식은 `토픽@ROS타입방향Gazebo타입`이고 방향 기호 `[`는 **Gazebo → ROS** 한 방향이다(`@`는 양방향, `]`는 ROS → Gazebo). 여기서는 시뮬레이션 시각 `/clock`을 ROS로 낸다. `use_sim_time`을 켠 노드들은 이 토픽으로 시간을 읽는다.
- `LaunchDescription([...])`: 목록의 순서대로 시작한다. **순서가 중요한 것**이 있다. `DeclareLaunchArgument`(`model_arg`)는 그 값을 쓰는 노드보다 앞에 있어야 하고(`robot_state_publisher_node`보다 뒤에 두면 `launch configuration 'model' does not exist`로 launch가 시작하지 못한다), `SetEnvironmentVariable`은 환경변수를 쓰는 `gazebo`(include)보다 앞에 있어야 한다(뒤에 두면 메시를 못 찾는 `Unable to find file with URI` 오류가 난다). 둘 다 이 가이드를 쓸 때 목록 순서를 바꿔 재현했다. 반대로 노드끼리(`robot_state_publisher_node`, `gz_spawn_entity`, `gz_ros2_bridge`)는 순서를 바꿔도 모델이 스폰되었다. `create`가 `robot_description` 토픽이 올 때까지 기다려 주기 때문이다.

### 빌드와 시험

사용자가 돌리는 원래 명령은 창(GUI)이 뜬다. 터미널 A에서 실행한다. launch는 끝나지 않는다. 시험을 마치면 이 터미널에서 `Ctrl+C`로 끝낸다. 다시 시험하기 전에 앞의 Gazebo가 모두 끝났는지 확인한다(`ps -eo pid,args | grep "gz sim"`에 남은 것이 없어야 한다).

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bumperbot_description gazebo.launch.py
```

창이 뜨면 바닥 위에 로봇이 놓여 있어야 한다. 이 창은 이 가이드를 쓸 때 직접 보지 못했다. 아래 출력은 창 없이 서버만 띄워(`gz_args`에 `-s`를 더한 launch 사본) 얻은 것이다. 사본은 `gazebo.launch.py`에서 `[" -v 4", " -r", " empty.sdf"]`를 `[" -v 3", " -s", " -r", " empty.sdf"]`로 바꾼 것이다. 사용자의 창이 있는 실행에서는 그래픽 관련 로그가 더 섞일 수 있다.

터미널 B에서:

```bash
source ~/bumperbot_ws/install/setup.bash
gz model --list
ros2 node list
ros2 topic echo /clock --once
gz model -m bumperbot -p
```

기대 결과(값은 실행할 때마다 다르다):

```text
Available models:
    - ground_plane
    - bumperbot
```

```text
/robot_state_publisher
/ros_gz_bridge
```

```text
clock:
  sec: 15
  nanosec: 916000000
```

```text
Model: [10]
  - Name: bumperbot
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [-0.001611 0.000000 0.000002]
    [0.000000 0.010548 0.000002]
```

`sec`는 시뮬레이션이 시작한 뒤 흐른 시간이고, `Pose`의 작은 값들(예: `-0.001611`)은 스폰 직후 로봇이 자리를 잡는 정도라 실행마다 조금 다르다.

터미널 A(launch)의 로그에서:

```text
[create-3] [INFO] [...] [ros_gz_sim]: Waiting messages on topic [robot_description].
[create-3] [INFO] [...] [ros_gz_sim]: Entity creation successful.
[INFO] [create-3]: process has finished cleanly [pid ...]
```

`[create-3]`의 번호는 launch 목록의 순서에 따른 것이라 다를 수 있다.

- 통과 조건 1: `gz model --list`에 `bumperbot`이 있고 `Entity creation successful`이 로그에 있다.
- 통과 조건 2: `/clock`이 나온다(`ros2 topic echo /clock --once`가 값을 출력한다).
- 통과 조건 3: 로그에 메시를 못 찾는다는 오류(`Unable to find file with URI`, `Failed to load mesh`)가 없다. (서버만 띄운 이 가이드의 시험에서는 `grep -iE "error|warn|unable|fail"`로 찾은 줄이 0건이었다.)
- 자세(pitch) 0.0105 rad는 단계 5에서 계산한 값과 같다. 로봇이 바퀴 위에서 약간 기울어 캐스터 하나가 바닥에 닿은 상태다.
- 아직 `/joint_states`를 내는 노드가 없다. 토픽 목록에 `/joint_states`가 보이는 것은 `robot_state_publisher`가 구독하기 때문이다. 03장에서 `joint_state_broadcaster`가 낸다.

## 단계 9. `package.xml`의 `<exec_depend>`

### 왜 필요한가

이 패키지는 코드를 빌드하지 않으므로 빌드할 때 필요한 의존성이 없다. 대신 launch 파일이 **실행할 때** 다른 패키지의 노드와 명령을 쓴다. `<exec_depend>`는 그 목록이다.

- 빌드·실행 자체는 이 태그가 없어도 된다. 해당 패키지가 설치되어 있으면 동작한다.
- 이 태그를 읽는 것은 `rosdep`(`rosdep install --from-paths src -y`처럼 의존 패키지를 자동으로 설치하는 도구)와 사람이다. 다른 컴퓨터(예: Raspberry Pi)에 이 패키지를 옮길 때 무엇을 설치해야 하는지 알려 주는 문서 역할을 한다.

### 코드

`package.xml`의 `<buildtool_depend>`와 `<test_depend>` 사이에 넣는다.

> 파일: `bumperbot_description/package.xml` — 수정
```xml
  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ros2launch</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
```

### 코드 설명

| `<exec_depend>` | 쓰이는 곳 |
|---|---|
| `robot_state_publisher` | 두 launch의 `robot_state_publisher` 노드 |
| `urdf` | 이 패키지의 파일에서는 쓰지 않는다(참조가 넣어 둔 것) |
| `joint_state_publisher_gui` | `display.launch.py`의 슬라이더 창 |
| `rviz2` | `display.launch.py`의 RViz |
| `xacro` | `Command(["xacro ", ...])`와 `$(find ...)`, `<xacro:include>` |
| `ros2launch` | `ros2 launch` 명령 자체 |
| `ros_gz_sim` | `gz_sim.launch.py`와 `create` |
| `ros_gz_bridge` | `parameter_bridge` |

03장에서 `gz_ros2_control`이 하나 더 추가된다.

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg xml bumperbot_description | grep -c exec_depend
```

기대 결과: `8`

- 통과 조건: 경고 없이 빌드되고 `exec_depend`가 8개로 센다(03장에서 `gz_ros2_control`을 더하면 9개가 된다).
- 위 `urdf` 줄은 이 가이드의 `CMakeLists.txt`(`find_package(urdf)` 없음)와 파일들이 `urdf` 패키지를 쓰지 않으므로 사실상 문서 역할만 한다. 참조는 `CMakeLists.txt`에서 `find_package(urdf REQUIRED)`까지 했으므로 엄밀히는 그쪽이 `<depend>urdf</depend>`여야 맞다.

## 참조 코드와 다른 점

| 참조 | 이 가이드 | 이유 |
|---|---|---|
| `CMakeLists.txt`: `cmake_minimum_required(VERSION 3.5)` | `3.8` | `ros2 pkg create`가 만드는 값을 그대로 쓴다. |
| `CMakeLists.txt`: `find_package(urdf REQUIRED)` | 없음 | 이 패키지에 `urdf`의 코드를 쓰는 곳이 없다. 빼고 빌드·시험이 통과했다. |
| `package.xml`의 `<maintainer>`(참조 저자의 이름과 이메일), `<license>Apache 2.0</license>` | `Your Name` / `you@example.com`, `Apache-2.0` | 본인 정보로 바꾸는 자리. 라이선스는 ament 식별자 `Apache-2.0`으로 썼다. 01장과 같다. |
| `LICENSE` 파일 없음 | 있음(Apache-2.0 전문) | `ros2 pkg create --license Apache-2.0`이 만든다. 01장과 같다. |
| 파일 안의 불필요한 공백·마지막 줄 줄바꿈 없음 | 정리한 공백, 마지막 줄바꿈 있음 | 내용은 같다. |
| `display.launch.py`, `gazebo.launch.py`, 두 xacro | 같음 | Section4 시점 파일을 그대로 쓴다. 이 장에는 Humble 분기가 없다(`is_ignition`은 Section5에서 나온다. 03장에서 다룬다). |

## 자주 나는 오류

| 증상 | 원인 | 고치는 법 |
|---|---|---|
| `colcon build`가 `CMake Error ... file INSTALL cannot find ".../meshes": No such file or directory.`로 실패한다. | `install(DIRECTORY launch meshes urdf rviz ...)`에 적은 폴더가 소스에 없다. | 네 폴더를 모두 만든다(`mkdir -p launch urdf rviz meshes`). 빈 폴더라도 있어야 한다. |
| `ros2 launch`가 `Package 'bumperbot_description' not found` 또는 `file 'display.launch.py' was not found in the share directory`라고 한다. | `source install/setup.bash`를 하지 않았거나, 새 launch 파일을 만든 뒤 `colcon build`를 다시 하지 않았다(`--symlink-install`은 새 파일의 링크를 빌드 때 만든다). | 새 터미널마다 `source ~/bumperbot_ws/install/setup.bash`. 새 파일을 만들면 `colcon build --symlink-install`을 다시 한다. |
| `xacro`가 오류를 내고 메시지 끝에 `Check that: - Your XML is well-formed - You have the xacro xmlns declaration: xmlns:xacro="http://www.ros.org/wiki/xacro"`가 있다. | 이 문구는 XML 문법 오류(태그를 닫지 않은 경우 등)에도 공통으로 나온다. `<robot>`에서 `xmlns:xacro=...`를 빼서도 같은 문구가 나오는 것을 확인했다. | 먼저 `<robot name="bumperbot" xmlns:xacro="http://ros.org/wiki/xacro">`처럼 `xmlns:xacro`가 있는지, 다음에 태그가 모두 닫혔는지 본다. (이 가이드의 파일은 `www` 없는 주소를 쓰고 그대로 동작했다.) |
| Gazebo 로그에 `[Err] [SystemPaths.cc:426] Unable to find file with URI [model://bumperbot_description/meshes/base_link.STL]`, `[Err] [MeshManager.cc:211] Unable to find file[...]`, `[Wrn] [Util.cc:884] Failed to load mesh from [...]`가 뜬다. | `GZ_SIM_RESOURCE_PATH`가 설정되지 않아 Gazebo가 메시 폴더를 찾지 못한다. 이때도 모델은 만들어지지만(`gz model --list`에 보인다) 메시 모양이 없다. `gazebo_resource_path`를 launch 목록에서 빼거나 `gazebo`(include)보다 뒤에 둔 사본으로 재현했다. | `SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ...)`가 launch 목록에 있고 `gazebo`보다 앞에 있는지 확인한다. |
| 바퀴 프레임이 TF에 없다. `ros2 run tf2_ros tf2_echo base_link wheel_left_link`가 `Invalid frame ID "wheel_left_link" ... frame does not exist`를 반복한다. RViz에서도 바퀴가 나오지 않는다(RViz 화면은 확인하지 못했다). | 바퀴는 `continuous` 조인트라 `/joint_states`가 와야 TF가 생긴다. | 이 장에서는 `joint_state_publisher_gui`가 떠 있어야 한다. 03장 이후에는 `joint_state_broadcaster`가 낸다. |
| `launch configuration 'model' does not exist` 오류로 `gazebo.launch.py`가 시작하지 못한다. | `DeclareLaunchArgument`(`model_arg`)를 `model`을 쓰는 노드보다 뒤에 두었다(재현했다). | 인자 선언을 `LaunchDescription` 목록의 맨 앞에 둔다. |
| 로봇 URDF를 고쳤는데 RViz가 그대로다. | `robot_state_publisher`는 시작할 때 URDF를 한 번 읽는다. | launch를 껐다가 다시 켠다. |
| `check_urdf`가 `Failed to find root link: Two root links found: [base_footprint] and [base_link]`를 낸다. | 링크가 조인트로 이어지지 않아 트리의 뿌리가 둘이다(`base_joint`를 지웠을 때 재현했다). | 조인트마다 `<parent>`와 `<child>` 이름이 링크 이름과 같은지 본다. |

## 정리

| 파일 | 역할 | 누가 읽는가 |
|---|---|---|
| `CMakeLists.txt` | 네 폴더를 `share/bumperbot_description`으로 설치 | colcon, CMake |
| `package.xml` | 패키지 이름·빌드 종류·실행 의존성 | colcon, rosdep |
| `meshes/*.STL` | 3D 모양 | RViz, Gazebo |
| `urdf/bumperbot.urdf.xacro` | 로봇의 링크·조인트 트리 | xacro → `robot_state_publisher`, Gazebo |
| `urdf/bumperbot_gazebo.xacro` | Gazebo 마찰 등 | Gazebo(SDF 변환) |
| `launch/display.launch.py` | RViz로 보기 | `ros2 launch` |
| `launch/gazebo.launch.py` | Gazebo 실행·스폰·시계 | `ros2 launch` |
| `rviz/display.rviz` | RViz 화면 설정 | RViz |

다음 장(03)에서 이 URDF에 `<ros2_control>` 태그와 Gazebo 플러그인을 더하고, `bumperbot_controller` 패키지로 바퀴를 움직인다. 이 장에서 정한 조인트 이름 `wheel_left_joint`, `wheel_right_joint`가 그곳의 기준이 된다.
