# 04. 운동학 맛보기: 두 거북이의 상대 자세 (저장소 Section6)

> 참조: `Section6_Kinematics/bumperbot_ws/src/bumperbot_cpp_examples/` (`include/bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp`, `src/simple_turtlesim_kinematics.cpp`, `CMakeLists.txt`, `package.xml`)

## 이 장에서 하는 일

turtlesim의 두 거북이 자세(pose)를 구독해 "거북이 1에서 거북이 2로 가는 이동 벡터와 회전 행렬"을 계산해 출력하는 노드를 만든다. 로봇의 자세를 **이동(translation)과 회전(rotation)** 으로 다루는 가장 작은 예다. 이 개념이 05장의 차동 구동 운동학과 07장의 오도메트리(`odom → base_footprint` 변환)에서 그대로 쓰인다.

패키지 관점에서는 처음으로 **헤더 파일**을 분리한다. `include/<패키지 이름>/` 폴더에 두는 이유와 CMake에서 그 폴더를 연결하는 방법을 배운다. 이후 장의 `simple_controller`, 하드웨어 인터페이스 플러그인이 모두 `hpp` + `cpp` 구조다.

## 이번 장의 파일

```text
~/bumperbot_ws/src/bumperbot_cpp_examples/
├── CMakeLists.txt                                       [수정]
├── package.xml                                          [수정]
├── include/bumperbot_cpp_examples/
│   └── simple_turtlesim_kinematics.hpp                  [새]
└── src/
    └── simple_turtlesim_kinematics.cpp                  [새]
```

## 단계 1. 헤더와 소스로 나눠 쓴 노드

### 왜 필요한가

노드 클래스가 길어지면 선언(어떤 멤버가 있는가)은 헤더(`.hpp`)에, 구현(무슨 일을 하는가)은 소스(`.cpp`)에 나눠 쓴다. 01장의 노드는 클래스 전체를 `.cpp` 한 파일에 넣었다. 나누면 다음이 달라진다.

- 컴파일러가 `.cpp`에서 `#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"`를 찾아야 한다. 이 헤더가 있는 `include/` 폴더를 컴파일러의 검색 경로에 넣어 줘야 한다. 그 일을 `CMakeLists.txt`의 `include_directories(include)`가 한다. 이 줄이 없으면 `fatal error: bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp: No such file or directory`가 난다.
- 헤더를 `include/<패키지 이름>/`처럼 한 단계 더 깊은 폴더에 두고 `#include "<패키지 이름>/<헤더>"`로 쓰는 것은 ROS 패키지의 관례다. 다른 패키지에 같은 이름의 헤더(`utils.hpp` 등)가 있어도 겹치지 않게 한다(일반 지식).
- 다른 패키지가 이 헤더를 가져다 쓰게 하려면 헤더를 `install/`에 설치해야 한다. 설치만으로는 부족하고, 다른 패키지가 `find_package()`로 이 패키지를 찾았을 때 include 경로를 알 수 있도록 `ament_export_include_directories(include)`(또는 타깃 export) 같은 선언이 더 필요하다(일반 지식이다. 이 패키지에서는 쓰지 않아 시험하지 않았다). 이 패키지의 헤더는 다른 곳에서 쓰지 않지만 참조 저장소가 설치까지 하므로 같이 한다.

### 운동학: 무엇을 계산하는가

두 거북이의 자세를 `(x1, y1, θ1)`, `(x2, y2, θ2)`라 하자(`x`, `y`는 화면 좌표, `θ`는 바라보는 방향의 각도, 라디안). 노드는 다음을 계산해 출력한다.

```text
Tx    = x2 - x1
Ty    = y2 - y1
theta = theta2 - theta1

        | cos(theta)  -sin(theta) |
R   =   | sin(theta)   cos(theta) |
```

`(Tx, Ty)`는 이동 벡터, `R`은 평면 회전 행렬이다. 두 값을 하나로 묶으면 3x3 동차 변환 행렬 `T`가 된다.

```text
        | R11  R12  Tx |
T   =   | R21  R22  Ty |
        |  0    0    1 |
```

이 행렬로 한 좌표계의 점 `(px, py)`를 다른 좌표계로 옮길 수 있다. `[x', y', 1]^T = T * [px, py, 1]^T`다. 07장에서 오도메트리가 `odom` 프레임 안의 로봇 위치를 이런 변환으로 다룬다.

주의할 점이 있다. 코드가 계산하는 `(Tx, Ty)`는 **화면(월드) 좌표의 차이**이고 `R`은 `θ2 - θ1`의 회전이다. 이 둘을 위 모양으로 묶은 `T`는 `θ1 = 0`일 때만 "거북이 1 좌표계에서 거북이 2 좌표계로" 가는 올바른 좌표 변환이다. `θ1 ≠ 0`이면 이동 벡터를 거북이 1 기준 좌표로 먼저 바꿔야 하는데, 그것은 `(Tx, Ty)`를 `-θ1`만큼 돌린 값이다.

```text
(t_x, t_y)^T = R(-theta1) * (Tx, Ty)^T
```

참조 코드는 이 회전을 하지 않고 월드 좌표의 차이를 그대로 출력한다. 아래 시험에서 이 차이를 확인한다.

### 코드

헤더부터 쓴다. 폴더 `include/bumperbot_cpp_examples/`는 01장의 `ros2 pkg create`가 이미 만들었다. 없으면 `mkdir -p ~/bumperbot_ws/src/bumperbot_cpp_examples/include/bumperbot_cpp_examples`로 만든다.

> 파일: `bumperbot_cpp_examples/include/bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp` — 전체
```cpp
#ifndef SIMPLE_TURTLESIM_KINEMATICS_HPP
#define SIMPLE_TURTLESIM_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class SimpleTurtlesimKinematics : public rclcpp::Node
{
public:
    SimpleTurtlesimKinematics(const std::string& name);

    void turtle1PoseCallback(const turtlesim::msg::Pose& pose);

    void turtle2PoseCallback(const turtlesim::msg::Pose& pose);

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;

    turtlesim::msg::Pose last_turtle1_pose_;
    turtlesim::msg::Pose last_turtle2_pose_;
};

#endif // SIMPLE_TURTLESIM_KINEMATICS_HPP
```

소스는 다음과 같다.

> 파일: `bumperbot_cpp_examples/src/simple_turtlesim_kinematics.cpp` — 전체
```cpp
#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"

using std::placeholders::_1;


SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string& name)
    : Node(name)
{
    turtle1_pose_sub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, _1));

    turtle2_pose_sub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle2/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, _1));
}


void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose& pose)
{
    last_turtle1_pose_ = pose;
}


void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose& pose)
{
    last_turtle2_pose_ = pose;
    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;
    float theta_rad = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
    float theta_deg = 180 * theta_rad / 3.14;
    RCLCPP_INFO_STREAM(get_logger(), "\nTranslation Vector turtle1 -> turtle2 \n" <<
                    "Tx: " << Tx << "\n" <<
                    "Ty: " << Ty << "\n" <<
                    "Rotation Matrix turtle1 -> turtle2 \n" << 
                    "theta (rad): " << theta_rad << "\n" <<
                    "theta (deg): " << theta_deg << "\n" <<
                    "|R11   R12|:  |" << std::cos(theta_rad) << "\t" << -std::sin(theta_rad) << "|\n" <<
                    "|R21   R22|   |" << std::sin(theta_rad) << "\t\t" << std::cos(theta_rad) << "|\n");
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 코드 설명

**헤더**

| 부분 | 뜻 |
|---|---|
| `#ifndef ... #define ... #endif` | 인클루드 가드. 같은 헤더가 한 번의 컴파일에서 두 번 읽혀도 클래스가 두 번 선언되지 않게 한다 |
| `#include <turtlesim/msg/pose.hpp>` | turtlesim의 자세 메시지 타입. 필드는 `x`, `y`, `theta`, `linear_velocity`, `angular_velocity`다 |
| `class SimpleTurtlesimKinematics : public rclcpp::Node` | 노드 클래스 선언. 생성자와 콜백 두 개를 공개로 선언한다 |
| `turtle1_pose_sub_`, `turtle2_pose_sub_` | 구독 핸들. 멤버로 두어야 노드가 사는 동안 구독이 유지된다 |
| `last_turtle1_pose_`, `last_turtle2_pose_` | 가장 최근에 받은 자세. 두 토픽이 따로 오므로 하나를 저장해 두고 다른 쪽이 올 때 함께 쓴다 |

**소스**

| 부분 | 뜻 |
|---|---|
| `SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string& name) : Node(name)` | 클래스 밖에서 멤버 함수를 정의할 때는 `클래스이름::함수이름`을 쓴다. 노드 이름을 인자로 받아 `Node`에 넘긴다 |
| `create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(..., this, _1))` | 01장 서브스크라이버와 같은 모양이다. 토픽 이름이 `/`로 시작하는 절대 이름이다 |
| `turtle1PoseCallback` | 받은 자세를 저장만 한다 |
| `turtle2PoseCallback` | 받은 자세를 저장하고, 거북이 1의 마지막 자세와의 차이를 계산해 출력한다. 거북이 2의 자세가 올 때마다(turtlesim은 자세를 계속 내보낸다) 로그가 나온다 |
| `theta_deg = 180 * theta_rad / 3.14` | 라디안을 도로 바꾼다. `π`를 `3.14`로 근사하므로 정확한 각도에서 조금 어긋난 값이 나온다(시험에서 확인한다) |
| `main()`의 `std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics")` | 노드 이름을 인자로 넘긴다 |

### `CMakeLists.txt`, `package.xml` 수정

해야 할 일은 네 가지다. 아래 "수정" 블록의 `# ...` 줄은 "원래 있던 줄을 그대로 둔다"는 뜻이다. 파일에 치지 않는다.

1. `find_package(turtlesim REQUIRED)`: `turtlesim/msg/pose.hpp`를 쓰므로 필요하다.
2. `include_directories(include)`: 헤더 폴더를 컴파일러 검색 경로에 넣는다.
3. 실행 파일 추가와 `install(TARGETS ...)` 목록 추가.
4. `install(DIRECTORY include/ DESTINATION include)`: 헤더를 `install/<패키지>/include/`에 설치한다.

> 파일: `bumperbot_cpp_examples/CMakeLists.txt` — 수정
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)
# ...
add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter rclcpp rcl_interfaces)

add_executable(simple_turtlesim_kinematics src/simple_turtlesim_kinematics.cpp)
ament_target_dependencies(simple_turtlesim_kinematics rclcpp turtlesim)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  simple_publisher
  simple_subscriber
  simple_parameter
  simple_turtlesim_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

`package.xml`에는 `<depend>turtlesim</depend>`를 넣는다.

> 파일: `bumperbot_cpp_examples/package.xml` — 수정
```xml
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>rcl_interfaces</depend>
  <depend>turtlesim</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
```

`include_directories(include)`는 이 `CMakeLists.txt`의 **모든 실행 파일**에 적용된다. 한 실행 파일에만 적용하려면 `target_include_directories(simple_turtlesim_kinematics PRIVATE include)`를 쓴다. 헤더를 이 실행 파일 안에서만 쓰므로 `PRIVATE`이 맞다. 이렇게 바꿔도 빌드되는 것을 확인했다. 이 패키지는 헤더를 쓰는 실행 파일이 하나뿐이어서 참조 저장소처럼 `include_directories(include)`를 쓴다.

`install(DIRECTORY include/ DESTINATION include)`의 `include/` 끝 슬래시가 의미를 가진다. 끝에 `/`가 있으면 `include` 폴더의 **내용**(`bumperbot_cpp_examples/` 폴더)이 `install/<패키지>/include/` 아래로 복사된다. 참조 코드처럼 `/`를 빼면 `include` 폴더 자체가 복사되어 `install/<패키지>/include/include/bumperbot_cpp_examples/...`처럼 `include`가 두 번 겹친다(아래 "참조 코드와 다른 점").

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
```

```text
Starting >>> bumperbot_cpp_examples
Finished <<< bumperbot_cpp_examples [3.75s]

Summary: 1 package finished [3.82s]
```

이 장까지 따라온 작업 공간에는 다른 패키지(`bumperbot_py_examples` 등)도 있어서 함께 빌드된다. 그 패키지의 `Starting`, `Finished` 줄도 나오고 `N packages finished`의 N도 달라진다. `bumperbot_cpp_examples`가 `Finished`이고 `Summary`에 실패가 없으면 된다.

설치된 헤더 위치를 본다.

```bash
find ~/bumperbot_ws/install/bumperbot_cpp_examples/include
```

```text
.../install/bumperbot_cpp_examples/include
.../install/bumperbot_cpp_examples/include/bumperbot_cpp_examples
.../install/bumperbot_cpp_examples/include/bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp
```

이제 노드를 시험한다. 터미널 셋을 쓴다. 각 터미널을 처음 열 때 `source /opt/ros/jazzy/setup.bash`와 `source ~/bumperbot_ws/install/setup.bash`를 먼저 한다.

터미널 A: turtlesim을 띄운다. 거북이 1이 있는 창이 뜬다. 끝나지 않는 명령이다.

```bash
ros2 run turtlesim turtlesim_node
```

터미널 B: 노드를 띄운다. 끝나지 않는 명령이다. 아직 거북이 2가 없으므로 출력이 없다.

```bash
ros2 run bumperbot_cpp_examples simple_turtlesim_kinematics
```

터미널 C: 거북이 2를 만든다. `/spawn`은 turtlesim이 제공하는 서비스다(서비스는 06장에서 직접 만든다).

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

```text
response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

거북이 1은 시작할 때 `(5.544, 5.544)`에서 `θ = 0`이다. 터미널 B에 다음이 계속 나온다. turtlesim이 `/turtle2/pose`를 빠르게 계속 내보내므로 같은 내용이 매우 빠르게 반복해서 올라간다.

```text
[INFO] [1795239146.216950098] [simple_turtlesim_kinematics]:
Translation Vector turtle1 -> turtle2
Tx: -3.54444
Ty: -3.54444
Rotation Matrix turtle1 -> turtle2
theta (rad): 0
theta (deg): 0
|R11   R12|:  |1	-0|
|R21   R22|   |0		1|
```

`Tx = 2.0 - 5.544 ≈ -3.544`다. 회전이 없으므로 `R`은 단위 행렬이다. 거북이 1이 바라보는 방향이 월드 x축과 같을 때(`θ1 = 0`)는 `(Tx, Ty)`를 거북이 1 기준 좌표로 읽어도 맞다.

회전이 있을 때 월드 좌표 차이와 거북이 1 기준 좌표의 차이가 갈린다. 터미널 C에서 두 거북이를 순간이동시킨다. 바로 끝나는 명령이다.

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.0, y: 5.0, theta: 1.5708}"
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 8.0, y: 5.0, theta: 0.0}"
```

```text
Translation Vector turtle1 -> turtle2
Tx: 3
Ty: 0
Rotation Matrix turtle1 -> turtle2
theta (rad): -1.5708
theta (deg): -90.0459
|R11   R12|:  |-3.61999e-06	1|
|R21   R22|   |-1		-3.61999e-06|
```

거북이 1은 `(5, 5)`에서 위(`+y`)를 보고, 거북이 2는 `(8, 5)`에서 오른쪽(`+x`)을 본다. 출력은 `Tx = 3`, `Ty = 0`이다. 그러나 거북이 1 입장에서 거북이 2는 **자기 오른쪽 3만큼**(거북이 1 기준 좌표로 `(0, -3)`)에 있다. 식으로 쓰면 `R(-90°) * (3, 0)^T = (0, -3)^T`다. 코드가 낸 `(3, 0)`은 월드 좌표의 차이이고, 이 값을 거북이 1 기준으로 바꾸려면 `-θ1 = -90°` 회전이 더 필요하다. 이 장의 코드는 그 변환을 하지 않는다. 같은 일을 TF 라이브러리가 해 주는 것을 06장에서 본다.

통과 조건: 거북이 2를 만들면 노드가 `Tx`, `Ty`, `theta`, 회전 행렬을 출력한다. `theta (deg)`가 `-90`이 아니라 `-90.0459`로 나오는 것은 `3.14` 근사 때문이다(`180 * 1.5708 / 3.14 ≈ 90.046`). 회전 행렬의 `-3.61999e-06`은 `cos(π/2)`가 0이 아닌 것이 아니라 `1.5708`이 `π/2`의 반올림 값이어서 생긴 오차다.

끝낼 때는 터미널 B의 노드를 Ctrl+C로 끄고, 터미널 A의 turtlesim도 Ctrl+C로 끈다(창이 함께 닫힌다).

Python판의 참조 위치: `Section6_Kinematics/bumperbot_ws/src/bumperbot_py_examples/bumperbot_py_examples/simple_turtlesim_kinematics.py`

## 참조 코드와 다른 점

| 참조 | 이 가이드 | 이유 |
|---|---|---|
| `install(DIRECTORY include DESTINATION include)` | `install(DIRECTORY include/ DESTINATION include)` | 참조대로 하면 `install/<패키지>/include/include/<패키지>/...`로 `include`가 두 번 겹쳐 설치된다. 끝에 `/`를 붙이면 `install/<패키지>/include/<패키지>/...`가 된다. 이 패키지의 헤더는 아직 다른 패키지가 쓰지 않으므로 어느 쪽이든 동작에는 영향이 없다 |
| 소스·헤더 코드 | 같다 | 없음 |

## 자주 나는 오류

| 증상 | 원인 | 고치는 법 |
|---|---|---|
| `fatal error: bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp: No such file or directory` | `include_directories(include)`가 없거나, 헤더가 `include/bumperbot_cpp_examples/` 안이 아닌 곳에 있다 | `CMakeLists.txt`에 줄을 넣고 헤더 경로를 확인한다 |
| `CMake Error ... ament_target_dependencies() the passed package name 'turtlesim' was not found before` | `find_package(turtlesim REQUIRED)`를 빼먹었다. `ament_target_dependencies`보다 앞에 있어야 한다 | `find_package` 줄을 `ament_target_dependencies` 줄보다 위에 둔다 |
| 노드를 띄웠는데 출력이 없다 | 거북이 2가 없다. 출력은 `/turtle2/pose`를 받을 때만 나온다 | `/spawn` 서비스로 `turtle2`를 만든다 |

## 정리

| 파일 | 역할 |
|---|---|
| `include/bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp` | 노드 클래스 선언 |
| `src/simple_turtlesim_kinematics.cpp` | 두 거북이 자세의 차이로 이동 벡터와 회전 행렬을 계산한다 |
| `CMakeLists.txt` | `turtlesim` 의존, `include_directories(include)`, 헤더 설치, 실행 파일 추가 |
| `package.xml` | `<depend>turtlesim</depend>` 추가 |

이 장에서 익힌 "이동 벡터 + 회전 행렬 = 좌표 변환"은 06장의 TF2에서 `tf2::Quaternion`과 `TransformStamped`로 다시 나온다. 다음 05장에서는 이 운동학을 바퀴 로봇에 적용해 속도 명령을 바퀴 속도로 바꾸는 `simple_controller`를 만든다. `bumperbot_cpp_examples` 패키지는 06장(서비스·TF)과 10장(라이프사이클 노드)에서 다시 자란다.
