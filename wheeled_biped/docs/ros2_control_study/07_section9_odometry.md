# 07. 오도메트리 (저장소 Section9)

> 참조: `Section9_Odometry/bumperbot_ws/src/bumperbot_controller/`, `Section9_Odometry/bumperbot_ws/src/bumperbot_description/urdf/bumperbot_ros2_control.xacro` (참조 저장소 최상위 기준 경로)

## 이 장에서 하는 일

- 바퀴의 회전각(`/joint_states`의 `position`)에서 로봇의 위치·방향을 계산하는 **오도메트리**를 `simple_controller`에 더한다.
- 결과를 `nav_msgs/msg/Odometry` 메시지(`/bumperbot_controller/odom`)와 TF(`odom → base_footprint`)로 낸다.
- Gazebo의 실제 위치와 비교하고, 표준 `diff_drive_controller`가 내는 오도메트리와도 비교한다.
- ros2_control 전체에서의 자리: 05장은 "명령 → 바퀴"였다. 이 장은 반대로 "바퀴 상태(state interface, `/joint_states`) → 로봇의 상태"다. 제어기가 읽는 state interface가 어떻게 쓰이는지 보여 주는 예이고, 09장(센서 융합)의 입력이 된다.

## 이번 장의 파일

```text
~/bumperbot_ws/src/
├── bumperbot_controller/
│   ├── CMakeLists.txt                              [수정] sensor_msgs, nav_msgs, tf2, tf2_ros
│   ├── package.xml                                 [수정]
│   ├── include/bumperbot_controller/simple_controller.hpp   [수정]
│   └── src/simple_controller.cpp                   [수정]
└── bumperbot_description/urdf/
    └── bumperbot_ros2_control.xacro                [수정] 조인트 선언 순서
```

## 단계 1. 식

### 왜 필요한가

엔코더(여기서는 `/joint_states`의 바퀴 `position`)는 바퀴가 **얼마나 돌았는지**만 알려 준다. 이것을 로봇이 **어디에 있는지**로 바꾸는 계산을 오도메트리라고 한다.

### 식

05장의 식을 거꾸로 쓴다. 바퀴의 회전각이 한 주기 사이에 `Δφ_R`, `Δφ_L`(rad)만큼 늘었다고 하자. 기호는 05장과 같다(`r = 0.033`, `L = 0.17`).

```text
Δs = r/2 · (Δφ_R + Δφ_L)      로봇이 앞으로 간 거리 (m)
Δθ = r/L · (Δφ_R − Δφ_L)      로봇이 돈 각도 (rad)
```

속도는 같은 식을 시간 `Δt`로 나눈 것이다.

```text
v = Δs / Δt        w = Δθ / Δt
```

위치와 방향은 이 증분을 누적한다. 방향 `θ`를 먼저 갱신하고 그 방향으로 거리 `Δs`만큼 간다.

```text
θ ← θ + Δθ
x ← x + Δs · cos θ
y ← y + Δs · sin θ
```

`(x, y, θ)`는 로봇이 처음 놓인 자리를 원점으로 하는 좌표계(`odom` 프레임)에서의 `base_footprint`의 위치다. 바퀴가 미끄러지거나 `r`, `L`이 실제와 다르면 오차가 계속 쌓인다. 이 오차가 08장에서 만드는 노이즈 제어기와 09장의 센서 융합의 주제다.

## 단계 2. `/joint_states`를 받아 위치 계산하기

### 왜 필요한가

`simple_controller`가 `/joint_states`를 구독하고 위 식을 계산하게 한다. 우선은 계산 결과를 로그로만 확인한다.

### 코드

헤더, 구현, 의존성을 모두 바꾼다. 이 단계가 끝난 시점의 파일 전체는 아래와 같다. 05장과 비교해서 달라진 줄은 아래 "코드 설명"에서 짚는다. 헤더의 `#include`와 멤버, 구현의 생성자와 `jointCallback`이 새로 생긴 부분이다. 구현 맨 위의 `#include <Eigen/Geometry>`는 05장에서 설명한 것과 같다.

> 파일: `bumperbot_controller/include/bumperbot_controller/simple_controller.hpp` — 전체 (단계 2 시점)
```cpp
#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string& name);

private:
    void velCallback(const geometry_msgs::msg::TwistStamped &msg);

    void jointCallback(const sensor_msgs::msg::JointState &msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    // Odometry
    double wheel_radius_;
    double wheel_separation_;
    Eigen::Matrix2d speed_conversion_;
    double right_wheel_prev_pos_;
    double left_wheel_prev_pos_;
    rclcpp::Time prev_time_;
    double x_;
    double y_;
    double theta_;
};

#endif // SIMPLE_CONTROLLER_HPP
```

> 파일: `bumperbot_controller/src/simple_controller.cpp` — 전체 (단계 2 시점)
```cpp
#include "bumperbot_controller/simple_controller.hpp"
#include <Eigen/Geometry>


using std::placeholders::_1;


SimpleController::SimpleController(const std::string& name)
                                  : Node(name)
                                  , right_wheel_prev_pos_(0.0)
                                  , left_wheel_prev_pos_(0.0)
                                  , x_(0.0)
                                  , y_(0.0)
                                  , theta_(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10, std::bind(&SimpleController::velCallback, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1));

    speed_conversion_ << wheel_radius_/2, wheel_radius_/2, wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;
    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion_);

    prev_time_ = get_clock()->now();
}


void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // Implements the differential kinematic model
    // Given v and w, calculate the velocities of the wheels
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));
    
    wheel_cmd_pub_->publish(wheel_speed_msg);
}


void SimpleController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    // Implements the inverse differential kinematic model
    // Given the position of the wheels, calculates their velocities
    // then calculates the velocity of the robot wrt the robot frame
    // and then converts it in the global frame and publishes the TF
    double dp_left = state.position.at(0) - left_wheel_prev_pos_;
    double dp_right = state.position.at(1) - right_wheel_prev_pos_;
    rclcpp::Time msg_time = state.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    // Actualize the prev pose for the next itheration
    left_wheel_prev_pos_ = state.position.at(0);
    right_wheel_prev_pos_ = state.position.at(1);
    prev_time_ = state.header.stamp;

    // Calculate the rotational speed of each wheel
    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();

    // Calculate the linear and angular velocity
    double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
    double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    // Calculate the position increment
    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;
    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "x=%.3f y=%.3f theta=%.3f linear=%.3f angular=%.3f",
                         x_, y_, theta_, linear, angular);
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleController>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

`CMakeLists.txt`에 `sensor_msgs`를 더한다.

> 파일: `bumperbot_controller/CMakeLists.txt` — 수정 (단계 2 시점)
```cmake
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# ...
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs sensor_msgs ${Eigen_LIBRARIES})
```

`package.xml`에도 더한다.

> 파일: `bumperbot_controller/package.xml` — 수정 (단계 2 시점)
```xml
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>eigen</depend>
```

### 코드 설명

헤더에서 새로 생긴 것:

- `#include <sensor_msgs/msg/joint_state.hpp>`: `/joint_states`의 메시지 `sensor_msgs/msg/JointState`. `header`, `name`(조인트 이름 목록), `position`, `velocity`, `effort`(각각 `name`과 같은 순서의 숫자 목록)를 가진다.
- `void jointCallback(const sensor_msgs::msg::JointState &msg);` 와 `joint_sub_`: 구독 함수와 구독자.
- `right_wheel_prev_pos_`, `left_wheel_prev_pos_`: 이전 주기의 바퀴 회전각. 이번 값과의 차이가 `Δφ`다.
- `rclcpp::Time prev_time_;`: 이전 메시지의 시각. `rclcpp::Time`은 ROS의 시각 값이다.
- `x_`, `y_`, `theta_`: 누적한 위치와 방향.

구현에서 새로 생긴 것:

- 생성자 초기화 목록 `, right_wheel_prev_pos_(0.0), left_wheel_prev_pos_(0.0), x_(0.0), ... theta_(0.0)`: 멤버 변수를 0으로 시작한다. 초기화하지 않으면 값이 불확정이다. **C++에서 멤버는 헤더에 선언한 순서대로 초기화된다.** 초기화 목록에 쓴 순서와는 상관이 없다. 두 순서가 다르면 컴파일러가 `-Wreorder` 경고를 낸다. 참조의 생성자는 목록을 `left_wheel_prev_pos_`, `right_wheel_prev_pos_` 순서로 썼는데 헤더의 선언은 `right_wheel_prev_pos_`, `left_wheel_prev_pos_` 순서다. 경고 옵션(`-Wall -Wextra`)을 켜고 빌드하면 `warning: ‘SimpleController::left_wheel_prev_pos_’ will be initialized after [-Wreorder]`가 나온다(이 가이드를 쓸 때 재현했다). 이 가이드는 **목록의 순서를 선언 순서에 맞게 고쳐서** 경고를 없앴다.
- `joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1));`: 05장의 `vel_sub_`와 같은 형식이다. 토픽은 `/joint_states`다.
- `prev_time_ = get_clock()->now();`: 노드의 시계로 현재 시각을 얻는다. `use_sim_time`이 켜져 있으면 Gazebo의 시뮬레이션 시각이다. 노드의 시계는 `/clock`을 받기 전에는 0을 줄 수 있다.
- `jointCallback`:
  - `state.position.at(0)`, `state.position.at(1)`: 왼쪽·오른쪽 바퀴의 회전각이다. **`position`의 0번이 왼쪽, 1번이 오른쪽이라고 가정한 코드다.** `/joint_states`의 `name` 순서가 `[wheel_left_joint, wheel_right_joint]`여야 맞다(단계 6에서 확인한다). `.at(i)`는 범위를 벗어나면 예외를 던지는 안전한 접근이다.
  - `dp_left`, `dp_right`: 이전 값과의 차이(`Δφ`). 계산한 뒤 이전 값을 갱신한다.
  - `rclcpp::Time msg_time = state.header.stamp; rclcpp::Duration dt = msg_time - prev_time_;`: 메시지 시각의 차이로 `Δt`를 구한다. 호출된 시각(`now()`)이 아니라 **메시지에 찍힌 시각**을 쓰므로 콜백이 늦게 불려도 속도가 틀어지지 않는다. `dt.seconds()`는 초 단위 실수다. (단계 3·4에서 `Odometry`와 TF의 `header.stamp`는 `dt`와 달리 `get_clock()->now()`를 **각각 따로 불러** 채운다. 그래서 오도메트리 메시지와 TF의 시각이 서로 아주 조금 다를 수 있다. 참조 그대로다.)
  - `fi_left = dp_left / dt.seconds()`: 바퀴 각속도(`φ̇`). 이어서 `linear`, `angular`(식의 `v`, `w`)를 구한다.
  - `d_s`, `d_theta`, `theta_ += d_theta; x_ += d_s * cos(theta_); y_ += d_s * sin(theta_);`: 식의 누적 부분이다.
  - 콜백 맨 위의 참조 주석은 `inverse differential kinematic model`이라 하고 끝에는 `publishes the TF`라고 적혀 있다. 이 계산은 바퀴 회전에서 로봇의 움직임을 구하는 방향이라 보통 **정기구학**(forward kinematics)이라 부르고(일반 지식), 05장의 식(로봇 속도에서 바퀴 속도를 구함)이 역기구학이다. 또 이 단계에는 TF를 내는 코드가 아직 없다(단계 4에서 생긴다). 코드는 참조 그대로 두고 주석은 따르지 않는다.
  - `RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "...", ...)`: 시험용 임시 로그다. 마지막 인자들이 `printf` 형식으로 들어가고, `1000`(ms)마다 최대 한 번만 출력한다(로그가 쏟아지지 않게 한다). 다음 단계에서 지운다.

`CMakeLists.txt`와 `package.xml`: 새로 쓴 `sensor_msgs`를 `find_package`, `ament_target_dependencies`, `<depend>`에 더한다. 메시지 패키지를 하나 더 쓸 때마다 같은 세 곳을 고친다. 위 블록에서 `find_package(std_msgs ...)`부터 `find_package(Eigen3 ...)`까지의 사이에서 새로 넣는 줄은 `find_package(sensor_msgs REQUIRED)` 하나이고, `ament_target_dependencies` 줄에서는 `sensor_msgs`가 새로 넣는 단어다. `package.xml`에서는 `<depend>sensor_msgs</depend>` 한 줄이 새로 넣는 줄이다. 블록의 `...` 줄은 "그 사이의 원래 있던 줄을 그대로 둔다"는 뜻이고 파일에 치지 않는다.

### 빌드와 시험

터미널 A에서 빌드하고 Gazebo를 띄운다(끝나지 않는다. 이 단계의 시험이 끝날 때까지 켜 둔다).

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bumperbot_description gazebo.launch.py
```

기대 결과(빌드. 이 패키지의 줄만):

```text
Finished <<< bumperbot_controller [5.17s]
```

작업 공간에 앞 장의 패키지도 있으면 그 패키지의 `Finished` 줄도 함께 나온다. `Summary`의 패키지 수는 사용자마다 다르므로 `Failed`가 없고 경고가 없으면 된다.

터미널 B에서 제어기 launch를 띄운다(끝나지 않는다). 임시 로그가 이 터미널에 1초에 한 줄씩 나온다.

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 launch bumperbot_controller controller.launch.py
```

터미널 C에서 직진한 뒤 회전한다. 첫 명령은 끝나지 않으므로 5초쯤 뒤 `Ctrl+C`로 끈다. 구성 A는 마지막 속도를 유지하므로 `Ctrl+C`로 끈 뒤에도 로봇은 계속 간다. 바로 이어서 회전 명령을 보내 속도를 바꾸고, 3초 뒤 멈춘다.

```bash
source ~/bumperbot_ws/install/setup.bash
gz model -m bumperbot -p
ros2 topic pub -r 20 /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

5초쯤 뒤 `Ctrl+C`로 끝내고, 이어서 터미널 C에서:

```bash
ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"
```

3초쯤 기다린 뒤:

```bash
ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
gz model -m bumperbot -p
```

기대 결과(터미널 B의 로그. 시각은 다르고 값은 명령을 보낸 시간에 따라 다르다. 아래는 이 가이드를 쓸 때의 값이다):

```text
[simple_controller-3] [INFO] [...] [simple_controller]: x=-0.000 y=0.000 theta=-0.000 linear=-0.000 angular=-0.000
[simple_controller-3] [INFO] [...] [simple_controller]: x=0.001 y=-0.000 theta=-0.000 linear=0.111 angular=-0.000
[simple_controller-3] [INFO] [...] [simple_controller]: x=0.101 y=-0.000 theta=0.000 linear=0.100 angular=0.000
[simple_controller-3] [INFO] [...] [simple_controller]: x=0.201 y=0.000 theta=0.000 linear=0.100 angular=0.000
...
[simple_controller-3] [INFO] [...] [simple_controller]: x=0.572 y=0.000 theta=0.650 linear=-0.000 angular=0.500
[simple_controller-3] [INFO] [...] [simple_controller]: x=0.572 y=0.000 theta=1.990 linear=0.000 angular=0.000
```

터미널 C에서 `gz model`의 첫 출력은 움직이기 전, 두 번째 출력은 멈춘 뒤다.

```text
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [0.000004 0.000000 0.000002]
    [0.000000 0.010548 -0.000001]
```

```text
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [0.572013 -0.000322 0.000002]
    [-0.000000 0.010548 1.982920]
```

- 통과 조건 1: 정지 상태에서 `x`, `y`, `theta`가 0이다(움직이지 않는다).
- 통과 조건 2: `v = 0.1`을 명령하면 `linear`가 `0.100` 근처이고 `x`가 초당 약 0.1씩 늘며, `w = 0.5`를 명령하면 `angular`가 `0.500`이고 `theta`가 초당 약 0.5씩 늘어난다.
- 통과 조건 3: 멈춘 뒤 로그의 `x`, `theta`가 Gazebo가 알려 주는 실제 위치와 1% 안쪽으로 같다. 위 시험에서 로그 `x = 0.572`, `theta = 1.990`, Gazebo `x = 0.572013`(`XYZ`의 첫 값), `yaw = 1.982920`(`RPY`의 마지막 값)이었다.
- 이 시험은 창 없이 서버만 띄운 Gazebo에서 했다. 보낸 시간이 달라서 값은 사용자마다 다르다. 비교할 것은 `simple_controller`의 로그와 Gazebo의 위치가 같은지다.
- 터미널 A와 B를 `Ctrl+C`로 끝낸다. 같은 노드를 다시 띄우기 전에 앞의 것이 끝났는지 확인한다.

## 단계 3. `nav_msgs/Odometry` 메시지 발행

### 왜 필요한가

로그로만 있는 위치는 다른 노드가 쓸 수 없다. 09장의 센서 융합이나 RViz가 읽을 수 있도록 표준 메시지 `nav_msgs/msg/Odometry`로 발행한다. 이 메시지는 `header.frame_id`(기준 프레임), `child_frame_id`(움직이는 프레임), `pose`(위치+방향+공분산), `twist`(선속도·각속도+공분산)로 되어 있다.

### 코드

헤더에서: 단계 2의 `#include <sensor_msgs/...>` 아래에 `nav_msgs`를 더하고, 구독자 `joint_sub_` 아래에 발행자 `odom_pub_`를, `prev_time_` 아래에 메시지 멤버 `odom_msg_`를 더한다. 아래 블록에서 새로 넣는 줄은 `#include <nav_msgs/msg/odometry.hpp>`, `rclcpp::Publisher<nav_msgs::...>::SharedPtr odom_pub_;`, `nav_msgs::msg::Odometry odom_msg_;` 세 줄이다(나머지는 단계 2에서 이미 있는 줄이고, `...` 줄은 파일에 치지 않는다).

> 파일: `bumperbot_controller/include/bumperbot_controller/simple_controller.hpp` — 수정 (단계 3 시점)
```cpp
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
// ...
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Odometry
    double wheel_radius_;
    double wheel_separation_;
    Eigen::Matrix2d speed_conversion_;
    double right_wheel_prev_pos_;
    double left_wheel_prev_pos_;
    rclcpp::Time prev_time_;
    nav_msgs::msg::Odometry odom_msg_;
```

구현에서: 맨 위 `#include`, 생성자, 콜백의 끝을 바꾼다. 생성자에서는 `joint_sub_ = create_subscription...` 줄 바로 아래에 `odom_pub_ = create_publisher...` 줄을 넣고, 05장의 `RCLCPP_INFO_STREAM(... conversion matrix ...)` 줄 아래, `prev_time_ = get_clock()->now();` 줄 위에 `// Fill the Odometry message ...` 블록을 넣는다. 콜백에서는 단계 2의 `RCLCPP_INFO_THROTTLE` 임시 로그 세 줄을 지우고 `// Compose and publish the odom message` 블록으로 바꾼다.

> 파일: `bumperbot_controller/src/simple_controller.cpp` — 수정 (단계 3 시점)
```cpp
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
// ...
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10);
// ...
    // Fill the Odometry message with invariant parameters
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;
// ...
    // Compose and publish the odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;
    odom_pub_->publish(odom_msg_);
```

`CMakeLists.txt`와 `package.xml`에 `nav_msgs`와 `tf2`(쿼터니언 계산용)를 더한다.

> 파일: `bumperbot_controller/CMakeLists.txt` — 수정 (단계 3 시점)
```cmake
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
# ...
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2 ${Eigen_LIBRARIES})
```

> 파일: `bumperbot_controller/package.xml` — 수정 (단계 3 시점)
```xml
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>eigen</depend>
```

### 코드 설명

- `odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10);`: 토픽 이름이 표준 제어기(`bumperbot_controller`)의 `~/odom`과 같다. 그래서 오도메트리를 읽는 쪽은 어느 구성을 쓰는지 몰라도 된다.
- 생성자의 `odom_msg_.header.frame_id = "odom"; odom_msg_.child_frame_id = "base_footprint";`: 이 메시지의 위치는 `odom` 프레임에서 본 `base_footprint`의 위치라는 뜻이다. 매번 바뀌지 않는 값은 생성자에서 한 번만 채운다. `orientation.w = 1.0`은 회전 없음(단위 쿼터니언)이다.
- 콜백의 끝:
  - `tf2::Quaternion q; q.setRPY(0, 0, theta_);`: 방향은 메시지에서 오일러각이 아니라 **쿼터니언**(`x, y, z, w`)으로 나타낸다. 평면 로봇은 `yaw`만 있으므로 `setRPY(roll=0, pitch=0, yaw=theta_)`로 만든다. 결과는 `(0, 0, sin(θ/2), cos(θ/2))`다.
  - `odom_msg_.header.stamp = get_clock()->now();`: 발행 시각.
  - `pose.pose.position.x/y`, `orientation`: 계산한 위치와 방향. `position.z`는 대입하지 않으므로 기본값 0이다. `orientation`은 생성자에서 `w = 1.0`으로 한 번 채웠지만 콜백마다 새 쿼터니언으로 **덮어쓴다**.
  - `twist.twist.linear.x = linear; ...angular.z = angular;`: 계산한 속도. 로봇 자신의 좌표계(앞으로·회전) 기준이다.
  - `odom_pub_->publish(odom_msg_);`: 매 `/joint_states`(약 100 Hz)마다 한 번 발행한다.
- 공분산(`pose.covariance`, `twist.covariance`)은 채우지 않아 모두 0이다. 센서 융합을 하려면 의미 있는 값이 필요하다(09장). 표준 `diff_drive_controller`는 yaml의 `pose_covariance_diagonal`로 채운다.
- `#include <tf2/LinearMath/Quaternion.h>`: 쿼터니언 계산 클래스. `tf2` 패키지에 있다. 그래서 `find_package(tf2 REQUIRED)`와 `ament_target_dependencies`의 `tf2`가 필요하다.

### 빌드와 시험

단계 2와 같은 세 터미널로 한다. 먼저 이전 단계의 launch를 모두 `Ctrl+C`로 끝낸다.

터미널 A(끝나지 않는다):

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bumperbot_description gazebo.launch.py
```

터미널 B(끝나지 않는다):

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 launch bumperbot_controller controller.launch.py
```

터미널 C에서 토픽부터 확인한다.

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 topic list | grep odom
ros2 topic info /bumperbot_controller/odom
ros2 topic echo /bumperbot_controller/odom --once --no-arr
```

```text
/bumperbot_controller/odom
```

```text
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 0
```

```text
header:
  stamp:
    sec: 16
    nanosec: 870000000
  frame_id: odom
child_frame_id: base_footprint
pose:
  pose:
    position:
      x: -1.4598544060727747e-13
      ...
    orientation:
      ...
      w: 1.0
  covariance: '<array type: double[36]>'
twist:
  ...
```

이어서 직진하고 멈춘 뒤 위치를 비교한다. 구성 A는 마지막 속도를 유지하므로 `0.0` 명령까지 보낸다.

```bash
gz model -m bumperbot -p
ros2 topic pub -r 20 /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

5초쯤 뒤 `Ctrl+C`로 끝내고 이어서(회전을 3초 보낸 뒤 멈춘다):

```bash
ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"
```

3초쯤 기다린 뒤:

```bash
ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
ros2 topic echo /bumperbot_controller/odom --once --field pose.pose
gz model -m bumperbot -p
```

기대 결과(값은 명령을 보낸 시간에 따라 다르다. 아래는 이 가이드를 쓸 때의 값이다):

```text
position:
  x: 0.5799999999984358
  y: 1.1152536231749354e-12
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.8277018881656836
  w: 0.561168053551663
---
```

```text
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [0.580000 -0.000327 0.000002]
    [-0.000000 0.010548 1.942970]
```

계산: 오도메트리의 `x`는 `0.5800`, Gazebo의 `x`는 `0.580000`이다. 방향은 쿼터니언 `(z, w) = (0.8277, 0.5612)`에서 평면 로봇의 `yaw = 2 · atan2(z, w) = 2 · atan2(0.8277, 0.5612) = 1.943` rad이고, Gazebo의 `yaw`(`RPY`의 마지막 값)는 `1.942970`이다.

- 통과 조건 1: `Type: nav_msgs/msg/Odometry`, `frame_id: odom`, `child_frame_id: base_footprint`.
- 통과 조건 2: 움직인 뒤의 `pose.pose.position.x`와 `yaw`가 Gazebo가 알려 주는 값과 1% 안쪽으로 같다. 값 자체는 사용자마다 다르다.
- 이 단계에서는 TF가 아직 없다. 터미널 C에서 `ros2 run tf2_ros tf2_echo odom base_footprint`를 치면 `Invalid frame ID "odom" ... frame does not exist`가 반복된다(확인했다). `Ctrl+C`로 끝낸다.
- 터미널 A와 B를 `Ctrl+C`로 끝낸다.

## 단계 4. TF 발행: `odom → base_footprint`

### 왜 필요한가

ROS의 위치 정보는 두 곳에 있다. 하나는 `Odometry` 메시지, 다른 하나는 **TF**(프레임 사이의 변환을 `/tf` 토픽으로 퍼뜨리는 체계)다. RViz나 내비게이션, `robot_localization` 같은 도구는 "`odom` 프레임에서 `base_footprint`까지의 변환"을 TF에서 찾는다. `Odometry` 메시지만 내면 이 변환이 없다. 02장의 `robot_state_publisher`는 `base_footprint → base_link → wheel_*_link`를 내므로, 이 장의 TF까지 이으면 `odom → base_footprint → base_link → ...`의 한 나무가 된다.

### 코드

헤더에서:

> 파일: `bumperbot_controller/include/bumperbot_controller/simple_controller.hpp` — 수정
```cpp
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
// ...
    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
```

구현에서(생성자와 콜백 끝):

> 파일: `bumperbot_controller/src/simple_controller.cpp` — 수정
```cpp
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    prev_time_ = get_clock()->now();
// ...
    // TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();
    transform_broadcaster_->sendTransform(transform_stamped_);
```

`CMakeLists.txt`와 `package.xml`에 `tf2_ros`를 더한다.

> 파일: `bumperbot_controller/CMakeLists.txt` — 수정
```cmake
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
# ...
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})
```

> 파일: `bumperbot_controller/package.xml` — 수정
```xml
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2</depend>
  <depend>eigen</depend>
```

이 단계가 끝난 시점의 헤더와 구현 전체는 아래와 같다(Section9 참조 파일과 같다).

> 파일: `bumperbot_controller/include/bumperbot_controller/simple_controller.hpp` — 전체
```cpp
#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string& name);

private:
    void velCallback(const geometry_msgs::msg::TwistStamped &msg);

    void jointCallback(const sensor_msgs::msg::JointState &msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Odometry
    double wheel_radius_;
    double wheel_separation_;
    Eigen::Matrix2d speed_conversion_;
    double right_wheel_prev_pos_;
    double left_wheel_prev_pos_;
    rclcpp::Time prev_time_;
    nav_msgs::msg::Odometry odom_msg_;
    double x_;
    double y_;
    double theta_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif // SIMPLE_CONTROLLER_HPP
```

> 파일: `bumperbot_controller/src/simple_controller.cpp` — 전체
```cpp
#include "bumperbot_controller/simple_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>


using std::placeholders::_1;


SimpleController::SimpleController(const std::string& name)
                                  : Node(name)
                                  , right_wheel_prev_pos_(0.0)
                                  , left_wheel_prev_pos_(0.0)
                                  , x_(0.0)
                                  , y_(0.0)
                                  , theta_(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10, std::bind(&SimpleController::velCallback, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10);

    speed_conversion_ << wheel_radius_/2, wheel_radius_/2, wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;
    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion_);

    // Fill the Odometry message with invariant parameters
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    prev_time_ = get_clock()->now();
}


void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // Implements the differential kinematic model
    // Given v and w, calculate the velocities of the wheels
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));
    
    wheel_cmd_pub_->publish(wheel_speed_msg);
}


void SimpleController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    // Implements the inverse differential kinematic model
    // Given the position of the wheels, calculates their velocities
    // then calculates the velocity of the robot wrt the robot frame
    // and then converts it in the global frame and publishes the TF
    double dp_left = state.position.at(0) - left_wheel_prev_pos_;
    double dp_right = state.position.at(1) - right_wheel_prev_pos_;
    rclcpp::Time msg_time = state.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    // Actualize the prev pose for the next itheration
    left_wheel_prev_pos_ = state.position.at(0);
    right_wheel_prev_pos_ = state.position.at(1);
    prev_time_ = state.header.stamp;

    // Calculate the rotational speed of each wheel
    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();

    // Calculate the linear and angular velocity
    double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
    double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    // Calculate the position increment
    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;
    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    // Compose and publish the odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;
    odom_pub_->publish(odom_msg_);

    // TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();
    transform_broadcaster_->sendTransform(transform_stamped_);
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleController>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 코드 설명

- `#include <tf2_ros/transform_broadcaster.h>`: TF를 `/tf` 토픽으로 내보내는 클래스 `tf2_ros::TransformBroadcaster`.
- `std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;`: `unique_ptr`은 한 곳만 소유하는 스마트 포인터다. 객체는 생성자에서 `std::make_unique<tf2_ros::TransformBroadcaster>(*this)`로 만든다. 인자 `*this`는 이 노드 자신이다. 브로드캐스터는 만들 때 노드를 받아 그 노드를 통해 `/tf` 발행자를 만든다. 이렇게 `unique_ptr`로 두고 생성자 본문에서 나중에 만드는 것은 참조의 선택이다. 객체를 멤버로 직접 두고 멤버 초기화 목록에서 `(*this)`를 넘겨 만들어도 된다고 생각하지만(일반 지식) 시험하지는 않았다.
- `geometry_msgs::msg::TransformStamped transform_stamped_;`: TF 한 개를 담는 메시지. `header.frame_id`(부모), `child_frame_id`(자식), `transform.translation`(위치), `transform.rotation`(쿼터니언)을 가진다.
- 생성자: `header.frame_id = "odom"`, `child_frame_id = "base_footprint"`. 부모가 `odom`, 자식이 `base_footprint`다. 이 값은 `odom_msg_`와 같아야 한다.
- 콜백의 `// TF` 부분: 위치(`x_`, `y_`; `translation.z`는 대입하지 않으므로 기본값 0), 방향(위에서 계산한 쿼터니언 `q`)을 채우고, 시각을 찍고 `sendTransform`으로 낸다. 한 프레임(`base_footprint`)의 부모는 하나뿐이어야 한다. 따라서 `odom → base_footprint`를 내는 노드는 하나여야 한다. 표준 제어기(구성 B)는 자기가 내므로, 구성 B를 쓸 때는 이 노드를 띄우지 않는다(`use_simple_controller:=False`가 이를 보장한다).
- `ament_target_dependencies`에 `tf2_ros`가 더해진다.

### 빌드와 시험

단계 3과 같은 세 터미널로 한다. 이전 단계의 launch를 먼저 `Ctrl+C`로 끝낸다.

터미널 A(끝나지 않는다):

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bumperbot_description gazebo.launch.py
```

기대 결과(빌드. 이 패키지의 줄만):

```text
Finished <<< bumperbot_controller [6.34s]
```

작업 공간에 앞 장의 패키지도 있으면 그 패키지의 `Finished` 줄도 함께 나온다. `Summary`의 패키지 수는 사용자마다 다르므로 `Failed`가 없고 경고가 없으면 된다.

터미널 B(끝나지 않는다):

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 launch bumperbot_controller controller.launch.py
```

터미널 C에서 단계 3과 같이 직진·회전한 뒤(`pub -r 20`을 `Ctrl+C`로 끄고 회전, 정지 명령) 멈춘 상태에서 Gazebo의 위치를 읽는다.

```bash
gz model -m bumperbot -p
```

```text
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [0.580995 -0.000329 0.000002]
    [-0.000000 0.010548 1.927990]
```

TF를 읽는다. 이 명령은 끝나지 않는다. 변환이 나오면 `Ctrl+C`로 끝낸다.

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

기대 결과(`At time`과 값은 실행마다 다르다. `tf2_echo`는 시작 직후에 `Waiting for transform ... Invalid frame ID "odom"`가 한두 번 나올 수 있다. TF를 받기 전의 대기다):

```text
At time 33.580000000
- Translation: [0.581, 0.000, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.823, 0.567]
- Rotation: in RPY (radian) [0.000, -0.000, 1.935]
- Rotation: in RPY (degree) [0.000, -0.000, 110.867]
```

`/tf`의 발행자를 본다.

```bash
ros2 topic info /tf -v | grep "Node name"
```

```text
Node name: simple_controller
Node name: robot_state_publisher
```

(08장 이후에는 `noisy_controller`도 보인다.)

- 통과 조건 1: `tf2_echo odom base_footprint`가 변환을 계속 출력하고, 그 값이 `/bumperbot_controller/odom`의 위치·방향과 같다(`Translation`의 `x`, `RPY`의 마지막 값 `yaw`).
- 통과 조건 2: 오도메트리가 Gazebo의 실제 값과 1% 안쪽으로 같다. 위 시험에서 TF `x = 0.581`, `yaw = 1.935`, Gazebo `x = 0.580995`, `yaw = 1.927990`이었다(`yaw` 차이 0.4%). 값 자체는 보낸 시간에 따라 다르다. 이 오차는 `wheel_separation`·`wheel_radius`가 실제와 약간 다르거나 바퀴가 아주 조금 미끄러지는 데서 온다. 이 값들이 실제와 다를수록 오차가 커진다.
- `/tf`의 발행자는 `robot_state_publisher`와 `simple_controller` 둘이다. 서로 다른 프레임 쌍을 낸다.
- RViz에서 보려면 `Fixed Frame`을 `base_footprint`에서 `odom`으로 바꾼다. 로봇이 움직이면 모델이 바닥(`odom` 기준 격자) 위를 이동한다. (이 가이드를 쓸 때 RViz는 띄우지 않았다.)
- 터미널 A와 B를 `Ctrl+C`로 끝낸다.

## 단계 5. `bumperbot_ros2_control.xacro`의 변경

### 왜 필요한가

참조 저장소의 Section7과 Section9 사이에서 `bumperbot_ros2_control.xacro`가 바뀌었다(`diff -r`로 확인했다). 바뀐 것은 **조인트를 선언한 순서**뿐이다. 03장·05장에서는 `wheel_right_joint`, `wheel_left_joint` 순서였고 이 장에서 `wheel_left_joint`, `wheel_right_joint` 순서가 된다. 참조 저장소의 마지막 커밋 제목이 `fix joint order`다. 단계 6에서 이 순서가 왜 문제가 되는지, 그리고 이 환경에서는 실제로 어떻게 나오는지 본다.

### 코드

`<ros2_control>` 안에서 두 `<joint>` 블록의 위치를 바꾼다.

> 파일: `bumperbot_description/urdf/bumperbot_ros2_control.xacro` — 수정
```xml
        <joint name="wheel_left_joint">
            <command_interface name="velocity">
              <param name="min">-1</param>
              <param name="max">1</param>
            </command_interface>
            <state_interface name="position" />
            <state_interface name="velocity" />
        </joint>

        <joint name="wheel_right_joint">
            <command_interface name="velocity">
              <param name="min">-1</param>
              <param name="max">1</param>
            </command_interface>
            <state_interface name="position" />
            <state_interface name="velocity" />
        </joint>

    </ros2_control>
```

### 코드 설명

- 달라진 것은 왼쪽 조인트가 먼저, 오른쪽 조인트가 나중에 선언된 것뿐이다. 내용(command/state interface)은 같다.
- 의도: `simple_controller.cpp`가 `/joint_states`의 `position.at(0)`을 왼쪽, `at(1)`을 오른쪽으로 읽는다. 조인트 순서를 왼쪽, 오른쪽으로 선언해 두면 `/joint_states`의 순서도 그렇게 나오리라고 기대한 변경이다.

### 빌드와 시험

새 터미널이라면 `source ~/bumperbot_ws/install/setup.bash`를 먼저 친다. 바꾼 파일이 `--symlink-install`로 이미 연결되어 있으므로 다시 빌드하지 않아도 되지만, 빌드해도 상관없다.

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
xacro src/bumperbot_description/urdf/bumperbot.urdf.xacro | grep -n "<joint name=\"wheel_"
```

기대 결과:

```text
55:    <joint name="wheel_left_joint">
63:    <joint name="wheel_right_joint">
116:  <joint name="wheel_right_joint" type="continuous">
141:  <joint name="wheel_left_joint" type="continuous">
```

앞의 두 줄(55, 63번째 줄)이 `<ros2_control>` 안의 조인트이고 **왼쪽, 오른쪽** 순서다. 뒤의 두 줄(116, 141번째 줄)은 02장에서 쓴 URDF의 링크 조인트이고 `type="continuous"`가 붙어 있으며 오른쪽, 왼쪽 순서다(줄 번호는 파일을 쓴 모양에 따라 조금 다를 수 있다).

- 통과 조건: `ros2_control` 안의 첫 `<joint>`가 `wheel_left_joint`다(`type="continuous"`가 없는 줄). 이 xacro는 Gazebo를 시작할 때 `robot_state_publisher`가 한 번 읽는다. 바꾼 뒤 Gazebo와 `controller.launch.py`를 **다시 띄워야** 반영된다.

## 단계 6. `/joint_states`의 조인트 순서

### 왜 필요한가

오도메트리 코드는 `position.at(0)`이 왼쪽 바퀴라고 가정한다. 순서가 반대이면 `Δφ_L`과 `Δφ_R`이 바뀌어 직진은 맞아도 회전 방향이 거꾸로 계산된다. 이 가정이 실제 `/joint_states`에서 맞는지 확인한다.

### 시험

단계 5의 변경을 반영하려면 Gazebo와 제어기 launch를 새로 띄워야 한다. 이전 launch를 `Ctrl+C`로 끝낸 뒤 터미널 A와 B에서 다시 띄운다.

```bash
cd ~/bumperbot_ws
source install/setup.bash
ros2 launch bumperbot_description gazebo.launch.py
```

터미널 B:

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 launch bumperbot_controller controller.launch.py
```

터미널 C에서(두 launch가 완전히 올라온 뒤. 제어기가 올라오기 전에 읽으면 `/joint_states`가 없어 `echo`가 기다린다):

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 topic echo /joint_states --once --field name
ros2 control list_hardware_components | tail -3
```

기대 결과(이 가이드의 xacro, 즉 왼쪽이 먼저):

```text
['wheel_left_joint', 'wheel_right_joint']
```

```text
	command interfaces
		wheel_left_joint/velocity [available] [claimed]
		wheel_right_joint/velocity [available] [claimed]
```

이 환경(Jazzy, `controller_manager` 4.50)에서 순서를 바꿔 가며 확인한 결과다.

| 경우 | `/joint_states`의 `name` |
|---|---|
| xacro에서 오른쪽을 먼저 선언(03장, 05장 상태) | `['wheel_left_joint', 'wheel_right_joint']` |
| xacro에서 왼쪽을 먼저 선언(이 장) | `['wheel_left_joint', 'wheel_right_joint']` |
| xacro는 왼쪽 먼저, `joint_state_broadcaster`의 `joints` 파라미터를 `[wheel_right_joint, wheel_left_joint]`로 지정 | `['wheel_left_joint', 'wheel_right_joint']` |

- 어느 경우에도 순서는 `[left, right]`였다. `ros2 control list_hardware_components`의 command interface 목록은 xacro 순서를 따라 오른쪽이 먼저 나오는 경우가 있는데도 `/joint_states`는 그렇지 않았다. **원인은 확인하지 못했다.**
- 따라서 이 환경에서는 `at(0)`이 왼쪽이라는 코드의 가정이 시험한 세 경우에 모두 맞았다. 이 장의 xacro 변경은 참조를 따랐고 동작에는 영향이 없었다.
- **순서에 기대지 말고 이름으로 찾는 것이 안전하다.** 참조 저장소가 이 문제로 `fix joint order` 커밋을 한 것이 근거다. 이 가이드는 참조 코드를 그대로 쓰므로 `position.at(0)`을 그대로 둔다. 고친다면 `state.name`에서 `wheel_left_joint`의 인덱스를 찾아 `position`을 읽는다(시험하지 않은 개선안이다). 순서에 기대는 오도메트리가 틀렸을 때의 증상은 "직진은 되는데 회전 방향이 반대"다.
- 11장에서 직접 쓰는 하드웨어 인터페이스는 조인트를 이름으로 찾는다. 그래도 `/joint_states`의 순서는 같은 방법으로 확인한다.

## 단계 7. 표준 `diff_drive_controller`의 오도메트리와 비교

### 왜 필요한가

05장에서 구성 B(`use_simple_controller:=False`)로 바꾸면 우리가 쓴 노드 없이도 `/bumperbot_controller/odom`과 `odom → base_footprint` TF가 나온다. 우리가 쓴 오도메트리가 표준과 얼마나 같은지 본다.

### 시험

앞 단계의 launch를 모두 `Ctrl+C`로 끝내고 Gazebo를 새로 띄운 뒤 구성 B로 올린다. 구성 A와 B는 같은 Gazebo에 번갈아 올릴 수 없다(05장).

터미널 A(끝나지 않는다):

```bash
cd ~/bumperbot_ws
source install/setup.bash
ros2 launch bumperbot_description gazebo.launch.py
```

터미널 B(끝나지 않는다):

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 launch bumperbot_controller controller.launch.py use_simple_controller:=False
```

터미널 C에서 회전 명령을 반복해서 보낸다(구성 B는 0.5초 안에 새 명령이 없으면 선다). 3초쯤 뒤 `Ctrl+C`로 끝낸다.

```bash
source ~/bumperbot_ws/install/setup.bash
gz model -m bumperbot -p
ros2 topic pub -r 20 /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"
```

멈춘 뒤(1~2초 기다린 다음):

```bash
gz model -m bumperbot -p
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic info /tf -v | grep "Node name"
```

(`tf2_echo`는 끝나지 않는다. 변환이 나오면 `Ctrl+C`로 끝낸다.)

기대 결과(이 가이드를 쓸 때의 값. 회전한 시간에 따라 다르다):

```text
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [0.000318 -0.000346 0.000002]
    [-0.000000 0.010548 1.473280]
```

```text
At time 25.870000000
- Translation: [-0.000, -0.000, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.673, 0.740]
- Rotation: in RPY (radian) [0.000, -0.000, 1.475]
- Rotation: in RPY (degree) [0.000, -0.000, 84.522]
```

```text
Node name: robot_state_publisher
Node name: bumperbot_controller
```

직진(05장 단계 4의 시험)에서는 오도메트리·TF의 `x`가 `1.485`, Gazebo의 `x`가 `1.483900`이었다.

| 항목 | `simple_controller`(우리 코드) | `diff_drive_controller`(표준) |
|---|---|---|
| 토픽 | `/bumperbot_controller/odom` | 같음 |
| 프레임 | `odom → base_footprint` | 같음(`base_frame_id: base_footprint`로 설정) |
| 발행 주기 | `/joint_states`마다(약 100 Hz. `ros2 topic hz`로 확인) | `publish_rate`(yaml에서 50 Hz) |
| 공분산 | 모두 0 | yaml의 `pose_covariance_diagonal`, `twist_covariance_diagonal` |
| TF를 내는 곳 | `simple_controller` 노드 | CM 안의 제어기(`/tf`의 발행자가 `bumperbot_controller`) |
| 적분 방식 | `θ`를 먼저 갱신하고 그 방향으로 `Δs`만큼 간다(사각형 근사) | 설치된 `odometry.hpp`에 `integrateRungeKutta2`와 `integrateExact` 두 함수가 있다. 속도는 롤링 평균(`velocity_rolling_window_size`)으로 구한다. 어느 쪽을 언제 쓰는지는 소스(`odometry.cpp`)를 보지 못했다 |
| Gazebo 실제 값과의 오차(이 가이드를 쓸 때) | 이동 0.1~0.2%, 회전 0.1~0.4% | 이동 0.1% 안팎, 회전 0.1% 안팎 |

- 통과 조건 1: 구성 B에서도 `odom → base_footprint`가 나오고 오도메트리·TF가 Gazebo의 실제 이동·회전과 1% 안쪽으로 같다.
- 통과 조건 2: 두 구현의 오차가 비슷하다. 적분 방식이 다르지만(위 표), 100 Hz의 작은 증분에서는 계산 결과가 거의 같게 나온다고 본다(일반 지식). 차이는 공분산, 발행 주기, 그리고 표준 제어기가 속도·가속도 제한을 하는 점이다.
- 두 구성을 동시에 띄우면 `odom → base_footprint`를 두 노드가 내므로 TF가 충돌한다. `use_simple_controller`로 하나만 고른다.
- 터미널 A와 B를 `Ctrl+C`로 끝낸다.

## 알려진 한계

이 장의 `simple_controller`는 참조를 그대로 따랐고 아래 한계가 있다. 코드는 고치지 않았다.

- `dt.seconds()`가 0이면 속도(`fi_left`, `fi_right`)를 0으로 나누게 된다. `/joint_states`가 같은 시각의 메시지를 두 번 주면 일어날 수 있다. 위치(`x_`, `y_`, `theta_`)는 `dt`를 쓰지 않으므로 영향이 없고 `twist`만 `inf`나 `nan`이 될 수 있다. 이 가이드를 쓸 때는 재현하지 못했다(일반 지식).
- 로봇이 이미 움직인 뒤에 이 노드를 띄우면 `left_wheel_prev_pos_`, `right_wheel_prev_pos_`가 0에서 시작하므로 첫 콜백에서 바퀴 위치 전체를 한 번에 움직인 것으로 계산해 위치가 튄다. `odom` 프레임의 원점이 "노드를 띄운 자리"가 아니라 "바퀴 위치가 0이던 자리"가 되는 셈이다. (이 가이드를 쓸 때 재현했다. 바퀴가 `15.30` rad 돈 뒤에 노드를 새로 띄우자 `odom`의 `x`가 처음부터 `0.505`(`0.033 × 15.30`)였다. 로봇은 그 자리에 있는데도 원점에서 출발한 값이 아니다.)
- 노드를 띄울 때 `use_sim_time`의 시계가 아직 `/clock`을 받지 않았다면 `prev_time_ = get_clock()->now()`가 0이 되어 첫 콜백의 `dt`가 비정상적으로 크다. 속도 값 하나가 이상할 뿐 위치에는 영향이 없다(코드를 읽고 한 설명이다. 재현하지 않았다).

## 참조 코드와 다른 점

| 참조 | 이 가이드 | 이유 |
|---|---|---|
| `simple_controller.cpp`의 생성자 초기화 목록: `left_wheel_prev_pos_(0.0)`, `right_wheel_prev_pos_(0.0)` 순서 | `right_wheel_prev_pos_(0.0)`, `left_wheel_prev_pos_(0.0)` 순서 | 헤더의 멤버 선언 순서(`right`, `left`)에 맞췄다. 맞추지 않으면 `-Wreorder` 경고가 난다(단계 2 설명). 다른 줄은 Section9 파일 그대로다. |
| `CMakeLists.txt`: `find_package(ament_cmake_python REQUIRED)`, `rclpy`, `ament_python_install_package`, `install(PROGRAMS ... .py)` | 없음 | 이 가이드는 C++ 노드만 쓴다. |
| `package.xml`: `<exec_depend>tf_transformations</exec_depend>`, `<depend>rclpy</depend>` | 없음 | `tf_transformations`는 Python 판이 쓰고 이 환경에 설치되어 있지 않다. C++은 `tf2::Quaternion`을 쓴다. |
| Python판 `bumperbot_controller/simple_controller.py` | 싣지 않음 | 같은 이유. 참조 위치는 `Section9_Odometry/bumperbot_ws/src/bumperbot_controller/bumperbot_controller/simple_controller.py`다. |
| `CMakeLists.txt`: `cmake_minimum_required(VERSION 3.5)`, `install(DIRECTORY include DESTINATION include)` | `3.8`, `install(DIRECTORY include/ DESTINATION include)`, 경고 옵션 블록 | 05장과 같다. |
| `package.xml`에 `teleop_twist_keyboard` 없음, 라이선스·maintainer 표기 | 05장·03장과 같다 | 05장·03장과 같다. |
| `bumperbot_ros2_control.xacro`의 `is_ignition` 분기 | 없음 | 이 가이드는 Jazzy만 쓴다. 이 장의 변경(조인트 순서)은 참조와 같다. |

## 자주 나는 오류

| 증상 | 원인 | 고치는 법 |
|---|---|---|
| 명령을 안 줬는데 오도메트리(`x`, `theta`)가 움직인다. | 이전 Gazebo가 남은 채로 새로 띄워 모델과 `controller_manager`가 둘이다(이 가이드를 쓰다가 겪었다). | 이전 launch를 모두 `Ctrl+C`로 끝낸다. `ps -eo pid,args \| grep "gz sim"`로 확인한다. |
| `controller.launch.py`를 한 번 더 실행하자 스포너가 `[WARN] Controller already loaded, skipping load_controller`, `[ERROR] Failed to configure controller`를 내고 종료 코드 1로 죽는다. | 이미 올라온 제어기에 같은 launch를 다시 실행했다(재현했다). | 제어기는 Gazebo를 새로 띄워야 새로 올릴 수 있다. 또는 `ros2 control switch_controllers`, `unload_controller`로 내린 뒤 올린다(03장). |
| 구성 A가 켜진 Gazebo에 구성 B를 올리자 `bumperbot_controller`가 `inactive`다. | 두 바퀴의 command interface를 이미 `simple_velocity_controller`가 점유하고 있어 둘째 제어기를 켤 수 없다(`list_controllers`에서 확인했다). | 구성을 바꿀 때는 Gazebo를 껐다가 다시 띄운다. |
| `tf2_echo odom base_footprint`가 계속 `Invalid frame ID "odom" ... does not exist`다. | 오도메트리를 내는 노드가 없다(`simple_controller`를 띄우지 않았거나 단계 4 이전이다), 또는 Gazebo를 껐다가 다시 켠 뒤 `controller.launch.py`를 띄우지 않았다. | `ros2 node list`에서 `/simple_controller`(구성 A)나 `/bumperbot_controller`(구성 B)를 확인한다. |
| 직진은 맞는데 회전이 반대로 계산된다. | `/joint_states`의 조인트 순서가 코드의 가정(`position.at(0)`이 왼쪽)과 다르다. | `ros2 topic echo /joint_states --once --field name`으로 순서를 확인하고 코드의 인덱스를 맞춘다. 이 가이드의 환경에서는 항상 `[left, right]`였다. |
| 오도메트리가 시간이 갈수록 실제와 크게 벌어진다. | `wheel_radius`, `wheel_separation` 값이 실제와 다르거나 바퀴가 미끄러진다. | 값을 측정해 `controller.launch.py`의 인자나 yaml에서 고친다. 오차가 쌓이는 것은 오도메트리의 본질이다(08장, 09장). |
| 오도메트리·TF 시각이 이상하게 크다(예: 1.79e9) 또는 RViz가 시간 오류를 낸다. | `simple_controller`의 `use_sim_time`이 꺼져 벽시계 시각으로 찍는다. (이 가이드에서는 재현하지 않았다. 일반 지식이다.) | launch 인자 `use_sim_time`이 `True`인지(기본값) 확인한다. |

## 정리

| 파일 | 변경 | 역할 |
|---|---|---|
| `include/bumperbot_controller/simple_controller.hpp`, `src/simple_controller.cpp` | `/joint_states` 구독, 오도메트리 계산, `Odometry` 발행, TF 발행 | `(x, y, θ)` 계산과 발행 |
| `CMakeLists.txt`, `package.xml` | `sensor_msgs`, `nav_msgs`, `tf2`, `tf2_ros` 추가 | 빌드·의존성 |
| `bumperbot_ros2_control.xacro` | 조인트 선언 순서 | `/joint_states` 순서에 대한 참조의 조치(이 환경에서는 영향 없음) |

다음 장(08)에서 같은 계산을 하되 엔코더에 잡음과 바퀴 크기 오차를 넣은 `noisy_controller`를 만든다. 이 장의 `/bumperbot_controller/odom`(정답에 가까운 값)과 그것이 만드는 `/bumperbot_controller/odom_noisy`(오차가 있는 값)를 비교한다. 이 두 값이 09장의 센서 융합 입력이 된다.
