# 08. 노이즈 제어기 (저장소 Section10)

> 참조: `Section10_Probability_for_Robotics/bumperbot_ws/src/bumperbot_controller/` (참조 저장소 최상위 기준 경로)

## 이 장에서 하는 일

- 07장의 오도메트리와 같은 계산을 하되, 바퀴 엔코더에 **잡음**을 더하고 바퀴 반지름·바퀴 간격에 **오차**를 넣은 노드 `noisy_controller`를 만든다.
- 그 결과를 `/bumperbot_controller/odom_noisy`로 낸다. 정확한 값(`odom`)과 오차가 든 값(`odom_noisy`)이 같은 로봇에서 동시에 나온다.
- launch 파일에서 `OpaqueFunction`으로 오차 인자를 숫자로 꺼내 더하는 방법을 쓴다.
- 왜 만드는가: 실제 로봇의 엔코더는 잡음이 있고 바퀴 지름·간격의 측정값은 틀린다. 오도메트리 오차가 시간이 갈수록 어떻게 쌓이는지 눈으로 보고, 09장(센서 융합)에서 이 나쁜 값을 IMU와 합쳐 개선하는 입력으로 쓴다. ros2_control의 구조는 바뀌지 않는다. 같은 `/joint_states`를 읽는 일반 노드가 하나 더 생길 뿐이다.

## 이번 장의 파일

```text
~/bumperbot_ws/src/bumperbot_controller/
├── CMakeLists.txt                                      [수정] 실행 파일 추가
├── include/bumperbot_controller/noisy_controller.hpp   [새]
├── src/noisy_controller.cpp                            [새]
└── launch/controller.launch.py                         [수정] OpaqueFunction, 오차 인자
```

## 단계 1. `noisy_controller` 노드

### 왜 필요한가

07장의 `simple_controller`는 `/joint_states`로 오도메트리를 계산한다. 같은 입력에 잡음을 더해 계산하는 노드를 하나 더 두면, 잡음이 위치 추정에 어떤 영향을 주는지를 같은 로봇에서 비교할 수 있다. 이 노드는 `cmd_vel`을 받지 않고 바퀴도 제어하지 않는다. `/joint_states`를 읽어서 오도메트리만 계산한다.

### 코드

> 파일: `bumperbot_controller/include/bumperbot_controller/noisy_controller.hpp` — 전체
```cpp
#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class NoisyController : public rclcpp::Node
{
public:
    NoisyController(const std::string& name);

private:

    void jointCallback(const sensor_msgs::msg::JointState &msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Odometry
    double wheel_radius_;
    double wheel_separation_;
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

#endif // NOISY_CONTROLLER_HPP
```

> 파일: `bumperbot_controller/src/noisy_controller.cpp` — 전체
```cpp
#include "bumperbot_controller/noisy_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <random>


using std::placeholders::_1;


NoisyController::NoisyController(const std::string& name)
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

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&NoisyController::jointCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom_noisy", 10);

    // Fill the Odometry message with invariant parameters
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_ekf";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";

    prev_time_ = get_clock()->now();
}


void NoisyController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    // Implements the inverse differential kinematic model
    // Given the position of the wheels, calculates their velocities
    // then calculates the velocity of the robot wrt the robot frame
    // and then converts it in the global frame and publishes the TF

    // Add noise to wheel readings
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.005);

    double wheel_encoder_left = state.position.at(0) + left_encoder_noise(noise_generator);
    double wheel_encoder_right = state.position.at(1) + right_encoder_noise(noise_generator);

    double dp_left = wheel_encoder_left - left_wheel_prev_pos_;
    double dp_right = wheel_encoder_right - right_wheel_prev_pos_;
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
  auto node = std::make_shared<NoisyController>("noisy_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 코드 설명

`simple_controller`(07장)와 같은 부분(구독, 식, `Odometry` 메시지와 TF를 채워 내는 부분)은 다시 설명하지 않는다. 달라진 곳만 짚는다.

- 헤더: `velCallback`, 바퀴 속도 발행자, `Eigen`, `speed_conversion_`이 없다. 명령을 받지 않으므로 필요 없다. `std_msgs`(`Float64MultiArray`)도 없다. 바퀴 속도 명령을 내지 않기 때문이다. 그래서 CMake의 `find_package`와 `ament_target_dependencies`에도 `std_msgs`와 Eigen이 없다.
- 생성자 초기화 목록도 07장과 같은 이유로 `right_wheel_prev_pos_`, `left_wheel_prev_pos_` 순서로 고쳤다(헤더의 선언 순서. `-Wreorder` 경고를 피한다).
- 노드 이름과 토픽:
  - 노드 이름 `noisy_controller`(`main`의 `std::make_shared<NoisyController>("noisy_controller")`).
  - 발행 토픽 `/bumperbot_controller/odom_noisy`(`nav_msgs/msg/Odometry`).
  - 프레임: 오도메트리 메시는 `header.frame_id = "odom"`, `child_frame_id = "base_footprint_ekf"`이고, TF는 `odom → base_footprint_noisy`다. 정확한 값을 내는 `simple_controller`의 TF(`odom → base_footprint`)와 **자식 프레임 이름이 달라서** 같은 프레임에 TF가 둘 겹치지 않는다. 메시지의 `child_frame_id`(`base_footprint_ekf`)와 TF의 자식 이름(`base_footprint_noisy`)이 다른 것은 참조 코드 그대로다. `child_frame_id`가 `base_footprint_ekf`인 까닭은 참조 저장소의 다음 섹션에서 확인된다. `Section11_Sensor_Fusion/bumperbot_ws/src/bumperbot_localization/config/ekf.yaml`이 `base_link_frame: base_footprint_ekf`로 설정하고 입력 `odom0`으로 `bumperbot_controller/odom_noisy`를 읽는다. 즉 이 오도메트리를 `robot_localization`의 필터에 넣을 때 쓰는 프레임 이름이다. 09장에서 이어진다. TF의 자식 이름 `base_footprint_noisy`가 왜 다른지는 확인하지 못했다.
- 파라미터: `wheel_radius`, `wheel_separation`의 기본값은 07장과 같다(0.033, 0.17). 실제로는 launch가 오차를 더한 값을 넘겨 준다(단계 2).
- 잡음 만들기(`jointCallback` 맨 앞):
  - `unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();`: 현재 시각으로 난수의 시작값을 정한다.
  - `std::default_random_engine noise_generator(seed);`: 난수 발생기.
  - `std::normal_distribution<double> left_encoder_noise(0.0, 0.005);`: 평균 0, 표준편차 0.005의 정규분포. 단위는 rad이다. 바퀴 반지름 0.033 m일 때 약 0.17 mm에 해당한다. 오른쪽도 같은 방식이다.
  - `double wheel_encoder_left = state.position.at(0) + left_encoder_noise(noise_generator);`: 바퀴 회전각에 잡음을 더한 값이 "측정값"이다.
  - `dp_left = wheel_encoder_left - left_wheel_prev_pos_;`: 잡음이 든 측정값과 이전 값의 차이.
  - `left_wheel_prev_pos_ = state.position.at(0);`: 이전 값으로는 **잡음이 없는 값**을 저장한다. 그래서 각 주기의 증분 `dp_left`에는 그 주기의 잡음 `n_k`가 한 번만 들어가고, 증분을 합하면 잡음이 누적된다. 잡음이 든 값을 이전 값으로 저장하면 증분이 `n_k − n_(k−1)`이 되어 합할 때 서로 상쇄되므로 오차가 쌓이지 않는다(계산상의 설명이고 그렇게 바꿔 시험해 보지는 않았다).
- 콜백이 불릴 때마다 난수 발생기를 새로 만든다. 간단하지만 좋은 습관은 아니다. 멤버로 한 번만 만드는 쪽이 낫다. 참조 코드를 그대로 옮겼다.
- `position.at(0)`이 왼쪽 바퀴라는 가정은 07장과 같다.

Python판: `Section10_Probability_for_Robotics/bumperbot_ws/src/bumperbot_controller/bumperbot_controller/noisy_controller.py`

### 코드: CMakeLists.txt

실행 파일을 하나 더 만들고 설치한다. `noisy_controller`는 Eigen과 `std_msgs`를 쓰지 않으므로 `ament_target_dependencies`에 그 둘이 없다. 아래 블록의 `...` 줄은 "그 사이에 원래 있던 줄을 그대로 둔다"는 뜻이다. 파일에 치지 않는다. 블록에서 새로 넣는 줄은 `add_executable(noisy_controller ...)`와 `ament_target_dependencies(noisy_controller ...)` 두 줄, 그리고 `install(TARGETS` 목록의 `noisy_controller` 한 줄이다. 첫 두 줄은 `simple_controller` 줄들 아래, `install(DIRECTORY include/ ...)` 위에 넣는다.

> 파일: `bumperbot_controller/CMakeLists.txt` — 수정
```cmake
add_executable(noisy_controller src/noisy_controller.cpp)
ament_target_dependencies(noisy_controller rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2)
# ...
install(TARGETS
  simple_controller
  noisy_controller
```

### 코드 설명: CMakeLists.txt

- `add_executable(noisy_controller src/noisy_controller.cpp)`: 실행 파일 `noisy_controller`를 만든다. 소스 파일이 두 개(`simple_controller.cpp`, `noisy_controller.cpp`)이고 각각 `main`을 가지므로 실행 파일도 두 개다. 한 실행 파일에 두 소스를 합치면 `main`이 둘이 되어 링크가 실패한다.
- `ament_target_dependencies(noisy_controller rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2)`: 이 실행 파일의 의존 패키지.
- `install(TARGETS simple_controller noisy_controller ...)`: 두 실행 파일을 함께 설치한다. 새 실행 파일을 만들 때 이 줄에 더하는 것을 잊으면 `ros2 run`/launch에서 `No executable found`가 난다(05장).
- `package.xml`은 바뀌지 않는다. 쓰는 패키지가 07장과 같다.

### 빌드와 시험

이 단계만 따로 시험하려면 launch 없이 노드를 직접 띄운다. 이 노드는 `use_sim_time`을 켜야 Gazebo의 시뮬레이션 시각을 쓰므로(이 단계에서는 Gazebo 없이 시작만 확인하지만 launch에서는 항상 켠다) `-p use_sim_time:=true`를 붙인다. 노드는 끝나지 않으므로 확인한 뒤 `Ctrl+C`로 끝낸다.

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 run bumperbot_controller noisy_controller --ros-args -p use_sim_time:=true
```

기대 결과(빌드. 이 패키지의 줄만):

```text
Finished <<< bumperbot_controller [4.19s]
```

작업 공간에 앞 장의 패키지도 있으면 그 패키지의 `Finished` 줄도 함께 나온다. `Summary`의 패키지 수는 사용자마다 다르므로 `Failed`가 없고 경고가 없으면 된다.

```text
[INFO] [...] [noisy_controller]: Using wheel radius 0.033
[INFO] [...] [noisy_controller]: Using wheel separation 0.17
```

- 통과 조건: 경고 없이 빌드되고, 노드가 뜨면서 `Using wheel radius 0.033`, `Using wheel separation 0.17` 로그가 나온다. 이 로그의 값이 기본값이다.

## 단계 2. launch: `OpaqueFunction`과 오차 인자

### 왜 필요한가

launch 인자로 오차를 받아 "원래 값 + 오차"를 파라미터로 넘기고 싶다. 그런데 `LaunchConfiguration("wheel_radius")`는 값이 아니라 **나중에 값으로 바뀔 자리표시자**다. launch 파일의 `generate_launch_description()`이 실행되는 시점에는 아직 사용자가 `wheel_radius:=...`로 준 값이 정해지지 않았다. 따라서 `float(LaunchConfiguration("wheel_radius")) + ...`처럼 파이썬 연산을 직접 할 수 없다. 값을 숫자로 꺼내려면 launch가 실제로 일을 시작한 뒤에 부르는 함수가 필요하고, 그것이 `OpaqueFunction`이다.

### 코드

import 줄:

> 파일: `bumperbot_controller/launch/controller.launch.py` — 수정
```python
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
```

`generate_launch_description` 앞에 함수를 하나 더한다.

> 파일: `bumperbot_controller/launch/controller.launch.py` — 수정
```python
def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_node = Node(
        package="bumperbot_controller",
        executable="noisy_controller",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "use_sim_time": use_sim_time}],
    )

    return [
        noisy_controller_node,
    ]
```

`generate_launch_description` 안: 인자 선언, `OpaqueFunction` 만들기, 반환 목록.

> 파일: `bumperbot_controller/launch/controller.launch.py` — 수정
```python
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
    )
# ...
    noisy_controller_launch = OpaqueFunction(function=noisy_controller)
# ...
    return LaunchDescription(
        [
            use_sim_time_arg,
            use_simple_controller_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
            noisy_controller_launch,
        ]
    )
```

### 코드 설명

- `def noisy_controller(context, *args, **kwargs):`: `OpaqueFunction`이 부르는 함수의 형식이다. 첫 인자 `context`는 현재 launch가 가진 값들(인자 값 포함)을 담은 객체다. 반환값은 launch 액션의 목록이다. `*args, **kwargs`는 `OpaqueFunction(function=..., args=[...], kwargs={...})`로 넘긴 추가 인자를 받는 자리다(설치된 `launch/actions/opaque_function.py`에서 `function(context, *args, **kwargs)`로 부르는 것을 확인했다). 이 함수는 추가 인자를 쓰지 않으므로 형식을 맞추는 용도일 뿐이다.
- `float(LaunchConfiguration("wheel_radius").perform(context))`: `perform(context)`가 자리표시자를 실제 값(문자열)으로 바꾼다. `float()`로 숫자로 바꾼 뒤에야 `wheel_radius + wheel_radius_error`처럼 더할 수 있다.
- `Node(package="bumperbot_controller", executable="noisy_controller", parameters=[{...}])`: 파라미터는 `wheel_radius + wheel_radius_error`(0.033 + 0.005 = 0.038)와 `wheel_separation + wheel_separation_error`(0.17 + 0.02 = 0.19). 바퀴가 실제보다 크고 간격이 넓다고 잘못 알고 있는 로봇이다. `use_sim_time`은 계산할 필요가 없으므로 `LaunchConfiguration` 그대로 넘긴다. 이 변수는 `generate_launch_description` 안의 `use_sim_time`과는 별개로 함수 안에서 다시 만든 것이다. 함수는 `generate_launch_description`의 지역 변수를 볼 수 없기 때문이다(`wheel_radius`도 같은 이유로 함수 안에서 다시 `LaunchConfiguration(...)`을 만든다). 문자열 `"True"`는 launch가 알아서 불리언으로 해석한다.
- `return [noisy_controller_node,]`: 만든 노드를 목록으로 돌려주면 launch가 그 노드를 실행한다. 참조에서는 이 변수의 이름이 `noisy_controller_cpp`이고 Python판 노드와 함께 반환한다. 이 가이드는 C++만 쓰므로 이름을 `noisy_controller_node`로 줄였다.
- 인자 `wheel_radius_error`(기본값 `0.005`)와 `wheel_separation_error`(기본값 `0.02`): 단위는 미터. 명령줄에서 `wheel_radius_error:=0.0 wheel_separation_error:=0.0`처럼 바꿀 수 있다. 위 함수 안의 `LaunchConfiguration(...)` 이름과 같아야 한다.
- `noisy_controller_launch = OpaqueFunction(function=noisy_controller)`: 함수를 launch 액션으로 감싼다. `LaunchDescription`의 목록 맨 뒤에 넣는다. 함수 이름(`noisy_controller`)과 이 변수의 이름이 다른 것에 주의한다. `function=`에는 호출하지 않은 함수 자체를 넘긴다. `noisy_controller()`처럼 괄호를 붙이면 launch 파일을 읽는 시점에 인자 없이 함수를 부르게 되어 `TypeError: noisy_controller() missing 1 required positional argument: 'context'`가 난다(재현했다).
- 이 노드에는 조건이 없다. `use_simple_controller:=False`(구성 B)에서도 같이 뜬다.

### 빌드와 시험

터미널 A에서 빌드하고 Gazebo를 띄운다(끝나지 않는다). 단계 1의 `ros2 run` 노드는 먼저 `Ctrl+C`로 끝낸다.

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bumperbot_description gazebo.launch.py
```

터미널 B에서 제어기 launch를 띄운다(끝나지 않는다).

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 launch bumperbot_controller controller.launch.py
```

기대 결과(터미널 B의 로그. `[noisy_controller-4]`의 번호는 launch 목록의 구성에 따라 다르다):

```text
[noisy_controller-4] [INFO] [...] [noisy_controller]: Using wheel radius 0.038
[noisy_controller-4] [INFO] [...] [noisy_controller]: Using wheel separation 0.19
```

터미널 C에서 노드와 토픽을 확인한다.

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 node list
ros2 topic list | grep odom
```

기대 결과(`ros2 node list`는 일부만 줄였다. 다른 노드도 함께 나온다):

```text
/controller_manager
/noisy_controller
/simple_controller
/simple_velocity_controller
...
```

```text
/bumperbot_controller/odom
/bumperbot_controller/odom_noisy
```

오차 인자가 값에 반영되는지 보려면 **앞의 Gazebo와 launch를 모두 `Ctrl+C`로 끄고**(같은 노드를 두 번 띄우지 않는다) 다시 띄운다. 이 확인은 Gazebo 없이 터미널 B의 launch만 띄워도 된다.

```bash
ros2 launch bumperbot_controller controller.launch.py wheel_radius_error:=0.0 wheel_separation_error:=0.0
```

```text
[noisy_controller-4] [INFO] [...] [noisy_controller]: Using wheel radius 0.033
[noisy_controller-4] [INFO] [...] [noisy_controller]: Using wheel separation 0.17
```

확인한 뒤 `Ctrl+C`로 끝낸다. **단계 3의 시험은 기본 오차(`0.005`, `0.02`)를 전제로 하므로** 단계 3에서는 인자 없이 터미널 A, B를 새로 띄운다.

- 통과 조건 1: 기본값에서 `Using wheel radius 0.038`, `Using wheel separation 0.19`이고, 오차를 0으로 주면 `0.033`, `0.17`이다.
- 통과 조건 2: `/bumperbot_controller/odom`과 `/bumperbot_controller/odom_noisy`가 모두 있고 `/noisy_controller` 노드가 있다.
- 숫자가 아닌 값(`wheel_radius_error:=abc`)을 주면 launch가 `[ERROR] [launch]: Caught exception in launch (see debug for traceback): could not convert string to float: 'abc'`를 내고 시작하지 않는다(확인했다).

## 단계 3. 두 오도메트리의 차이 보기

### 왜 필요한가

이 장의 목적이다. 같은 로봇의 `odom`과 `odom_noisy`가 시간이 갈수록 벌어지는 것을 숫자로 본다.

### 시험

터미널 A에서 Gazebo, 터미널 B에서 `controller.launch.py`를 **기본 인자로** 새로 띄운다(단계 2의 마지막 시험을 했다면 모두 끄고 다시 띄운다. 오차 인자는 기본값 `0.005`, `0.02`다).

터미널 C에서 직진 명령을 보내고(끝나지 않는다. 10초쯤 뒤 `Ctrl+C`), 그동안 터미널 D에서 두 토픽을 번갈아 읽는다.

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 topic pub -r 20 /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 topic echo /bumperbot_controller/odom --once --field pose.pose.position
ros2 topic echo /bumperbot_controller/odom_noisy --once --field pose.pose.position
```

두 `echo`를 2초 간격으로 몇 번 반복하면 `odom_noisy`의 `x`가 `odom`의 `x`보다 항상 크고 그 차이가 커진다. (직진 중이므로 `odom`의 `y`는 0 근처에 머물고 `odom_noisy`의 `y`는 0에서 벗어난다.) 직진을 끝낸 뒤 회전도 본다. 구성 A는 마지막 속도를 유지하므로 터미널 C에서 `Ctrl+C` 뒤에 이어서:

```bash
ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.3}}}"
```

8초쯤 기다린 뒤 멈추고, 두 토픽을 다시 읽는다.

```bash
ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
ros2 topic echo /bumperbot_controller/odom --once --field pose.pose.orientation
ros2 topic echo /bumperbot_controller/odom_noisy --once --field pose.pose.orientation
```

방향은 쿼터니언 `(z, w)`로 나온다. 평면 로봇의 `yaw`는 `2 · atan2(z, w)`다(07장 단계 3).

아래 표는 두 토픽을 2초마다 읽어 한 줄로 출력하는 짧은 파이썬 스크립트(이 가이드를 쓸 때만 만들었다)로 얻은 것이다. 사용자는 위 `echo`를 여러 번 반복하거나 RViz에서 두 `Odometry` 표시를 함께 켜서 확인한다.

기대 결과(`v = 0.2`로 약 10초, `w = 0.3`으로 약 8초. `t`는 측정을 시작한 뒤 흐른 시뮬레이션 시간(초)이고 각 열은 `(x, y, yaw)`. 잡음은 난수이므로 값은 실행마다 다르다):

```text
t   odom(x,y,yaw)          noisy(x,y,yaw)         |diff_xy|
 0.0 (-0.000,0.000,-0.000)  (-0.003,0.000,-0.013)  0.003
 4.0 (0.212,-0.000,-0.000)  (0.244,-0.003,-0.024)  0.032
 6.0 (0.612,-0.000,0.000)  (0.706,-0.014,-0.038)  0.095
 8.0 (1.012,0.000,0.000)  (1.164,-0.045,-0.087)  0.158
10.0 (1.412,0.000,0.000)  (1.618,-0.077,-0.070)  0.220
12.0 (1.812,0.000,0.000)  (2.076,-0.121,-0.121)  0.291
14.0 (2.072,0.000,0.210)  (2.376,-0.159,0.099)  0.343
20.0 (2.072,0.000,2.007)  (2.375,-0.156,1.954)  0.340
odom frame: odom -> base_footprint | noisy frame: odom -> base_footprint_ekf
```

`odom_noisy`는 `/bumperbot_controller/odom_noisy`를 읽은 값이다. 마지막 줄의 프레임 이름은 `Odometry` 메시지의 `header.frame_id -> child_frame_id`다.

- 통과 조건 1: 직진하는 동안 두 값의 거리 차이(`|diff_xy|`)가 시간에 비례해 커진다. 위 시험에서 12초에 0.29 m였다.
- 통과 조건 2: 이동한 거리의 비율이 `0.038/0.033 = 1.15`에 가깝다. 위 시험에서 `2.376/2.072 = 1.147`이었다. 바퀴 반지름을 15% 크게 알고 있으므로 같은 바퀴 회전을 15% 더 멀리 간 것으로 계산한다.
- 통과 조건 3(실행마다 달라진다): 직진하는 동안 `odom`의 `y`와 `yaw`는 0에 머물지만 `odom_noisy`는 0에서 벗어난다(위 시험에서 12초에 `y = -0.121`, `yaw = -0.121`). 엔코더 잡음이 쌓인 결과다. 잡음은 난수이므로 실행할 때마다 값이 다르다.
- 제자리 회전에서는 두 값의 회전량이 다르다. 위 시험의 14초에서 20초 사이에 `odom`은 `2.007 − 0.210 = 1.797` rad, `odom_noisy`는 `1.954 − 0.099 = 1.855` rad 돌았다(3.2% 더 크다). `r/L` 비율이 `0.038/0.19 = 0.2`와 `0.033/0.17 = 0.194`로 약 3% 다르기 때문이다. 위치 오차와 달리 각도의 어긋남은 직진 중에 쌓인 값(`yaw = -0.121`)에서 시작한다.
- 회전하는 동안 위치 차이(0.343 → 0.340 m)는 거의 늘지 않는다. 위치 오차는 이동한 거리에 비례하기 때문이다.
- TF로 보면 `odom → base_footprint`(정확한 값)와 `odom → base_footprint_noisy`(오차가 든 값)가 따로 나온다. `ros2 run tf2_ros tf2_echo odom base_footprint_noisy`로 읽을 수 있다. (이 가이드에서는 이 TF를 직접 읽어 보지 않았다.)

## 참조 코드와 다른 점

| 참조 | 이 가이드 | 이유 |
|---|---|---|
| `noisy_controller.cpp`의 생성자 초기화 목록: `left_wheel_prev_pos_`, `right_wheel_prev_pos_` 순서 | `right_wheel_prev_pos_`, `left_wheel_prev_pos_` 순서 | 헤더의 멤버 선언 순서에 맞췄다(`-Wreorder` 경고. 07장 단계 2). 나머지는 Section10 파일 그대로다. |
| `CMakeLists.txt`: `install(PROGRAMS ${PROJECT_NAME}/noisy_controller.py ...)` | 없음 | 이 가이드는 C++ 노드만 쓴다. |
| Python판 `bumperbot_controller/noisy_controller.py` | 싣지 않음 | 같은 이유. 참조 위치는 `Section10_Probability_for_Robotics/bumperbot_ws/src/bumperbot_controller/bumperbot_controller/noisy_controller.py`다. |
| `controller.launch.py`의 `noisy_controller` 함수: `use_python` 인자와 `noisy_controller_py` 노드, `noisy_controller_cpp` 이름, `IfCondition`/`UnlessCondition` | `noisy_controller_node` 하나, 조건 없음 | C++만 쓴다. 변수 이름만 바꿨다. |
| `CMakeLists.txt`: `cmake_minimum_required(VERSION 3.5)`, `install(DIRECTORY include DESTINATION include)` | `3.8`, `install(DIRECTORY include/ DESTINATION include)`, 경고 옵션 블록 | 05장과 같다. |
| `bumperbot_ros2_control.xacro`의 공백 한 줄 수정(Section10) | 반영하지 않음 | 내용에 영향이 없는 공백 변경이다. |

## 자주 나는 오류

| 증상 | 원인 | 고치는 법 |
|---|---|---|
| launch가 `could not convert string to float: 'abc'`로 시작하지 않는다. | 오차 인자에 숫자가 아닌 값을 줬다. | 숫자를 준다(`0.005`). |
| `noisy_controller` 실행 파일이 없다(`No executable found`). | `install(TARGETS ... noisy_controller ...)`에 이름을 더하지 않았다. | `CMakeLists.txt`의 `install(TARGETS` 목록에 더하고 다시 빌드한다. |
| 링크 오류 `multiple definition of 'main'` 같은 오류가 난다. | 두 `main`이 한 실행 파일에 합쳐졌다(`add_executable`에 소스를 둘 넣었다). | 실행 파일마다 `add_executable`을 따로 쓴다. (일반 지식이다. 이 가이드에서는 재현하지 않았다.) |
| `odom_noisy`가 `odom`과 똑같이 나온다. | 오차 인자를 0으로 줬고 잡음(표준편차 0.005 rad)도 작아서 구분이 안 된다. 또는 이전 빌드 결과를 돌리고 있다. | 기본 오차(`0.005`, `0.02`)로 돌린다. 코드를 고쳤다면 `colcon build`를 다시 한다. |
| `ros2 launch`가 `TypeError: noisy_controller() missing 1 required positional argument: 'context'`와 `InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown`을 낸다. | `OpaqueFunction(function=noisy_controller())`처럼 괄호를 붙여 함수를 launch 파일을 읽는 시점에 인자 없이 불렀다(재현했다). | `function=noisy_controller`처럼 괄호 없이 함수 자체를 넘긴다. |

## 정리

| 파일 | 역할 | 누가 읽는가 |
|---|---|---|
| `include/bumperbot_controller/noisy_controller.hpp`, `src/noisy_controller.cpp` | 엔코더 잡음과 바퀴 크기 오차가 든 오도메트리 계산, `odom_noisy` 발행, TF `odom → base_footprint_noisy` | 컴파일러 → `noisy_controller` |
| `CMakeLists.txt` | 실행 파일 추가·설치 | colcon |
| `launch/controller.launch.py` | 오차 인자와 `OpaqueFunction`으로 노드 실행 | `ros2 launch` |

다음은 센서 융합(09장)이다. 이 장의 `/bumperbot_controller/odom_noisy`와 IMU 값을 칼만 필터나 `robot_localization`으로 합쳐 오차를 줄인다. 이 장이 끝난 시점의 `bumperbot_controller` 패키지가 다음 장들의 출발점이다.
