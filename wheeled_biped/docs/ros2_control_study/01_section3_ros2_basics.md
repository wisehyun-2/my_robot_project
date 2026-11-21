# 01. ROS 2 기초: 작업 공간과 패키지 구조 (저장소 Section3)

> 참조: `Section3_Introduction_to_ROS_2/bumperbot_ws/src/bumperbot_cpp_examples/`, `Section3_Introduction_to_ROS_2/bumperbot_ws/src/bumperbot_py_examples/`, `Section4_Locomotion/bumperbot_ws/src/bumperbot_cpp_examples/src/simple_parameter.cpp`

## 이 장에서 하는 일

빈 폴더에서 시작해 ROS 2 작업 공간 `~/bumperbot_ws`를 만들고, 그 안에 C++ 패키지 `bumperbot_cpp_examples`를 `package.xml`과 `CMakeLists.txt`부터 손으로 쳐서 완성한다. 그 위에 토픽 퍼블리셔, 서브스크라이버, 파라미터 노드를 하나씩 얹는다. 마지막으로 Python 패키지(`ament_python`)를 한 번만 만들어 C++ 패키지와 구조를 비교한다.

이 장의 목적은 노드 로직이 아니라 **패키지 구조**다. 뒤 장에서 만드는 `bumperbot_controller`, `bumperbot_firmware` 같은 패키지는 모두 여기서 배우는 `package.xml` + `CMakeLists.txt` + `install()` 구조 위에 선다. ros2_control의 설정 파일(yaml)과 launch 파일을 패키지에 설치하는 일도 같은 `install()` 명령으로 한다.

이 가이드의 나머지는 C++ 노드만 친다. Python 노드는 이 장의 단계 6에서 한 번 만들어 보고 끝낸다.

## 이번 장의 파일

```text
~/bumperbot_ws/
├── src/
│   ├── bumperbot_cpp_examples/            [새]
│   │   ├── CMakeLists.txt                 [새]
│   │   ├── package.xml                    [새]
│   │   ├── LICENSE                        [새] (ros2 pkg create가 만든다)
│   │   ├── include/bumperbot_cpp_examples/  (비어 있다. 04장에서 쓴다)
│   │   └── src/
│   │       ├── simple_publisher.cpp       [새]
│   │       ├── simple_subscriber.cpp      [새]
│   │       └── simple_parameter.cpp       [새]
│   └── bumperbot_py_examples/             [새]
│       ├── LICENSE                        [새] (ros2 pkg create가 만든다)
│       ├── package.xml                    [새]
│       ├── setup.py                       [새]
│       ├── setup.cfg                      [새]
│       ├── resource/bumperbot_py_examples [새] (빈 파일)
│       ├── bumperbot_py_examples/
│       │   ├── __init__.py                [새] (빈 파일)
│       │   └── simple_publisher.py        [새]
│       └── test/                          [새] (ros2 pkg create가 만든다)
├── build/    (colcon이 만든다)
├── install/  (colcon이 만든다)
└── log/      (colcon이 만든다)
```

## 준비

ROS 2 Jazzy가 `/opt/ros/jazzy`에 설치돼 있어야 한다. 새 터미널을 열 때마다 아래를 먼저 한다. `~/.bashrc`에 첫 줄을 넣어 두면 편하다.

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
```

기대 결과는 다음 한 줄이다.

```text
jazzy
```

같은 네트워크에 다른 ROS 2 컴퓨터가 있으면 서로의 토픽이 보일 수 있다. 그럴 때는 두 컴퓨터가 다른 `ROS_DOMAIN_ID`(0~101)를 쓰도록 `export ROS_DOMAIN_ID=<번호>`를 한다(저장소 코드에는 없지만 일반 지식이다).

## 단계 1. 작업 공간 만들기

### 왜 필요한가

ROS 2 패키지는 한 폴더의 `src/` 아래에 모아 두고 `colcon`이 한꺼번에 빌드한다. `colcon`은 `src/` 아래를 훑어 `package.xml`이 있는 폴더를 패키지로 인식하고, `package.xml`에 적힌 의존 관계로 빌드 순서를 정한다.

빌드 결과는 `/opt/ros/jazzy`(설치된 ROS, underlay)를 고치지 않고 작업 공간 안의 `install/`(overlay)에 쌓인다. 내가 만든 패키지를 `ros2 run`이 찾으려면 이 `install/`을 환경 변수에 올려야 하고, 그 일을 `source install/setup.bash`가 한다.

### 명령

```bash
mkdir -p ~/bumperbot_ws/src
cd ~/bumperbot_ws
colcon build --symlink-install
```

기대 결과는 다음과 같다. 패키지가 없으므로 빌드할 것이 없다.

```text

Summary: 0 packages finished [0.08s]
```

### 만들어진 것

```bash
ls ~/bumperbot_ws
```

```text
build  install  log  src
```

| 폴더 | 누가 만드나 | 역할 |
|---|---|---|
| `src/` | 사용자 | 소스. 직접 쓰는 곳은 여기뿐이다 |
| `build/` | colcon | 패키지별 중간 산출물. CMake 캐시, 오브젝트 파일, 컴파일된 실행 파일이 있다. 지워도 다시 빌드하면 된다 |
| `install/` | colcon | 실행에 쓰는 결과. 실행 파일, 헤더, `share/` 파일, `setup.bash`가 있다. `source`하는 대상이다 |
| `log/` | colcon | 빌드 로그. `log/latest_build/<패키지>/`에 `stdout.log`, `stderr.log` 등이 있다 |

`build/`, `install/`, `log/` 안에는 빈 파일 `COLCON_IGNORE`가 하나씩 있다. colcon이 이 폴더들을 패키지 탐색에서 빼기 위한 표식이다(이 역할은 일반 지식이다. 파일이 있다는 것은 확인했다).

빈 작업 공간의 `install/`에는 아직 패키지가 없고 환경 설정 스크립트만 있다.

```bash
ls -A ~/bumperbot_ws/install
```

```text
COLCON_IGNORE
.colcon_install_layout
local_setup.bash
local_setup.ps1
local_setup.sh
_local_setup_util_ps1.py
_local_setup_util_sh.py
local_setup.zsh
setup.bash
setup.ps1
setup.sh
setup.zsh
```

`.colcon_install_layout`의 내용은 `isolated`다. 패키지마다 `install/<패키지 이름>/`에 따로 설치한다는 뜻이다(colcon 기본값).

### `source install/setup.bash`가 하는 일

`install/setup.bash`를 열어 보면 하는 일이 두 가지다.

1. `/opt/ros/jazzy/local_setup.bash`를 먼저 source한다. 이 작업 공간을 빌드할 때 `/opt/ros/jazzy`가 source돼 있었기 때문에 그 경로가 파일에 박혀 있다(`COLCON_CURRENT_PREFIX="/opt/ros/jazzy"`).
2. 그다음 이 작업 공간의 `local_setup.bash`를 source한다.

그래서 `setup.bash` 한 번으로 underlay(`/opt/ros/jazzy`) 위에 overlay(`install/`)가 얹힌다. `local_setup.bash`는 앞의 1번 없이 이 작업 공간 것만 올린다.

환경 변수 `AMENT_PREFIX_PATH`가 바뀌는 모습은 단계 2에서 패키지를 만든 뒤에 확인한다. 패키지가 하나도 없는 지금은 source해도 값이 그대로 `/opt/ros/jazzy`다.

### `--symlink-install`이 바꾸는 것

`colcon build`에 `--symlink-install`을 붙이면 `install/` 안의 파일을 복사하지 않고 `src/`나 `build/`의 원본을 가리키는 심볼릭 링크로 만든다. 그러면 Python 파일, launch 파일, yaml, urdf 같은 **컴파일이 필요 없는 파일**을 고쳤을 때 다시 빌드하지 않아도 바로 반영된다. C++ 소스는 어차피 컴파일해야 하므로 다시 빌드한다. 실제 링크 모습은 단계 2(`package.xml`)와 단계 3(실행 파일), 단계 6(Python)에서 본다.

이 가이드는 모든 빌드를 `colcon build --symlink-install`로 한다.

## 단계 2. C++ 패키지 뼈대

### 왜 필요한가

패키지는 "`package.xml`이 있는 폴더"다. `package.xml`이 없으면 colcon이 그 폴더를 패키지로 보지 않는다. C++ 패키지는 여기에 `CMakeLists.txt`가 더 필요하다. `ros2 pkg create`가 이 둘의 뼈대를 만들어 준다. 뼈대의 각 줄이 무슨 뜻인지 아는 것이 이 단계의 목표다.

### 명령

```bash
cd ~/bumperbot_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 \
  --description "ROS 2 Code Examples" \
  --maintainer-name "Your Name" --maintainer-email "you@example.com" \
  bumperbot_cpp_examples
```

`Your Name`, `you@example.com`은 자기 이름과 이메일로 바꾼다. 옵션을 주지 않으면 `ros2 pkg create`가 git 설정에서 읽은 값을 쓰고 설명과 라이선스에 `TODO`를 넣는다. 이 가이드는 `package.xml`을 그대로 옮겨 적으므로 옵션을 줘서 같은 모양으로 만든다.

기대 결과(줄임)는 다음과 같다.

```text
package name: bumperbot_cpp_examples
package format: 3
version: 0.0.0
description: ROS 2 Code Examples
maintainer: ['Your Name <you@example.com>']
licenses: ['Apache-2.0']
build type: ament_cmake
dependencies: []
creating folder ./bumperbot_cpp_examples
creating ./bumperbot_cpp_examples/package.xml
creating source and include folder
creating folder ./bumperbot_cpp_examples/src
creating folder ./bumperbot_cpp_examples/include/bumperbot_cpp_examples
creating ./bumperbot_cpp_examples/CMakeLists.txt
```

만들어진 **파일**은 `CMakeLists.txt`, `package.xml`, `LICENSE`(Apache-2.0 전문) 셋이다. 만들어진 **폴더**는 빈 `src/`와 빈 `include/bumperbot_cpp_examples/` 둘이다. `include/`는 04장에서 헤더를 둘 때 쓴다.

### `package.xml` 설명

아래는 `ros2 pkg create`가 만든 그대로의 파일이다.

> 파일: `bumperbot_cpp_examples/package.xml` — 전체 (단계 2 시점)
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bumperbot_cpp_examples</name>
  <version>0.0.0</version>
  <description>ROS 2 Code Examples</description>
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

`package.xml`은 colcon, ament, rosdep, `ros2` 명령이 읽는 패키지의 신분증이다.

| 줄 | 뜻 |
|---|---|
| `<?xml version="1.0"?>` | XML 문서 선언 |
| `<?xml-model href=".../package_format3.xsd" ...?>` | 이 파일이 따르는 스키마(format 3) 주소. 편집기가 문법 검사에 쓴다. 없어도 colcon은 동작한다 |
| `<package format="3">` | `package.xml` 형식 버전. `ros2 pkg create`의 기본값이 3이다. 이 가이드에서 쓰는 `<member_of_group>`(06장), 조건부 의존성 같은 기능이 format 3에 있다(일반 지식) |
| `<name>` | 패키지 이름. 폴더 이름, `CMakeLists.txt`의 `project()` 이름과 같게 둔다. colcon과 `ros2 run <패키지> <실행 파일>`이 이 이름을 쓴다 |
| `<version>` | 버전. 이 가이드에서는 쓰지 않는다 |
| `<description>`, `<maintainer>`, `<license>` | 설명, 관리자, 라이선스. `<license>`에는 ament 라이선스 식별자(`Apache-2.0` 등)를 쓴다. `ros2 pkg create --license ?`로 목록이 나온다 |
| `<buildtool_depend>ament_cmake</buildtool_depend>` | 이 패키지를 **빌드하는 도구**. `CMakeLists.txt`가 `find_package(ament_cmake)`로 쓴다 |
| `<test_depend>` | 테스트할 때만 필요한 패키지. `ament_lint_auto`와 `ament_lint_common`은 코드 모양을 검사하는 린터 모음이다. 이 가이드는 `colcon test`를 쓰지 않으므로 사실상 쓰이지 않는다 |
| `<export><build_type>ament_cmake</build_type></export>` | colcon이 이 패키지를 **어떤 방식으로 빌드할지** 읽는 곳. `ament_cmake`면 CMake를, `ament_python`이면 `setup.py`를 쓴다. 값이 틀리면 colcon이 엉뚱한 파일을 찾는다(아래 "자주 나는 오류") |

의존성 태그는 다음처럼 나뉜다. 단계 3부터 `<depend>`를 쓴다.

| 태그 | 언제 필요한가 | 예 |
|---|---|---|
| `<buildtool_depend>` | 빌드를 돌리는 도구 | `ament_cmake` |
| `<build_depend>` | 컴파일할 때(헤더, 라이브러리) | `rosidl_default_generators`(06장) |
| `<exec_depend>` | 실행할 때만 | `rosidl_default_runtime`(06장) |
| `<test_depend>` | 테스트할 때만 | `ament_lint_auto` |
| `<depend>` | 컴파일과 실행에 모두 필요. `build_depend`, `build_export_depend`(이 패키지에 의존하는 쪽도 헤더를 보려면 필요한 빌드 의존성), `exec_depend`를 한 줄로 쓴 것이다(일반 지식) | `rclcpp`, `std_msgs` |

`package.xml`의 의존성은 두 곳에서 읽힌다. colcon은 이것으로 **빌드 순서**를 정하고, rosdep은 빠진 패키지를 설치한다. `CMakeLists.txt`의 `find_package()`는 CMake에게 헤더와 라이브러리 위치를 찾게 하는 일이고, `package.xml`의 선언과는 별개다. 둘 다 써 줘야 한다.

### `CMakeLists.txt` 설명

> 파일: `bumperbot_cpp_examples/CMakeLists.txt` — 전체 (단계 2 시점)
```cmake
cmake_minimum_required(VERSION 3.8)
project(bumperbot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

| 줄 | 뜻 |
|---|---|
| `cmake_minimum_required(VERSION 3.8)` | 필요한 CMake 최소 버전 |
| `project(bumperbot_cpp_examples)` | CMake 프로젝트 이름. `package.xml`의 `<name>`과 같아야 한다. 뒤에서 `${PROJECT_NAME}`으로 다시 쓴다 |
| `add_compile_options(-Wall -Wextra -Wpedantic)` | GCC나 Clang일 때 경고를 많이 켠다. 경고가 없는 코드를 쓰는 습관을 들이려는 것이다 |
| `find_package(ament_cmake REQUIRED)` | ament의 CMake 매크로(`ament_package`, `ament_target_dependencies` 등)를 불러온다. `REQUIRED`는 없으면 설정 단계에서 바로 멈추라는 뜻이다 |
| `# find_package(<dependency> REQUIRED)` | 의존하는 패키지를 여기에 적으라는 안내 주석. 단계 3부터 채운다 |
| `if(BUILD_TESTING) ... endif()` | `BUILD_TESTING` 옵션이 켜져 있을 때만 린터를 불러온다. 이 옵션의 기본값은 **켜짐**이라 보통의 `colcon build`에서도 이 블록이 실행된다(`build/<패키지>/CMakeCache.txt`에 `BUILD_TESTING:BOOL=ON`이 있고, 설치된 `ament_cmake_test-extras.cmake`가 `option(BUILD_TESTING ... ON)`으로 정의한다). 끄려면 `colcon build --cmake-args -DBUILD_TESTING=OFF`를 쓴다. 이 블록은 린터 테스트를 **등록**할 뿐 린터는 `colcon test`에서 돌아간다(일반 지식). 두 개의 `set(... TRUE)`는 저작권 검사와 cpplint를 끈다 |
| `ament_package()` | **반드시 마지막에 둔다.** 이 패키지를 다른 패키지가 `find_package()`로 찾을 수 있게 하는 파일(`<pkg>Config.cmake`), 패키지 존재를 알리는 표식 파일, `package.xml` 설치, 환경 변수 훅을 `install/`에 만든다 |

`ament_package()`가 만드는 것은 `install/`에서 확인한다. 아래는 빌드한 뒤 모습이다.

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
```

기대 결과는 다음과 같다.

```text
Starting >>> bumperbot_cpp_examples
Finished <<< bumperbot_cpp_examples [0.60s]

Summary: 1 package finished [0.67s]
```

빈 패키지가 ROS 환경에 등록됐는지 본다.

```bash
ros2 pkg list | grep bumperbot
ros2 pkg prefix bumperbot_cpp_examples
ros2 pkg executables bumperbot_cpp_examples
echo $AMENT_PREFIX_PATH
```

```text
bumperbot_cpp_examples
/home/<user>/bumperbot_ws/install/bumperbot_cpp_examples
/home/<user>/bumperbot_ws/install/bumperbot_cpp_examples:/opt/ros/jazzy
```

`ros2 pkg executables`는 실행 파일이 없으므로 아무것도 출력하지 않는다. 마지막 줄이 `source`의 효과다. 이 작업 공간의 패키지 폴더가 `/opt/ros/jazzy` 앞에 붙었다. `ros2 pkg list`와 `ros2 pkg prefix`는 `AMENT_PREFIX_PATH`의 각 폴더에서 `share/ament_index/resource_index/packages/<패키지 이름>` 파일을 찾는다. 이 파일이 `ament_package()`가 만든 "이 패키지가 여기 있다"는 표식이다.

`install/`에 무엇이 생겼는지 본다.

```bash
cd ~/bumperbot_ws/install/bumperbot_cpp_examples
find . -type f -o -type l | sort | grep -v -E '/(hook|environment)/'
```

```text
./share/ament_index/resource_index/package_run_dependencies/bumperbot_cpp_examples
./share/ament_index/resource_index/packages/bumperbot_cpp_examples
./share/ament_index/resource_index/parent_prefix_path/bumperbot_cpp_examples
./share/bumperbot_cpp_examples/cmake/bumperbot_cpp_examplesConfig.cmake
./share/bumperbot_cpp_examples/cmake/bumperbot_cpp_examplesConfig-version.cmake
./share/bumperbot_cpp_examples/local_setup.bash
./share/bumperbot_cpp_examples/local_setup.dsv
./share/bumperbot_cpp_examples/local_setup.sh
./share/bumperbot_cpp_examples/local_setup.zsh
./share/bumperbot_cpp_examples/package.bash
./share/bumperbot_cpp_examples/package.dsv
./share/bumperbot_cpp_examples/package.ps1
./share/bumperbot_cpp_examples/package.sh
./share/bumperbot_cpp_examples/package.xml
./share/bumperbot_cpp_examples/package.zsh
./share/colcon-core/packages/bumperbot_cpp_examples
```

(`hook/`, `environment/` 폴더는 뺐다.) 이것이 전부 `ament_package()`와 colcon이 만든 것이다. `packages/bumperbot_cpp_examples`가 위에서 말한 표식이고, `cmake/...Config.cmake`는 다른 패키지가 `find_package(bumperbot_cpp_examples)`로 이 패키지를 찾게 한다.

`--symlink-install`의 효과는 `package.xml`에서 보인다.

```bash
ls -l ~/bumperbot_ws/install/bumperbot_cpp_examples/share/bumperbot_cpp_examples/package.xml
```

```text
... package.xml -> /home/<user>/bumperbot_ws/src/bumperbot_cpp_examples/package.xml
```

`install/`의 `package.xml`은 복사본이 아니라 `src/`의 원본을 가리키는 링크다.

## 단계 3. 퍼블리셔 노드 `simple_publisher`

### 왜 필요한가

패키지 뼈대가 실제로 동작하는지 가장 작은 노드로 확인한다. 1초에 한 번 `chatter` 토픽에 문자열을 내보내는 노드다. 이 단계에서 "C++ 파일 하나가 `ros2 run`으로 실행되기까지" 필요한 `CMakeLists.txt` 줄과 `package.xml` 줄을 모두 익힌다.

### 코드

파일을 만드는 순서는 소스, `CMakeLists.txt`, `package.xml`이다.

> 파일: `bumperbot_cpp_examples/src/simple_publisher.cpp` — 전체
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>


using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("simple_publisher"), counter_(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
  }

  void timerCallback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS 2 - counter:" + std::to_string(counter_++);
    pub_->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  unsigned int counter_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimplePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 코드 설명

| 부분 | 뜻 |
|---|---|
| `#include <rclcpp/rclcpp.hpp>` | ROS 2 C++ 클라이언트 라이브러리(`rclcpp`) 헤더. `rclcpp::Node` 등이 들어 있다 |
| `#include <std_msgs/msg/string.hpp>` | 메시지 타입 `std_msgs/msg/String`. 필드는 `data`(문자열) 하나다 |
| `using namespace std::chrono_literals;` | `1s`처럼 단위가 붙은 시간 표기를 쓰게 한다 |
| `class SimplePublisher : public rclcpp::Node` | `rclcpp::Node`를 상속한 클래스. 노드가 하는 일(퍼블리셔, 타이머, 로그)을 이 클래스의 멤버로 둔다 |
| `Node("simple_publisher"), counter_(0)` | 부모 생성자에 **노드 이름**을 넘기고, 멤버 `counter_`를 0으로 초기화한다(생성자 이름 뒤 `:` 목록을 초기화 목록이라 한다). `ros2 node list`에 `/simple_publisher`로 보이는 이름이 이것이다 |
| `create_publisher<std_msgs::msg::String>("chatter", 10)` | `chatter` 토픽에 `String`을 내보내는 퍼블리셔를 만든다. `10`은 큐 깊이(QoS history depth)다. 받는 쪽이 느릴 때 쌓아 둘 메시지 수다 |
| `create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this))` | 1초마다 `timerCallback`을 부르는 타이머. `std::bind(&클래스::함수, this)`는 "이 객체의 이 멤버 함수를 인자 없이 부를 수 있는 호출 가능 객체"로 묶는 C++ 표현이다 |
| `RCLCPP_INFO(get_logger(), "...")` | 노드 로거로 INFO 로그를 터미널에 낸다 |
| `message.data = "..." + std::to_string(counter_++)` | `counter_`를 문자열로 바꿔 붙이고 1 올린다 |
| `pub_->publish(message)` | 메시지를 내보낸다 |
| `rclcpp::Publisher<...>::SharedPtr pub_` | `std::shared_ptr`의 별칭. ROS 객체는 대부분 공유 포인터로 들고 다닌다. 멤버로 잡아 두어야 노드가 살아 있는 동안 퍼블리셔도 산다 |
| `rclcpp::init` → `make_shared` → `spin` → `shutdown` | `main`의 정해진 순서다. `init`이 ROS 통신을 준비하고, `spin`이 타이머·구독 콜백을 부르며 Ctrl+C가 올 때까지 돈다 |

### `CMakeLists.txt` 수정

`ros2 pkg create`가 만든 파일에서 다음을 한다.

1. 주석(`# find dependencies` 등)과 `if(BUILD_TESTING) ... endif()` 블록을 지운다. 이 가이드는 린터를 돌리지 않는다.
2. `find_package(ament_cmake REQUIRED)` 아래에 `rclcpp`, `std_msgs`를 찾는 줄을 넣는다.
3. 실행 파일을 정의하고 `install()`을 넣는다.

결과는 다음과 같다. 전체를 그대로 쓴다.

> 파일: `bumperbot_cpp_examples/CMakeLists.txt` — 전체 (단계 3 시점)
```cmake
cmake_minimum_required(VERSION 3.8)
project(bumperbot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

install(TARGETS
  simple_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

| 줄 | 뜻 |
|---|---|
| `find_package(rclcpp REQUIRED)`, `find_package(std_msgs REQUIRED)` | `#include <rclcpp/rclcpp.hpp>`와 `<std_msgs/...>`가 쓰는 헤더·라이브러리 위치를 CMake에 알린다. 소스에서 `#include`하는 패키지는 모두 여기에 있어야 한다 |
| `add_executable(simple_publisher src/simple_publisher.cpp)` | `src/simple_publisher.cpp`를 컴파일해 `simple_publisher`라는 실행 파일을 만든다. 첫 인자가 **실행 파일 이름**이고 `ros2 run <패키지> <이 이름>`에 쓰인다 |
| `ament_target_dependencies(simple_publisher rclcpp std_msgs)` | 이 실행 파일에 `rclcpp`, `std_msgs`의 include 경로와 라이브러리를 연결한다. 이 줄이 없으면 `#include <rclcpp/rclcpp.hpp>`를 찾지 못한다 |
| `install(TARGETS simple_publisher DESTINATION lib/${PROJECT_NAME})` | 만든 실행 파일을 `install/<패키지>/lib/<패키지>/`에 설치한다 |

**왜 `lib/${PROJECT_NAME}`이어야 하나.** `ros2 run`은 ament 표식 파일로 패키지 폴더(`install/bumperbot_cpp_examples`)를 찾은 다음 그 아래 `lib/<패키지 이름>/`에서 실행 파일을 찾는다. `DESTINATION bin`으로 바꿔 빌드하고 실행해 보면 다음과 같이 된다.

```text
No executable found
```

`ros2 pkg executables`에도 나오지 않는다. 실행 파일은 빌드는 되지만 `ros2`가 찾는 자리에 없는 것이다.

`ament_target_dependencies` 대신 `target_link_libraries(simple_publisher PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})` 같은 타깃 방식도 쓸 수 있고, Jazzy에서 빌드되는 것을 확인했다. 이 가이드는 참조 저장소와 같게 `ament_target_dependencies`를 쓴다. 이 방식은 Jazzy에서 경고 없이 빌드된다. Jazzy 다음 배포판인 Kilted부터는 `ament_target_dependencies`가 deprecated이고 `target_link_libraries`에 타깃을 직접 적는 방식이 권장된다(저장소 코드와 설치본에는 없는 일반 지식이다). 그래서 다른 배포판으로 옮길 때는 이 줄을 `target_link_libraries`로 바꾸게 된다.

### `package.xml` 수정

`<buildtool_depend>` 아래에 `rclcpp`, `std_msgs` 의존성을 넣는다. 전체는 다음과 같다.

> 파일: `bumperbot_cpp_examples/package.xml` — 전체 (단계 3 시점)
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bumperbot_cpp_examples</name>
  <version>0.0.0</version>
  <description>ROS 2 Code Examples</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`CMakeLists.txt`의 `find_package(rclcpp)`, `find_package(std_msgs)`와 짝이다. `<depend>`가 없으면 이 패키지만 따로 빌드할 때는 문제가 없어 보이다가, 작업 공간 안의 다른 패키지에 의존할 때 빌드 순서가 어긋난다. 이 일은 06장에서 직접 보게 된다.

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 run bumperbot_cpp_examples simple_publisher
```

기대 결과는 다음과 같다. 단계 2에서 `source`한 터미널이면 다시 하지 않아도 된다. 새 터미널이면 `source`부터 한다.

```text
[INFO] [1795238921.105759956] [simple_publisher]: Publishing at 1 Hz
```

이 터미널(터미널 A)은 그대로 두고 **새 터미널(터미널 B)** 을 연다. 새 터미널에서는 처음 한 번 `source /opt/ros/jazzy/setup.bash`와 `source ~/bumperbot_ws/install/setup.bash`를 한다. 터미널 B에서 다음을 한다. 모두 바로 끝나는 명령이다.

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /chatter --once
```

```text
/simple_publisher
```

```text
/chatter [std_msgs/msg/String]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

```text
data: Hello ROS 2 - counter:4
---
```

`counter:` 뒤의 번호는 퍼블리셔를 띄운 지 얼마나 지났는지에 따라 다르다(예: 4). 이어서 같은 터미널 B에서 주기를 잰다. 이 명령은 **끝나지 않는다.** 몇 줄이 나온 뒤 Ctrl+C로 끈다.

```bash
ros2 topic hz /chatter
```

```text
average rate: 1.000
	min: 1.000s max: 1.000s std dev: 0.00007s window: 3
```

통과 조건: 노드 이름 `/simple_publisher`, 토픽 `/chatter [std_msgs/msg/String]`, 주기 약 1.000 Hz다. `/parameter_events`와 `/rosout`은 모든 노드가 자동으로 만드는 토픽이다.

`--symlink-install`의 효과를 실행 파일에서 본다.

```bash
ls -l ~/bumperbot_ws/install/bumperbot_cpp_examples/lib/bumperbot_cpp_examples/
```

```text
... simple_publisher -> /home/<user>/bumperbot_ws/build/bumperbot_cpp_examples/simple_publisher
```

`install/`의 실행 파일은 `build/`에서 컴파일된 파일을 가리키는 링크다. 소스를 고치고 다시 `colcon build`하면 `build/`의 파일이 새로 만들어지고 링크는 그대로다.

터미널 A의 퍼블리셔는 Ctrl+C로 끝낸다.

## 단계 4. 서브스크라이버 노드 `simple_subscriber`

### 왜 필요한가

노드 두 개가 토픽으로 연결되는 가장 작은 예다. 이후 장의 거의 모든 노드(제어기, 오도메트리)가 "토픽을 받아서 무언가 계산해 내보내는" 구조이므로 구독 콜백의 모양을 먼저 익힌다. 패키지 관점에서는 실행 파일이 둘 이상일 때 `CMakeLists.txt`가 어떻게 자라는지 보는 단계다.

### 코드

> 파일: `bumperbot_cpp_examples/src/simple_subscriber.cpp` — 전체
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber() : Node("simple_subscriber")
  {
    sub_ = create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  void msgCallback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data.c_str());
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### 코드 설명

| 부분 | 뜻 |
|---|---|
| `using std::placeholders::_1;` | `std::bind`에서 "첫 번째 인자 자리"를 뜻하는 `_1`을 쓰게 한다 |
| `create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1))` | `chatter`를 구독하고, 메시지가 올 때마다 `msgCallback`을 부른다. `_1` 자리에 받은 메시지가 들어간다. 큐 깊이 `10`은 퍼블리셔와 같은 의미다 |
| `void msgCallback(const std_msgs::msg::String &msg) const` | 콜백의 모양. 메시지를 상수 참조로 받는다. 멤버를 바꾸지 않으므로 `const`다 |
| `RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data.c_str())` | `<<`로 이어 붙이는 형태의 로그 매크로다 |

### `CMakeLists.txt` 수정

`simple_publisher` 블록 아래에 같은 모양으로 `simple_subscriber` 블록을 추가하고, `install(TARGETS ...)` 목록에 이름을 더한다. `package.xml`은 바뀌지 않는다(같은 `rclcpp`, `std_msgs`만 쓴다).

> 파일: `bumperbot_cpp_examples/CMakeLists.txt` — 수정 (단계 4 시점)
```cmake
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

install(TARGETS
  simple_publisher
  simple_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
```

실행 파일 하나당 `add_executable`과 `ament_target_dependencies` 한 쌍이 필요하고, 이름은 `install(TARGETS ...)` 목록에도 빠짐없이 넣는다. 목록에서 빠지면 빌드는 되지만 `ros2 run`이 찾지 못한다(위의 `No executable found`와 같은 증상이다).

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
```

터미널 A에서 퍼블리셔를 띄운다. 새 터미널을 처음 여는 경우 `source /opt/ros/jazzy/setup.bash`와 `source ~/bumperbot_ws/install/setup.bash`부터 한다. 이 명령은 끝나지 않는다.

```bash
ros2 run bumperbot_cpp_examples simple_publisher
```

터미널 B(처음 한 번 위와 같이 `source`한다)에서 서브스크라이버를 띄운다. 이것도 끝나지 않는다.

```bash
ros2 run bumperbot_cpp_examples simple_subscriber
```

기대 결과(터미널 B)는 다음과 같다. 처음 받는 `counter:` 번호는 두 노드를 띄운 시간차에 따라 다르다. 번호가 1씩 늘면서 1초마다 한 줄이 나오면 된다.

```text
[INFO] [1795238967.160066021] [simple_subscriber]: I heard: Hello ROS 2 - counter:0
[INFO] [1795238968.155369335] [simple_subscriber]: I heard: Hello ROS 2 - counter:1
[INFO] [1795238969.155360415] [simple_subscriber]: I heard: Hello ROS 2 - counter:2
```

두 노드를 계속 띄워 둔 채 **터미널 C**(처음 한 번 `source`한다)에서 노드 정보를 본다. 둘 다 바로 끝나는 명령이다.

```bash
ros2 node info /simple_subscriber
ros2 topic info /chatter
```

```text
/simple_subscriber
  Subscribers:
    /chatter: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /simple_subscriber/describe_parameters: rcl_interfaces/srv/DescribeParameters
    ...
```

```text
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

통과 조건: 서브스크라이버가 `I heard:` 줄을 1초마다 낸다. `ros2 topic info`가 `Publisher count: 1`, `Subscription count: 1`을 보인다. 두 노드는 서로를 모르고 토픽 이름과 타입으로만 연결된다. `ros2 node info`에 `Service Servers`가 일곱 개 보인다. 이 중 여섯 개(`describe_parameters`, `get_parameter_types`, `get_parameters`, `list_parameters`, `set_parameters`, `set_parameters_atomically`)는 모든 노드가 자동으로 갖는 파라미터 서비스이고(다음 단계에서 쓴다), 나머지 하나 `get_type_description`은 노드의 타입 정보를 알려 주는 서비스다. 시험이 끝나면 터미널 A, B의 노드를 Ctrl+C로 끈다.

## 단계 5. 파라미터 노드 `simple_parameter`

참조 저장소에서는 Section4에 있는 파일이다. 기초 개념이라 이 장으로 당겼다. 이름과 코드는 참조 그대로다.

### 왜 필요한가

노드의 동작 값(바퀴 반지름, 제어 주기 등)을 코드에 박지 않고 실행할 때 바꾸게 하는 장치가 파라미터다. 03장에서 `controller_manager`와 제어기의 설정(`bumperbot_controllers.yaml`)이 전부 파라미터이고, launch 파일이 yaml 파일을 노드에 파라미터로 넘겨 준다. 그 구조를 가장 작은 노드로 먼저 본다.

### 코드

> 파일: `bumperbot_cpp_examples/src/simple_parameter.cpp` — 전체
```cpp
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <vector>
#include <string>
#include <memory>


using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node
{
public:
    SimpleParameter() : Node("simple_parameter")
    {
        declare_parameter<int>("simple_int_param", 28);
        declare_parameter<std::string>("simple_string_param", "Antonio");
        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
    }

private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        for(const auto& param : parameters)
        {
            if(param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed! New value is " << param.as_int());
                result.successful = true;
            }
            if(param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed! New value is " << param.as_string());
                result.successful = true;
            }
        }
        
        return result;
    }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleParameter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 코드 설명

| 부분 | 뜻 |
|---|---|
| `#include <rcl_interfaces/msg/set_parameters_result.hpp>` | 파라미터 변경 콜백이 돌려주는 결과 타입. 그래서 `rcl_interfaces` 패키지가 필요하다 |
| `declare_parameter<int>("simple_int_param", 28)` | 파라미터를 **선언**한다. 이름, 타입, 기본값을 정한다. 선언하지 않은 파라미터는 `ros2 param set`으로도 설정할 수 없다 |
| `add_on_set_parameters_callback(...)` | 파라미터가 바뀌기 **직전**에 불리는 콜백을 등록한다. 이 콜백이 `successful = true`를 돌려줘야 변경이 반영되고, `false`면 거부된다 |
| `OnSetParametersCallbackHandle::SharedPtr param_callback_handle_` | 등록 핸들. 멤버로 들고 있어야 콜백이 유지된다 |
| `param.get_name()`, `param.get_type()`, `param.as_int()` | 바뀌는 파라미터의 이름, 타입, 값 |
| `rcl_interfaces::msg::SetParametersResult result;` | 결과 구조체. `successful` 기본값이 `false`다 |

참조 코드는 이름이 `simple_int_param`, `simple_string_param`인 경우에만 `successful = true`를 넣는다. 그 밖의 파라미터(`use_sim_time` 등)를 바꾸려 하면 거부된다(아래 시험에서 확인한다).

### `CMakeLists.txt`, `package.xml` 수정

`rcl_interfaces`를 찾고(`find_package`), 실행 파일을 추가하고, `install` 목록에 넣는다.

아래 "수정" 블록의 `# ...` 줄은 "원래 있던 줄을 그대로 둔다"는 뜻이다. 파일에 치지 않는다. 이 가이드의 다른 "수정" 블록에서 `...`, `# ...`, `// ...`, `<!-- ... -->`도 같은 뜻이다.

> 파일: `bumperbot_cpp_examples/CMakeLists.txt` — 수정
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
# ...
add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter rclcpp rcl_interfaces)

install(TARGETS
  simple_publisher
  simple_subscriber
  simple_parameter
  DESTINATION lib/${PROJECT_NAME}
)
```

`package.xml`에도 `<depend>rcl_interfaces</depend>`를 더한다.

> 파일: `bumperbot_cpp_examples/package.xml` — 수정
```xml
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>rcl_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
```

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
```

터미널 A에서 노드를 띄운다. 이 명령은 끝나지 않는다. 새 터미널이면 `source /opt/ros/jazzy/setup.bash`와 `source ~/bumperbot_ws/install/setup.bash`를 먼저 한다.

```bash
ros2 run bumperbot_cpp_examples simple_parameter
```

터미널 B(처음 한 번 `source`한다)에서 다음을 한다. 모두 바로 끝나는 명령이다.

```bash
ros2 param list /simple_parameter
ros2 param get /simple_parameter simple_int_param
ros2 param get /simple_parameter simple_string_param
ros2 param set /simple_parameter simple_int_param 30
ros2 param set /simple_parameter simple_string_param "ROS"
```

```text
  qos_overrides./parameter_events.publisher.depth
  ...
  simple_int_param
  simple_string_param
  start_type_description_service
  use_sim_time
```

```text
Integer value is: 28
String value is: Antonio
Set parameter successful
Set parameter successful
```

터미널 A에는 콜백이 낸 로그가 나온다.

```text
[INFO] [1795238992.441125150] [simple_parameter]: Param simple_int_param changed! New value is 30
[INFO] [1795238993.286112926] [simple_parameter]: Param simple_string_param changed! New value is ROS
```

거부되는 경우도 터미널 B에서 확인한다.

```bash
ros2 param set /simple_parameter simple_int_param hello
ros2 param set /simple_parameter nope 1
ros2 param set /simple_parameter use_sim_time true
```

```text
Setting parameter failed: Wrong parameter type, parameter {simple_int_param} is of type {integer}, setting it to {string} is not allowed.
Setting parameter failed: parameter 'nope' cannot be set because it was not declared
Setting parameter failed
```

첫째는 선언한 타입(정수)과 다른 타입이라서, 둘째는 선언하지 않아서 거부된다. 셋째는 `use_sim_time`이 이름 검사에 걸려 `successful`이 `false`로 남아 거부된 것이다.

**실행할 때 값을 주는 방법 두 가지.** 같은 이름의 노드를 두 개 띄우지 않도록 먼저 터미널 A의 노드를 Ctrl+C로 끈다. 그다음 터미널 A에서 값을 주며 다시 띄운다. 이 명령은 끝나지 않는다.

```bash
ros2 run bumperbot_cpp_examples simple_parameter --ros-args -p simple_int_param:=50 -p simple_string_param:=Hello
```

터미널 B에서 값을 읽는다.

```bash
ros2 param get /simple_parameter simple_int_param
```

```text
Integer value is: 50
```

yaml 파일로도 줄 수 있다. 파일의 최상위 키는 **노드 이름**이고 그 아래 `ros__parameters`가 온다. 터미널 B에서 파일을 만든다.

```bash
cat > /tmp/simple_parameter.yaml <<'EOF'
simple_parameter:
  ros__parameters:
    simple_int_param: 77
    simple_string_param: from_yaml
EOF
```

터미널 A의 노드를 Ctrl+C로 끄고 yaml을 주며 다시 띄운다. 끝나지 않는 명령이다.

```bash
ros2 run bumperbot_cpp_examples simple_parameter --ros-args --params-file /tmp/simple_parameter.yaml
```

터미널 B에서 읽는다.

```bash
ros2 param get /simple_parameter simple_string_param
```

```text
String value is: from_yaml
```

통과 조건: `ros2 param get`이 기본값 28/Antonio를 보이고, `set`하면 노드 로그에 `changed!`가 찍히며, 타입이 다르거나 선언하지 않은 이름은 거부되고, `-p`와 `--params-file`로 시작값을 줄 수 있다. 시작값은 `declare_parameter` 때 적용되며 이 노드에서는 그때 변경 콜백이 아직 등록되지 않아 `changed!` 로그가 찍히지 않는다(시험에서 확인했다).

시험이 끝나면 터미널 A의 노드를 Ctrl+C로 끈다.

이 yaml 형식이 03장에서 `bumperbot_controllers.yaml`로, launch 파일의 `parameters=[...]` 인자로 다시 나온다.

## 단계 6. Python 패키지 구조 한 번 만들어 보기

이 가이드는 이후 Python 노드를 치지 않는다. 참조 저장소에는 모든 노드의 Python판이 있지만 여기서는 패키지 구조를 비교하려고 `simple_publisher` 하나만 옮긴다. 이후 각 노드 설명 끝에 Python판의 참조 위치만 적는다.

### 왜 필요한가

ROS 2 패키지는 C++(`ament_cmake`)와 Python(`ament_python`)의 구조가 완전히 다르다. 어느 쪽이든 colcon이 같은 작업 공간에서 빌드하고 `ros2 run`이 같은 방식으로 찾는다는 점은 같고, 그것을 달성하는 파일이 다르다. 두 구조를 나란히 봐 두면 `CMakeLists.txt`의 각 줄이 왜 필요한지 대비해서 이해된다. 참조 저장소의 `bumperbot_controller`에서 쓰는 Python 노드를 읽을 때도 도움이 된다.

### 명령

패키지 이름을 **먼저** 쓴다. `--dependencies`는 값을 여러 개 받기 때문에 이름을 뒤에 두면 이름까지 의존성으로 읽는다.

```bash
cd ~/bumperbot_ws/src
ros2 pkg create bumperbot_py_examples --build-type ament_python --license Apache-2.0 \
  --description "ROS 2 Code Examples" \
  --maintainer-name "Your Name" --maintainer-email "you@example.com" \
  --dependencies rclpy std_msgs
```

기대 결과(줄임)는 다음과 같다.

```text
package name: bumperbot_py_examples
build type: ament_python
dependencies: ['rclpy', 'std_msgs']
creating ./bumperbot_py_examples/package.xml
creating source folder
creating folder ./bumperbot_py_examples/bumperbot_py_examples
creating ./bumperbot_py_examples/setup.py
creating ./bumperbot_py_examples/setup.cfg
creating folder ./bumperbot_py_examples/resource
creating ./bumperbot_py_examples/resource/bumperbot_py_examples
creating ./bumperbot_py_examples/bumperbot_py_examples/__init__.py
creating folder ./bumperbot_py_examples/test
creating ./bumperbot_py_examples/test/test_copyright.py
creating ./bumperbot_py_examples/test/test_flake8.py
creating ./bumperbot_py_examples/test/test_pep257.py
```

### 뼈대 파일 설명

`ros2 pkg create`가 만든 파일을 하나씩 본다. 노드 파일(`simple_publisher.py`)은 아직 없다. 아래에서 뼈대를 이해한 뒤 노드를 쓰고 `setup.py`에 등록한다.

**`package.xml`**

> 파일: `bumperbot_py_examples/package.xml` — 전체
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bumperbot_py_examples</name>
  <version>0.0.0</version>
  <description>ROS 2 Code Examples</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

C++ 패키지와 다른 곳은 둘이다. `<buildtool_depend>`가 없고(Python은 CMake가 필요 없다), `<export><build_type>`이 `ament_python`이다. colcon은 이 값을 보고 `setup.py`를 실행한다. `<depend>rclpy</depend>`는 C++의 `rclcpp`에 해당하는 Python 클라이언트 라이브러리다. `<test_depend>` 넷은 단계 2의 린터에 해당한다.

**`setup.py`** — C++의 `CMakeLists.txt` 역할을 한다. 아래는 `ros2 pkg create`가 만든 그대로다. `entry_points`의 `console_scripts` 목록이 비어 있다.

> 파일: `bumperbot_py_examples/setup.py` — 전체 (단계 6 시점)
```python
from setuptools import find_packages, setup

package_name = 'bumperbot_py_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 Code Examples',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
```

| 줄 | 뜻 |
|---|---|
| `packages=find_packages(exclude=['test'])` | 설치할 Python 패키지(`__init__.py`가 있는 폴더)를 찾는다. `test/`는 뺀다. `bumperbot_py_examples/__init__.py`가 없으면 이 폴더가 패키지로 인식되지 않는다 |
| `data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ...]` | `resource/bumperbot_py_examples`(빈 파일)를 `install/<pkg>/share/ament_index/resource_index/packages/`에 복사한다. C++의 `ament_package()`가 만드는 **표식 파일**과 같은 것이다. 이 파일이 있어야 `ros2 pkg list`와 `ros2 run`이 패키지를 찾는다. 파일을 지우고 빌드하면 `error: can't copy '.../resource/bumperbot_py_examples': doesn't exist or not a regular file`로 빌드가 실패한다 |
| `('share/' + package_name, ['package.xml'])` | `package.xml`을 `install/<pkg>/share/<pkg>/`에 설치한다. C++에서는 `ament_package()`가 한다 |
| `install_requires=['setuptools']` | pip 방식 의존성. ROS 패키지 의존성은 `package.xml`에 쓴다 |
| `entry_points={'console_scripts': [...]}` | **실행 파일 등록.** 목록의 한 항목이 실행 파일 하나가 된다. C++의 `add_executable` + `install(TARGETS)`에 해당한다. 지금은 비어 있어서 `ros2 run`으로 실행할 것이 없다. 노드를 쓴 뒤 채운다 |
| `maintainer`, `description`, `license` | 메타데이터. `package.xml`과 같은 값을 쓴다 |

**`setup.cfg`** — 실행 파일을 어디에 설치할지 정한다.

> 파일: `bumperbot_py_examples/setup.cfg` — 전체
```text
[develop]
script_dir=$base/lib/bumperbot_py_examples
[install]
install_scripts=$base/lib/bumperbot_py_examples
```

`console_scripts`로 만든 스크립트를 기본값(`bin/`)이 아니라 `lib/bumperbot_py_examples/`에 설치하라는 설정이다. C++의 `DESTINATION lib/${PROJECT_NAME}`과 같은 이유로 필요하다. 이 파일을 지우고 빌드하면 스크립트가 `install/bumperbot_py_examples/bin/`에 생기고 `ros2 run bumperbot_py_examples simple_publisher`는 `No executable found`를 낸다.

**`resource/bumperbot_py_examples`** — 내용이 없는 파일이다. 위 `data_files`가 복사해 표식으로 쓴다. 이름은 패키지 이름과 같아야 한다.

**`bumperbot_py_examples/__init__.py`** — 내용이 없는 파일이다. Python이 이 폴더를 패키지로 읽게 한다. 노드 소스는 이 폴더에 둔다. 폴더 이름이 패키지 이름과 같다.

**`test/`** — 린터 테스트 세 개. 이 가이드에서는 쓰지 않는다. 지워도 된다.

### 코드

노드 파일 `bumperbot_py_examples/bumperbot_py_examples/simple_publisher.py`를 쓴다.

> 파일: `bumperbot_py_examples/bumperbot_py_examples/simple_publisher.py` — 전체
```python
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(self):
        super().__init__("simple_publisher")
        self.pub_ = self.create_publisher(String, "chatter", 10)
        self.counter_ = 0
        self.frequency_ = 1.0
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "Hello ROS 2 - counter: %d" % self.counter_
        self.pub_.publish(msg)
        self.counter_ += 1


def main():
    rclpy.init()

    simple_publisher = SimplePublisher()
    try:
        rclpy.spin(simple_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    simple_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

### 코드 설명

C++ `simple_publisher.cpp`와 같은 일을 한다. 참조 코드에서 `import` 한 줄과 `main()`의 마지막 부분만 바꿨다(아래 "참조 코드와 다른 점").

| 부분 | 뜻 |
|---|---|
| `class SimplePublisher(Node)` | `rclpy.node.Node`를 상속한 노드 클래스. C++의 `class SimplePublisher : public rclcpp::Node`에 해당한다 |
| `super().__init__("simple_publisher")` | 부모 생성자에 노드 이름을 넘긴다. C++의 `Node("simple_publisher")`와 같다 |
| `create_publisher(String, "chatter", 10)` | 인자 순서는 메시지 타입, 토픽 이름, 큐 깊이다. C++에서는 타입이 `<...>` 안에 들어가고 이름과 깊이가 인자다 |
| `self.frequency_ = 1.0`, `create_timer(self.frequency_, self.timerCallback)` | `create_timer`의 첫 인자는 **주기(초)** 다. 변수 이름은 `frequency_`지만 값은 주기라서, 1.0일 때만 1 Hz가 된다. 0.5로 바꾸면 2 Hz, 2.0으로 바꾸면 0.5 Hz다. 로그의 `"Publishing at %d Hz"`도 1.0일 때만 맞다. 참조 코드 그대로라서 고치지 않았다 |
| `timerCallback` | 타이머가 부른다. 메시지를 만들어 `publish`하고 `counter_`를 1 올린다 |
| `main()` | `rclpy.init()`, 노드 생성, `rclpy.spin()` 순서다. C++ `main`과 같은 순서다. `try/except`는 Ctrl+C로 `spin`이 끝날 때 트레이스백이 나오지 않게 한다 |
| `if __name__ == '__main__': main()` | 파일을 직접 실행할 때만 `main()`을 부른다. `ros2 run`으로 실행할 때는 `setup.py`의 `entry_points`가 `main`을 직접 부른다 |

Python판의 참조 위치: `Section3_Introduction_to_ROS_2/bumperbot_ws/src/bumperbot_py_examples/bumperbot_py_examples/simple_publisher.py`

### `setup.py`에 실행 파일 등록

`entry_points`의 `console_scripts` 목록에 한 줄을 넣는다. 파일을 열어 `'console_scripts': [` 아래에 넣는다.

> 파일: `bumperbot_py_examples/setup.py` — 수정
```python
    entry_points={
        'console_scripts': [
            'simple_publisher = bumperbot_py_examples.simple_publisher:main',
        ],
    },
```

`'simple_publisher = bumperbot_py_examples.simple_publisher:main'`은 `실행 파일 이름 = 패키지.모듈:함수` 형식이다. "`simple_publisher`라는 실행 파일을 만들고, 실행하면 `bumperbot_py_examples/simple_publisher.py`의 `main()`을 부른다"는 뜻이다. 이 줄이 없으면 파일이 있어도 `ros2 run`이 찾지 못한다.

### ament_cmake와 ament_python 대응표

| 하는 일 | `ament_cmake` (C++) | `ament_python` (Python) |
|---|---|---|
| 빌드 설정 파일 | `CMakeLists.txt` | `setup.py` + `setup.cfg` |
| `colcon`이 보는 빌드 방식 | `package.xml`의 `<build_type>ament_cmake` | `package.xml`의 `<build_type>ament_python` |
| 빌드 도구 의존성 | `<buildtool_depend>ament_cmake` | 없음 |
| 라이브러리 의존성 선언 | `find_package()` + `ament_target_dependencies()` + `package.xml`의 `<depend>` | `package.xml`의 `<depend>` (`import`는 소스에서) |
| 실행 파일 등록 | `add_executable()` + `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` | `entry_points`의 `console_scripts` + `setup.cfg`의 `install_scripts` |
| 소스 위치 | `src/*.cpp`, `include/<패키지>/*.hpp` | `<패키지 이름>/*.py` (+ `__init__.py`) |
| 패키지 표식 파일 | `ament_package()`가 생성 | `resource/<패키지 이름>` 파일을 `data_files`로 설치 |
| `package.xml` 설치 | `ament_package()`가 설치 | `data_files`의 `('share/<패키지>', ['package.xml'])` |
| 다른 파일(launch, yaml) 설치 | `install(DIRECTORY ... DESTINATION share/${PROJECT_NAME})` | `data_files`에 항목 추가 |
| 컴파일 | 있음 (`build/`에 실행 파일) | 없음 |
| 소스를 고친 뒤 | 다시 `colcon build` | `--symlink-install`이면 바로 반영. `setup.py`를 고쳤으면 다시 빌드 |

### 빌드와 시험

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables bumperbot_py_examples
```

기대 결과는 다음과 같다. 경고 없이 빌드된다. `Summary`에 실패가 없으면 된다.

```text
Starting >>> bumperbot_cpp_examples
Starting >>> bumperbot_py_examples
Finished <<< bumperbot_py_examples [0.75s]
Finished <<< bumperbot_cpp_examples [0.13s]

Summary: 2 packages finished [0.89s]
```

```text
bumperbot_py_examples simple_publisher
```

터미널 A에서 노드를 띄운다. 끝나지 않는 명령이다. 단계 3의 C++ 퍼블리셔와 같은 이름(`/simple_publisher`)·같은 토픽(`/chatter`)이므로 **C++ 퍼블리셔가 떠 있으면 먼저 Ctrl+C로 끈다.**

```bash
ros2 run bumperbot_py_examples simple_publisher
```

```text
[INFO] [1795239038.899865250] [simple_publisher]: Publishing at 1 Hz
```

터미널 B(처음 한 번 `source`한다)에서 토픽을 확인한다. 바로 끝나는 명령이다.

```bash
ros2 topic echo /chatter --once
```

```text
data: 'Hello ROS 2 - counter: 4'
---
```

`counter:` 번호는 실행 시점에 따라 다르다. Python판은 `counter:` 뒤에 공백이 있고 값이 따옴표로 감싸여 나온다. 확인이 끝나면 터미널 A의 노드를 Ctrl+C로 끈다. 조용히 끝나면 `main()`의 `try/except`가 동작한 것이다.

`--symlink-install`이 Python 패키지에서 한 일을 본다.

```bash
ls -l ~/bumperbot_ws/install/bumperbot_py_examples/lib/python3.12/site-packages/
cat ~/bumperbot_ws/install/bumperbot_py_examples/lib/python3.12/site-packages/*.egg-link
ls -l ~/bumperbot_ws/build/bumperbot_py_examples/
```

```text
... bumperbot-py-examples.egg-link
/home/<user>/bumperbot_ws/build/bumperbot_py_examples
```

```text
... bumperbot_py_examples -> /home/<user>/bumperbot_ws/src/bumperbot_py_examples/bumperbot_py_examples
... package.xml -> /home/<user>/bumperbot_ws/src/bumperbot_py_examples/package.xml
... setup.py -> ...
```

`install/`에는 `.egg-link` 파일 하나만 있고, `build/`의 `bumperbot_py_examples` 폴더가 `src/`의 소스 폴더를 가리킨다. 그래서 `simple_publisher.py`를 고치고 `ros2 run`만 다시 해도 바뀐 코드가 돈다. `setup.py`의 `entry_points`를 바꿨을 때는 실행 스크립트를 다시 만들어야 하므로 `colcon build`가 필요하다.

`--symlink-install` 없이 같은 패키지를 빌드하면 `install/.../site-packages/bumperbot_py_examples/`에 소스가 복사되고, 빌드 때 `warning: build_py: byte-compiling is disabled, skipping.` 같은 경고가 stderr에 나온다.

## 참조 코드와 다른 점

| 참조 | 이 가이드 | 이유 |
|---|---|---|
| `package.xml`의 `<license>Apache 2.0</license>`, 관리자 `Antonio Brandi` | `<license>Apache-2.0</license>`, 관리자는 자기 이름 | `ros2 pkg create`가 안내하는 ament 라이선스 식별자를 쓴다. 관리자는 사용자 본인 정보다 |
| Python `setup.py`: `packages=[package_name]`, `tests_require=['pytest']`, `maintainer='user'` | `packages=find_packages(exclude=['test'])`, `extras_require={'test': ['pytest']}`, 관리자 정보를 직접 입력 | Jazzy의 `ros2 pkg create`가 만드는 템플릿을 그대로 쓴다 |
| Python `simple_publisher.py`의 `main()`: `rclpy.spin(...)` 뒤에 `destroy_node()`와 `rclpy.shutdown()` | `rclpy.spin(...)`을 `try/except (KeyboardInterrupt, ExternalShutdownException)`로 감싸고 끝에 `rclpy.try_shutdown()` | 참조 코드는 Jazzy에서 Ctrl+C를 누르면 `ExternalShutdownException` 트레이스백이 나오며 `Process exited with failure 1`로 끝난다. 위처럼 고치면 조용히 끝난다 |
| C++ 코드(`simple_publisher.cpp`, `simple_subscriber.cpp`, `simple_parameter.cpp`) | 같다 | 없음 |
| Python판 `simple_subscriber.py`, `simple_parameter.py`가 있다(Section3, Section4) | 싣지 않는다 | 이 가이드는 C++ 노드만 친다. `simple_publisher.py` 하나만 구조 비교용으로 옮겼다 |
| `simple_parameter.cpp`와 `rcl_interfaces` 의존성은 Section4에서 생긴다 | 이 장(단계 5)으로 당겼다 | 파라미터는 기초 개념이고 뒤 장의 제어기 설정에서 쓰인다 |
| `LICENSE` 파일이 없다 | 두 패키지에 `LICENSE`(Apache-2.0 전문)가 있다 | `ros2 pkg create --license Apache-2.0`가 만든다 |
| Python `setup.py`: `license='Apache 2.0'` | `license='Apache-2.0'` | `package.xml`과 같은 식별자로 맞춘다 |
| `ament_target_dependencies` | 같다 | Jazzy에서 경고가 없다 |

## 자주 나는 오류

| 증상 | 원인 | 고치는 법 |
|---|---|---|
| `Package 'bumperbot_cpp_examples' not found` | 이 터미널에서 `source ~/bumperbot_ws/install/setup.bash`를 하지 않았다 | `source`한다. 새 터미널마다 필요하다 |
| `No executable found` | `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})`가 없거나 목적지가 다르다. 실행 파일 이름이 `install()` 목록에서 빠졌다. Python이면 `setup.cfg`가 없거나 `entry_points`에 안 썼다 | `CMakeLists.txt`의 `install()`을 확인하고 다시 빌드한다 |
| `fatal error: rclcpp/rclcpp.hpp: No such file or directory` | `find_package(rclcpp REQUIRED)`나 `ament_target_dependencies(<실행 파일> rclcpp ...)`를 빼먹었다 | 두 줄을 확인한다 |
| `ros2 pkg executables <패키지>`가 출력이 없다 | 실행 파일이 없거나 설치되지 않았다 | `install(TARGETS ...)`와 `ls install/<패키지>/lib/<패키지>/`를 확인한다 |
| `colcon build`가 `setup.py: No such file or directory`로 실패한다 | C++ 패키지의 `package.xml`에서 `<build_type>`이 `ament_python`이다 | `<build_type>ament_cmake</build_type>`로 고친다 |
| `ros2 pkg create`가 `usage: ...`와 `the following arguments are required: package_name`을 출력하고 패키지를 만들지 않는다 | `--dependencies rclpy std_msgs 패키지이름`처럼 이름을 뒤에 썼다. `--dependencies`가 이름까지 의존성으로 읽는다 | 패키지 이름을 `--dependencies`보다 앞에 쓴다 |
| 소스를 고쳤는데 결과가 같다 | C++은 다시 빌드해야 한다. 다시 빌드했는데도 같으면 실행 중인 노드가 옛 프로세스다 | `colcon build` 후 노드를 끄고 다시 띄운다 |
| 새로 만든 패키지를 `ros2`가 못 찾는다 | 이미 열린 터미널은 새 패키지 폴더가 `AMENT_PREFIX_PATH`에 없다 | `source ~/bumperbot_ws/install/setup.bash`를 다시 한다 |

## 정리

| 파일 | 역할 |
|---|---|
| `bumperbot_cpp_examples/package.xml` | 패키지 이름, 의존성, 빌드 방식을 colcon과 rosdep에 알린다 |
| `bumperbot_cpp_examples/CMakeLists.txt` | C++ 실행 파일 3개를 빌드하고 `lib/bumperbot_cpp_examples/`에 설치한다 |
| `bumperbot_cpp_examples/src/simple_publisher.cpp` | `chatter`에 1 Hz로 문자열을 낸다 |
| `bumperbot_cpp_examples/src/simple_subscriber.cpp` | `chatter`를 구독해 로그로 찍는다 |
| `bumperbot_cpp_examples/src/simple_parameter.cpp` | 파라미터 선언과 변경 콜백 |
| `bumperbot_py_examples/{package.xml, setup.py, setup.cfg, resource/, __init__.py}` | `ament_python` 패키지의 구조. `simple_publisher.py`는 C++ 퍼블리셔의 Python판이다 |

이 장에서 익힌 `package.xml`의 `<depend>`, `CMakeLists.txt`의 `find_package` → `add_executable` → `ament_target_dependencies` → `install` 흐름은 이후 모든 패키지에서 반복된다. 다음 장(02장)에서는 로봇 모양을 정의하는 `bumperbot_description` 패키지를 만든다. 그 패키지는 실행 파일이 없고 `urdf/`, `launch/` 폴더를 `install(DIRECTORY ...)`로 설치하는 패키지다.
