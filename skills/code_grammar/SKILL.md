---
name: code_grammar
description: >-
  Explain C and C++ syntax in a given code file at a beginner-friendly level
  with simple standalone examples and real-world ROS 2 (rclcpp) usage patterns,
  referencing https://cppreference.com/ and https://github.com/ros2/rclcpp. Use
  when the user invokes /code_grammar or asks for easy C/C++ grammar
  explanations with simple examples and ROS 2 rclcpp examples.
---

# Code Grammar (`/code_grammar`)

사용자가 제공한 C 또는 C++ 코드 파일 속 문법을 입문자도 이해할 수 있게 쉽게 풀어 설명하고, 단순한 별도 예시 코드와 실제 ROS 2(`rclcpp`) 활용 예시를 함께 제시하는 스킬입니다.

## 레퍼런스 기준

설명의 정확성을 위해 다음 두 레퍼런스를 기준으로 삼습니다. (정확한 표준 사양이나 최신 `rclcpp` API 형태를 확인해야 할 경우 웹 검색 등을 통해 실제 문서를 확인하고, 확실하지 않은 API나 문법 사항을 추측으로 지어내지 않습니다.)

- **C / C++ 표준 문법 레퍼런스**: `https://cppreference.com/`
- **ROS 2 C++ 클라이언트 라이브러리 레퍼런스**: `https://github.com/ros2/rclcpp`

## 수행 절차

사용자가 지정한 C/C++ 파일(경로 또는 첨부/붙여넣은 코드)을 읽고, 코드에 등장하는 핵심 문법 요소별로 아래 3가지 조건을 모두 충족하도록 설명합니다.

### 1. 입문자 눈높이의 문법 설명
- C 또는 C++를 처음 공부하는 사람도 막힘없이 이해할 수 있도록 쉽고 친절하게 설명합니다.
- 어려운 전문 용어만 나열하지 말고, **"이 문법이 무엇인지"**, **"왜 쓰는지(어떤 문제를 해결하는지)"**, **"코드를 어떻게 읽으면 되는지"**를 일상적인 비유나 직관적인 표현으로 풀어씁니다.
- 원본 코드의 어떤 줄/표현에서 해당 문법이 쓰였는지 짚어주고, 필요 시 `cppreference.com` 기준의 정확한 용어·표준(예: C++11, C++17 등)을 간단히 덧붙입니다.

### 2. 이해를 돕기 위한 단순하고 쉬운 별도 예시 코드
- 원본 코드의 복잡한 문맥을 그대로 가져오지 말고, **해당 문법의 원리만 명확히 보여주는 아주 짧고 단순한 새로운 예시 코드**를 작성합니다.
- **작성 원칙**:
  - 복잡한 중첩 구조, 불필요한 최적화, 과도한 템플릿/추상화는 피합니다.
  - 입문자가 한눈에 보고 바로 실행 흐름과 결과를 머릿속으로 따라갈 수 있는 수준(보통 5~15줄 내외의 `main()` 포함 최소 예제)으로 작성합니다.
  - 예시 코드 바로 아래에 실행 시 어떤 동작/결과가 나오는지 1~2줄로 덧붙입니다.

### 3. ROS 2 (`rclcpp`)에서의 실제 활용 예시
- 설명한 문법이 실제 ROS 2(`https://github.com/ros2/rclcpp`) 코드에서는 **어떤 상황에서 어떤 형태로 쓰이는지** 예시 코드와 함께 알려줍니다.
- 예시 매핑 방향:
  - 포인터 / 스마트 포인터(`std::shared_ptr`, `std::make_shared`) → `rclcpp::Node::SharedPtr`, `rclcpp::Publisher<...>::SharedPtr`, `std::make_shared<MyNode>()`
  - 람다 표현식 / `std::bind` / 함수 포인터 → `create_wall_timer(...)`, `create_subscription(...)`의 콜백 등록
  - 상속 / 가상 함수(`public`, `override`) → `class MinimalPublisher : public rclcpp::Node`
  - 네임스페이스 / `using` / `auto` / `std::chrono_literals` → `using namespace std::chrono_literals;`, `500ms` 등
  - C 문법(구조체, 포인터, 콜백, 매크로, 비트/메모리 연산 등) → `rclcpp` 내부 또는 하위 `rcl`/`rcutils` 계층이나 ROS 2 노드/메시지 처리에서 대응되는 활용 패턴
- 각 ROS 2 예시에는 "ROS 2에서 이 문법이 왜 유용한지"를 짧게 설명합니다.

## 출력 형식 가이드

코드에서 핵심 문법 항목(보통 중요도/등장 순서대로 3~7개 내외, 코드 크기에 따라 조절)을 뽑아 각 문법마다 아래 형식으로 정리합니다.

```markdown
### [번호]. <문법 이름> (원본 코드 위치)
1. **쉬운 개념 설명** (입문자용 설명 + cppreference 기준 핵심 개념)
2. **단순 예시 코드** (짧고 쉬운 독립 예제 + 동작 설명)
3. **ROS 2 (`rclcpp`) 활용 예시** (실제 ROS 2 패턴 코드 + 쓰임새 설명)
```
