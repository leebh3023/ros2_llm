# llm_ros

## 개요

`llm_ros`는 사용자의 자연어(한국어) 명령을 해석하여 ROS 2 시스템에서 로봇을 제어하기 위한 패키지입니다. 음성 인식 등으로 입력된 텍스트를 필터링하고, 명령의 의도를 파악하여 대규모 언어 모델(LLM)을 통해 실행 가능한 로봇 명령(JSON)으로 변환하는 기능을 제공합니다.

## 주요 기능

이 패키지는 여러 ROS 2 노드로 구성되어 각자의 역할을 수행합니다.

1.  **`filter_input_node` (입력 필터 노드)**
    *   **/voice2text** 토픽 또는 터미널에서 직접 입력된 텍스트를 수신합니다.
    *   지정된 호출 단어(예: "도리")로 시작하는 명령만 필터링하여 **/text_command** 토픽으로 전달합니다.

2.  **`intent_router` (의도 분류 노드)**
    *   **/text_command** 토픽에서 필터링된 명령을 수신합니다.
    *   명령의 의도를 분석합니다.
        *   **장소 관련 명령** (예: "여기를 거실로 저장해", "부엌으로 가"): `/text_to_location` 토픽으로 목적지를 전달합니다.
        *   **일반 동작 명령** (예: "앞으로 1미터 가"): `/text_to_llm` 토픽으로 전달합니다.

3.  **`llm_node` (LLM 연동 노드)**
    *   **/text_to_llm** 토픽의 자연어 명령을 수신합니다.
    *   OpenAI의 GPT 모델 API를 호출하여, 자연어 명령을 로봇이 이해할 수 있는 **JSON 형식**의 명령으로 변환합니다.
    *   변환된 JSON 명령을 **/voice_cmd** 토픽으로 발행합니다.
    *   **주의**: 이 노드를 사용하려면 `OPENAI_API_KEY` 환경변수 설정이 필요합니다.

4.  **`location_command_node` (장소 명령 처리 노드)**
    *   **/text_to_location** 토픽에서 장소 관련 명령을 수신합니다.
    *   추후 장소 저장, 조회, 이동 등을 처리하는 네비게이션 노드와 연동하기 위해 해당 목적지를 **/target_location** 토픽으로 발행합니다.

## 사전 준비

1.  **ROS 2 설치**: ROS 2 Foxy, Humble 등 환경 구성
2.  **OpenAI 라이브러리 설치**:
    ```bash
    pip install openai
    ```
3.  **OpenAI API 키 설정**:
    ```bash
    export OPENAI_API_KEY="여기에_API_키를_입력하세요"
    ```

## 빌드 방법

ROS 2 워크스페이스의 루트 디렉토리에서 아래 명령어를 실행합니다.

```bash
colcon build --packages-select llm_ros
```

## 실행 명령어

빌드 후, 새 터미널을 열고 워크스페이스의 `setup.bash` 파일을 소싱합니다.

```bash
source install/setup.bash
```

각 노드를 별도의 터미널에서 실행합니다.

```bash
# 입력 필터 노드 실행
ros2 run llm_ros filter_input_node

# 의도 분류 노드 실행
ros2 run llm_ros intent_router

# LLM 연동 노드 실행
ros2 run llm_ros llm_node

# 장소 명령 처리 노드 실행
ros2 run llm_ros location_command_node
```

이제 다른 노드(예: `turtlesim` 제어 노드)에서 `/voice_cmd` 토픽을 구독하여 로봇을 제어할 수 있습니다.
