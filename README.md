# ROS2 LLM 기반 자율 주행 시스템 (ROS2 LLM-based Autonomous Navigation System)

## 1. 프로젝트 개요

이 프로젝트는 ROS2(Robot Operating System 2)와 대규모 언어 모델(LLM, Large Language Model)을 통합하여 로봇을 자연어 명령으로 제어하는 시스템입니다. 사용자는 텍스트 명령을 통해 로봇의 이동, 회전뿐만 아니라 특정 장소를 기억하고 해당 장소로 자율 주행하도록 지시할 수 있습니다.

## 2. 주요 기능

*   **자연어 명령 처리**: LLM(OpenAI GPT-3.5-turbo)을 활용하여 사용자의 자연어 명령을 로봇이 이해할 수 있는 구조화된 JSON 명령으로 변환합니다.
*   **장소 관리**:
    *   현재 로봇의 위치를 특정 이름으로 저장 (`여기를 [장소명]으로 저장해줘`).
    *   저장된 장소 목록 조회 (`저장된 장소 목록 보여줘`).
    *   저장된 장소 삭제 (`[장소명] 지워줘`).
*   **자율 주행**: 저장된 장소로 Nav2 스택을 활용하여 자율적으로 이동 (`[장소명]으로 가자`).
*   **수동 제어**: 기본적인 로봇의 선형 이동 및 회전 제어 (`앞으로 1미터 가줘`, `시계 방향으로 90도 회전해줘`).
*   **견고한 시스템**: 상태 머신 기반의 제어 로직, 안전한 파일 입출력, LLM 응답 오류 처리 등 안정성을 위한 다양한 기능이 포함되어 있습니다.

## 3. 아키텍처 개요

시스템은 다음과 같은 주요 ROS2 노드들로 구성되어 상호작용합니다.

```
[사용자 입력 (텍스트)]
       ↓
[client_input_node] (텍스트 명령 발행: /text_command)
       ↓
[llm_node] (LLM 호출, JSON 명령 발행: /voice_cmd)
       ↓
[navigator_node] (JSON 명령 해석, 로봇 제어 명령 발행: /cmd_vel 또는 Nav2 목표 전송)
       ↓
[location_manager] (장소 저장/조회/삭제/목록 서비스 제공)
       ↑
[Nav2 스택] (자율 주행, /amcl_pose 발행)
```

## 4. 개발 환경 및 설정

### 4.1. 전제 조건

*   ROS2 Humble Hawksbill (또는 호환 버전)
*   Nav2 스택 설치
*   OpenAI API Key (환경 변수 `OPENAI_API_KEY`로 설정)

### 4.2. 빌드 및 환경 설정

1.  **작업 공간 클론 (또는 기존 작업 공간 사용)**:
    ```bash
    # cd ~/your_ros2_ws/src
    # git clone [repository_url] # 해당되는 경우
    ```
2.  **패키지 빌드**: `ros_llm` 작업 공간의 루트 디렉터리에서 다음 명령을 실행합니다.
    ```bash
    cd ~/ros_llm # 프로젝트 루트 디렉터리
    colcon build
    ```
3.  **환경 소싱**: 빌드 후, ROS2 환경 변수를 설정합니다.
    ```bash
    source install/setup.bash
    ```

## 5. 사용법

### 5.1. 필수 노드 실행

새로운 기능을 사용하려면 다음 노드들을 실행해야 합니다.

```bash
# 터미널 1: 위치 관리자 노드
ros2 run llmros location_manager

# 터미널 2: LLM 인터페이스 노드 (OPENAI_API_KEY 설정 필수)
# export OPENAI_API_KEY="YOUR_API_KEY_HERE"
ros2 run llmros llm_node

# 터미널 3: 내비게이터 노드 (로봇 제어 및 Nav2 연동)
ros2 run llmros navigator_node

# 터미널 4: 사용자 입력 노드
ros2 run llmros client_input_node
```

### 5.2. 시뮬레이터 또는 로봇 환경 설정

*   **Turtlesim (간단한 테스트)**:
    ```bash
    ros2 run turtlesim turtlesim_node
    ros2 run turtlesim turtle_teleop_key # 수동 제어용
    ```
    *   **참고**: `turtlesim`은 `/amcl_pose`를 발행하지 않으므로, 위치 저장/자율 주행 기능을 테스트하려면 `/amcl_pose`를 발행하는 다른 노드(예: `fake_localization` 또는 실제 AMCL)가 필요합니다.
*   **실제 로봇 또는 Isaac Sim (Nav2 연동)**:
    *   Nav2 스택이 실행 중이어야 합니다.
    *   로봇의 위치 추정 노드(예: AMCL)가 `/amcl_pose` 토픽을 발행하고 있어야 합니다.

### 5.3. 예시 명령 (client_input_node 터미널에서 입력)

*   **위치 저장**:
    *   `여기를 내 자리로 저장해줘`
    *   `여기를 화장실로 저장해줘`
*   **저장된 장소 목록 조회**:
    *   `저장된 장소 목록 보여줘`
*   **특정 장소로 이동 (Nav2 필요)**:
    *   `내 자리로 가자`
    *   `화장실로 이동해줘`
*   **특정 장소 삭제**:
    *   `내 자리 지워줘`
*   **수동 제어**:
    *   `앞으로 1미터 가줘`
    *   `시계 방향으로 90도 회전해줘`

## 6. 최근 업데이트 내용 (이번 세션)

이번 개발 세션에서 다음과 같은 주요 업데이트가 이루어졌습니다.

*   **새로운 `ros_llm_interfaces` 패키지 추가**:
    *   `SaveLocation`, `GetLocation`, `DeleteLocation`, `ListLocations`와 같은 커스텀 ROS2 서비스 정의를 포함합니다. 이는 노드 간의 통신을 표준화하고 모듈성을 높입니다.
*   **새로운 `location_manager` 노드 구현**:
    *   `locations.json` 파일을 사용하여 명명된 장소의 위치 데이터를 영구적으로 저장하고 관리하는 서비스를 제공합니다. 파일 입출력 시 동시성 문제를 방지하기 위한 락(Lock) 메커니즘이 적용되었습니다.
*   **새로운 `navigator_node` 구현**:
    *   기존의 단순 파서 노드를 대체하는 핵심 제어 노드입니다.
    *   상태 머신(State Machine)을 도입하여 로봇의 상태(IDLE, NAVIGATING, MANUAL_CONTROL 등)를 관리하고, 명령 충돌을 방지합니다.
    *   `location_manager` 서비스 클라이언트로서 장소 저장/조회/삭제/목록 기능을 사용합니다.
    *   Nav2 스택의 `NavigateToPose` 액션 클라이언트로서 자율 주행 목표를 전송하고 피드백 및 결과를 처리합니다.
    *   `/amcl_pose` 토픽을 구독하여 로봇의 현재 위치를 추적하고, 위치 데이터의 신선도를 검사하는 안전 로직이 포함되었습니다.
    *   수동 제어 시 `cmd_vel` 타임아웃을 두어 안전성을 강화했습니다.
*   **`llm_node` 업데이트**:
    *   `ask_gpt` 함수의 프롬프트에 장소 관리 명령(저장, 이동, 삭제, 목록)에 대한 새로운 온톨로지와 예시가 추가되었습니다.
    *   LLM 응답이 유효한 JSON이 아닐 경우, 표준화된 에러 JSON을 반환하도록 오류 처리 로직이 강화되었습니다.
*   **`setup.py` 업데이트**:
    *   `location_manager`와 `navigator_node`가 ROS2 시스템에서 실행 가능한 노드로 등록되었습니다.
*   **`locations.json` 파일 생성**:
    *   `location_manager` 노드가 사용할 빈 JSON 파일이 생성되었습니다.

