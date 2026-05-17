# ODIN Docker: Full Distributed Jetson Deployment

이 구성은 역할별 Docker compose 파일을 각 장비에서 실행하는 풀 분산 배포형입니다.

권장 역할:

- Laptop/Desktop: Gazebo, map merge, coordinator, detection, GUI
- Jetson 1: `robot_1` scout stack
- Jetson 2: `robot_2` scout stack
- Jetson 3: `robot_3` rescue/Nav2 stack
- Jetson 4: Qwen server + `virtual_qwen_planner`

모든 장비는 같은 유선망, 같은 `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`을 사용합니다.

## 준비

각 장비에 저장소를 같은 위치로 준비합니다.

```bash
cd /home/odin/robotics_ws/ros2_ws/src/odin_rescue/docker/full_distributed
cp .env.example .env
```

`.env`에서 공통 domain과 Qwen IP를 맞춥니다.

```text
ROS_DOMAIN_ID=42
QWEN_HOST_IP=10.42.0.14
QWEN_API_URL=http://10.42.0.14:8081/v1/chat/completions
```

## 빌드

각 장비에서 자신의 compose 파일을 빌드합니다.

Laptop/Desktop:

```bash
xhost +local:docker
docker compose --env-file .env -f compose.desktop.yaml build
```

Jetson 1:

```bash
docker compose --env-file .env -f compose.robot_1.yaml build
```

Jetson 2:

```bash
docker compose --env-file .env -f compose.robot_2.yaml build
```

Jetson 3:

```bash
docker compose --env-file .env -f compose.robot_3.yaml build
```

Jetson 4:

```bash
docker compose --env-file .env -f compose.qwen.yaml build
```

## 실행 순서

1. Jetson 4에서 Qwen server와 AI planner 실행

```bash
docker compose --env-file .env -f compose.qwen.yaml up
```

2. Laptop/Desktop에서 Gazebo와 중앙 기능 실행

```bash
xhost +local:docker
docker compose --env-file .env -f compose.desktop.yaml up
```

3. Jetson 1에서 robot_1 stack 실행

```bash
docker compose --env-file .env -f compose.robot_1.yaml up
```

4. Jetson 2에서 robot_2 stack 실행

```bash
docker compose --env-file .env -f compose.robot_2.yaml up
```

5. Jetson 3에서 robot_3 stack 실행

```bash
docker compose --env-file .env -f compose.robot_3.yaml up
```

## 확인

Qwen:

```bash
curl http://<QWEN_JETSON_IP>:8081/health
```

ROS graph:

```bash
docker compose -f compose.desktop.yaml exec desktop_central ros2 topic list
docker compose -f compose.desktop.yaml exec desktop_central ros2 run odin_bringup deployment_view
```

핵심 topic:

```bash
ros2 topic echo /robot_1/map --once --field info
ros2 topic echo /robot_2/map --once --field info
ros2 topic echo /merged_map --once --field info
ros2 topic echo /ai/mission_policy
ros2 topic echo /ai/selected_path
ros2 topic echo /robot_3/dispatch_status
```

## 종료

각 장비에서:

```bash
docker compose --env-file .env -f <compose-file>.yaml down
```

## 주의

- 같은 역할의 node를 두 장비에서 동시에 실행하지 않습니다.
- Gazebo 시뮬레이션 기반 분산 데모에서는 센서 topic이 Laptop/Desktop Gazebo에서 나오고, Jetson의 ROS node들이 DDS로 이를 구독합니다.
- 실제 로봇 배포에서는 Gazebo 대신 각 로봇의 driver가 `/robot_i/scan`, `/robot_i/odom`, `/robot_i/cmd_vel`을 담당해야 합니다.
