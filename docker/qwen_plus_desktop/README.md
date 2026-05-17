# ODIN Docker: Qwen Jetson + Desktop Full Demo

이 구성은 최소 배포형입니다.

- Qwen server: Jetson 또는 같은 머신의 Docker container
- ODIN ROS graph: Laptop/Desktop container에서 전체 실행
- Gazebo/RViz/GUI: Laptop/Desktop X11 화면 사용

## 준비

```bash
cd /home/odin/robotics_ws/ros2_ws/src/odin_rescue/docker/qwen_plus_desktop
cp .env.example .env
```

`.env`에서 모델 경로와 Qwen API URL을 맞춥니다.

```text
QWEN_MODEL_DIR=/home/odin
QWEN_API_URL=http://127.0.0.1:8081/v1/chat/completions
```

Qwen Jetson을 별도 장비에서 이미 실행 중이면 compose의 `qwen` service는 쓰지 않고, `.env`의 URL만 Jetson IP로 바꿉니다.

```text
QWEN_API_URL=http://10.42.0.14:8081/v1/chat/completions
```

## 빌드 및 실행

```bash
xhost +local:docker
docker compose --env-file .env build
docker compose --env-file .env up
```

Qwen server를 별도 Jetson에서 실행하는 경우:

```bash
xhost +local:docker
docker compose --env-file .env up --build odin
```

## 확인

```bash
curl http://127.0.0.1:8081/health
docker compose exec odin ros2 topic list
docker compose exec odin ros2 run odin_bringup deployment_view
```

## 종료

```bash
docker compose down
```
