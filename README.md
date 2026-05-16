# ODIN

ODIN은 ROS 2 Humble과 Gazebo Classic 11 기반의 멀티로봇 전술 인질 구출 시뮬레이션 프로젝트입니다.

두 대의 scout 로봇이 전술 arena를 정찰하며 SLAM과 map merge를 수행하고, ArUco marker로 표현된 인질을 발견하면 coordinator가 rescue 후보 경로를 생성합니다. Qwen 기반 AI 노드는 미션 상황과 후보 경로를 평가해 rescue 정책과 최종 경로를 선택하며, 최종 제어 권한은 coordinator와 `robot_3` Nav2 stack이 담당합니다.

이 프로젝트는 단일 PC 시뮬레이션에서 시작하지만, 각 기능을 ROS 2 package와 topic 단위로 분리하여 Jetson 기반 분산 실행으로 확장할 수 있도록 구성되어 있습니다.

## 프로젝트 목표

- Gazebo Classic 기반 전술 인질 구출 시나리오 구현
- `robot_1`, `robot_2` scout 로봇의 독립 SLAM 및 map merge
- ArUco marker 기반 hostage event 발행
- coordinator 기반 event 검증, 후보 경로 생성, rescue dispatch
- Qwen 기반 mission policy 선택 및 경로 선택
- `robot_3` Nav2 기반 rescue 이동
- GUI를 통한 mission intent 입력, Qwen 의사결정, route visualization, mission status 표시
- 추후 Jetson 분산 환경으로 확장 가능한 ROS 2 구조 유지

## 전장 시나리오

기본 월드는 20 m x 20 m 평면 arena입니다.

- 오른쪽 위 구역은 enemy stronghold입니다.
- scout 로봇은 위험 구역을 직접 공략하지 않고 hostage 후보 구역을 중심으로 정찰합니다.
- ArUco ID `0`은 hostage surrogate입니다.
- `robot_1`, `robot_2`는 scout 역할로 SLAM과 탐색을 수행합니다.
- `robot_3`는 rescue 역할로, coordinator dispatch 전에는 이동하지 않습니다.
- Qwen은 작전 의도를 해석해 rescue 정책을 선택하고, 후보 경로 중 하나를 추천합니다.
- coordinator는 Qwen 결과를 그대로 실행하지 않고, ROS frame, 좌표, map, 위험 구역, robot 상태를 검증한 뒤 dispatch합니다.

## 전체 동작 흐름

```text
Mission Intent GUI
  -> /mission/intent
  -> Qwen policy selection
  -> /ai/mission_policy

robot_1, robot_2
  -> scout exploration
  -> per-robot SLAM
  -> /robot_1/map, /robot_2/map
  -> /merged_map

ArUco detection
  -> /hostage_events

coordinator
  -> event validation
  -> candidate route generation
  -> /coordinator/candidate_routes

Qwen planner
  -> route evaluation
  -> /ai/selected_path
  -> /ai/waypoint_recommendation

coordinator
  -> waypoint validation
  -> /robot_3/spawn_trigger
  -> /robot_3/goal_pose

robot_3 Nav2
  -> selected path following
  -> mission success
```

## Mission Policy

미션 시작 전에 GUI에서 자연어로 작전 상황을 입력합니다. Qwen은 입력 문장을 기반으로 다음 정책 중 하나를 선택합니다.

- `FAST_RESCUE`: 인질 생존 시간이 중요하여 신속한 rescue를 우선합니다.
- `SAFE_RESCUE`: 정찰된 영역과 map 안정성을 우선합니다.
- `STEALTH_RESCUE`: enemy stronghold와 적 시야망 회피를 우선합니다.

정책 선택 이후 scout 주행이 시작됩니다. 즉, 사용자가 intent를 전송하기 전에는 scout 로봇이 대기합니다.

Qwen이 사용할 수 없거나 응답이 유효하지 않은 경우에도 같은 ROS topic contract를 유지해 전체 rescue pipeline이 중단되지 않도록 구성되어 있습니다.

## Route Evaluation

Coordinator는 후보 경로들을 생성하고, Qwen planner는 각 후보 경로의 상태 변수를 요약해 정책별 평가 함수에 넣습니다. 현재 구현은 값이 낮을수록 더 선호되는 cost 기반 평가식입니다.

```text
J(route) =
  w_length   * path_length
+ w_unknown  * unknown_area_exposure
+ w_occupied * obstacle_intersection
+ w_red      * enemy_area_exposure
+ w_vision   * enemy_vision_exposure
+ w_clearance * enemy_area_separation
+ w_turn     * turning_cost
+ w_route    * route_type_cost
```

각 항목의 의미는 다음과 같습니다.

- `path_length`: rescue robot이 따라가야 하는 경로 길이
- `unknown_area_exposure`: 아직 충분히 정찰되지 않은 영역을 지나는 정도
- `obstacle_intersection`: 장애물 또는 점유 cell과 충돌하는 정도
- `enemy_area_exposure`: enemy stronghold 또는 red zone과 겹치는 정도
- `enemy_vision_exposure`: 적 시야망으로 간주되는 영역에 노출되는 정도
- `enemy_area_separation`: enemy stronghold와의 이격 정도
- `turning_cost`: 경로의 회전량과 조향 복잡도
- `route_type_cost`: 후보 경로가 어떤 planner/context에서 생성됐는지를 반영하는 보정항

`FAST_RESCUE`, `SAFE_RESCUE`, `STEALTH_RESCUE`는 같은 평가식을 사용하지만 서로 다른 정책 가중치 집합을 적용합니다. 예를 들어 `FAST_RESCUE`는 신속한 rescue를, `SAFE_RESCUE`는 정찰된 영역을, `STEALTH_RESCUE`는 enemy area와 vision exposure 회피 및 이격을 더 강하게 반영합니다.

Qwen은 전체 후보를 그대로 받지 않고, coordinator와 planner가 압축한 후보 요약을 입력받습니다. Qwen이 선택한 경로는 다시 coordinator validation을 거쳐 `robot_3` dispatch에 사용됩니다.

## GUI 구성

현재 데모는 다음 GUI를 사용합니다.

- `Mission Intent`: 사용자가 작전 상황을 입력하고 Qwen에게 정책 선택을 요청합니다.
- `ODIN Coordinator / Qwen Dialog`: coordinator와 Qwen 사이의 request, response, dispatch log를 표시합니다.
- `ODIN Qwen Route Decision`: merged map 위에 후보 경로, Qwen 선택 경로, hostage 위치, robot_3 trail, enemy area overlay를 표시합니다.
- `ODIN Mission Timeline`: `SCOUTING`, `ROBOT DETECT HOSTAGE`, `COORDINATOR VALIDATING`, `ROBOT3 DISPATCH`, `MISSION SUCCESS` 흐름을 표시합니다.
- `Mission Success Popup`: rescue 완료 시 별도 성공 팝업을 표시합니다.

## 패키지 구조

### `odin_bringup`

전체 시뮬레이션을 실행하는 상위 launch 패키지입니다.

- `sim_multi_slam_map_merge.launch.py`
  - Gazebo
  - scout SLAM
  - map merge
  - exploration
  - RGB ArUco detection
  - coordinator
  - Qwen planner
  - robot_3 Nav2 dispatch
  - GUI panels

- `multi_slam_map_merge.launch.py`
  - Gazebo 없이 SLAM과 map merge 관련 노드만 실행합니다.

### `odin_gazebo`

Gazebo world와 robot spawn을 담당합니다.

- `worlds/odin_rescue_20x20_c.world`
  - 기본 전술 arena
  - enemy stronghold
  - obstacle layout
  - hostage ArUco marker

- `house_easier_three_robots.launch.py`
  - scout 로봇을 알려진 초기 위치에 spawn합니다.
  - `robot_3`는 rescue phase에서 별도 spawn됩니다.

### `odin_description`

로봇 모델, URDF/Xacro, 센서 구성을 담당합니다.

- scout 로봇은 LiDAR와 RGB camera를 사용합니다.
- RGB camera는 ArUco marker detection에 사용됩니다.

### `odin_slam`

로봇별 `slam_toolbox` 실행을 담당합니다.

- `robot_1`, `robot_2`는 각 namespace 아래에서 독립적으로 SLAM을 수행합니다.
- 주요 topic:
  - `/robot_1/map`
  - `/robot_2/map`

### `odin_map_merge`

Scout 로봇의 odom과 scan 정보를 이용해 `/merged_map`을 생성합니다.

- 주요 입력:
  - `/robot_1/odom`
  - `/robot_1/scan`
  - `/robot_2/odom`
  - `/robot_2/scan`

- 주요 출력:
  - `/merged_map`

Merged map은 coordinator, Qwen route visualization, robot_3 navigation context에 사용됩니다.

### `odin_exploration`

Scout 로봇의 자동 탐색 주행을 담당합니다.

- `reactive_scout.py`
  - LiDAR 기반 reactive navigation
  - 장애물 회피
  - event 이후 추가 map coverage 유도
  - enemy stronghold 접근 억제

- `reactive_scouts.launch.py`
  - `robot_1`, `robot_2` namespace 아래에 scout node를 실행합니다.

Scout 로봇은 mission policy가 선택되기 전까지 대기합니다.

### `odin_detection`

ArUco hostage event 발행을 담당합니다.

- `rgb_aruco_event_detector.py`
  - RGB camera image에서 ArUco ID `0`을 감지합니다.
  - 벽 뒤 감지처럼 시나리오상 부적절한 event를 줄이기 위해 visibility 조건을 함께 확인합니다.
  - 동일 marker event가 반복 발행되지 않도록 관리합니다.

- `gazebo_aruco_event_detector.py`
  - Gazebo state 기반 detector입니다.
  - RGB detector를 보조하거나 테스트할 때 사용할 수 있습니다.

주요 출력:

- `/hostage_events`

### `odin_coordinator`

Hostage event 검증, candidate route 생성, AI waypoint 검증, robot_3 dispatch를 담당합니다.

주요 입력:

- `/hostage_events`
- `/merged_map`
- `/robot_1/odom`
- `/robot_2/odom`
- `/robot_3/odom`
- `/ai/mission_policy`
- `/ai/waypoint_recommendation`

주요 출력:

- `/coordinator/status`
- `/coordinator/candidate_path`
- `/coordinator/candidate_routes`
- `/coordinator/validated_waypoint`
- `/robot_3/spawn_trigger`
- `/robot_3/goal_pose`

Coordinator는 Qwen이 선택한 경로를 그대로 실행하지 않고, frame, 좌표, map accessibility, duplicate event, robot availability, 위험 구역 여부를 검증한 뒤 robot_3를 dispatch합니다.

### `odin_ai`

Qwen 기반 mission policy 선택과 route selection을 담당합니다.

- `mission_intent_panel.py`
  - 미션 시작 전 사용자의 자연어 intent를 입력받습니다.

- `virtual_qwen_planner.py`
  - `/mission/intent`를 받아 Qwen에게 rescue policy 선택을 요청합니다.
  - `/coordinator/candidate_routes`와 `/merged_map`을 기반으로 후보 경로를 압축해 Qwen에게 전달합니다.
  - Qwen 선택 결과를 `/ai/selected_path`와 `/ai/waypoint_recommendation`으로 발행합니다.
  - Qwen 연결 상태와 무관하게 동일한 ROS topic interface를 유지합니다.

Qwen은 직접 `/cmd_vel` 또는 `/robot_3/goal_pose`를 발행하지 않습니다.

### `odin_navigation`

`robot_3` spawn, Nav2 dispatch, mission GUI를 담당합니다.

- `robot_3_spawn_on_goal.py`
  - coordinator의 spawn trigger를 받아 Gazebo에 `robot_3`를 생성합니다.

- `nav2_goal_dispatcher.py`
  - coordinator가 검증한 goal과 Qwen이 선택한 selected path를 Nav2 goal sequence로 전달합니다.

- `robot_scan_filter.py`
  - robot_3 Nav2 costmap에 들어가는 scan을 정리합니다.
  - scout 로봇이 robot_3의 동적 장애물로 과도하게 반영되는 현상을 줄입니다.

- `robot_3_speed_policy.py`
  - mission policy에 따라 `robot_3` Nav2 속도 제한을 조정합니다.
  - 일반 rescue 모드에서는 안정성을 우선하고, stealth rescue 모드에서는 rescue 기동 속도를 높입니다.

- `mission_success_marker.py`
  - mission success 상태와 RViz marker를 발행합니다.

- `mission_status_panel.py`
  - mission timeline GUI입니다.

- `qwen_dialog_panel.py`
  - coordinator와 Qwen의 대화 log GUI입니다.

- `qwen_route_map_panel.py`
  - route decision map GUI입니다.

## 주요 Topic

### Scout

- `/robot_1/scan`
- `/robot_1/odom`
- `/robot_1/cmd_vel`
- `/robot_1/map`
- `/robot_2/scan`
- `/robot_2/odom`
- `/robot_2/cmd_vel`
- `/robot_2/map`

### Map

- `/merged_map`

### Detection

- `/hostage_events`

### Coordinator

- `/coordinator/status`
- `/coordinator/candidate_path`
- `/coordinator/candidate_routes`
- `/coordinator/validated_waypoint`

### AI

- `/mission/intent`
- `/ai/mission_policy`
- `/ai/status`
- `/ai/selected_path`
- `/ai/waypoint_recommendation`

### Robot 3

- `/robot_3/spawn_trigger`
- `/robot_3/goal_pose`
- `/robot_3/scan`
- `/robot_3/scan_nav2`
- `/robot_3/odom`
- `/robot_3/cmd_vel`
- `/robot_3/dispatch_status`

### Mission UI

- `/mission_status`
- `/mission_marker`

## Qwen Jetson 연결

Qwen은 OpenAI-compatible Chat Completions endpoint를 제공하는 `llama.cpp` server로 연결합니다.

Qwen Jetson은 ROS 2 node를 실행하지 않아도 됩니다. 이 구성에서는 laptop/desktop이 ROS graph를 실행하고, `odin_ai`가 HTTP로 Qwen server에 request를 보내는 구조입니다.

Jetson에서 Qwen server 실행 예시:

```bash
llama-server \
  -m ~/Qwen3-VL-4B-Instruct-Q4_K_M.gguf \
  --mmproj ~/Qwen3-VL-4B-Instruct-mmproj.gguf \
  --host 0.0.0.0 \
  --port 8081 \
  --ctx-size 1024 \
  --gpu-layers 0 \
  --no-mmproj-offload \
  --jinja \
  --reasoning off
```

노트북에서 health check:

```bash
curl http://<JETSON_IP>:8081/health
```

프로젝트의 기본 Qwen API URL은 `odin_ai/config/virtual_qwen_planner.yaml`에서 수정할 수 있습니다.

Qwen server만 Jetson에서 올리는 경우 laptop/desktop 실행 흐름:

```bash
cd /home/odin/robotics_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
curl http://<JETSON_IP>:8081/health
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py
```

이때 주요 통신은 다음과 같습니다.

```text
Laptop ROS graph
  /mission/intent
  /coordinator/candidate_routes
  /merged_map
  -> odin_ai/virtual_qwen_planner
  -> HTTP request to Qwen Jetson
  <- HTTP response from Qwen Jetson
  /ai/mission_policy
  /ai/selected_path
  /ai/waypoint_recommendation
```

## Jetson 분산 실행 가이드

ODIN은 topic, namespace, package 단위로 분리되어 있어 Jetson 분산 실행으로 확장할 수 있습니다. 현재 Gazebo 기반 데모에서는 laptop/desktop이 시뮬레이션과 시각화를 담당하고, Jetson은 Qwen server 또는 일부 ROS node 그룹을 담당하는 방식부터 적용하는 것을 권장합니다.

### 네트워크 공통 설정

모든 ROS 2 장비에서 같은 네트워크와 같은 DDS domain을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/robotics_ws/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

CycloneDDS를 사용할 경우 모든 장비에 패키지가 설치되어 있어야 합니다.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

한 장비에 해당 RMW가 설치되어 있지 않으면 `RMW_IMPLEMENTATION`을 설정하지 않거나, 모든 장비에 동일하게 설치해야 합니다.

네트워크 확인:

```bash
ping <OTHER_DEVICE_IP>
ros2 multicast receive
ros2 multicast send
ros2 topic list
```

### 권장 분산 배치

| 장비 | 역할 | 주요 node/topic |
| --- | --- | --- |
| Laptop/Desktop | Gazebo, RViz, GUI, map merge, coordinator | `/merged_map`, `/coordinator/*`, GUI panels |
| Jetson 1 | `robot_1` scout stack | `/robot_1/scan`, `/robot_1/odom`, `/robot_1/map`, `/robot_1/cmd_vel` |
| Jetson 2 | `robot_2` scout stack | `/robot_2/scan`, `/robot_2/odom`, `/robot_2/map`, `/robot_2/cmd_vel` |
| Jetson 3 | `robot_3` rescue stack | `/robot_3/scan`, `/robot_3/odom`, `/robot_3/goal_pose`, `/robot_3/cmd_vel` |
| Jetson 4 | Qwen server 또는 AI node | HTTP Qwen endpoint 또는 `/ai/*` |

### 현재 데모 기준 실행 예시

가장 안정적인 데모 방식은 Qwen server만 Jetson에서 실행하고 나머지 ROS graph를 laptop/desktop에서 실행하는 방식입니다.

Jetson 4:

```bash
cd ~/llama.cpp/build
./bin/llama-server \
  -m ~/Qwen3-VL-4B-Instruct-Q4_K_M.gguf \
  --mmproj ~/Qwen3-VL-4B-Instruct-mmproj.gguf \
  --host 0.0.0.0 \
  --port 8081 \
  --ctx-size 1024 \
  --gpu-layers 0 \
  --no-mmproj-offload \
  --jinja \
  --reasoning off
```

Laptop/Desktop:

```bash
cd /home/odin/robotics_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py
```

ROS node를 실제 Jetson들에 나누는 경우에는 중복 실행을 피해야 합니다. 예를 들어 laptop에서 전체 launch를 그대로 실행하면서 Jetson에서도 같은 node를 실행하면 topic과 TF가 중복됩니다. 분산 실행 시에는 아래처럼 기능 그룹을 나누어 실행합니다.

Laptop/Desktop, Gazebo와 중앙 기능:

```bash
ros2 launch odin_gazebo house_easier_three_robots.launch.py
ros2 launch odin_map_merge scenario_scan_map_merge.launch.py
ros2 launch odin_coordinator rescue_coordinator.launch.py \
  battlefield_config_file:=/home/odin/robotics_ws/ros2_ws/install/odin_bringup/share/odin_bringup/config/battlefield_rules.yaml
ros2 launch odin_ai virtual_qwen_planner.launch.py \
  battlefield_config_file:=/home/odin/robotics_ws/ros2_ws/install/odin_bringup/share/odin_bringup/config/battlefield_rules.yaml
```

Jetson 1, `robot_1` scout stack 예시:

```bash
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -r __ns:=/robot_1 \
  -p use_sim_time:=true \
  -p scan_topic:=scan \
  -p map_frame:=robot_1/map \
  -p odom_frame:=robot_1/odom \
  -p base_frame:=robot_1/base_footprint \
  --params-file /home/odin/robotics_ws/ros2_ws/install/odin_slam/share/odin_slam/config/slam_toolbox.yaml

ros2 run odin_exploration reactive_scout \
  --ros-args -r __ns:=/robot_1 -p use_sim_time:=true
```

Jetson 2, `robot_2` scout stack 예시:

```bash
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -r __ns:=/robot_2 \
  -p use_sim_time:=true \
  -p scan_topic:=scan \
  -p map_frame:=robot_2/map \
  -p odom_frame:=robot_2/odom \
  -p base_frame:=robot_2/base_footprint \
  --params-file /home/odin/robotics_ws/ros2_ws/install/odin_slam/share/odin_slam/config/slam_toolbox.yaml

ros2 run odin_exploration reactive_scout \
  --ros-args -r __ns:=/robot_2 -p use_sim_time:=true
```

Jetson 3, `robot_3` rescue stack 예시:

```bash
ros2 launch odin_navigation robot_3_nav2_dispatch.launch.py
```

분산 실행에서 확인할 핵심 topic:

```bash
ros2 topic echo /robot_1/map --once
ros2 topic echo /robot_2/map --once
ros2 topic echo /merged_map --once --field info
ros2 topic echo /hostage_events
ros2 topic echo /ai/mission_policy
ros2 topic echo /ai/selected_path
ros2 topic echo /robot_3/dispatch_status
```

## 빌드

workspace root에서 실행합니다.

```bash
cd /home/odin/robotics_ws/ros2_ws
colcon build --packages-select \
  odin_ai \
  odin_bringup \
  odin_coordinator \
  odin_detection \
  odin_exploration \
  odin_gazebo \
  odin_interfaces \
  odin_map_merge \
  odin_navigation \
  odin_slam
source install/setup.bash
```

개발 중 일부 패키지만 확인할 때:

```bash
colcon build --packages-select odin_ai odin_coordinator odin_navigation odin_bringup
```

## 실행

전체 시뮬레이션:

```bash
cd /home/odin/robotics_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py
```

실행 후 `Mission Intent` GUI에서 작전 상황을 선택하거나 직접 입력하고 `Send Intent To Qwen`을 누르면 scout 로봇이 정찰을 시작합니다.

Gazebo GUI 없이 실행:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py gui:=false
```

Mission Intent GUI 없이 실행:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_mission_intent_gui:=false
```

일부 기능 비활성화:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_detection:=false
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_coordinator:=false
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_ai:=false
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_robot_3_dispatch:=false
```

SLAM과 map merge만 실행:

```bash
ros2 launch odin_bringup multi_slam_map_merge.launch.py
```

## 수동 테스트 명령

Mission intent 직접 발행:

```bash
ros2 topic pub --once /mission/intent std_msgs/msg/String "{data: '안전하게 구출해야 해. 이미 정찰된 경로를 우선해.'}"
ros2 topic pub --once /mission/intent std_msgs/msg/String "{data: '최대한 빠르게 구출해야 해.'}"
ros2 topic pub --once /mission/intent std_msgs/msg/String "{data: '적에게 들키지 않게 은밀하게 접근해야 해.'}"
```

Topic 확인:

```bash
ros2 topic list | grep -E 'mission|ai|coordinator|robot_3|hostage|merged_map'
ros2 topic echo /ai/status
ros2 topic echo /coordinator/status
ros2 topic echo /robot_3/dispatch_status
```

Merged map 정보 확인:

```bash
ros2 topic echo /merged_map --once --field info
```

## RViz 확인

Fixed Frame:

```text
map
```

권장 표시 topic:

- `/merged_map`
- `/robot_1/map`
- `/robot_2/map`
- `/coordinator/candidate_path`
- `/coordinator/candidate_routes`
- `/ai/selected_path`
- `/robot_3/goal_pose`
- `/mission_marker`

## 현재 완성된 기능

- 전술 arena Gazebo world 실행
- scout robot 2대 spawn
- robot namespace 분리
- `slam_toolbox` 기반 robot별 SLAM
- odom/scan 기반 `/merged_map` 생성
- RGB camera 기반 ArUco hostage detection
- 중복 hostage event 방지
- coordinator event validation
- coordinator candidate route generation
- Qwen mission policy selection
- Qwen route selection
- Qwen route decision GUI
- coordinator/Qwen dialog GUI
- robot_3 on-demand spawn
- robot_3 Nav2 dispatch
- robot_3 selected path following
- robot_3 scan filtering for Nav2
- mission policy 기반 robot_3 speed policy
- Mission timeline GUI
- Mission success popup

## 현재 제한 사항

- Gazebo simulation 중심 구현입니다.
- 실제 Jetson 분산 배치는 네트워크, ROS_DOMAIN_ID, RMW 설정, launch 중복 실행 여부를 장비별로 맞춰야 합니다.
- Scout 주행은 연구용 exploration stack이 아니라 데모 시나리오에 맞춘 lightweight reactive navigation입니다.
- `/merged_map`은 현재 시나리오 world를 기준으로 안정성을 우선한 map merge 방식입니다.
- ArUco marker는 hostage surrogate이며 실제 사람 인식 모델은 포함하지 않습니다.
- Qwen은 정책 선택과 경로 선택을 보조하며, 최종 robot command 권한은 갖지 않습니다.
- `robot_3`는 현재 rescue dispatch 역할에 집중하며 scout SLAM에는 참여하지 않습니다.

## 확장 방향

- Jetson별 namespace stack 분리 실행
- ROS 2 DDS discovery 및 네트워크 설정 정리
- Qwen/VLM 입력에 map image overlay 추가
- enemy vision model 고도화
- mission policy별 시나리오 확장
- 실제 로봇 또는 Jetson Isaac/ROS 연동
- `robot_3` spawn point 다양화
- map merge 방식 일반화

## 참고

프로젝트는 다음 원칙을 유지합니다.

- AI는 의사결정 보조 역할을 담당합니다.
- coordinator가 최종 검증과 dispatch 권한을 가집니다.
- 로봇 간 통신은 ROS 2 topic, service, action 기반으로 유지합니다.
- 각 로봇 stack은 namespace로 분리합니다.
- 추후 Jetson 분산 환경을 고려해 monolithic script 구조를 피합니다.
