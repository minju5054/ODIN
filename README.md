# ODIN

ODIN은 ROS 2 Humble과 Gazebo Classic 11 기반의 멀티로봇 전술 인질 구출 시뮬레이션 프로젝트입니다.

현재 단계의 목표는 두 대의 scout 로봇이 20 m x 20 m 전술 arena를 정찰하면서 각자 SLAM을 수행하고, odom과 LiDAR scan을 이용해 하나의 `/merged_map`을 안정적으로 생성하는 것입니다. 오른쪽 위 구역은 적 요충지로 가정하며, scout 로봇은 해당 위험 구역을 직접 공략하지 않고 인질이 있을 가능성이 높은 중앙 구역으로 서서히 좁혀 들어가는 방식으로 수색합니다.

각 로봇은 namespace로 분리되어 있으며, 이후 Jetson 기반 분산 환경으로 확장할 수 있는 구조를 전제로 합니다.

## 전장 시나리오

- 전장 맵은 20 m x 20 m 평면 arena입니다.
- 적 요충지는 맵 오른쪽 위 구역입니다.
- 맵 C에는 오른쪽 위에 enemy stronghold, sentry post, barricade, red zone marker, visual-only vision ray가 배치되어 있습니다.
- `robot_1`, `robot_2`는 scout 로봇으로, 적 요충지를 피하면서 주변을 정찰하고 인질 가능 구역으로 맵을 확장합니다.
- scout 주행은 중앙 인질 후보 구역을 향해 점진적으로 좁혀 들어가는 center-spiral bias를 사용합니다.
- ArUco ID `0`은 hostage surrogate입니다. 현재 월드 C의 중앙 벽에 visual marker로 배치되어 있습니다.
- coordinator는 `/merged_map`, hostage event, robot state를 바탕으로 `robot_3` 침투 경로 후보를 생성합니다.
- Qwen/VLM은 지도, 적 요충지, red zone, 후보 경로를 보고 적 시야망을 피하는 경로를 평가하거나 추천합니다.
- 최종 dispatch 권한은 coordinator가 가지며, Qwen은 직접 로봇을 제어하지 않습니다.
- `robot_3`는 초기에는 왼쪽 아래 safe insertion point에서 spawn된다고 가정합니다.

## 현재 구현 상태

- Gazebo Classic에서 전술 월드 C를 실행합니다.
- `robot_1`, `robot_2` 두 scout 로봇을 서로 다른 namespace로 spawn합니다.
- 각 scout 로봇은 `slam_toolbox`로 독립 SLAM을 수행합니다.
- `odin_map_merge`는 `/robot_1/scan`, `/robot_1/odom`, `/robot_2/scan`, `/robot_2/odom`을 이용해 `/merged_map`을 생성합니다.
- merged map에서는 로봇끼리 서로 LiDAR에 감지되어 장애물로 남는 현상을 줄이기 위해 robot footprint와 robot-on-robot scan hit를 필터링합니다.
- `odin_exploration`은 reactive gap-following과 center-spiral bias를 이용해 scout 로봇을 자동 주행시킵니다.
- `odin_detection`은 Gazebo world state 기반으로 ArUco ID `0` hostage surrogate 감지 조건을 판단하고 `/hostage_events`를 발행합니다.
- `odin_coordinator`는 hostage event, `/merged_map`, robot odom을 구독해 중복 event, frame, 좌표, map 접근성, robot_3 availability를 검증하고 rescue 후보 경로를 생성합니다.
- `odin_ai`는 Qwen/VLM 연결 전 local heuristic fallback으로 `/coordinator/candidate_path`를 보고 `/ai/waypoint_recommendation`을 발행합니다.
- `robot_3`는 시작 시 바로 spawn되지 않고, coordinator가 검증한 `/robot_3/goal_pose`가 발행된 뒤 왼쪽 아래 safe insertion point에 spawn되어 dispatch를 시작합니다.

## 앞으로 구현할 목록

1. `odin_ai` 추가
   - Qwen/VLM은 `/merged_map`, 적 요충지 위치, red zone, 후보 경로, robot_3 초기 위치를 입력으로 받습니다.
   - 적 시야망을 피하는 waypoint 후보를 평가하거나 추천합니다.
   - Qwen/VLM은 직접 `/cmd_vel`이나 goal topic을 발행하지 않습니다.

2. `robot_3` dispatch 구현
   - coordinator 검증이 통과된 경우에만 `robot_3`를 spawn 또는 활성화합니다.
   - `robot_3`는 SLAM을 하지 않고, 알려진 초기 위치와 merged map 기반 경로를 사용합니다.
   - 초기 구현은 고정 safe insertion point에서 시작합니다.
   - 이후 확장은 `y = -x` 영역의 랜덤 insertion point를 지원합니다.

3. 전장 규칙 고도화
   - 오른쪽 위 enemy stronghold를 explicit red zone으로 config화합니다.
   - 적 시야 cone 또는 위험 영역을 coordinator와 AI 입력에 포함합니다.
   - scout 로봇은 red zone 직접 진입을 피하고, 중앙 hostage 후보 구역으로 수색 범위를 좁힙니다.

## 개발 환경

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10
- TurtleBot3 Gazebo 패키지

작업 경로:

```bash
/home/odin/robotics_ws/ros2_ws
└── src
    └── odin_rescue
```

## 패키지 구조

### `odin_bringup`

전체 시스템 실행을 담당하는 상위 launch 패키지입니다.

- `sim_multi_slam_map_merge.launch.py`: Gazebo, robot별 SLAM, merged map 노드, scout 주행 노드를 함께 실행합니다.
- `multi_slam_map_merge.launch.py`: Gazebo 없이 SLAM과 map merge만 실행합니다.

### `odin_gazebo`

Gazebo world와 로봇 spawn을 담당합니다.

- `house_easier_three_robots.launch.py`: 현재는 `robot_1`, `robot_2` 두 대만 spawn합니다.
- `worlds/odin_rescue_20x20_c.world`: 기본 전술 맵 C입니다. 20x20 평면 구조에 오른쪽 위 적 요충지 오브젝트와 ArUco hostage marker가 포함되어 있습니다.

현재 scout robot spawn pose:

- `robot_1`: `x=-7.5`, `y=7.5`, `yaw=-1.5708`
- `robot_2`: `x=7.5`, `y=-7.5`, `yaw=1.5708`

### `odin_slam`

로봇별 `slam_toolbox` 실행을 담당합니다.

- `multi_slam.launch.py`: `robot_1`, `robot_2` namespace 아래에 각각 `async_slam_toolbox_node`를 실행합니다.
- `config/slam_toolbox.yaml`: 2D 시뮬레이션 SLAM용 설정 파일입니다.

주요 map topic:

- `/robot_1/map`
- `/robot_2/map`

### `odin_map_merge`

현재 프로젝트에서 사용하는 merged map 생성 패키지입니다.

현재 고정 방식은 `merge_B`입니다.

- `scenario_scan_map_merge.py`
- `scenario_scan_map_merge.launch.py`
- `config/scenario_scan_map_merge.yaml`

이 노드는 다음 topic을 구독합니다.

- `/robot_1/odom`
- `/robot_1/scan`
- `/robot_2/odom`
- `/robot_2/scan`

그리고 다음 topic을 발행합니다.

- `/merged_map`

`/merged_map`은 20 m x 20 m 전역 occupancy grid에 각 로봇의 scan을 직접 누적해서 생성합니다. 또한 다른 로봇이 LiDAR에 감지됐을 때 merged map에 장애물로 남지 않도록 robot footprint와 robot-on-robot scan hit를 필터링합니다.

### `odin_exploration`

SLAM coverage를 위한 간단한 scout 자율 주행 패키지입니다.

- `reactive_scout.py`: LiDAR 기반 gap-following, escape behavior, center-spiral bias를 포함한 주행 노드입니다.
- `reactive_scouts.launch.py`: `robot_1`, `robot_2` namespace 아래에 scout 주행 노드를 실행합니다.

현재 scout 주행 규칙:

- `robot_1`은 왼쪽 위에서 아래 방향으로 출발합니다.
- `robot_2`는 오른쪽 아래에서 위 방향으로 출발합니다.
- 두 로봇은 중앙 근처에 hostage가 있다고 가정하고, 장애물 회피를 유지하면서 중앙으로 서서히 좁혀 들어가는 방향 bias를 받습니다.
- 오른쪽 위 enemy stronghold는 현재 Gazebo world와 시나리오에서 위험 구역으로 취급합니다.

### `odin_detection`

Gazebo 기반 ArUco hostage event 발행 패키지입니다.

- `gazebo_aruco_event_detector.py`: `/model_states`에서 scout 로봇과 `hostage_aruco_marker_0` 위치를 확인하고, 감지 거리/FOV 조건이 맞으면 event를 발행합니다.
- `gazebo_aruco_event_detector.launch.py`: detector 노드를 실행합니다.
- `config/gazebo_aruco_event_detector.yaml`: marker id, 대상 scout robot, 감지 거리, 감지 FOV, event topic을 설정합니다.

발행 topic:

- `/hostage_events`

event에는 다음 정보가 포함됩니다.

- marker id
- 감지 로봇 이름
- 추정 좌표 pose
- frame id
- timestamp

### `odin_coordinator`

Hostage event 검증과 `robot_3` rescue 후보 생성을 담당합니다.

- `rescue_coordinator.py`: `/hostage_events`, `/merged_map`, `/robot_1/odom`, `/robot_2/odom`, `/robot_3/odom`을 구독합니다.
- 중복 event, frame id, 좌표 범위, quaternion validity, map 접근성, robot_3 availability를 검증합니다.
- 왼쪽 아래 safe insertion point에서 hostage marker 근처 standoff goal까지의 후보 path를 생성합니다.
- Qwen/VLM waypoint는 `/ai/waypoint_recommendation`으로 받고, 검증을 통과한 경우에만 `/coordinator/validated_waypoint`로 발행합니다.

주요 topic:

- `/coordinator/status`
- `/coordinator/candidate_path`
- `/coordinator/validated_waypoint`
- `/robot_3/goal_pose`

### `odin_ai`

Qwen/VLM 연결 전까지 사용하는 local heuristic fallback입니다.

- `heuristic_waypoint_recommender.py`: `/coordinator/candidate_path`와 `/merged_map`을 구독합니다.
- 후보 경로의 goal 쪽 waypoint를 추천하고 `/ai/waypoint_recommendation`으로 발행합니다.
- 실제 로봇 제어 topic은 발행하지 않습니다.

Jetson Nano의 Qwen/VLM을 붙일 때는 이 fallback 노드 대신 같은 타입의 `/ai/waypoint_recommendation` publisher를 구현하면 됩니다.

### `robot_3` Dispatch

초기 구현은 Nav2가 아닌 보수적인 goal follower입니다.

- `robot_3`는 시작 시 Gazebo에 spawn되지 않으며 SLAM과 scout exploration을 수행하지 않습니다.
- `/robot_3/goal_pose`가 들어오면 spawn manager가 Gazebo에 `robot_3`를 생성하고, follower가 같은 goal을 따라갑니다.
- `simple_goal_follower.py`는 `/robot_3/odom`, `/robot_3/scan`, `/robot_3/goal_pose`를 구독하고 `/robot_3/cmd_vel`을 발행합니다.
- 전방 장애물이 가까우면 정지하고 `/robot_3/dispatch_status`에 상태를 발행합니다.

### 향후 추가 예정 패키지

다음 패키지는 이후 milestone에서 추가할 예정입니다.

- `odin_ai`: Qwen/VLM 기반 경로 후보 평가 또는 위험 구역 판단 보조

### 참고용 패키지

다음 패키지들은 현재 기본 실행 경로에는 포함되지 않지만, 실험 및 참고용으로 남겨져 있습니다.

- `multirobot_map_merge`
- `explore_lite`
- `explore_lite_msgs`
- `odin_navigation`

이전에 실험한 A/C/D map merge 방식은 프로젝트 패키지에서 제거하고 아래 위치에 따로 보관했습니다.

```bash
/home/odin/robotics_ws/ros2_ws/odin_rescue_map_merge_archive
```

## 주요 노드와 함수

### `ScenarioScanMapMerge`

파일:

```bash
odin_map_merge/odin_map_merge/scenario_scan_map_merge.py
```

역할:

- 로봇별 `/odom`과 `/scan`을 받아 전역 `/merged_map`을 생성합니다.
- 현재 구현은 map size를 알고 있는 scenario 기반 방식입니다.
- 다른 로봇이 장애물처럼 merged map에 찍히지 않도록 필터링합니다.

주요 함수:

- `_odom_callback`: `/robot_i/odom`에서 각 로봇의 현재 위치와 yaw를 저장합니다.
- `_scan_callback`: LiDAR scan endpoint를 전역 grid cell로 변환하고 free/occupied cell을 갱신합니다.
- `_is_other_robot_hit`: scan hit가 다른 로봇 위치 근처이면 occupied로 찍지 않도록 판단합니다.
- `_clear_robot_footprints`: publish 전 각 로봇 현재 위치 주변을 free로 지웁니다.
- `_raytrace_free`: LiDAR ray가 지나간 cell을 free로 표시합니다.
- `_publish_map`: 최종 `/merged_map`을 발행합니다.

### `ReactiveScout`

파일:

```bash
odin_exploration/odin_exploration/reactive_scout.py
```

역할:

- 로봇별 `/scan`, `/odom`을 이용해 `/cmd_vel`을 발행합니다.
- 기본적으로 LiDAR gap-following 방식으로 장애물을 피합니다.
- 너무 가까운 장애물이 있으면 후진 및 회전 escape behavior를 수행합니다.
- 중앙 hostage 후보 구역 가정을 반영한 center-spiral bias를 추가로 적용합니다.

주요 함수:

- `_scan_callback`: scan에서 전방/좌측/우측 거리와 열린 gap 후보를 계산합니다.
- `_control_loop`: 현재 mode와 scan 상태를 바탕으로 속도 명령을 생성합니다.
- `_best_gap_angle`: 가장 안전하게 진행할 수 있는 열린 방향을 선택합니다.
- `_enter_escape`: 장애물에 너무 가까울 때 후진 및 회전 mode로 진입합니다.
- `_apply_center_spiral_bias`: 중앙 hostage 후보 구역을 향해 서서히 좁혀 들어가는 방향 bias를 추가합니다.
- `_odom_callback`: center-spiral 계산에 사용할 현재 odom pose를 저장합니다.

## 빌드

workspace root에서 실행합니다.

```bash
cd /home/odin/robotics_ws/ros2_ws
colcon build --packages-select \
  odin_gazebo \
  odin_slam \
  odin_map_merge \
  odin_exploration \
  odin_interfaces \
  odin_detection \
  odin_coordinator \
  odin_ai \
  odin_navigation \
  odin_bringup
source install/setup.bash
```

## 실행

전체 시뮬레이션 실행:

```bash
cd /home/odin/robotics_ws/ros2_ws
source install/setup.bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py
```

위 명령은 Gazebo, scout SLAM, `/merged_map`, scout 자율주행, Gazebo 기반 ArUco event detector, coordinator, local AI fallback, robot_3 dispatch follower를 함께 실행합니다.

Gazebo GUI 없이 실행:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py gui:=false
```

감지 노드를 끄고 기존 자율주행 + merged map만 실행:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_detection:=false
```

coordinator를 끄고 실행:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_coordinator:=false
```

local AI fallback 또는 robot_3 dispatch follower를 끄고 실행:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_ai:=false
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py start_robot_3_dispatch:=false
```

SLAM과 merged map만 실행:

```bash
ros2 launch odin_bringup multi_slam_map_merge.launch.py
```

Gazebo와 로봇 spawn만 실행:

```bash
ros2 launch odin_gazebo house_easier_three_robots.launch.py
```

기본 실행은 맵 C를 사용합니다.

ArUco detector만 별도 실행:

```bash
ros2 launch odin_detection gazebo_aruco_event_detector.launch.py
```

Coordinator만 별도 실행:

```bash
ros2 launch odin_coordinator rescue_coordinator.launch.py
```

## RViz 확인

유용한 topic 확인:

```bash
ros2 topic list | grep -E 'robot_1|robot_2|merged_map'
ros2 topic echo /merged_map --once --field info
ros2 topic echo /hostage_events
ros2 topic echo /coordinator/status
ros2 topic echo /ai/status
ros2 topic echo /robot_3/dispatch_status
```

RViz fixed frame:

```text
map
```

권장 표시 topic:

- `/merged_map`
- `/robot_1/map`
- `/robot_2/map`
- `/hostage_events`
- `/coordinator/candidate_path`
- `/ai/waypoint_recommendation`
- `/robot_3/goal_pose`

## 현재 제한 사항

- `robot_3`는 검증된 dispatch goal이 발행된 뒤에만 spawn됩니다.
- `/merged_map`은 scan 기반 scenario 방식이며, 20 m x 20 m 전역 map bounds를 사용합니다.
- 개별 SLAM map(`/robot_i/map`)에는 다른 로봇이 동적 장애물로 남을 수 있습니다.
- merged map에서는 robot footprint와 robot-on-robot scan hit를 필터링합니다.
- ArUco detection은 현재 실제 카메라 영상 처리 대신 Gazebo world state 기반 감지 조건으로 event를 발행합니다.
- robot_3 dispatch는 현재 단순 goal follower 기반이며, 복잡한 장애물 우회와 global planning은 이후 Nav2 통합에서 고도화합니다.
- Qwen/VLM은 아직 Jetson Nano와 연결하지 않았고, 현재는 local heuristic fallback이 같은 topic 계약을 사용합니다.
- enemy zone은 현재 Gazebo world에 시각/장애물 오브젝트로 표현되어 있으며, 이후 coordinator config에서 명시적인 red zone 좌표로도 관리할 예정입니다.
