# ODIN-RESCUE

ODIN-RESCUE is a ROS 2 Humble + Gazebo Classic 11 multi-robot rescue simulation.

The current milestone focuses on two scout robots that explore a known 20 m x 20 m rescue arena, run per-robot SLAM, and publish a shared `/merged_map` from robot odometry and laser scans. The design keeps robot topics separated by namespace so the system can later be split across Jetson devices.

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10
- TurtleBot3 Gazebo packages

Workspace layout:

```bash
/home/odin/robotics_ws/ros2_ws
└── src
    └── odin_rescue
```

## Packages

### `odin_bringup`

Top-level launch package.

- `sim_multi_slam_map_merge.launch.py`: launches Gazebo, two robot SLAM nodes, the scan-based merged map node, and delayed scout motion.
- `multi_slam_map_merge.launch.py`: launches only SLAM and map merge.

### `odin_gazebo`

Gazebo worlds and robot spawning.

- `house_easier_three_robots.launch.py`: currently spawns `robot_1` and `robot_2`.
- `worlds/odin_rescue_20x20.world`: default map A.
- `worlds/odin_rescue_20x20_b.world`: alternate asymmetric map B kept for testing.

Current scout spawn poses:

- `robot_1`: `x=-7.5`, `y=7.5`, `yaw=-1.5708`
- `robot_2`: `x=7.5`, `y=-7.5`, `yaw=1.5708`

### `odin_slam`

Per-robot `slam_toolbox` integration.

- `multi_slam.launch.py`: starts one `async_slam_toolbox_node` for each scout namespace.
- `config/slam_toolbox.yaml`: SLAM parameters tuned for quick 2D simulation mapping.

Published map topics:

- `/robot_1/map`
- `/robot_2/map`

### `odin_map_merge`

Scan-based merged map generation.

Current fixed implementation:

- `scenario_scan_map_merge.py`
- `scenario_scan_map_merge.launch.py`
- `config/scenario_scan_map_merge.yaml`

This node subscribes to:

- `/robot_1/odom`
- `/robot_1/scan`
- `/robot_2/odom`
- `/robot_2/scan`

It publishes:

- `/merged_map`

The merged map is built directly in a known 20 m x 20 m global occupancy grid. It also filters robot-on-robot detections so scout robots do not remain as obstacles in the merged map.

### `odin_exploration`

Simple autonomous scout motion for SLAM coverage.

- `reactive_scout.py`: reactive gap-following controller with escape behavior and center-spiral bias.
- `reactive_scouts.launch.py`: starts one scout controller in each robot namespace.

Current behavior:

- `robot_1` starts from the upper-left area and initially drives downward.
- `robot_2` starts from the lower-right area and initially drives upward.
- Both robots bias toward the center while keeping obstacle avoidance active.

### Other Reference Packages

The repository still contains reference packages such as `multirobot_map_merge`, `explore_lite`, `explore_lite_msgs`, and `odin_navigation`. They are not part of the current default bringup path.

Earlier map merge experiments are archived outside this project at:

```bash
/home/odin/robotics_ws/ros2_ws/odin_rescue_map_merge_archive
```

## Main Nodes And Functions

### `ScenarioScanMapMerge`

File:

```bash
odin_map_merge/odin_map_merge/scenario_scan_map_merge.py
```

Important methods:

- `_odom_callback`: stores each robot pose from `/robot_i/odom`.
- `_scan_callback`: projects each laser scan into the global occupancy grid.
- `_is_other_robot_hit`: skips occupied hits near the other robot position.
- `_clear_robot_footprints`: clears each robot's current footprint from `/merged_map`.
- `_raytrace_free`: marks free cells along each laser ray.
- `_publish_map`: publishes the final `/merged_map`.

### `ReactiveScout`

File:

```bash
odin_exploration/odin_exploration/reactive_scout.py
```

Important methods:

- `_scan_callback`: extracts front, left, right, and gap information from `/scan`.
- `_control_loop`: publishes `/cmd_vel` based on obstacle clearance and target direction.
- `_best_gap_angle`: chooses the safest open scan direction.
- `_enter_escape`: backs up and turns when the robot is too close to an obstacle.
- `_apply_center_spiral_bias`: nudges the robot toward a center-focused spiral scan pattern.
- `_odom_callback`: stores odometry for the center-spiral bias.

## Build

From the workspace root:

```bash
cd /home/odin/robotics_ws/ros2_ws
colcon build --packages-select \
  odin_gazebo \
  odin_slam \
  odin_map_merge \
  odin_exploration \
  odin_bringup
source install/setup.bash
```

## Run

Full simulation:

```bash
cd /home/odin/robotics_ws/ros2_ws
source install/setup.bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py
```

Headless simulation:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py gui:=false
```

SLAM + merged map only:

```bash
ros2 launch odin_bringup multi_slam_map_merge.launch.py
```

Gazebo only:

```bash
ros2 launch odin_gazebo house_easier_three_robots.launch.py
```

Use alternate map B:

```bash
ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py \
  world:=/home/odin/robotics_ws/ros2_ws/install/odin_gazebo/share/odin_gazebo/worlds/odin_rescue_20x20_b.world
```

## RViz Checks

Useful topics:

```bash
ros2 topic list | grep -E 'robot_1|robot_2|merged_map'
ros2 topic echo /merged_map --once --field info
```

Recommended RViz fixed frame:

```text
map
```

Map displays:

- `/merged_map`
- `/robot_1/map`
- `/robot_2/map`

## Notes

- `robot_3` is intentionally not spawned in the current SLAM milestone.
- The current merged map is scan-based and assumes known global map bounds.
- Per-robot SLAM maps may still contain dynamic robot artifacts, but `/merged_map` filters robot footprints and robot-on-robot scan hits.
- Victim detection, coordinator dispatch, robot_3 rescue behavior, and AI integration are planned later milestones.
