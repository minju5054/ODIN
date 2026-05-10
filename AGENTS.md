# Project Rules: ODIN

These rules apply to the ODIN project in addition to the global rules in `/home/odin/.codex/AGENTS.md`.

## Project Context

ODIN is a ROS 2 Humble and Gazebo Classic 11 based multi-robot tactical hostage rescue simulation project.

The core goal is to build an on-device AI based multi-robot tactical rescue system that can grow from a single desktop simulation into a distributed Jetson-based deployment.

Development environment:

- OS: Ubuntu 22.04
- ROS: ROS 2 Humble
- Python: Python 3.10
- Simulator: Gazebo Classic 11
- Workspace root: `/home/odin/robotics_ws/ros2_ws`
- Project path: `/home/odin/robotics_ws/ros2_ws/src/odin_rescue`

## Architecture Principles

Design for a distributed ROS 2 system from the beginning.

- Do not assume all nodes run on one machine.
- Do not rely on shared memory between robots or nodes.
- Do not hardcode `localhost` as a system assumption.
- Do not rely on absolute machine-specific paths.
- Avoid monolithic scripts that combine unrelated responsibilities.
- Communicate through ROS 2 topics, services, actions, parameters, and launch files.
- Each robot stack must be able to run independently under its own namespace.

Target deployment model:

- Jetson 1: `robot_1` scout stack
- Jetson 2: `robot_2` scout stack
- Jetson 3: `robot_3` hostage rescue stack
- Jetson 4: optional Qwen / LLM / VLM node
- Jetson 5: optional monitoring / backup AI node
- Desktop/Laptop: Gazebo, RViz, visualization, and development

## Robot Roles

The baseline system has three robots.

- `robot_1`: scout robot A
- `robot_2`: scout robot B
- `robot_3`: hostage rescue robot C

`robot_1` and `robot_2` perform exploration and SLAM.

`robot_3` is the hostage rescue robot. It waits at the safe insertion point and must not move until dispatched by the coordinator. `robot_3` does not perform SLAM during the initial system design. It uses its known spawn pose and later navigates to a validated rescue goal only after a hostage event and coordinator command.

All three robots have known fixed spawn poses inside the simulation world:

- `robot_1` knows its initial spawn pose.
- `robot_2` knows its initial spawn pose.
- `robot_3` spawns in a safe corner of the map and knows its initial spawn pose.

Use this known-pose assumption to keep the first multi-robot SLAM and map merge implementation simple. Do not start with unknown-initial-pose map merging unless explicitly requested.

## Current Priority

The current priority is stable multi-robot structure, not SLAM quality or AI integration.

Use the custom flat tactical arena as the initial simulation world. The default world is a flat 20 m x 20 m map with walls, obstacles, a right-top enemy stronghold area, and an ArUco ID `0` hostage surrogate marker.

The right-top area is the enemy stronghold / red zone. Scout robots should avoid directly targeting that area and should instead search toward the central hostage candidate area.

Default world:

- `odin_gazebo/worlds/odin_rescue_20x20_c.world`

Implement in this order unless the user explicitly changes the priority:

1. Minimal ROS 2 package structure
2. Successful `colcon build`
3. Spawn `robot_1` and `robot_2` in Gazebo
4. Separate robot namespaces
5. Verify per-robot topics
6. Move each robot individually through `cmd_vel` or teleop
7. Single and multi-robot SLAM for `robot_1` and `robot_2`
8. Map sharing or map merge
9. ArUco based hostage detection
10. Coordinator node
11. `robot_3` hostage rescue dispatch
12. Optional Qwen / LLM / VLM integration

Do not jump to AI, VLM, LLM, advanced navigation, or complex coordination before the basic three-robot namespace and topic structure works.

## Package Boundaries

Prefer role-based ROS 2 packages. Each package should have one clear responsibility.

Expected package split:

- `odin_bringup`: top-level launch and system composition
- `odin_description`: robot URDF/Xacro, meshes, robot model assets
- `odin_gazebo`: Gazebo worlds, spawn logic, simulation plugins
- `odin_slam`: SLAM configuration and SLAM launch integration
- `odin_map_merge`: known-pose occupancy grid merging from scout robot maps
- `odin_exploration`: simple scout motion or exploration goal generation for `robot_1` and `robot_2`
- `odin_detection`: ArUco hostage detection and hostage event publishing
- `odin_coordinator`: hostage event handling, validation, dispatch decisions
- `odin_navigation`: navigation, goal execution, Nav2 integration
- `odin_ai`: optional AI waypoint recommendation or ranking

Do not put all functionality into one script or one package for convenience.

## ROS Naming And Namespaces

Use explicit namespace-based topic naming.

Expected topic examples:

- `/robot_1/scan`
- `/robot_1/odom`
- `/robot_1/cmd_vel`
- `/robot_1/map`
- `/robot_2/scan`
- `/robot_2/odom`
- `/robot_2/cmd_vel`
- `/robot_2/map`
- `/robot_3/scan`
- `/robot_3/odom`
- `/robot_3/cmd_vel`
- `/robot_3/goal_pose`
- `/hostage_events`
- `/merged_map`
- `/coordinator/status`

When adding nodes or launch files:

- Use namespaces explicitly.
- Avoid hidden remaps that make topic ownership unclear.
- Keep frame names and topic names readable.
- Prefer launch arguments for robot name, namespace, pose, and world configuration.

## Hostage Detection

Hostage detection uses ArUco markers as a surrogate for real human detection.

- Use ArUco marker ID `0` as the hostage surrogate.
- Do not introduce YOLO, RGB-D human detection, or heavyweight perception dependencies unless explicitly requested.
- The goal is to implement the multi-robot event-driven hostage rescue flow, not high-accuracy perception.

## Coordinator Rules

The coordinator is the system mediator.

The coordinator receives hostage events, checks robot and map context, validates candidate goals, and dispatches `robot_3`.

AI modules may recommend waypoints or rank victim candidates, but they must not directly control robots. Any AI-produced waypoint must be validated by the coordinator before use.

Coordinator validation must include:

- `frame_id`
- `x`, `y`, and `yaw` validity
- coordinate bounds
- duplicate hostage event detection
- `robot_3` availability
- map accessibility

If a command, waypoint, event, or state is unsafe or ambiguous, the coordinator must reject it in a fail-safe way.

## Coding Rules

Write maintainable research-grade code, not disposable demos.

- Make small, focused changes.
- Implement the minimum working feature first.
- Keep launch-based workflows repeatable.
- Use explicit topic names and namespaces.
- Keep node structure readable.
- Use ROS loggers instead of bare `print` for ROS nodes.
- Avoid monolithic scripts.
- Avoid hardcoded paths.
- Avoid unrelated file rewrites.
- Minimize heavy dependencies.
- Prefer parameters and config files over runtime constants when values may change between robots or machines.

## Git And Collaboration Rules

This project is collaborative. Protect the shared repository history.

- Do not commit automatically after making changes.
- Only create commits when the user explicitly asks for a commit.
- Do not push directly to `main` unless the user explicitly asks for it.
- Prefer feature branches for collaboration, for example `feature/<short-task-name>`.
- Do not use force push, force-with-lease, history rewrite, or destructive git commands unless the user explicitly asks for that exact operation.
- Before starting new work intended for GitHub, check the current branch and status.
- Keep unrelated user changes out of commits.

## Build And Verification

Run commands from the workspace root unless there is a specific reason not to.

Build:

```bash
cd /home/odin/robotics_ws/ros2_ws
colcon build --packages-select odin_rescue
```

When the project is split into multiple packages, build the relevant package or packages explicitly:

```bash
cd /home/odin/robotics_ws/ros2_ws
colcon build --packages-select <package_name>
```

Test:

```bash
cd /home/odin/robotics_ws/ros2_ws
colcon test --packages-select <package_name>
colcon test-result --verbose
```

Minimum verification should match the current milestone:

- Package structure exists and builds.
- Gazebo launches without crashing.
- `robot_1` and `robot_2` spawn for the current scout SLAM milestone.
- `robot_1` and `robot_2` spawn at their configured known poses.
- Robot namespaces are separated.
- Per-robot topics are visible.
- Each robot can be moved independently.
- SLAM and map topics are visible when that milestone is reached.
- `robot_3` does not run SLAM in the initial architecture.
- Hostage events are published when ArUco detection is implemented.
- Coordinator dispatches `robot_3` only after validation.

If a verification step cannot be run, state the reason clearly.

## Safety And Scope

Treat motion, navigation, dispatch, and actuator-like behavior conservatively.

- Do not move `robot_3` unless the coordinator dispatch path requires it.
- Do not bypass coordinator validation for convenience.
- Do not make AI modules authoritative over robot motion.
- Prefer simulation or dry-run validation before assuming real robot behavior.
- Keep the project aligned with distributed multi-robot ROS architecture before adding advanced AI.
