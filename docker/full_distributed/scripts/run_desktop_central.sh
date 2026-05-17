#!/usr/bin/env bash
set -e

BATTLEFIELD_CONFIG="/workspace/install/odin_bringup/share/odin_bringup/config/battlefield_rules.yaml"

ros2 launch odin_gazebo house_easier_three_robots.launch.py gui:="${GAZEBO_GUI:-true}" &
ros2 launch odin_map_merge scenario_scan_map_merge.launch.py &
ros2 launch odin_detection rgb_aruco_event_detector.launch.py &
ros2 launch odin_coordinator rescue_coordinator.launch.py battlefield_config_file:="${BATTLEFIELD_CONFIG}" &

if [[ "${START_MISSION_INTENT_GUI:-true}" == "true" ]]; then
  ros2 run odin_ai mission_intent_panel --ros-args -p use_sim_time:=true &
fi
if [[ "${START_GUI_PANELS:-true}" == "true" ]]; then
  ros2 run odin_navigation qwen_dialog_panel --ros-args -p use_sim_time:=true &
  ros2 run odin_navigation qwen_route_map_panel --ros-args -p use_sim_time:=true &
  ros2 run odin_navigation mission_status_panel --ros-args -p use_sim_time:=true &
fi
if [[ "${START_DEPLOYMENT_VIEW:-true}" == "true" ]]; then
  ros2 run odin_bringup deployment_view --ros-args -p refresh_period_sec:=5.0 &
fi

wait -n
