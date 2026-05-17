#!/usr/bin/env bash
set -e

ROBOT_NAME="${1:-${ROBOT_NAME:-robot_1}}"
SLAM_PARAMS="/workspace/install/odin_slam/share/odin_slam/config/slam_toolbox.yaml"

ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -r __ns:=/"${ROBOT_NAME}" \
  --params-file "${SLAM_PARAMS}" \
  -p use_sim_time:=true \
  -p scan_topic:=scan \
  -p map_frame:="${ROBOT_NAME}/map" \
  -p odom_frame:="${ROBOT_NAME}/odom" \
  -p base_frame:="${ROBOT_NAME}/base_footprint" &

ros2 run odin_exploration reactive_scout \
  --ros-args \
  -r __ns:=/"${ROBOT_NAME}" \
  -p use_sim_time:=true
