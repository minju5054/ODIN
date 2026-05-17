#!/usr/bin/env bash
set -e

AI_CONFIG="/workspace/install/odin_ai/share/odin_ai/config/virtual_qwen_planner.yaml"
if [[ -n "${QWEN_API_URL:-}" && -f "${AI_CONFIG}" ]]; then
  sed -i "s#^    qwen_api_url: .*#    qwen_api_url: ${QWEN_API_URL}#g" "${AI_CONFIG}"
fi

exec ros2 launch odin_bringup sim_multi_slam_map_merge.launch.py \
  start_deployment_view:="${START_DEPLOYMENT_VIEW:-true}" \
  gui:="${GAZEBO_GUI:-true}" \
  "${ODIN_LAUNCH_ARGS[@]}"
