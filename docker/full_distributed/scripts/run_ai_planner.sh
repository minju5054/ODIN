#!/usr/bin/env bash
set -e

AI_CONFIG="/workspace/install/odin_ai/share/odin_ai/config/virtual_qwen_planner.yaml"
BATTLEFIELD_CONFIG="/workspace/install/odin_bringup/share/odin_bringup/config/battlefield_rules.yaml"

if [[ -n "${QWEN_API_URL:-}" && -f "${AI_CONFIG}" ]]; then
  sed -i "s#^    qwen_api_url: .*#    qwen_api_url: ${QWEN_API_URL}#g" "${AI_CONFIG}"
fi

exec ros2 launch odin_ai virtual_qwen_planner.launch.py \
  start_mission_intent_gui:=false \
  battlefield_config_file:="${BATTLEFIELD_CONFIG}"
