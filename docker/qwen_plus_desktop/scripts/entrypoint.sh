#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

exec "$@"
