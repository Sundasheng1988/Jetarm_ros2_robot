#!/usr/bin/env bash
set -e

ENV_SCRIPT="$HOME/ros2_ws/scripts/ros_env_pc.sh"

if [ ! -r "$ENV_SCRIPT" ]; then
  echo "ERROR: 找不到 ROS 环境脚本："
  echo "  $ENV_SCRIPT"
  exit 1
fi

# Camera / perception cross-host mode must use the unified Robot environment.
source "$ENV_SCRIPT" robot

# This wrapper is specifically for cross-host camera access.
# If the verified robot DDS interface is unavailable, do not silently continue.
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  echo "ERROR: Robot DDS environment is not ready."
  echo "       CYCLONEDDS_URI is unset."
  echo "       请检查 eno1 网线、192.168.100.x 地址和 Cyclone XML。"
  exit 2
fi

unset ROS2CLI_DISABLE_DAEMON

exec "$@"