#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &&
  pwd
)"

exec "$SCRIPT_DIR/with_cyclone_camera.sh" \
  ros2 launch app perception_bringup.launch.py \
  start_grounding:=false
