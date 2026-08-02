#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &&
  pwd
)"

exec "$SCRIPT_DIR/with_cyclone_camera.sh" \
  ros2 run rqt_image_view rqt_image_view
