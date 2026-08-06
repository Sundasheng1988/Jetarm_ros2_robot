#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/sundasheng/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/sundasheng/ros2_ws/config/cyclonedds/pc_camera_eno1.xml

unset ROS2CLI_DISABLE_DAEMON

exec "$@"
