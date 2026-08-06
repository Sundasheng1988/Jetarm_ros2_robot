#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("toy_block_perception")
    )
    config_path = package_share / "config" / "toy_block_color_detector.yaml"

    return LaunchDescription([
        Node(
            package="toy_block_perception",
            executable="toy_block_color_detector",
            name="toy_block_color_detector",
            output="screen",
            parameters=[str(config_path)],
        )
    ])
