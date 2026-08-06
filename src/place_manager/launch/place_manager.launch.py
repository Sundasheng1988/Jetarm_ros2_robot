"""
Launch file for place_manager node.

Usage::

    ros2 launch place_manager place_manager.launch.py
    ros2 launch place_manager place_manager.launch.py places_file:=/path/to/custom.yaml
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_places_file = os.path.join(
        str(Path.home()),
        'ros2_ws',
        'config',
        'places.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'places_file',
            default_value=default_places_file,
            description='路径：places YAML 数据文件（默认 ~/ros2_ws/config/places.yaml）',
        ),
        Node(
            package='place_manager',
            executable='place_manager_node',
            name='place_manager',
            output='screen',
            parameters=[{
                'places_file': LaunchConfiguration('places_file'),
            }],
        ),
    ])
