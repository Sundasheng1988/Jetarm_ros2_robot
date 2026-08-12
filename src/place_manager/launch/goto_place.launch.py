"""
Launch file for goto_place_node — semantic navigation via Nav2.

Usage::

    ros2 launch place_manager goto_place.launch.py
    ros2 launch place_manager goto_place.launch.py places_file:=/path/to/custom.yaml
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
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom_combined',
            description='到达验证用里程计话题（当前 JetArm 实机为 /odom_combined）',
        ),
        Node(
            package='place_manager',
            executable='goto_place_node',
            name='goto_place',
            output='screen',
            parameters=[{
                'places_file': LaunchConfiguration('places_file'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }],
        ),
    ])
