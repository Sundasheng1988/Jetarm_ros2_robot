#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    test_command_arg = DeclareLaunchArgument(
        "test_command",
        default_value="pick red cup to right side",
        description="Fake user command for the test chain",
    )

    return LaunchDescription([
        test_command_arg,
        Node(
            package="sketch_runtime",
            executable="runtime_test_node",
            name="runtime_test_node",
            output="screen",
            parameters=[{
                "test_command": LaunchConfiguration("test_command"),
                "hover_height": 0.08,
                "approach_z": 0.015,
                "lift_height": 0.08,
                "auto_confirm": True,
            }],
        ),
    ])
