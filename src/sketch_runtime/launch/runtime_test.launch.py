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
    run_once_arg = DeclareLaunchArgument(
        "run_once",
        default_value="true",
        description="Run once (true) or loop (false)",
    )
    interval_sec_arg = DeclareLaunchArgument(
        "interval_sec",
        default_value="1.0",
        description="Loop interval in seconds (only used when run_once=false)",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="true",
        description="Use RuntimeAdapter in dry_run mode (no real hardware)",
    )

    return LaunchDescription([
        test_command_arg,
        run_once_arg,
        interval_sec_arg,
        dry_run_arg,
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
                "run_once": LaunchConfiguration("run_once"),
                "interval_sec": LaunchConfiguration("interval_sec"),
                "dry_run": LaunchConfiguration("dry_run"),
            }],
        ),
    ])
