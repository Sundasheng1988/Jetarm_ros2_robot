#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    grounding_launch = os.path.join(
        get_package_share_directory("grounding"),
        "launch",
        "ground_bringup.launch.py",
    )

    model = LaunchConfiguration("model")
    ollama_base = LaunchConfiguration("ollama_base")
    params_file = LaunchConfiguration("params_file")
    start_ollama = LaunchConfiguration("start_ollama")
    start_keyboard = LaunchConfiguration("start_keyboard")
    use_dummy_wm = LaunchConfiguration("use_dummy_wm")
    use_tf_wm = LaunchConfiguration("use_tf_wm")
    use_vision_wm = LaunchConfiguration("use_vision_wm")
    parser_pkg = LaunchConfiguration("parser_pkg")
    parser_exe = LaunchConfiguration("parser_exe")
    world_frame = LaunchConfiguration("world_frame")

    dry_run = LaunchConfiguration("dry_run")
    run_once = LaunchConfiguration("run_once")

    return LaunchDescription([
        # ── grounding pipeline args (passthrough) ──
        DeclareLaunchArgument("model", default_value="qwen:1.8b"),
        DeclareLaunchArgument("ollama_base", default_value="http://127.0.0.1:11434"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("grounding"),
                "config",
                "grounding_params.yaml",
            ),
        ),
        DeclareLaunchArgument("start_ollama", default_value="false"),
        DeclareLaunchArgument("start_keyboard", default_value="true"),
        DeclareLaunchArgument("use_dummy_wm", default_value="true"),
        DeclareLaunchArgument("use_tf_wm", default_value="false"),
        DeclareLaunchArgument("use_vision_wm", default_value="false"),
        DeclareLaunchArgument("parser_pkg", default_value="llm_parser"),
        DeclareLaunchArgument("parser_exe", default_value="llm_command_parser_node"),
        DeclareLaunchArgument("world_frame", default_value="table"),

        # ── runtime node args ──
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("run_once", default_value="true"),

        # ── grounding bringup ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(grounding_launch),
            launch_arguments={
                "model": model,
                "ollama_base": ollama_base,
                "params_file": params_file,
                "start_ollama": start_ollama,
                "start_keyboard": start_keyboard,
                "use_dummy_wm": use_dummy_wm,
                "use_tf_wm": use_tf_wm,
                "use_vision_wm": use_vision_wm,
                "parser_pkg": parser_pkg,
                "parser_exe": parser_exe,
                "world_frame": world_frame,
            }.items(),
        ),

        # ── runtime bridge node ──
        Node(
            package="sketch_runtime",
            executable="real_grounded_runtime_node",
            name="real_grounded_runtime_node",
            output="screen",
            parameters=[{
                "dry_run": dry_run,
                "run_once": run_once,
                "input_topic": "/grounded_task_context",
            }],
        ),
    ])
