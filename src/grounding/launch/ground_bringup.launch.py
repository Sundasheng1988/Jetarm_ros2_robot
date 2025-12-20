#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 默认参数文件
    default_params = os.path.join(
        get_package_share_directory('grounding'),
        'config', 'grounding_params.yaml'
    )

    # ---- launch args ----
    model            = LaunchConfiguration('model')
    ollama_base      = LaunchConfiguration('ollama_base')
    params_file      = LaunchConfiguration('params_file')
    start_ollama     = LaunchConfiguration('start_ollama')
    start_keyboard   = LaunchConfiguration('start_keyboard')
    use_dummy_wm     = LaunchConfiguration('use_dummy_wm')
    use_tf_wm        = LaunchConfiguration('use_tf_wm')
    use_vision_wm    = LaunchConfiguration('use_vision_wm')
    parser_pkg       = LaunchConfiguration('parser_pkg')
    parser_exe       = LaunchConfiguration('parser_exe')
    world_frame      = LaunchConfiguration('world_frame')

    return LaunchDescription([
        # ---- args ----
        DeclareLaunchArgument('model',           default_value='qwen:1.8b'),
        DeclareLaunchArgument('ollama_base',     default_value='http://127.0.0.1:11434'),
        DeclareLaunchArgument('params_file',     default_value=default_params),
        DeclareLaunchArgument('start_ollama',    default_value='false'),
        DeclareLaunchArgument('start_keyboard',  default_value='true'),
        DeclareLaunchArgument('use_dummy_wm',    default_value='true'),
        DeclareLaunchArgument('use_tf_wm',       default_value='false'),
        DeclareLaunchArgument('use_vision_wm',   default_value='false'),
        DeclareLaunchArgument('parser_pkg',      default_value='llm_parser'),
        DeclareLaunchArgument('parser_exe',      default_value='llm_command_parser_node'),
        DeclareLaunchArgument('world_frame',     default_value='table'),

        # 传环境变量给解析节点
        SetEnvironmentVariable('LLM_MODEL',   model),
        SetEnvironmentVariable('OLLAMA_BASE', ollama_base),

        # 可选：把大模型本地服务也一起拉起
        ExecuteProcess(
            cmd=['bash', '-lc', 'ollama serve'],
            output='screen',
            condition=IfCondition(start_ollama)
        ),

        # 1) 解析节点（包/可执行体可替换）
        Node(
            package=parser_pkg,
            executable=parser_exe,
            name='llm_command_parser_node',
            output='screen'
        ),

        # 2) grounding 节点
        Node(
            package='grounding',
            executable='grounding_node',
            name='grounding_node',
            parameters=[params_file],
            output='screen'
        ),

        # 3a) 世界模型：dummy（仿真）
        Node(
            package='grounding',
            executable='wm_dummy_pub',
            name='wm_dummy_pub',
            condition=IfCondition(use_dummy_wm),
            output='screen'
        ),

        # 3b) 世界模型：从 TF 桥接（如果你用了 grounding/wm_from_tf.py）
        Node(
            package='grounding',
            executable='wm_from_tf',
            name='wm_from_tf',
            condition=IfCondition(use_tf_wm),
            parameters=[{
                'world_frame': world_frame,
                # 这里给出常用前缀，按需改
                'frame_prefixes': ['cup', 'ball', 'obj']
            }],
            output='screen'
        ),

        # 3c) 世界模型：从 vision_msgs 桥接（如果你用了 grounding/wm_from_vision_msgs.py）
        Node(
            package='grounding',
            executable='wm_from_vision',
            name='wm_from_vision',
            condition=IfCondition(use_vision_wm),
            parameters=[{'world_frame': world_frame}],
            output='screen'
        ),

    ])

