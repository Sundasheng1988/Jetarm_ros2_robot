"""
Launch file for the voice-controlled named-place navigation stack.

启动内容：

1. ``goto_place_node`` — 提供 ``/goto_place``（地点名称 → Nav2）与
   ``/cancel_navigation``（取消当前 Nav2 目标）两个服务。
2. ``navigation_executor_node`` — 消费 ``/parsed_command`` 中的导航动作，
   路由到 ``/goto_place``，并把结果发布到 ``/runtime/execution_result``。
3. ``executor_done_sayer``（可选）— 把导航结果翻译成语音反馈。

用法::

    # 独立测试导航栈（自带播报）
    ros2 launch place_manager navigation.launch.py

    # 与 voice_stack.launch.py 同时运行时：voice_stack 已自带
    # executor_done_sayer（机械臂完成 + 导航结果双通道），这里关闭它，
    # 避免出现两个 done_sayer 节点导致重复播报。
    ros2 launch place_manager navigation.launch.py launch_done_sayer:=false

    # 自定义地点文件
    ros2 launch place_manager navigation.launch.py places_file:=/path/to/places.yaml

注意：本 launch 不启动 Nav2 本身、不发布 /cmd_vel，也不启动语音识别 /
Parser。完整闭环还需 ``voice_stack.launch.py``（ASR + Rebecca +
llm_command_parser + done_sayer）与 Nav2 bringup。
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_places_file = os.path.join(
        str(Path.home()),
        'ros2_ws',
        'config',
        'places.yaml',
    )

    places_file = DeclareLaunchArgument(
        'places_file',
        default_value=default_places_file,
        description='路径：places YAML 数据文件（默认 ~/ros2_ws/config/places.yaml）',
    )
    # 到达验证用的里程计话题。当前 JetArm 实机为 /odom_combined；这里显式声明，
    # 避免依赖隐藏默认值，也便于在话题不同时直接覆盖。
    odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom_combined',
        description='到达验证用里程计话题（当前 JetArm 实机为 /odom_combined）',
    )
    parsed_command_topic = DeclareLaunchArgument(
        'parsed_command_topic',
        default_value='/parsed_command',
        description='导航执行器订阅的解析命令话题',
    )
    execution_result_topic = DeclareLaunchArgument(
        'execution_result_topic',
        default_value='/runtime/execution_result',
        description='导航执行器发布结果 / done_sayer 订阅结果的话题',
    )
    reply_topic = DeclareLaunchArgument(
        'reply_topic',
        default_value='/speech_reply',
        description='done_sayer 语音播报话题',
    )
    launch_done_sayer = DeclareLaunchArgument(
        'launch_done_sayer',
        default_value='true',
        description='是否在本 launch 中启动 executor_done_sayer。'
                    '与 voice_stack.launch.py 同跑时应设为 false。',
    )

    goto_place_node = Node(
        package='place_manager',
        executable='goto_place_node',
        name='goto_place',
        output='screen',
        parameters=[{
            'places_file': LaunchConfiguration('places_file'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }],
    )

    navigation_executor_node = Node(
        package='place_manager',
        executable='navigation_executor_node',
        name='navigation_executor_node',
        output='screen',
        parameters=[{
            'parsed_command_topic': LaunchConfiguration('parsed_command_topic'),
            'execution_result_topic': LaunchConfiguration('execution_result_topic'),
        }],
    )

    done_sayer = Node(
        package='llm_voice_agent',
        executable='executor_done_sayer',
        name='executor_done_sayer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_done_sayer')),
        parameters=[{
            'done_topic': '/executor/done',
            'execution_result_topic': LaunchConfiguration('execution_result_topic'),
            'reply_topic': LaunchConfiguration('reply_topic'),
        }],
    )

    return LaunchDescription([
        places_file,
        odom_topic,
        parsed_command_topic,
        execution_result_topic,
        reply_topic,
        launch_done_sayer,
        goto_place_node,
        navigation_executor_node,
        done_sayer,
    ])
