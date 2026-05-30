#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    transform_yaml_arg = DeclareLaunchArgument(
        'transform_yaml',
        default_value='/home/sundasheng/ros2_ws/src/app/config/transform.yaml',
    )

    lab_config_arg = DeclareLaunchArgument(
        'lab_config',
        default_value='/home/sundasheng/ros2_ws/src/app/config/lab_config.yaml',
    )

    roi_node = Node(
        package='app',
        executable='roi_color_detector_node',
        name='roi_color_detector_node',
        output='screen',
        parameters=[
            {'transform_yaml': LaunchConfiguration('transform_yaml')},
            {'lab_config': LaunchConfiguration('lab_config')},
        ],
    )

    yolo_node = Node(
        package='vision_yolo',
        executable='simple_yolo_node',
        name='simple_yolo_node',
        output='screen',
    )

    fusion_node = Node(
        package='app',
        executable='perception_fusion_node',
        name='perception_fusion_node',
        output='screen',
    )

    tracker_node = Node(
        package='app',
        executable='stable_object_tracker_node',
        name='stable_object_tracker_node',
        output='screen',
    )

    grounding_node = Node(
        package='grounding',
        executable='grounding_node',
        name='grounding_node',
        output='screen',
        parameters=[
            {'publish_runtime': True},
            {'world_model_topic': '/world_model/stable_objects'},
        ],
    )

    return LaunchDescription([
        transform_yaml_arg,
        lab_config_arg,
        roi_node,
        yolo_node,
        fusion_node,
        tracker_node,
        grounding_node,
    ])
