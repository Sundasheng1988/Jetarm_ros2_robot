# face_follow_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='social_robot',
            executable='face_follow_node',
            name='face_follow',
            output='screen',
            parameters=[
                {'camera_topic': '/depth_cam/rgb/image_raw'},
                {'display': False},
            ]
        ),
        
        # 手势动作节点（点头 nod）
        Node(
            package='social_robot',
            executable='gesture_player_node',
            name='gesture_player',
            output='screen',
        ),
        
        # 环境扫描
        #Node(
        #    package='social_robot',
        #    executable='env_scan_node',
        #    name='env_scan',
        #    output='screen',
        #),
        # 环境扫描
        Node(
            package='social_robot',
            executable='static_env_report_node',
            name='static_env_report',
            output='screen',
        ),
        # 环境扫描
        #Node(
        #    package='social_robot',
        #    executable='world_model_node',
        #    name='world_model_node',
        #    output='screen',
        #    parameters=[
        #        # ===== 联调期参数（强烈建议）=====
        #        {'merge_distance': 0.8},   # 米：允许较大抖动
        #        {'min_hits': 1},           # 一次命中就发布
        #        {'ttl': 5.0},              # 秒：避免被快速清理
        #    ]
        #),
    ])

