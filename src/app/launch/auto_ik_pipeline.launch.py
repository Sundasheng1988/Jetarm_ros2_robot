from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='app',
            executable='object_detection',
            name='object_detection',
            output='screen',
        ),
        Node(
            package='kinematics',
            executable='ik_solver_node',
            name='ik_solver_node',
            output='screen',
        ),
        Node(
            package='kinematics',
            executable='vision_ik_client',
            name='vision_ik_client',
            output='screen',
        )
    ])

