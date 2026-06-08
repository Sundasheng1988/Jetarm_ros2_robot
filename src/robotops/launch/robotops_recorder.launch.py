import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="robotops",
            executable="robotops_recorder_node",
            name="robotops_recorder_node",
            output="screen",
            parameters=[{
                "db_path": "robotops.db",
            }],
        ),
    ])
