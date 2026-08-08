from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start only the Agent; ASR, TTS and task runtime are not launched."""
    return LaunchDescription([
        DeclareLaunchArgument('glm_model', default_value='glm-4.5-air'),
        DeclareLaunchArgument('enable_robot_side_effects', default_value='false'),
        Node(
            package='llm_voice_agent',
            executable='llm_voice_agent_node',
            name='llm_voice_agent',
            output='screen',
            parameters=[{
                'query_topic': '/speech_query',
                'reply_topic': '/speech_reply',
                'publish_topics': ['/voice_input/input'],
                'enable_robot_side_effects': ParameterValue(
                    LaunchConfiguration('enable_robot_side_effects'),
                    value_type=bool,
                ),
                'use_llm': True,
                'llm_backend': 'glm',
                'llm_fallback_backend': 'ollama',
                'glm_api_base': 'https://open.bigmodel.cn/api/paas/v4',
                'glm_api_key_env': 'ZHIPUAI_API_KEY',
                'glm_model': LaunchConfiguration('glm_model'),
                'glm_thinking': False,
                'llm_stream': True,
                'stream_to_tts': True,
                'ollama_base': 'http://127.0.0.1:11434',
                'model': 'qwen3:8b',
                'temperature': 0.45,
                'max_tokens': 256,
            }],
        )
    ])
