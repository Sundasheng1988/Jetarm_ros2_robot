from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_voice_agent',
            executable='llm_voice_agent_node',
            name='llm_voice_agent',
            output='screen',
            parameters=[{
                'query_topic': '/speech_query',
                'reply_topic': '/speech_reply',
                'publish_topics': ['/voice_input/input', '/keyboard_input/input'],
                'ollama_base': 'http://127.0.0.1:11434',
                'model': 'qwen:1.8b',
                'temperature': 0.05,
                'max_tokens': 512,
            }],
        )
    ])
