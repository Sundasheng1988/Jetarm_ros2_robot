from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start only Agent + TTS for text-in and silent WAV development.

    This launch intentionally excludes ASR, parser, grounding, runtime and
    hardware executors. Publish String messages to /speech_query for testing.
    """
    return LaunchDescription([
        DeclareLaunchArgument('glm_model', default_value='glm-4.5-air'),
        DeclareLaunchArgument(
            'cosyvoice_prompt_wav',
            default_value='/home/sundasheng/tools/CosyVoice/asset/zero_shot_prompt.wav',
        ),
        DeclareLaunchArgument(
            'cosyvoice_prompt_text',
            default_value=(
                'You are a helpful assistant.<|endofprompt|>'
                '希望你以后能够做的比我还好呦。'
            ),
        ),
        DeclareLaunchArgument('play_audio', default_value='false'),
        DeclareLaunchArgument('wav_output_dir', default_value='/tmp/jetarm_tts_qa'),
        Node(
            package='llm_voice_agent',
            executable='llm_voice_agent_node',
            name='llm_voice_agent',
            output='screen',
            parameters=[{
                'query_topic': '/speech_query',
                'reply_topic': '/speech_reply',
                'publish_topics': ['/voice_input/input'],
                'enable_robot_side_effects': False,
                'start_system_active': True,
                'use_llm': True,
                'use_wakeword': False,
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
        ),
        Node(
            package='llm_voice_agent',
            executable='tts_speaker_node',
            name='tts_speaker_node',
            output='screen',
            parameters=[{
                'reply_topic': '/speech_reply',
                'tts_backend': 'cosyvoice',
                'tts_fallback_backend': 'piper',
                'cosyvoice_base_url': 'http://127.0.0.1:50000',
                'cosyvoice_mode': 'zero_shot',
                'cosyvoice_sample_rate': 24000,
                'cosyvoice_prompt_wav': LaunchConfiguration('cosyvoice_prompt_wav'),
                'cosyvoice_prompt_text': LaunchConfiguration('cosyvoice_prompt_text'),
                'play_audio': ParameterValue(
                    LaunchConfiguration('play_audio'), value_type=bool
                ),
                'wav_output_dir': LaunchConfiguration('wav_output_dir'),
                'sentence_silence_s': 0.14,
                'dedup_window_s': 1.5,
                'queue_max': 16,
            }],
        ),
    ])
