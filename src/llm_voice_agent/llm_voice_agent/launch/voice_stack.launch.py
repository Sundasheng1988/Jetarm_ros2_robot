# voice_stack.launch
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # ===== 唤醒词 =====
    wake_yaml = os.path.join(
        get_package_share_directory('llm_voice_agent'),
        'config', 'wake_words.yaml'
    )
    # ===== 通用/选择参数 =====
    which_model = DeclareLaunchArgument('which_model', default_value='fast')  # fast | deep
    use_llm     = DeclareLaunchArgument('use_llm',     default_value='true')
    llm_base    = DeclareLaunchArgument('llm_base',    default_value='http://127.0.0.1:11434')
    llm_timeout = DeclareLaunchArgument('llm_timeout_s', default_value='60.0')
    llm_connect_timeout = DeclareLaunchArgument('llm_connect_timeout_s', default_value='8.0')
    llm_backend = DeclareLaunchArgument('llm_backend', default_value='glm')
    llm_fallback_backend = DeclareLaunchArgument('llm_fallback_backend', default_value='ollama')
    glm_api_base = DeclareLaunchArgument(
        'glm_api_base', default_value='https://open.bigmodel.cn/api/paas/v4'
    )
    glm_api_key_env = DeclareLaunchArgument('glm_api_key_env', default_value='ZHIPUAI_API_KEY')
    glm_model = DeclareLaunchArgument('glm_model', default_value='glm-4.5-air')
    glm_thinking = DeclareLaunchArgument('glm_thinking', default_value='false')
    llm_stream = DeclareLaunchArgument('llm_stream', default_value='true')
    stream_to_tts = DeclareLaunchArgument('stream_to_tts', default_value='true')
    enable_robot_side_effects = DeclareLaunchArgument(
        'enable_robot_side_effects', default_value='false'
    )
    
    use_wakeword_arg    = DeclareLaunchArgument('use_wakeword',    default_value='true')
    wake_window_s_arg   = DeclareLaunchArgument('wake_window_s',   default_value='20.0')
    wake_cooldown_s_arg = DeclareLaunchArgument('wake_cooldown_s', default_value='2.0')
    
    wake_fallback_ms_arg = DeclareLaunchArgument('wake_fallback_ms', default_value='500')
    wake_fallback_ms = LaunchConfiguration('wake_fallback_ms')

    use_wakeword    = LaunchConfiguration('use_wakeword')
    wake_window_s   = LaunchConfiguration('wake_window_s')
    wake_cooldown_s = LaunchConfiguration('wake_cooldown_s')

    query_topic = DeclareLaunchArgument('query_topic', default_value='/speech_query')
    reply_topic = DeclareLaunchArgument('reply_topic', default_value='/speech_reply')

    # ===== ASR(FunASR) 参数 =====
    base_dir   = DeclareLaunchArgument('base_dir',   default_value='/home/sundasheng/ros2_ws/models/funasr')
    model_path = DeclareLaunchArgument('model_path', default_value='iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch')
    vad_path   = DeclareLaunchArgument('vad_path',   default_value='iic/speech_fsmn_vad_zh-cn-16k-common-pytorch')
    punc_path  = DeclareLaunchArgument('punc_path',  default_value='iic/punc_ct-transformer_cn-en-common-vocab471067-large')

    device        = DeclareLaunchArgument('device',        default_value='cpu')
    device_index  = DeclareLaunchArgument('device_index',  default_value='-1')
    sample_rate   = DeclareLaunchArgument('sample_rate',   default_value='16000')
    frame_ms      = DeclareLaunchArgument('frame_ms',      default_value='20')
    vad_lv        = DeclareLaunchArgument('vad_aggressiveness', default_value='3')
    min_utt_ms    = DeclareLaunchArgument('min_utt_ms',    default_value='900')
    max_sil_ms    = DeclareLaunchArgument('max_sil_ms',    default_value='500')
    max_utt_ms    = DeclareLaunchArgument('max_utt_ms',    default_value='6000')
    min_text_len  = DeclareLaunchArgument('min_text_len',  default_value='5')
    min_avg_conf  = DeclareLaunchArgument('min_avg_conf',  default_value='0.75')
    disable_update= DeclareLaunchArgument('disable_update',default_value='true')

    # ===== Agent 稳定参数 =====
    strict_intent        = DeclareLaunchArgument('strict_intent',        default_value='true')
    llm_hint_enabled     = DeclareLaunchArgument('llm_hint_enabled',     default_value='true')
    min_reply_interval_s = DeclareLaunchArgument('min_reply_interval_s', default_value='1.2')
    reply_dedup_window_s = DeclareLaunchArgument('reply_dedup_window_s', default_value='2.0')
    input_dedup_window_s = DeclareLaunchArgument('dedup_window_s',       default_value='3.0')
    num_ctx              = DeclareLaunchArgument('num_ctx',              default_value='2048')
    temperature_arg      = DeclareLaunchArgument('temperature',          default_value='0.45')
    max_tokens_arg       = DeclareLaunchArgument('max_tokens',           default_value='256')

    # ===== TTS 参数（更稳 & 支持打断清队列）=====
    piper_bin   = DeclareLaunchArgument('piper_bin',   default_value='/home/sundasheng/.local/bin/piper')
    piper_model = DeclareLaunchArgument('piper_model', default_value='/home/sundasheng/tools/piper/models/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx')
    tts_player  = DeclareLaunchArgument('tts_player',  default_value='ffplay -autoexit -nodisp -loglevel quiet')
    # ↓ 默认切到 pipe（你日志里实际用的是 pipe，延迟更低）
    tts_emit_mode = DeclareLaunchArgument('tts_emit_mode', default_value='pipe')  # wav | pipe
    length_scale= DeclareLaunchArgument('length_scale',default_value='1.04')
    noise_scale = DeclareLaunchArgument('noise_scale', default_value='0.65')
    noise_w     = DeclareLaunchArgument('noise_w',     default_value='0.90')
    # ↓ 句间静音从 80ms → 60ms（打断体感更利落）
    sent_sil_ms = DeclareLaunchArgument('sentence_silence_ms', default_value='140')
    max_sent_len= DeclareLaunchArgument('max_sentence_len', default_value='220')
    tts_dedup_s = DeclareLaunchArgument('tts_dedup_window_s', default_value='1.5')
    # ↓ 新增两个参数：打断即清队列 & 限制队列长度
    tts_drop_on_interrupt = DeclareLaunchArgument('tts_drop_queue_on_interrupt', default_value='true')
    tts_queue_max_arg     = DeclareLaunchArgument('tts_queue_max', default_value='16')

    # ===== CosyVoice / silent QA =====
    tts_backend = DeclareLaunchArgument('tts_backend', default_value='cosyvoice')
    tts_fallback_backend = DeclareLaunchArgument('tts_fallback_backend', default_value='piper')
    cosyvoice_base_url = DeclareLaunchArgument(
        'cosyvoice_base_url', default_value='http://127.0.0.1:50000'
    )
    cosyvoice_mode = DeclareLaunchArgument('cosyvoice_mode', default_value='zero_shot')
    cosyvoice_sample_rate = DeclareLaunchArgument(
        'cosyvoice_sample_rate', default_value='24000'
    )
    cosyvoice_speaker_id = DeclareLaunchArgument('cosyvoice_speaker_id', default_value='')
    cosyvoice_prompt_wav = DeclareLaunchArgument(
        'cosyvoice_prompt_wav',
        default_value='/home/sundasheng/tools/CosyVoice/asset/zero_shot_prompt.wav'
    )
    cosyvoice_prompt_text = DeclareLaunchArgument(
        'cosyvoice_prompt_text',
        default_value='You are a helpful assistant.<|endofprompt|>希望你以后能够做的比我还好呦。'
    )
    cosyvoice_instruct_text = DeclareLaunchArgument(
        'cosyvoice_instruct_text',
        default_value='You are a helpful assistant. 请用自然、清晰、亲切的普通话播报。<|endofprompt|>'
    )
    cosyvoice_connect_timeout = DeclareLaunchArgument(
        'cosyvoice_connect_timeout_s', default_value='5.0'
    )
    cosyvoice_timeout = DeclareLaunchArgument('cosyvoice_timeout_s', default_value='90.0')
    play_audio = DeclareLaunchArgument('play_audio', default_value='true')
    wav_output_dir = DeclareLaunchArgument('wav_output_dir', default_value='')
    sentence_silence_s = DeclareLaunchArgument('sentence_silence_s', default_value='0.14')

    # ----- 节点定义 -----

    # ASR
    asr_node = Node(
        package='llm_voice_agent',
        executable='speech_dialog_funasr_node',
        name='speech_dialog_funasr_node',
        output='screen',
        parameters=[
            wake_yaml,   # ✅ 加载 YAML
            {
                'base_dir': LaunchConfiguration('base_dir'),
                'model_path': LaunchConfiguration('model_path'),
                'vad_path': LaunchConfiguration('vad_path'),
                'punc_path': LaunchConfiguration('punc_path'),
                'disable_update': LaunchConfiguration('disable_update'),

                'device': LaunchConfiguration('device'),
                'device_index': LaunchConfiguration('device_index'),
                'sample_rate': LaunchConfiguration('sample_rate'),
                'frame_ms': LaunchConfiguration('frame_ms'),
                'vad_aggressiveness': LaunchConfiguration('vad_aggressiveness'),
                'min_utt_ms': LaunchConfiguration('min_utt_ms'),
                'max_sil_ms': LaunchConfiguration('max_sil_ms'),
                'max_utt_ms': LaunchConfiguration('max_utt_ms'),

                'min_text_len': LaunchConfiguration('min_text_len'),
                'min_avg_conf': LaunchConfiguration('min_avg_conf'),

                'topic_out':  LaunchConfiguration('query_topic'),
                'reply_topic': LaunchConfiguration('reply_topic'),

                'respect_tts_gate': True,
                'tts_gate_release_ms': 80,
            
                # —— 唤醒相关 —— 
                'use_wakeword': LaunchConfiguration('use_wakeword'),
                'wake_window_s': LaunchConfiguration('wake_window_s'),
                'wake_cooldown_s': LaunchConfiguration('wake_cooldown_s'),

            }
        ]
    )

    # LLM（快：qwen2.5:7b-instruct）
    node_fast = Node(
        package='llm_voice_agent',
        executable='llm_voice_agent_node',
        name='llm_voice_agent',
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('which_model'), '" == "fast"'])),
        # ✅ 只加这一行：把 LLM 发往 /tts/interrupt 的“自动打断”改道
        remappings=[('/tts/interrupt', '/tts/interrupt_from_llm')],
        parameters=[
            wake_yaml,   # ★ 新增：把唤醒词 YAML 文件注入
            {
                'use_llm': LaunchConfiguration('use_llm'),
                'llm_backend': LaunchConfiguration('llm_backend'),
                'llm_fallback_backend': LaunchConfiguration('llm_fallback_backend'),
                'glm_api_base': LaunchConfiguration('glm_api_base'),
                'glm_api_key_env': LaunchConfiguration('glm_api_key_env'),
                'glm_model': LaunchConfiguration('glm_model'),
                'glm_thinking': ParameterValue(
                    LaunchConfiguration('glm_thinking'), value_type=bool
                ),
                'llm_stream': ParameterValue(
                    LaunchConfiguration('llm_stream'), value_type=bool
                ),
                'stream_to_tts': ParameterValue(
                    LaunchConfiguration('stream_to_tts'), value_type=bool
                ),
                'ollama_base': LaunchConfiguration('llm_base'),
                'model': 'qwen2.5:7b-instruct',
                'temperature': LaunchConfiguration('temperature'),
                'max_tokens': LaunchConfiguration('max_tokens'),
                'llm_connect_timeout_s': LaunchConfiguration('llm_connect_timeout_s'),
                'llm_timeout_s': LaunchConfiguration('llm_timeout_s'),
                'num_ctx': LaunchConfiguration('num_ctx'),

                'query_topic': LaunchConfiguration('query_topic'),
                'reply_topic': LaunchConfiguration('reply_topic'),
                'publish_topics': ['/voice_input/input'],
                'enable_robot_side_effects': ParameterValue(
                    LaunchConfiguration('enable_robot_side_effects'), value_type=bool
                ),

                'strict_intent': LaunchConfiguration('strict_intent'),
                'llm_hint_enabled': LaunchConfiguration('llm_hint_enabled'),
                'min_reply_interval_s': LaunchConfiguration('min_reply_interval_s'),
                'reply_dedup_window_s': LaunchConfiguration('reply_dedup_window_s'),
                'dedup_window_s': LaunchConfiguration('dedup_window_s'),
            
                'use_wakeword': LaunchConfiguration('use_wakeword'),
                'wake_window_s': LaunchConfiguration('wake_window_s'),
                'wake_cooldown_s': LaunchConfiguration('wake_cooldown_s'),
                'wake_fallback_ms': LaunchConfiguration('wake_fallback_ms'),
               
                'chat_system_prompt': (
                    '你叫 Rebecca，是自然、可靠的中文机器人助手。直接回答用户，'
                    '默认一到三句口语化中文，约40到120字；简单问题更短。'
                    '不要输出内部思考、元叙述或Markdown表格。'
                )
            }]
    )

    # LLM（深：qwen3:8b）
    node_deep = Node(
        package='llm_voice_agent',
        executable='llm_voice_agent_node',
        name='llm_voice_agent',
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('which_model'), '" == "deep"'])),
        # ✅ 只加这一行：把 LLM 发往 /tts/interrupt 的“自动打断”改道
        remappings=[('/tts/interrupt', '/tts/interrupt_from_llm')],
        parameters=[
            wake_yaml,
            {
            'use_llm': LaunchConfiguration('use_llm'),
            'llm_backend': LaunchConfiguration('llm_backend'),
            'llm_fallback_backend': LaunchConfiguration('llm_fallback_backend'),
            'glm_api_base': LaunchConfiguration('glm_api_base'),
            'glm_api_key_env': LaunchConfiguration('glm_api_key_env'),
            'glm_model': LaunchConfiguration('glm_model'),
            'glm_thinking': ParameterValue(
                LaunchConfiguration('glm_thinking'), value_type=bool
            ),
            'llm_stream': ParameterValue(
                LaunchConfiguration('llm_stream'), value_type=bool
            ),
            'stream_to_tts': ParameterValue(
                LaunchConfiguration('stream_to_tts'), value_type=bool
            ),
            'ollama_base': LaunchConfiguration('llm_base'),
            'model': 'qwen3:8b',
            'temperature': LaunchConfiguration('temperature'),
            'max_tokens': LaunchConfiguration('max_tokens'),
            'llm_connect_timeout_s': LaunchConfiguration('llm_connect_timeout_s'),
            'llm_timeout_s': LaunchConfiguration('llm_timeout_s'),
            'num_ctx': LaunchConfiguration('num_ctx'),

            'query_topic': LaunchConfiguration('query_topic'),
            'reply_topic': LaunchConfiguration('reply_topic'),
            'publish_topics': ['/voice_input/input'],
            'enable_robot_side_effects': ParameterValue(
                LaunchConfiguration('enable_robot_side_effects'), value_type=bool
            ),

            'strict_intent': LaunchConfiguration('strict_intent'),
            'llm_hint_enabled': LaunchConfiguration('llm_hint_enabled'),
            'min_reply_interval_s': LaunchConfiguration('min_reply_interval_s'),
            'reply_dedup_window_s': LaunchConfiguration('reply_dedup_window_s'),
            'dedup_window_s': LaunchConfiguration('dedup_window_s'),
            
            'use_wakeword': use_wakeword,
            'wake_window_s': wake_window_s,
            'wake_cooldown_s': wake_cooldown_s,

            'chat_system_prompt': (
                '你叫 Rebecca，是自然、可靠的中文机器人助手。直接回答用户，'
                '默认一到三句口语化中文，约40到120字；简单问题更短。'
                '不要输出内部思考、元叙述或Markdown表格。'
            )
        }]
    )

    # TTS
    tts_node = Node(
        package='llm_voice_agent',
        executable='tts_speaker_node',
        name='tts_speaker_node',
        output='screen',
        parameters=[{
            'reply_topic': LaunchConfiguration('reply_topic'),

            'tts_backend': LaunchConfiguration('tts_backend'),
            'tts_fallback_backend': LaunchConfiguration('tts_fallback_backend'),
            'cosyvoice_base_url': LaunchConfiguration('cosyvoice_base_url'),
            'cosyvoice_mode': LaunchConfiguration('cosyvoice_mode'),
            'cosyvoice_sample_rate': LaunchConfiguration('cosyvoice_sample_rate'),
            'cosyvoice_speaker_id': LaunchConfiguration('cosyvoice_speaker_id'),
            'cosyvoice_prompt_wav': LaunchConfiguration('cosyvoice_prompt_wav'),
            'cosyvoice_prompt_text': LaunchConfiguration('cosyvoice_prompt_text'),
            'cosyvoice_instruct_text': LaunchConfiguration('cosyvoice_instruct_text'),
            'cosyvoice_connect_timeout_s': LaunchConfiguration('cosyvoice_connect_timeout_s'),
            'cosyvoice_timeout_s': LaunchConfiguration('cosyvoice_timeout_s'),
            'play_audio': ParameterValue(
                LaunchConfiguration('play_audio'), value_type=bool
            ),
            'wav_output_dir': LaunchConfiguration('wav_output_dir'),

            'piper_bin':  LaunchConfiguration('piper_bin'),
            'model_path': LaunchConfiguration('piper_model'),
            'length_scale': LaunchConfiguration('length_scale'),
            'noise_scale':  LaunchConfiguration('noise_scale'),
            'noise_w':      LaunchConfiguration('noise_w'),
            'sentence_silence_ms': LaunchConfiguration('sentence_silence_ms'),
            'sentence_silence_s': LaunchConfiguration('sentence_silence_s'),
            'player': LaunchConfiguration('tts_player'),
            'emit_mode': LaunchConfiguration('tts_emit_mode'),
            'max_sentence_len': LaunchConfiguration('max_sentence_len'),
            'dedup_window_s': LaunchConfiguration('tts_dedup_window_s'),

            # ★ 新增：打断即清队列 + 限制队列深度
            'drop_queue_on_interrupt': LaunchConfiguration('tts_drop_queue_on_interrupt'),
            'queue_max': LaunchConfiguration('tts_queue_max'),
        }]
    )

    # 执行完成播报器
    done_sayer = Node(
        package='llm_voice_agent',
        executable='executor_done_sayer',
        name='executor_done_sayer',
        output='screen',
        parameters=[{
            'done_topic': '/executor/done',
            'reply_topic': LaunchConfiguration('reply_topic')
        }]
    )

    return LaunchDescription([
        # 通用
        which_model, use_llm, llm_base, llm_timeout, llm_connect_timeout,
        llm_backend, llm_fallback_backend,
        glm_api_base, glm_api_key_env, glm_model, glm_thinking,
        llm_stream, stream_to_tts, enable_robot_side_effects,
        use_wakeword_arg, wake_window_s_arg, wake_cooldown_s_arg, wake_fallback_ms_arg,
        query_topic, reply_topic,

        # ASR
        base_dir, model_path, vad_path, punc_path,
        device, device_index, sample_rate, frame_ms, vad_lv,
        min_utt_ms, max_sil_ms, max_utt_ms, min_text_len, min_avg_conf, disable_update,

        # Agent
        strict_intent, llm_hint_enabled,
        min_reply_interval_s, reply_dedup_window_s, input_dedup_window_s,
        num_ctx, temperature_arg, max_tokens_arg,

        # TTS
        piper_bin, piper_model, tts_player, tts_emit_mode, length_scale, noise_scale, noise_w,
        sent_sil_ms, sentence_silence_s, max_sent_len, tts_dedup_s,
        tts_drop_on_interrupt, tts_queue_max_arg,
        tts_backend, tts_fallback_backend, cosyvoice_base_url, cosyvoice_mode,
        cosyvoice_sample_rate, cosyvoice_speaker_id,
        cosyvoice_prompt_wav, cosyvoice_prompt_text, cosyvoice_instruct_text,
        cosyvoice_connect_timeout, cosyvoice_timeout, play_audio, wav_output_dir,

        # 节点
        asr_node, node_fast, node_deep, tts_node, done_sayer
    ])
