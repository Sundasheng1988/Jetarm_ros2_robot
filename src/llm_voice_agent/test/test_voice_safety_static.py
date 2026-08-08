import ast
import os
import re
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class VoiceSafetyStaticTests(unittest.TestCase):
    def test_task_output_has_one_canonical_topic(self):
        source = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py'
        ).read_text()
        self.assertIn("default_publish_topics = ['/voice_input/input']", source)
        self.assertNotIn("'/keyboard_input/input'", source)

    def test_robot_side_effects_default_off_and_use_guarded_helpers(self):
        source = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py'
        ).read_text()
        self.assertIn(
            "declare_parameter('enable_robot_side_effects', False)", source
        )
        tree = ast.parse(source)

        def publisher_call_count(attribute_name):
            count = 0
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                    continue
                owner = node.func.value
                if (
                    node.func.attr == 'publish'
                    and isinstance(owner, ast.Attribute)
                    and owner.attr == attribute_name
                ):
                    count += 1
            return count

        self.assertEqual(publisher_call_count('face_ctrl_pub'), 1)
        self.assertEqual(publisher_call_count('gesture_pub'), 1)

    def test_task_mode_switch_occurs_after_attention_gate(self):
        source = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py'
        ).read_text()
        gate = source.index('if self.use_wakeword and self.system_active')
        switch = source.index('if self._maybe_switch_mode(norm):')
        self.assertLess(gate, switch)

    def test_cosyvoice3_rate_is_24khz_everywhere(self):
        tts = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'tts_speaker_node.py'
        ).read_text()
        launch = (PACKAGE_ROOT / 'launch' / 'voice_stack.launch.py').read_text()
        config = (PACKAGE_ROOT / 'config' / 'tts_params.yaml').read_text()
        self.assertIn("'cosyvoice_sample_rate', 24000", tts)
        self.assertIn("default_value='24000'", launch)
        self.assertIn('cosyvoice_sample_rate: 24000', config)

    def test_voice_stack_has_no_interrupt_from_llm_remap(self):
        launch = (PACKAGE_ROOT / 'launch' / 'voice_stack.launch.py').read_text()
        self.assertNotIn('interrupt_from_llm', launch)
        self.assertNotRegex(
            launch,
            r"['\"]?/tts/interrupt['\"]?\s*,\s*['\"]?/tts/interrupt_from_llm",
        )

    def test_voice_stack_start_system_active_default_true_and_passed(self):
        launch = (PACKAGE_ROOT / 'launch' / 'voice_stack.launch.py').read_text()
        agent = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py'
        ).read_text()
        # 声明默认 true
        self.assertIn("'start_system_active', default_value='true'", launch)
        self.assertIn("declare_parameter('start_system_active', True)", agent)
        # 同时传给 fast 和 deep 两个 Agent 节点（出现两次 ParameterValue 注入）
        self.assertEqual(
            launch.count("'start_system_active': ParameterValue("), 2
        )

    def test_agent_interrupts_active_glm_stream_concurrently(self):
        source = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py'
        ).read_text()
        # 并发前提：MultiThreadedExecutor + 独立 ReentrantCallbackGroup
        self.assertIn('from rclpy.executors import MultiThreadedExecutor', source)
        self.assertIn('from rclpy.callback_groups import ReentrantCallbackGroup', source)
        self.assertIn('MultiThreadedExecutor(num_threads=2)', source)
        self.assertIn('ReentrantCallbackGroup()', source)
        # 订阅 /tts/interrupt 取消 GLM
        self.assertIn('self._on_tts_interrupt', source)
        self.assertIn("Bool, '/tts/interrupt', self._on_tts_interrupt", source)
        # cancel_check 真正传入 glm_chat
        self.assertIn('cancel_check=cancel_check', source)
        self.assertIn('cancel_event=cancel_event', source)
        # 令牌由无 ROS 控制器管理，并在 request finally 进行 identity release。
        self.assertIn('CancellationTokenSlot()', source)
        self.assertIn('finally:', source)
        self.assertIn('self._llm_cancel_tokens.release(cancel_event)', source)
        self.assertNotIn('cancel_event.clear()', source)
        self.assertNotIn('_llm_cancel_tokens.current.clear()', source)

    def test_playback_only_is_not_an_agent_cancel_subscription(self):
        agent = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py'
        ).read_text()
        tts = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'tts_speaker_node.py'
        ).read_text()

        self.assertIn(
            "Bool, '/tts/interrupt_playback_only', 1", agent
        )
        self.assertIn(
            "Bool, '/tts/interrupt', self._on_tts_interrupt", agent
        )
        self.assertNotIn(
            "Bool, '/tts/interrupt_playback_only', self._on_tts_interrupt",
            agent,
        )
        self.assertNotIn('_suppress_self_interrupt', agent)
        self.assertIn(
            "Bool, '/tts/interrupt', self._on_interrupt", tts
        )
        self.assertIn(
            "Bool, '/tts/interrupt_playback_only', self._on_interrupt", tts
        )

    def test_executor_shutdown_order_is_explicit(self):
        source = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py'
        ).read_text()
        shutdown = source.rindex('executor.shutdown()')
        destroy = source.rindex('node.destroy_node()')
        rclpy_shutdown = source.rindex('rclpy.shutdown()')
        self.assertLess(shutdown, destroy)
        self.assertLess(destroy, rclpy_shutdown)
        self.assertIn('if rclpy.ok():', source[destroy:rclpy_shutdown])

    def test_asr_interrupt_duration_is_configurable_and_launched(self):
        asr = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'speech_dialog_funasr_node.py'
        ).read_text()
        launch = (PACKAGE_ROOT / 'launch' / 'voice_stack.launch.py').read_text()
        self.assertIn(
            'declare_parameter("interrupt_max_utt_ms", 1800)', asr
        )
        self.assertIn(
            'declare_parameter("interrupt_max_sil_ms", 100)', asr
        )
        self.assertNotIn(
            '900 if self._interrupt_listen_only', asr
        )
        self.assertIn(
            "'interrupt_max_utt_ms', default_value='1800'", launch
        )
        self.assertIn(
            "'interrupt_max_utt_ms': LaunchConfiguration('interrupt_max_utt_ms')",
            launch,
        )

    def test_tts_interrupt_uses_generation_queue_and_owned_process_stop(self):
        source = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'tts_speaker_node.py'
        ).read_text()
        self.assertIn('GenerationQueue[str]', source)
        self.assertIn('generation, dropped = self._q.interrupt()', source)
        self.assertIn('stop_owned_process_group(', source)
        self.assertIn('start_new_session=True', source)
        self.assertNotIn('preexec_fn=os.setsid', source)
        self.assertNotIn('except Exception:\n            pass', source)

    def test_request_llm_does_not_fallback_on_user_cancel(self):
        source = (
            PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py'
        ).read_text()
        # 用户取消必须早于 VoiceBackendError 被捕获并 re-raise，避免回退 Ollama。
        idx_cancel = source.find('except VoiceBackendCancelled:')
        idx_err = source.find('except VoiceBackendError as exc:')
        self.assertGreater(idx_cancel, -1)
        self.assertGreater(idx_err, -1)
        self.assertLess(idx_cancel, idx_err)
        self.assertIn('不回退其它后端', source)

    def _voice_text_files(self):
        for dirpath, dirnames, filenames in os.walk(PACKAGE_ROOT):
            dirnames[:] = [d for d in dirnames if d not in (
                '__pycache__', 'build', 'install', 'log')]
            for name in filenames:
                if name.endswith(('.py', '.yaml', '.xml', '.md', '.env', '.sh')):
                    yield Path(dirpath) / name

    def test_no_literal_api_key_in_voice_sources(self):
        # ZHIPUAI_API_KEY 不能被赋予明文字面量；也不得出现 sk- 形态密钥。
        assign_re = re.compile(r'ZHIPUAI_API_KEY\s*=\s*["\'][^"\']{8,}["\']')
        sk_re = re.compile(r'sk-[A-Za-z0-9]{16,}')
        offenders = []
        for path in self._voice_text_files():
            text = path.read_text(errors='replace')
            for m in assign_re.finditer(text):
                offenders.append((str(path), m.group(0)))
            for m in sk_re.finditer(text):
                offenders.append((str(path), m.group(0)))
        self.assertFalse(offenders, f'发现疑似明文 API key：{offenders}')

    def test_cosyvoice_launcher_uses_fp16_and_localhost(self):
        launcher = (PACKAGE_ROOT / 'scripts' / 'cosyvoice_fp16_server.py').read_text()
        self.assertIn('fp16=True', launcher)
        self.assertIn('load_trt=False', launcher)
        self.assertIn('load_vllm=False', launcher)
        # 只绑定本机，不暴露到外网
        self.assertIn('"127.0.0.1"', launcher)
        self.assertIn('拒绝绑定外部网络', launcher)

    def test_secrets_env_is_defensively_ignored(self):
        ignore = (PACKAGE_ROOT / '.gitignore').read_text()
        self.assertIn('secrets.env', ignore)


if __name__ == '__main__':
    unittest.main()
