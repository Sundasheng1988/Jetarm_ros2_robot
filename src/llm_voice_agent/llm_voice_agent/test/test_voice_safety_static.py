import ast
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


if __name__ == '__main__':
    unittest.main()
