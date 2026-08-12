#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rebecca 导航状态机的离线测试。

分两部分：

1. 模块级纯函数（``is_stop_movement`` / ``extract_navigation_place``）行为。
2. 源码静态断言：保证“停止说话/安静”不被当作导航取消、导航处理在模式切换
   之前、``_pending_nav`` 的初始化与复位点齐全。
"""

import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from llm_voice_agent.llm_voice_agent_node import (  # noqa: E402
    extract_navigation_place,
    is_stop_movement,
)

SOURCE = (PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py').read_text()


class NavHelperTests(unittest.TestCase):
    def test_is_stop_movement(self):
        for text in ('停止移动', '停止移动吧', '取消导航', '别走了', '停下移动'):
            with self.subTest(text=text):
                self.assertTrue(is_stop_movement(text), f'应识别为停止移动: {text}')

    def test_speech_not_treated_as_stop(self):
        # “停止说话 / 安静”只静音对话，绝不能取消导航。
        for text in ('停止说话', '安静', '别说话了', '停止', ''):
            with self.subTest(text=text):
                self.assertFalse(is_stop_movement(text), f'误判为停止移动: {text!r}')

    def test_extract_navigation_place(self):
        self.assertEqual(extract_navigation_place('去客厅点1'), '客厅点1')
        self.assertEqual(extract_navigation_place('导航到阳台'), '阳台')
        self.assertEqual(extract_navigation_place('前往厨房。'), '厨房')

    def test_extract_rejects_empty_filler_and_non_lead(self):
        for text in ('去', '去那边', '去这里', '把杯子移动到右边', '介绍一下移动机器人'):
            with self.subTest(text=text):
                self.assertIsNone(
                    extract_navigation_place(text), f'不应提取出地点: {text!r}'
                )


class NavStateMachineStaticTests(unittest.TestCase):
    def test_navigation_handled_before_mode_switch(self):
        # _on_query 中必须先 _handle_navigation 再 _maybe_switch_mode，
        # 否则“去客厅”可能被 start_keywords 误吃或模式切换抢先。
        call_nav = SOURCE.index('self._handle_navigation(norm, raw_text)')
        call_switch = SOURCE.index('self._maybe_switch_mode(norm)')
        self.assertLess(call_nav, call_switch)

    def test_speech_guard_at_top_of_nav_handler(self):
        # “停止说话/安静”守卫必须在 is_stop_movement 之前。
        guard = SOURCE.index("'说话' in norm or '安静' in norm")
        stop_check = SOURCE.index('if is_stop_movement(raw_text)')
        self.assertLess(guard, stop_check)

    def test_pending_nav_initialized_and_reset(self):
        # 初始化为 False。
        self.assertIn('self._pending_nav = False', SOURCE)
        nav_handler = SOURCE.index('def _handle_navigation')
        reset_fn = SOURCE.index('def _reset_nav_confirm')
        # 导航确认（确认 / 取消）必须复位 _pending_nav；复位函数必须存在且在
        # 导航处理函数之后定义。
        self.assertGreater(reset_fn, nav_handler)
        self.assertIn('self._pending_nav = False', SOURCE[reset_fn:reset_fn + 200])

    def test_stop_movement_publishes_canonical_phrase(self):
        # 停止移动必须下发规范短语“停止移动”（由 Parser 转 cancel_navigation），
        # 且不下发 /cmd_vel。
        self.assertIn("self._publish_command('停止移动')", SOURCE)
        self.assertNotIn("'/cmd_vel'", SOURCE)

    def test_nav_command_uses_single_input_topic(self):
        # 导航自然语言只能走 /voice_input/input，不引入第二个输入话题。
        self.assertIn("default_publish_topics = ['/voice_input/input']", SOURCE)


if __name__ == '__main__':
    unittest.main()
