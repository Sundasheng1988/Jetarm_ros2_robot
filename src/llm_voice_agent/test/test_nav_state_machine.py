#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rebecca 导航状态机的离线测试。

分两部分：

1. 模块级纯函数（``is_stop_movement`` / ``extract_navigation_place``）行为。
2. 源码静态断言：保证“停止说话/安静”不被当作导航取消、导航处理在模式切换
   之前、``_pending_nav`` 的初始化与复位点齐全。
"""

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from llm_voice_agent.llm_voice_agent_node import (  # noqa: E402
    LlmVoiceAgent,
    canonical_navigation_stop_command,
    extract_navigation_place,
    is_resume_navigation,
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


class CanonicalStopCommandTests(unittest.TestCase):
    """Patch 4C.1a：Agent 侧 STOP canonical 语义保持（pause / cancel 分流）。"""

    def test_pause_semantics_map_to_stop_move(self):
        for text in ('停止移动', '停止导航', '暂停导航'):
            with self.subTest(text=text):
                self.assertEqual(
                    canonical_navigation_stop_command(text), '停止移动'
                )

    def test_cancel_semantics_map_to_cancel_navigation(self):
        for text in ('取消导航', '取消移动', '放弃导航'):
            with self.subTest(text=text):
                self.assertEqual(
                    canonical_navigation_stop_command(text), '取消导航'
                )

    def test_non_stop_semantics_return_none(self):
        # “停止说话 / 安静”只中止 LLM/TTS；讨论句不得进入 navigation action。
        for text in ('停止说话', '安静', '停止移动是什么意思？', '取消导航是什么意思？'):
            with self.subTest(text=text):
                self.assertIsNone(
                    canonical_navigation_stop_command(text),
                    f'误产生导航停止命令: {text!r}',
                )


class ResumeNavigationTests(unittest.TestCase):
    """Patch 4C.3：恢复导航整句识别。"""

    def test_resume_phrases(self):
        self.assertTrue(
            is_resume_navigation('恢复导航')
        )
        self.assertTrue(
            is_resume_navigation('继续导航')
        )
        self.assertTrue(
            is_resume_navigation('恢复导航。')
        )

    def test_resume_discussion_not_command(self):
        for text in (
            '恢复导航是什么意思？',
            '继续导航怎么用？',
            '介绍一下恢复导航',
            '停止说话',
        ):
            with self.subTest(text=text):
                self.assertFalse(
                    is_resume_navigation(text)
                )


class ResumeFlowStaticTests(unittest.TestCase):
    """Patch 4C.3：恢复导航两阶段（先确认、后下发）源码接线断言。"""

    def test_resume_entry_sets_pending_and_asks_confirm(self):
        self.assertIn("self.pending_cmd_text = '恢复导航'", SOURCE)
        self.assertIn("self._set_state('nav_wait_confirm')", SOURCE)
        self.assertIn("'要继续刚才暂停的导航吗？'", SOURCE)

    def test_confirm_branch_handles_resume_cmd(self):
        self.assertIn("if cmd_text == '恢复导航':", SOURCE)
        self.assertIn('self._publish_command(cmd_text)', SOURCE)


class NavConfirmTimeoutTests(unittest.TestCase):
    """Patch 4C.4b：导航确认 15s 超时窗口（无 ROS 最小 stub）。"""

    def make_agent(self, **kw):
        ag = SimpleNamespace(
            waiting_confirm=True,
            pending_cmd_text='恢复导航',
            _pending_nav=True,
            _pending_nav_place='',
            _nav_confirm_deadline=0.0,
            nav_confirm_timeout_s=15.0,
            mobile_robot_name='Eric',
            states=[],
            commands=[],
            said=[],
            get_logger=lambda: SimpleNamespace(
                info=lambda msg: None,
                warning=lambda msg: None,
            ),
        )
        ag._set_state = lambda s: ag.states.append(s)
        ag._say = lambda t: ag.said.append(t)
        ag._publish_command = lambda t: ag.commands.append(t)
        ag._reset_nav_confirm = lambda: LlmVoiceAgent._reset_nav_confirm(ag)
        ag._arm_nav_confirm_timeout = (
            lambda: LlmVoiceAgent._arm_nav_confirm_timeout(ag)
        )
        for key, value in kw.items():
            setattr(ag, key, value)
        return ag

    def test_pending_not_yet_expired_keeps_confirm(self):
        # CASE 1：窗口期内 pending 保留，确认仍可用。
        ag = self.make_agent(_nav_confirm_deadline=time.monotonic() + 10.0)
        LlmVoiceAgent._check_nav_confirm_timeout(ag)
        self.assertTrue(ag.waiting_confirm)
        self.assertTrue(ag._pending_nav)

    def test_pending_expired_clears_confirm_and_state(self):
        # CASE 2：超时后 pending 全清，state 收到 nav_confirm_timeout。
        ag = self.make_agent(_nav_confirm_deadline=time.monotonic() - 1.0)
        LlmVoiceAgent._check_nav_confirm_timeout(ag)
        self.assertFalse(ag.waiting_confirm)
        self.assertFalse(ag._pending_nav)
        self.assertEqual(ag.pending_cmd_text, '')
        self.assertEqual(ag._pending_nav_place, '')
        self.assertEqual(ag._nav_confirm_deadline, 0.0)
        self.assertIn('nav_confirm_timeout', ag.states)

    def test_non_nav_waiting_confirm_not_cleared(self):
        # 机械臂 waiting_confirm（_pending_nav=False）不受导航超时影响。
        ag = self.make_agent(
            _pending_nav=False,
            _nav_confirm_deadline=time.monotonic() - 1.0,
        )
        LlmVoiceAgent._check_nav_confirm_timeout(ag)
        self.assertTrue(ag.waiting_confirm)
        self.assertEqual(ag.pending_cmd_text, '恢复导航')

    def test_reset_nav_confirm_clears_deadline(self):
        ag = self.make_agent(_nav_confirm_deadline=123.456)
        LlmVoiceAgent._reset_nav_confirm(ag)
        self.assertEqual(ag._nav_confirm_deadline, 0.0)

    def test_arm_nav_confirm_timeout_sets_future_deadline(self):
        ag = self.make_agent()
        LlmVoiceAgent._arm_nav_confirm_timeout(ag)
        self.assertGreater(ag._nav_confirm_deadline, time.monotonic())

    def test_stale_yes_rejected_by_execution_path_without_timer(self):
        # FIX 1：deadline 已过但 timer callback 尚未运行的竞态下，
        # execution path 自己必须拒绝 stale confirmation。
        # （不调用 _check_nav_confirm_timeout，直接走 _handle_navigation。）
        ag = self.make_agent(
            _nav_confirm_deadline=time.monotonic() - 0.1,
        )
        result = LlmVoiceAgent._handle_navigation(
            ag, '是的'.lower(), '是的'
        )
        self.assertFalse(result)
        # 没有 publish '恢复导航'（或任何命令）
        self.assertEqual(ag.commands, [])
        self.assertFalse(ag.waiting_confirm)
        self.assertFalse(ag._pending_nav)
        self.assertEqual(ag.pending_cmd_text, '')
        self.assertEqual(ag._nav_confirm_deadline, 0.0)
        self.assertIn('nav_confirm_timeout', ag.states)


class UnrelatedTopicFallbackStaticTests(unittest.TestCase):
    """Patch 4C.4b：waiting_confirm 无关输入清除 pending 源码接线。"""

    def test_robot_stop_precedes_waiting_confirm_branch(self):
        # Robot STOP 必须在 waiting_confirm 分支之前
        # （限定 _handle_navigation 内：_on_query 的 STOP 路径
        # 也含 waiting_confirm 分支，不能参与本次比较）。
        nav_handler = SOURCE.index('def _handle_navigation')
        stop_check = SOURCE.index('if is_stop_movement(raw_text):', nav_handler)
        wait_branch = SOURCE.index(
            'if self.waiting_confirm and self._pending_nav:', nav_handler
        )
        self.assertLess(stop_check, wait_branch)

    def test_fallback_clears_pending_and_returns_false(self):
        # fallback 必须清 pending 并 return False 交回正常流程。
        marker = SOURCE.index('无关输入清除导航确认 pending')
        fallback = SOURCE[marker:marker + 400]
        self.assertIn('self._reset_nav_confirm()', fallback)
        self.assertIn('return False', fallback)


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

    def test_stop_paths_publish_canonical_command(self):
        # Patch 4C.1a：两个 Robot STOP 路径（_on_query 顶层与
        # _handle_navigation）都必须经 canonical_navigation_stop_command
        # 分流 pause/cancel，并最终下发 stop_command，不得再固定
        # publish '停止移动'；且不下发 /cmd_vel。
        self.assertIn('canonical_navigation_stop_command', SOURCE)
        self.assertNotIn("self._publish_command('停止移动')", SOURCE)
        # 两个 stop 路径都实际调用 canonical 分流。
        on_query = SOURCE.index('def _on_query')
        nav_handler = SOURCE.index('def _handle_navigation')
        self.assertIn(
            'canonical_navigation_stop_command(raw_text)',
            SOURCE[on_query:nav_handler],
        )
        self.assertIn(
            'canonical_navigation_stop_command(raw_text)',
            SOURCE[nav_handler:],
        )
        # 最终机器命令必须走 stop_command 变量发布。
        self.assertIn('self._publish_command(stop_command)', SOURCE)
        self.assertNotIn("'/cmd_vel'", SOURCE)

    def test_nav_command_uses_single_input_topic(self):
        # 导航自然语言只能走 /voice_input/input，不引入第二个输入话题。
        self.assertIn("default_publish_topics = ['/voice_input/input']", SOURCE)


if __name__ == '__main__':
    unittest.main()
