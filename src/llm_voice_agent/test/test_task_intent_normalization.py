#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patch 3 Task Intent Normalization 离线测试。

三部分：

1. 纯函数：``_strip_navigation_lead`` / ``is_unsupported_manip_request``
   / ``extract_navigation_place`` 前缀归一化后的行为。
2. stub agent 绑定真实 ``_handle_navigation``：覆盖回归清单
   （Eric 前缀、Rebecca 角色边界、复合任务降级、纯拿取拦截、
   确认后下发、既有导航行为不变）。
3. 源码静态断言：能力表作用域限定在 chat/navigation 链路。
"""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from llm_voice_agent.llm_voice_agent_node import (  # noqa: E402
    VOICE_CHAT_CAPABILITIES,
    LlmVoiceAgent,
    _strip_navigation_lead,
    extract_navigation_place,
    is_unsupported_manip_request,
)

SOURCE = (PACKAGE_ROOT / 'llm_voice_agent' / 'llm_voice_agent_node.py').read_text()


# ---------------------------------------------------------------------------
# stub 基础设施：不依赖 ROS graph，把真实方法绑定到 SimpleNamespace 上。
# ---------------------------------------------------------------------------
class _StubLogger:
    def __init__(self):
        self.records = []

    def info(self, msg):
        self.records.append(msg)


def make_stub_agent(mode='chat'):
    logger = _StubLogger()
    ag = SimpleNamespace(
        mode=mode,
        mobile_robot_name='Eric',
        waiting_confirm=False,
        pending_cmd_text='',
        _pending_nav=False,
        _pending_nav_place='',
        said=[],
        commands=[],
        states=[],
        get_logger=lambda: logger,
    )
    ag._say = lambda t: ag.said.append(t)
    ag._publish_command = lambda t: ag.commands.append(t)
    ag._set_state = lambda s: ag.states.append(s)
    # 复位逻辑用真实实现，验证 slot 状态被正确清理。
    ag._reset_nav_confirm = lambda: LlmVoiceAgent._reset_nav_confirm(ag)
    return ag


def handle(ag, text):
    return LlmVoiceAgent._handle_navigation(ag, text.lower(), text)


# ---------------------------------------------------------------------------
# 1) 前缀归一化：Eric 系列剥离，Rebecca 系列保留
# ---------------------------------------------------------------------------
class LeadStripTests(unittest.TestCase):
    def test_eric_prefixes_stripped(self):
        cases = {
            '让Eric去餐厅': '去餐厅',
            'Eric去餐厅': '去餐厅',
            '麻烦Eric去客厅': '去客厅',
            '让小车去餐厅': '去餐厅',
            '让机器人去客厅': '去客厅',
            '请去餐厅': '去餐厅',
            '麻烦去客厅': '去客厅',
            '帮我拿杯子': '拿杯子',
        }
        for text, want in cases.items():
            with self.subTest(text=text):
                self.assertEqual(_strip_navigation_lead(text), want)

    def test_rebecca_prefix_preserved(self):
        # Rebecca 是 PC 侧语音助手，不是实体导航执行主体；
        # 其前缀绝不能被剥离成“去餐厅”。
        for text in (
            '让Rebecca去餐厅',
            'Rebecca去餐厅',
            '让rebecca去餐厅',
            'rebecca去餐厅',
            '让瑞贝卡去餐厅',
            '瑞贝卡去餐厅',
        ):
            with self.subTest(text=text):
                self.assertEqual(_strip_navigation_lead(text), text)


# ---------------------------------------------------------------------------
# 2) 纯拿取动词识别
# ---------------------------------------------------------------------------
class ManipRequestTests(unittest.TestCase):
    def test_manip_requests_detected(self):
        for text in ('帮我拿杯子', '帮我拿个东西过来', '抓一下那个杯子', '帮我搬个凳子'):
            with self.subTest(text=text):
                self.assertTrue(is_unsupported_manip_request(text))

    def test_non_manip_not_detected(self):
        for text in (
            '去餐厅',
            '取消导航',
            '导航到客厅',
            '今天天气怎么样',
            '讲个笑话',
        ):
            with self.subTest(text=text):
                self.assertFalse(is_unsupported_manip_request(text))


# ---------------------------------------------------------------------------
# 3) 能力表作用域
# ---------------------------------------------------------------------------
class CapabilityScopeTests(unittest.TestCase):
    def test_voice_chat_capabilities_pinned(self):
        self.assertEqual(
            VOICE_CHAT_CAPABILITIES, {'navigation': True, 'manipulation': False}
        )

    def test_capability_table_is_scoped_not_global(self):
        # 必须是 chat/navigation 链路私有能力表，不得引入全局
        # ROBOT_CAPABILITIES（task mode 的机械臂能力不受影响）。
        self.assertNotIn('ROBOT_CAPABILITIES', SOURCE)
        # 纯拿取拦截必须带 chat mode 门控。
        self.assertIn(
            "self.mode == 'chat'\n"
            "                and not VOICE_CHAT_CAPABILITIES['manipulation']",
            SOURCE,
        )


# ---------------------------------------------------------------------------
# 4) _handle_navigation 回归清单（stub 绑定真实方法）
# ---------------------------------------------------------------------------
class HandleNavigationIntentTests(unittest.TestCase):
    def test_eric_lead_navigation_enter_confirm_wait(self):
        # “让Eric去餐厅” → place=餐厅，进入 nav_wait_confirm
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '让Eric去餐厅'))
        self.assertEqual(ag.pending_cmd_text, '导航到餐厅')
        self.assertEqual(ag._pending_nav_place, '餐厅')
        self.assertTrue(ag.waiting_confirm and ag._pending_nav)
        self.assertIn('nav_wait_confirm', ag.states)
        # 未确认前不得下发 command
        self.assertEqual(ag.commands, [])

    def test_polite_eric_lead_living_room(self):
        # “麻烦Eric去客厅” → place=客厅
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '麻烦Eric去客厅'))
        self.assertEqual(ag._pending_nav_place, '客厅')
        self.assertEqual(ag.pending_cmd_text, '导航到客厅')
        self.assertIn('nav_wait_confirm', ag.states)

    def test_robot_alias_lead_navigation(self):
        # “让机器人去餐厅” → place=餐厅
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '让机器人去餐厅'))
        self.assertEqual(ag._pending_nav_place, '餐厅')
        self.assertIn('nav_wait_confirm', ag.states)

    def test_rebecca_lead_no_eric_navigation(self):
        # “让Rebecca去餐厅” / “Rebecca去餐厅” 不得触发 Eric 导航：
        # 不归一化、不进入确认、不下发 command，交给 chat LLM。
        for text in ('让Rebecca去餐厅', 'Rebecca去餐厅', '让瑞贝卡去餐厅'):
            with self.subTest(text=text):
                ag = make_stub_agent()
                self.assertIsNone(extract_navigation_place(text))
                self.assertFalse(handle(ag, text))
                self.assertEqual(ag.commands, [])
                self.assertFalse(ag.waiting_confirm)

    def test_composite_request_downgrades_to_navigation_only(self):
        # “帮我去餐厅拿个东西” → 降级为仅导航协商
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '帮我去餐厅拿个东西'))
        self.assertEqual(ag._pending_nav_place, '餐厅')
        self.assertEqual(ag.pending_cmd_text, '导航到餐厅')
        self.assertTrue(ag.waiting_confirm and ag._pending_nav)
        self.assertIn('nav_wait_confirm', ag.states)
        self.assertEqual(ag.commands, [])
        self.assertTrue(
            any('餐厅' in s and ('拿' in s or '取' in s) for s in ag.said),
            f'协商话术应说明拿取限制与导航提议: {ag.said}',
        )

    def test_pure_manip_request_no_navigation_no_llm_claim(self):
        # “帮我拿杯子” → 不产生 navigation command；确定性回复拦截，
        # return True 短路后续 chat LLM（调用点见 _on_query）。
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '帮我拿杯子'))
        self.assertEqual(ag.commands, [])
        self.assertFalse(ag.waiting_confirm)
        self.assertEqual(ag.said, ['我现在还不能直接帮你完成拿取任务。'])
        self.assertIn('chat_manip_unsupported', ag.states)

    def test_plain_navigation_unchanged(self):
        # “导航到餐厅” → 原行为不变（Patch 4A：询问话术不含控制 token）
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '导航到餐厅'))
        self.assertEqual(ag._pending_nav_place, '餐厅')
        self.assertEqual(ag.pending_cmd_text, '导航到餐厅')
        self.assertEqual(ag.said, ['要让Eric去餐厅吗？'])
        self.assertEqual(ag.states[-1], 'nav_wait_confirm')
        self.assertEqual(ag.commands, [])

    def test_cancel_navigation_unchanged(self):
        # “取消导航” → 原行为不变：立即下发规范短语
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '取消导航'))
        self.assertEqual(ag.commands, ['停止移动'])
        self.assertEqual(ag.said, ['好的，正在停止移动。'])
        self.assertEqual(ag.states[-1], 'nav_stop')

    def test_eric_lead_confirm_dispatches_navigation(self):
        # 端到端：让Eric去餐厅 → “可以”确认 → 下发一次导航 command
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '让Eric去餐厅'))
        self.assertTrue(handle(ag, '可以'))
        self.assertEqual(ag.commands, ['导航到餐厅'])
        self.assertEqual(ag.states[-1], 'nav_confirmed')
        self.assertFalse(ag.waiting_confirm)
        self.assertIn('我让Eric去餐厅', ''.join(ag.said))

    def test_task_mode_pure_manip_not_intercepted(self):
        # task mode 的机械臂链路不受 chat 能力表影响：
        # “帮我拿杯子”在 task mode 下不归导航处理。
        ag = make_stub_agent(mode='task')
        self.assertFalse(handle(ag, '帮我拿杯子'))
        self.assertEqual(ag.commands, [])
        self.assertEqual(ag.said, [])


if __name__ == '__main__':
    unittest.main()
