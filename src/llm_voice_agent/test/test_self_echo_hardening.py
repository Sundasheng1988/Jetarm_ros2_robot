#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patch 4A：Self-Echo Control Hardening 离线回归测试。

覆盖：

1. ASR TTS-time 控制门（强确认窄集合 / 取消集合 / TTS 打断集合）
   全部整句精确匹配，回声碎片不得穿透。
2. Agent 导航取消改严格整句集合；task 取消语义保持 Patch 4A 前
   行为（is_task_cancel_reply 分离）。
3. Agent 事件顺序：Robot STOP command-first；nav cancel/confirm
   先关 pending state 再播报。
4. nav_wait_confirm 期间 Rebecca 话术不含确认/取消 token。
"""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from llm_voice_agent.llm_voice_agent_node import (  # noqa: E402
    LlmVoiceAgent,
    VOICE_CHAT_CAPABILITIES,  # noqa: F401  (确保 Patch 3 基线仍在)
    is_nav_cancel_reply,
    is_nav_confirm_reply,
    is_task_cancel_reply,
)

# ASR 模块导入较重（funasr），但只 import 不加载模型。
from llm_voice_agent.speech_dialog_funasr_node import (  # noqa: E402
    VOICE_STOP_PHRASES,
    VOICE_TTS_NAV_CANCEL_REPLIES,
    VOICE_TTS_NAV_CONFIRM_REPLIES,
    VOICE_TTS_STOP_FORBIDDEN,
    VOICE_TTS_STOP_PHRASES,
    SpeechDialogFunASR,
    classify_tts_control_reply,
    is_tts_stop_phrase,
)

ASR_SOURCE = (
    PACKAGE_ROOT / 'llm_voice_agent' / 'speech_dialog_funasr_node.py'
).read_text()

CONTROL_TOKENS = ('确认', '取消', '可以', '好的')


# ---------------------------------------------------------------------------
# Agent stub 基础设施（同 test_task_intent_normalization，加事件时序）
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
        events=[],
        get_logger=lambda: logger,
    )
    ag._say = lambda t: (ag.said.append(t), ag.events.append(('say', t)))
    ag._publish_command = lambda t: (
        ag.commands.append(t), ag.events.append(('cmd', t))
    )
    ag._set_state = lambda s: (ag.states.append(s), ag.events.append(('state', s)))
    ag._reset_nav_confirm = lambda: LlmVoiceAgent._reset_nav_confirm(ag)
    return ag


def handle(ag, text):
    return LlmVoiceAgent._handle_navigation(ag, text.lower(), text)


def arm_pending_nav(ag, place='餐厅'):
    """把 stub 置于 nav_wait_confirm（模拟已提出导航询问）。"""
    assert handle(ag, f'导航到{place}')
    ag.said.clear()
    ag.commands.clear()
    ag.states.clear()
    ag.events.clear()
    return ag


# ---------------------------------------------------------------------------
# 1) Agent 纯函数：导航取消严格化 + task 取消语义保持 + 正常确认不受影响
# ---------------------------------------------------------------------------
class NavCancelExactTests(unittest.TestCase):
    def test_exact_cancel_replies(self):
        for text in ('取消', '不去', '算了', '不去了', '停止', '不用了'):
            with self.subTest(text=text):
                self.assertTrue(is_nav_cancel_reply(text))

    def test_echo_fragments_not_cancel(self):
        for text in ('或取消放弃', '请说取消', '我不确定', '不是餐厅', '确认'):
            with self.subTest(text=text):
                self.assertFalse(
                    is_nav_cancel_reply(text),
                    f'回声/噪声句不得取消导航: {text}',
                )


class TaskCancelSemanticsPreservedTests(unittest.TestCase):
    """Patch 4A 不得静默改变 task 模式既有取消语义。"""

    def test_task_cancel_keeps_pref45a_behavior(self):
        # 与 Patch 4A 前 is_nav_cancel_reply 行为一致：
        # startswith 不/别 的安全侧策略 + 取消关键词 substring。
        self.assertTrue(is_task_cancel_reply('取消'))
        self.assertTrue(is_task_cancel_reply('不确定'))   # startswith 不
        self.assertTrue(is_task_cancel_reply('别这样'))   # startswith 别
        self.assertTrue(is_task_cancel_reply('算了算了'))  # marker substring
        self.assertFalse(is_task_cancel_reply('我不确定'))  # 旧行为即 False
        self.assertFalse(is_task_cancel_reply('换个颜色'))
        self.assertFalse(is_task_cancel_reply('好的'))
        self.assertFalse(is_task_cancel_reply('确认'))

    def test_nav_and_task_cancel_are_separate_tiers(self):
        # 同一句 “不确定”：navigation 严格集合不取消，
        # task 仍按旧行为（startswith 不）取消。
        self.assertFalse(is_nav_cancel_reply('不确定'))
        self.assertTrue(is_task_cancel_reply('不确定'))


class NormalConfirmUnchangedTests(unittest.TestCase):
    def test_weak_confirms_still_work_in_normal_listening(self):
        # 弱确认保留在 Agent 正常 listening 的 _NAV_CONFIRM_REPLIES，
        # 正常对话体验不变（与 TTS-time 窄集合是两个安全等级）。
        for text in ('好', '好的', '行', 'ok', '是的', '没问题', '确认', '可以'):
            with self.subTest(text=text):
                self.assertTrue(is_nav_confirm_reply(text))

    def test_noisy_confirm_never_confirms(self):
        for text in ('确认开始导航', '请说确认', '可以让Eric去餐厅', '是否确认'):
            with self.subTest(text=text):
                self.assertFalse(is_nav_confirm_reply(text))


# ---------------------------------------------------------------------------
# 2) Agent _handle_navigation：pending 导航状态机回归
# ---------------------------------------------------------------------------
class PendingNavStateMachineTests(unittest.TestCase):
    def test_pending_cancel_exact(self):
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '取消'))
        self.assertFalse(ag.waiting_confirm)
        self.assertIn('nav_cancel', ag.states)
        self.assertEqual(ag.commands, [])

    def test_pending_echo_fragment_does_not_cancel(self):
        # 问题 1 实测路径：回声“或取消放弃”不得取消 pending 导航。
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '或取消放弃'))
        self.assertTrue(ag.waiting_confirm and ag._pending_nav)
        self.assertEqual(ag._pending_nav_place, '餐厅')
        self.assertEqual(ag.commands, [])

    def test_pending_uncertain_does_not_cancel(self):
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '我不确定'))
        self.assertTrue(ag.waiting_confirm and ag._pending_nav)
        self.assertEqual(ag.commands, [])

    def test_pending_confirm_dispatches_exactly_once(self):
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '确认'))
        self.assertEqual(ag.commands, ['导航到餐厅'])
        self.assertIn('nav_confirmed', ag.states)
        self.assertFalse(ag.waiting_confirm)

    def test_pending_noisy_confirm_no_command(self):
        # 问题 1 实测路径：“确认开始导航”不得确认下发。
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '确认开始导航'))
        self.assertEqual(ag.commands, [])
        self.assertTrue(ag.waiting_confirm and ag._pending_nav)


class NavReplyTokenFreeTests(unittest.TestCase):
    def _assert_token_free(self, reply):
        for token in CONTROL_TOKENS:
            self.assertNotIn(
                token, reply, f'nav_wait_confirm 话术含控制 token {token!r}: {reply}'
            )

    def test_composite_negotiation_reply_token_free(self):
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '帮我去餐厅拿个东西'))
        self.assertEqual(len(ag.said), 1)
        self._assert_token_free(ag.said[0])
        self.assertNotIn('但是可以', ag.said[0])

    def test_plain_nav_prompt_token_free(self):
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '导航到餐厅'))
        self.assertEqual(ag.said, ['要让Eric去餐厅吗？'])
        self._assert_token_free(ag.said[0])

    def test_place_change_prompt_token_free(self):
        ag = arm_pending_nav(make_stub_agent(), place='餐厅')
        self.assertTrue(handle(ag, '导航到客厅'))
        self.assertEqual(ag.said, ['那改让Eric去客厅，这样安排吗？'])
        self._assert_token_free(ag.said[0])

    def test_fallback_reply_token_free_with_place(self):
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '我想想'))
        self.assertEqual(ag.said, ['我没听清。还要让Eric去餐厅吗？'])
        self._assert_token_free(ag.said[0])


# ---------------------------------------------------------------------------
# 3) Agent 事件顺序：STOP command-first；cancel/confirm 先关 state 再播报
# ---------------------------------------------------------------------------
class EventOrderTests(unittest.TestCase):
    def test_robot_stop_command_first(self):
        # 修正 3：物理 STOP 命令必须先于 Rebecca 的语音反馈。
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '停止移动'))
        self.assertEqual(
            ag.events,
            [
                ('cmd', '停止移动'),
                ('state', 'nav_stop'),
                ('say', '好的，正在停止移动。'),
            ],
        )

    def test_nav_cancel_state_closed_before_say(self):
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '取消'))
        self.assertEqual(
            ag.events,
            [
                ('state', 'nav_cancel'),
                ('say', '好的，已取消导航。'),
            ],
        )

    def test_nav_confirm_state_then_say_then_command(self):
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '确认'))
        self.assertEqual(
            ag.events,
            [
                ('state', 'nav_confirmed'),
                ('say', '好的，我让Eric去餐厅。'),
                ('cmd', '导航到餐厅'),
            ],
        )


# ---------------------------------------------------------------------------
# 4) ASR TTS-time 控制门（整句精确匹配）
# ---------------------------------------------------------------------------
_norm_ctrl = SpeechDialogFunASR._norm_control_text

_FAKE_WAKE_SELF = SimpleNamespace(_wake_words_lc=['rebecca', '瑞贝卡'])

NAV_WAIT = 'nav_wait_confirm'


class TtsConfirmGateTests(unittest.TestCase):
    def test_strong_confirms_pass(self):
        for text in ('确认', '我确认', '确定', '我确定', '可以', '执行'):
            with self.subTest(text=text):
                self.assertEqual(
                    classify_tts_control_reply(_norm_ctrl(text), NAV_WAIT),
                    'confirm',
                )

    def test_weak_confirms_do_not_pass(self):
        # 修正 1：弱确认不开放给 TTS-time barge-in。
        for text in ('好', '好的', '行', 'ok', '是的', '没问题'):
            with self.subTest(text=text):
                self.assertIsNone(
                    classify_tts_control_reply(_norm_ctrl(text), NAV_WAIT)
                )

    def test_echo_fragments_do_not_pass(self):
        # 问题 1 实测路径：Rebecca 自播报回声碎片不得误触确认。
        for text in ('确认开始导航', '请说确认', '可以让Eric去餐厅', '是否确认'):
            with self.subTest(text=text):
                self.assertIsNone(
                    classify_tts_control_reply(_norm_ctrl(text), NAV_WAIT)
                )

    def test_gate_only_opens_in_nav_wait_confirm(self):
        for state in ('nav_confirmed', 'nav_cancel', 'chat_idle', ''):
            with self.subTest(state=state):
                self.assertIsNone(classify_tts_control_reply('确认', state))


class TtsCancelGateTests(unittest.TestCase):
    def test_exact_cancels_pass(self):
        for text in ('取消', '不去', '不要', '别去', '算了', '不用了', '不确认'):
            with self.subTest(text=text):
                self.assertEqual(
                    classify_tts_control_reply(_norm_ctrl(text), NAV_WAIT),
                    'cancel',
                )

    def test_echo_fragments_do_not_cancel(self):
        for text in ('或取消放弃', '请说取消', '我不确定'):
            with self.subTest(text=text):
                self.assertIsNone(
                    classify_tts_control_reply(_norm_ctrl(text), NAV_WAIT)
                )

    def test_confirm_and_cancel_sets_disjoint(self):
        self.assertFalse(
            VOICE_TTS_NAV_CONFIRM_REPLIES & VOICE_TTS_NAV_CANCEL_REPLIES
        )


class TtsStopPhraseTests(unittest.TestCase):
    def test_exact_stop_phrases_hit(self):
        for text in ('别说了', '停止播放', '打断', '闭嘴', 'stop', 'pause'):
            with self.subTest(text=text):
                self.assertTrue(
                    is_tts_stop_phrase(_norm_ctrl(text), VOICE_TTS_STOP_PHRASES)
                )

    def test_wake_prefix_stop_hits(self):
        # “瑞贝卡，别说了”：normalize + strip wake prefix + exact。
        reply = SpeechDialogFunASR._strip_wake_prefix(
            _FAKE_WAKE_SELF, _norm_ctrl('瑞贝卡，别说了')
        )
        self.assertEqual(reply, '别说了')
        self.assertTrue(is_tts_stop_phrase(reply, VOICE_TTS_STOP_PHRASES))

    def test_self_echo_not_tts_stop(self):
        # 问题 2 实测路径：Rebecca 自播报回声不得触发打断。
        for text in ('正在停止移动', '停在卧室了', '机器人已经停止了'):
            with self.subTest(text=text):
                self.assertFalse(
                    is_tts_stop_phrase(_norm_ctrl(text), VOICE_TTS_STOP_PHRASES)
                )

    def test_forbidden_phrases_excluded(self):
        self.assertEqual(VOICE_TTS_STOP_FORBIDDEN, {'停', '停止', '好了'})
        for text in ('停', '停止', '好了', '下一条'):
            with self.subTest(text=text):
                self.assertFalse(
                    is_tts_stop_phrase(_norm_ctrl(text), VOICE_TTS_STOP_PHRASES)
                )

    def test_robot_stop_separate_from_tts_stop(self):
        # “停止移动”归 Robot STOP path，不属于普通 TTS STOP。
        self.assertIn('停止移动', VOICE_STOP_PHRASES)
        self.assertNotIn('停止移动', VOICE_TTS_STOP_PHRASES)


class AsrHardeningStaticTests(unittest.TestCase):
    def test_wide_matching_machinery_removed(self):
        self.assertNotIn('_interrupt_combo_re', ASR_SOURCE)
        self.assertNotIn('_interrupt_words_lc', ASR_SOURCE)
        self.assertNotIn('NAV_CONFIRM_TOKENS', ASR_SOURCE)
        self.assertNotIn('NAV_CONFIRM_BLOCKERS', ASR_SOURCE)

    def test_robot_stop_precedes_tts_control_gates(self):
        # Robot STOP 检测必须先于 TTS-time 确认/取消/打断所有逻辑。
        stop_call = ASR_SOURCE.index('self._handle_robot_stop(text)')
        tts_gate = ASR_SOURCE.index('classify_tts_control_reply(reply')
        self.assertLess(stop_call, tts_gate)

    def test_forbidden_filter_warns_at_startup(self):
        # 参数重新带入 forbidden 短语时必须过滤并告警，不静默接受。
        self.assertIn('in VOICE_TTS_STOP_FORBIDDEN', ASR_SOURCE)
        self.assertIn('已过滤', ASR_SOURCE)

    def test_interrupt_params_pinned(self):
        # 既有静态安全基线：打断端点参数保持可配置。
        self.assertIn('declare_parameter("interrupt_max_utt_ms", 1800)', ASR_SOURCE)
        self.assertIn('declare_parameter("interrupt_max_sil_ms", 100)', ASR_SOURCE)


if __name__ == '__main__':
    unittest.main()
