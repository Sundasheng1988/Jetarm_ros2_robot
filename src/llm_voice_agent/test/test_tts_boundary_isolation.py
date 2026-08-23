#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patch 4A.1：TTS 边界 Utterance 隔离 + 地点候选净化 离线测试。

覆盖：

1. ASR utterance 级 restricted 锁存不变量（真实 _finalize_utterance
   绑定 stub 驱动，不是只测字符串 helper）：
   - 起始帧 restricted 的 utterance，即使全局 _interrupt_listen_only
     在 finalize 前回 False，也保持 restricted（不进 NORMAL 发布）。
   - min_utt / max_utt / max_sil 端点参数跟随锁存值，不随全局开关切换。
   - 跨界 “去吗？可以。” 永不进入 NORMAL 发布路径。
   - 跨界 “停止移动” 仍是 Robot STOP（最高优先级不丢）。
   - 全新 post-TTS utterance “可以” 仍走 NORMAL 控制直通。
2. Agent 地点候选净化（B1-B5）：问句尾词只在 place 路径剥离、
   句内标点 / 控制词污染拒绝、复合提取器同样校验。
3. pending 导航不被垃圾地点污染（C）+ 机台命令最终防线（D）。
"""

import io
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from llm_voice_agent.llm_voice_agent_node import (  # noqa: E402
    LlmVoiceAgent,
    _NAV_TAIL_NOISE,
    extract_navigation_place,
    extract_navigation_with_unsupported_manip,
    is_valid_navigation_place_candidate,
    normalize_navigation_place_candidate,
)

from llm_voice_agent.speech_dialog_funasr_node import (  # noqa: E402
    VOICE_MODE_NORMAL,
    SpeechDialogFunASR,
    merged_utt_restricted,
)

ASR_SOURCE = (
    PACKAGE_ROOT / 'llm_voice_agent' / 'speech_dialog_funasr_node.py'
).read_text()


# ---------------------------------------------------------------------------
# stub 基础设施
# ---------------------------------------------------------------------------
class _StubLogger:
    def __init__(self):
        self.records = []

    def info(self, msg):
        self.records.append(msg)

    def debug(self, msg):
        self.records.append(msg)

    def warn(self, msg):
        self.records.append(msg)

    def error(self, msg):
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
    ag._reset_nav_confirm = lambda: LlmVoiceAgent._reset_nav_confirm(ag)
    return ag


def handle(ag, text):
    return LlmVoiceAgent._handle_navigation(ag, text.lower(), text)


def arm_pending_nav(ag, place='餐厅'):
    assert handle(ag, f'导航到{place}')
    ag.said.clear()
    ag.commands.clear()
    ag.states.clear()
    return ag


def make_stub_asr(text='去吗？可以。', conf=1.0):
    """绑定真实 _finalize_utterance 等方法的 ASR stub（无 ROS graph）。"""
    logger = _StubLogger()
    asr = SimpleNamespace(
        # 端点参数（与节点默认一致）
        min_utt_ms=600,
        max_utt_ms=6000,
        max_sil_ms=600,
        interrupt_min_utt_ms=120,
        interrupt_max_utt_ms=1800,
        interrupt_max_sil_ms=100,
        sample_rate=16000,
        # 语音段状态
        speeching=True,
        cur_pcm=io.BytesIO(b'x' * 32000),
        ms_in_cur_utt=900,
        ms_sil=200,
        # 全局开关（测试中按需翻转）
        _interrupt_listen_only=False,
        _utt_restricted=False,
        # 识别引擎
        asr=SimpleNamespace(infer_pcm=lambda pcm, sr: (asr_next_text(), asr_next_conf())),
        # 控制门上下文
        _voice_mode=VOICE_MODE_NORMAL,
        _voice_agent_state='nav_wait_confirm',
        _wake_words_lc=['rebecca', '瑞贝卡'],
        _tts_stop_phrases=frozenset(),
        min_avg_conf=0.70,
        min_text_len=4,
        mute_until=0.0,
        last_tts_text='',
        last_tts_time=0.0,
        self_speech_window_s=4.0,
        # 记录器
        published=[],
        interrupts=[],
        get_logger=lambda: logger,
    )
    asr.pub_interrupt = SimpleNamespace(
        publish=lambda msg: asr.interrupts.append(msg.data)
    )
    asr._publish = lambda t: asr.published.append(t)

    next_state = {'text': text, 'conf': conf}

    def asr_next_text():
        return next_state['text']

    def asr_next_conf():
        return next_state['conf']

    # 绑定真实实例方法（lambda 包装模拟绑定调用）
    for name in (
        '_finalize_utterance',
        '_handle_robot_stop',
        '_handle_mode_control',
        '_handle_mode_enter',
        '_strip_wake_prefix',
        '_reset_vad_buffers_if_any',
    ):
        fn = getattr(SpeechDialogFunASR, name)
        setattr(asr, name, lambda *a, _fn=fn, _self=asr, **kw: _fn(_self, *a, **kw))
    # staticmethod：类访问即未绑定函数，直接挂到实例上
    asr._norm_control_text = SpeechDialogFunASR._norm_control_text
    return asr


def finalize(asr, reason='endpoint'):
    return SpeechDialogFunASR._finalize_utterance(asr, reason)


# ---------------------------------------------------------------------------
# 1) ASR：utterance 级 restricted 锁存不变量
# ---------------------------------------------------------------------------
class UttRestrictedLatchPureTests(unittest.TestCase):
    def test_any_restricted_frame_latches(self):
        self.assertFalse(merged_utt_restricted(False, False))
        self.assertTrue(merged_utt_restricted(False, True))

    def test_latch_never_falls_back(self):
        # 锁存后即使后续帧都是非 restricted（TTS 已结束）也不回退。
        self.assertTrue(merged_utt_restricted(True, False))

    def test_vad_reset_clears_latch(self):
        asr = make_stub_asr()
        asr._utt_restricted = True
        asr.speeching = True
        SpeechDialogFunASR._reset_vad_buffers_if_any(asr)
        self.assertFalse(asr._utt_restricted)
        self.assertFalse(asr.speeching)

    def test_on_tts_speaking_true_resets_buffers(self):
        # 保留既有 TTS-True 缓冲复位（不是对整个队列做 blanket 清空）。
        asr = make_stub_asr()
        asr.respect_tts_gate = True
        asr.enable_voice_interrupt = True
        asr._tts_speaking = False
        asr._interrupt_listen_only = False
        asr.speeching = True
        asr._utt_restricted = False
        asr.vad = SimpleNamespace(set_mode=lambda m: None)
        asr.vad_aggressiveness = 2
        asr.tts_gate_release_ms = 300.0
        SpeechDialogFunASR._on_tts_speaking(asr, SimpleNamespace(data=True))
        self.assertTrue(asr._interrupt_listen_only)
        self.assertFalse(asr.speeching)


class BoundaryIsolationTests(unittest.TestCase):
    """不变量 1/3：跨界混音 utterance 不得落入 NORMAL 发布路径。"""

    def test_cross_boundary_merged_speech_never_published_normal(self):
        # “去吗？可以。”：起始帧在 TTS 期间采集（锁存 True），
        # finalize 时 TTS 已结束（全局开关 False）→ 仍按 restricted 丢弃。
        asr = make_stub_asr(text='去吗？可以。', conf=1.0)
        asr._utt_restricted = True
        asr._interrupt_listen_only = False
        finalize(asr, 'max_utt')
        self.assertEqual(asr.published, [])
        self.assertEqual(asr.interrupts, [])
        # A4：finalize 后锁存复位，下一段从新初值开始
        self.assertFalse(asr._utt_restricted)

    def test_restricted_latch_survives_global_flag_flip(self):
        # 不变量 1 的等价观察：锁存 True + 全局 False →
        # min_utt 用 restricted 值（120ms），300ms 短句不被 normal 600ms 丢弃，
        # 以 Robot STOP 作为可观测信号证明走了 restricted 处理路径。
        asr = make_stub_asr(text='停止移动', conf=1.0)
        asr._utt_restricted = True
        asr._interrupt_listen_only = False
        asr.ms_in_cur_utt = 300
        finalize(asr, 'endpoint')
        self.assertEqual(asr.published, ['停止移动'])

    def test_normal_min_utt_still_applies_without_latch(self):
        # 对照组：同样 300ms，锁存 False（全新正常 utterance）→
        # 用 normal min_utt=600 丢弃，什么都不发布。
        asr = make_stub_asr(text='停止移动', conf=1.0)
        asr._utt_restricted = False
        asr._interrupt_listen_only = False
        asr.ms_in_cur_utt = 300
        finalize(asr, 'endpoint')
        self.assertEqual(asr.published, [])

    def test_cross_boundary_robot_stop_preserved(self):
        # 不变量 4：跨界 “停止移动” 仍是 Robot STOP（最高优先级）。
        asr = make_stub_asr(text='停止移动', conf=1.0)
        asr._utt_restricted = True
        asr._interrupt_listen_only = False
        finalize(asr, 'max_utt')
        self.assertEqual(asr.published, ['停止移动'])
        self.assertEqual(asr.interrupts, [True])
        self.assertGreater(asr.mute_until, 0.0)

    def test_fresh_post_tts_utterance_goes_normal(self):
        # 不变量 5：TTS 结束后的全新 utterance（锁存 False）走 NORMAL
        # 控制直通，“可以” 正常送达 Agent。
        asr = make_stub_asr(text='可以', conf=1.0)
        asr._utt_restricted = False
        asr._interrupt_listen_only = False
        finalize(asr, 'endpoint')
        self.assertEqual(asr.published, ['可以'])
        self.assertEqual(asr.interrupts, [])

    def test_tts_time_exact_confirm_still_barge_in(self):
        # 预期行为 A2：TTS 期间真实用户 “确认” barge-in 仍有效。
        asr = make_stub_asr(text='确认', conf=1.0)
        asr._utt_restricted = True
        asr._interrupt_listen_only = True
        finalize(asr, 'endpoint')
        self.assertEqual(asr.published, ['确认'])
        self.assertEqual(asr.interrupts, [True])


class LatchWiringStaticTests(unittest.TestCase):
    """锁存接线静态断言：端点参数与分支必须读锁存值。"""

    def test_queue_carries_capture_time_flag(self):
        self.assertIn('captured_restricted = bool(self._interrupt_listen_only)', ASR_SOURCE)
        self.assertIn('self.q.put_nowait((b, captured_restricted))', ASR_SOURCE)
        self.assertIn('chunk, captured_restricted = self.q.get(timeout=0.1)', ASR_SOURCE)

    def test_endpoint_params_use_utt_latch(self):
        self.assertIn(
            'if self._utt_restricted\n                    else self.max_utt_ms',
            ASR_SOURCE,
        )
        self.assertIn(
            'if self._utt_restricted\n                        else self.max_sil_ms',
            ASR_SOURCE,
        )
        self.assertIn(
            'if utt_restricted else self.min_utt_ms', ASR_SOURCE
        )

    def test_finalize_branch_uses_saved_latch_after_robot_stop(self):
        stop_call = ASR_SOURCE.index('self._handle_robot_stop(text)')
        branch = ASR_SOURCE.index('if utt_restricted:')
        self.assertLess(stop_call, branch)

    def test_no_blanket_queue_clear_on_tts_end(self):
        # 不得用清空整个音频队列的方式处理边界（会丢跨界 Robot STOP）。
        self.assertNotIn('queue.clear()', ASR_SOURCE)
        self.assertNotIn('.q.queue.clear()', ASR_SOURCE)


# ---------------------------------------------------------------------------
# 2) Agent：地点候选净化（B1-B5）
# ---------------------------------------------------------------------------
class PlaceCandidatePureTests(unittest.TestCase):
    def test_normalize_strips_question_tail_only(self):
        cases = {'餐厅吗': '餐厅', '客厅呢': '客厅', '厨房吧': '厨房', '吗': ''}
        for src, want in cases.items():
            with self.subTest(src=src):
                self.assertEqual(normalize_navigation_place_candidate(src), want)

    def test_valid_candidates(self):
        for place in ('餐厅', '客厅', '卧室', '厨房', '书房'):
            with self.subTest(place=place):
                self.assertTrue(is_valid_navigation_place_candidate(place))

    def test_invalid_candidates(self):
        invalid = [
            '', '吗', '呢', '吧', '好', '好的', '行',
            '确认', '取消', '可以', '执行',
            '那边', '地方',                      # filler
            '吗？可以', '吗?可以', '去吗？可以',  # 句内标点残留（跨界混音）
            '吗可以', '去确认',                  # 控制词污染
        ]
        for place in invalid:
            with self.subTest(place=place):
                self.assertFalse(
                    is_valid_navigation_place_candidate(place),
                    f'{place!r} 不应是有效地点候选',
                )

    def test_shared_tail_noise_not_widened(self):
        # B1：不得把 吗 加进共享 _NAV_TAIL_NOISE（会改变 Robot STOP 等
        # 控制路径的语义边界）。
        self.assertNotIn('吗', _NAV_TAIL_NOISE.pattern)


class PlaceExtractionMatrixTests(unittest.TestCase):
    def test_valid_navigation_returns_place(self):
        cases = {
            '去餐厅': '餐厅',
            '去餐厅吧': '餐厅',
            '去餐厅吗？': '餐厅',
            '去餐厅可以吗？': '餐厅',
            '让Eric去餐厅': '餐厅',
        }
        for text, want in cases.items():
            with self.subTest(text=text):
                self.assertEqual(extract_navigation_place(text), want)

    def test_garbage_navigation_returns_none(self):
        for text in ('去吗', '去吗？可以。', '去吗可以', '导航到吗?可以', '去确认', '去取消'):
            with self.subTest(text=text):
                self.assertIsNone(
                    extract_navigation_place(text),
                    f'{text!r} 不得产生地点（防止垃圾机台命令）',
                )

    def test_compound_extractor_same_validation(self):
        # B5：复合提取器必须用同一套校验。
        self.assertEqual(
            extract_navigation_with_unsupported_manip('帮我去餐厅拿点东西'), '餐厅'
        )
        self.assertEqual(
            extract_navigation_with_unsupported_manip('让Eric去客厅取东西'), '客厅'
        )
        self.assertIsNone(
            extract_navigation_with_unsupported_manip('去吗可以拿东西')
        )
        self.assertIsNone(
            extract_navigation_with_unsupported_manip('帮我去确认拿东西')
        )


# ---------------------------------------------------------------------------
# 3) Agent：pending 不被污染（C）+ 机台命令最终防线（D）
# ---------------------------------------------------------------------------
class PendingNavContaminationTests(unittest.TestCase):
    def test_merged_garbage_does_not_overwrite_pending(self):
        # 实测问题 3：nav_wait_confirm 时 “去吗？可以。” 不得把 pending
        # 餐厅改写成垃圾地点。pending 保留，安全 fallback。
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '去吗？可以。'))
        self.assertTrue(ag.waiting_confirm and ag._pending_nav)
        self.assertEqual(ag._pending_nav_place, '餐厅')
        self.assertEqual(ag.pending_cmd_text, '导航到餐厅')
        self.assertEqual(ag.commands, [])
        self.assertEqual(ag.said, ['我没听清。还要让Eric去餐厅吗？'])

    def test_halfwidth_garbage_does_not_overwrite_pending(self):
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '导航到吗?可以'))
        self.assertTrue(ag.waiting_confirm and ag._pending_nav)
        self.assertEqual(ag._pending_nav_place, '餐厅')
        self.assertEqual(ag.commands, [])

    def test_question_tail_place_still_recognized(self):
        # “去餐厅吗？” 在等待确认时应识别为地点变更协商，而非垃圾。
        ag = arm_pending_nav(make_stub_agent())
        self.assertTrue(handle(ag, '去餐厅吗？'))
        self.assertEqual(ag._pending_nav_place, '餐厅')
        self.assertEqual(ag.said, ['那改让Eric去餐厅，这样安排吗？'])


class MachineCommandGuardTests(unittest.TestCase):
    def test_garbage_never_produces_navigation_command(self):
        # D：以下输入在 pending 存在时绝不产生 导航到X 机台命令。
        for text in ('去吗', '去吗可以', '去吗？可以。', '导航到吗?可以', '确认确认导航'):
            with self.subTest(text=text):
                ag = arm_pending_nav(make_stub_agent())
                self.assertTrue(handle(ag, text))
                self.assertEqual(
                    ag.commands, [],
                    f'{text!r} 不得下发任何机台命令',
                )

    def test_valid_path_pending_then_confirm_single_command(self):
        # 有效路径：去餐厅 → 仅 pending；确认 → 恰好一次 导航到餐厅。
        ag = make_stub_agent()
        self.assertTrue(handle(ag, '去餐厅'))
        self.assertEqual(ag.commands, [])
        self.assertTrue(handle(ag, '确认'))
        self.assertEqual(ag.commands, ['导航到餐厅'])
        self.assertEqual(ag.states[-1], 'nav_confirmed')


if __name__ == '__main__':
    unittest.main()
