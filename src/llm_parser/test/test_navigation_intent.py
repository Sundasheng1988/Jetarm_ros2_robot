#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航意图解析的离线单元测试（纯函数，不依赖 ROS 运行时）。"""

import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from llm_parser.navigation_intent import (  # noqa: E402
    is_cancel_navigation,
    is_pause_navigation,
    is_resume_navigation,
    parse_cancel_navigation,
    parse_navigation,
    parse_pause_navigation,
    parse_resume_navigation,
)


class ParseNavigationTests(unittest.TestCase):
    def test_recognizes_common_nav_verbs(self):
        cases = {
            '去客厅点1': '客厅点1',
            '导航到阳台': '阳台',
            '前往厨房': '厨房',
            '移动到门口': '门口',
            '到达客厅': '客厅',
            '到卧室': '卧室',
        }
        for text, expected_place in cases.items():
            with self.subTest(text=text):
                cmd = parse_navigation(text)
                self.assertIsNotNone(cmd, f'应识别为导航: {text}')
                self.assertEqual(cmd['action'], 'navigate_to_place')
                self.assertEqual(cmd['place_name'], expected_place)
                self.assertEqual(cmd['source'], 'voice')
                self.assertEqual(cmd['raw_text'], text)

    def test_normalizes_asr_spaces_and_trailing_punct(self):
        # Vosk 常在中文之间插入空格；归一化应合并。
        self.assertEqual(parse_navigation('去客厅 点1')['place_name'], '客厅点1')
        # 末尾标点应被去掉。
        self.assertEqual(parse_navigation('去客厅点1。')['place_name'], '客厅点1')

    def test_rejects_empty_and_filler_places(self):
        for text in ('去', '去那边', '去这里', '去哪里', '', '到一下'):
            with self.subTest(text=text):
                self.assertIsNone(parse_navigation(text), f'不应产生空地点导航: {text!r}')

    def test_does_not_misclassify_arm_or_chat(self):
        # 机械臂任务：动词不在句首。
        self.assertIsNone(parse_navigation('把蓝色杯子移动到右边'))
        # 普通聊天：句首是“介绍”。
        self.assertIsNone(parse_navigation('介绍一下移动机器人技术'))


class PauseNavigationTests(unittest.TestCase):
    """Patch 4C.1：停止类短语统一归 pause_navigation（不再是 cancel）。"""

    def test_stop_move_phrases_are_pause(self):
        for text in (
            '停止移动',
            '停止导航',
            '暂停导航',
            '停止移动吧',
            '别走了',
            '别走',
            '停下移动',
            '停下导航',
        ):
            with self.subTest(text=text):
                self.assertTrue(is_pause_navigation(text), f'应识别为暂停: {text}')
                cmd = parse_pause_navigation(text)
                self.assertIsNotNone(cmd)
                self.assertEqual(cmd['action'], 'pause_navigation')
                self.assertEqual(cmd['source'], 'voice')
                # 语义拆分后不得再落入 cancel。
                self.assertFalse(is_cancel_navigation(text))
                self.assertIsNone(parse_cancel_navigation(text))

    def test_stops_tail_noise(self):
        self.assertTrue(is_pause_navigation('停止移动吧'))
        self.assertTrue(is_pause_navigation('停止移动。'))
        self.assertTrue(is_pause_navigation('暂停导航！'))


class CancelNavigationTests(unittest.TestCase):
    def test_exact_cancel_phrases(self):
        for text in ('取消导航', '取消导航！', '取消移动', '放弃导航'):
            with self.subTest(text=text):
                self.assertTrue(is_cancel_navigation(text), f'应识别为取消: {text}')
                cmd = parse_cancel_navigation(text)
                self.assertIsNotNone(cmd)
                self.assertEqual(cmd['action'], 'cancel_navigation')
                self.assertEqual(cmd['source'], 'voice')

    def test_stop_move_no_longer_cancel(self):
        # Patch 4C.1：停止移动/停止导航属于 pause，不再触发 cancel。
        for text in ('停止移动', '停止导航', '别走了', '停下移动'):
            with self.subTest(text=text):
                self.assertFalse(is_cancel_navigation(text), f'误判为取消: {text!r}')
                self.assertIsNone(parse_cancel_navigation(text))


class ResumeNavigationTests(unittest.TestCase):
    def test_exact_resume_phrases(self):
        for text in ('恢复导航', '恢复导航。', '继续导航'):
            with self.subTest(text=text):
                self.assertTrue(is_resume_navigation(text), f'应识别为恢复: {text}')
                cmd = parse_resume_navigation(text)
                self.assertIsNotNone(cmd)
                self.assertEqual(cmd['action'], 'resume_navigation')
                self.assertEqual(cmd['source'], 'voice')


class NavigationSpeechSafetyTests(unittest.TestCase):
    def test_speech_commands_enter_no_navigation_action(self):
        # “停止说话 / 安静”只中止 LLM/TTS，绝不能进入任何 navigation action。
        for text in ('停止说话', '安静', '别说话了', '停止'):
            with self.subTest(text=text):
                self.assertFalse(is_cancel_navigation(text), f'误判为取消: {text!r}')
                self.assertFalse(is_pause_navigation(text), f'误判为暂停: {text!r}')
                self.assertFalse(is_resume_navigation(text), f'误判为恢复: {text!r}')
                self.assertIsNone(parse_cancel_navigation(text))
                self.assertIsNone(parse_pause_navigation(text))
                self.assertIsNone(parse_resume_navigation(text))


class ParserNodeWiringTests(unittest.TestCase):
    """源码静态断言：parse() 导航分流顺序 cancel < pause < resume < navigate。"""

    def test_parser_node_routes_navigation_first(self):
        source = (
            PACKAGE_ROOT / 'llm_parser' / 'llm_command_parser_node.py'
        ).read_text()
        self.assertIn('from llm_parser.navigation_intent import', source)
        # 锚定到 parse() 内的调用点（而非 import 列表），确认固定顺序。
        cancel_call = source.index('parse_cancel_navigation(')
        pause_call = source.index('parse_pause_navigation(')
        resume_call = source.index('parse_resume_navigation(')
        nav_call = source.index('parse_navigation(')
        self.assertLess(
            cancel_call, pause_call,
            '取消判断必须在暂停判断之前',
        )
        self.assertLess(
            pause_call, resume_call,
            '暂停判断必须在恢复判断之前',
        )
        self.assertLess(
            resume_call, nav_call,
            '恢复判断必须在命名地点导航之前',
        )


if __name__ == '__main__':
    unittest.main()
