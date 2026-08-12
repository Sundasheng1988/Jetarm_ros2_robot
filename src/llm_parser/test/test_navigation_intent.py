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
    parse_cancel_navigation,
    parse_navigation,
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


class CancelNavigationTests(unittest.TestCase):
    def test_recognizes_stop_move_phrases(self):
        for text in (
            '停止移动',
            '停止导航',
            '取消导航',
            '取消移动',
            '别走了',
            '别走',
            '停下移动',
        ):
            with self.subTest(text=text):
                self.assertTrue(is_cancel_navigation(text), f'应识别为取消: {text}')
                cmd = parse_cancel_navigation(text)
                self.assertIsNotNone(cmd)
                self.assertEqual(cmd['action'], 'cancel_navigation')
                self.assertEqual(cmd['source'], 'voice')

    def test_strips_tail_noise(self):
        self.assertTrue(is_cancel_navigation('停止移动吧'))
        self.assertTrue(is_cancel_navigation('停止移动。'))
        self.assertTrue(is_cancel_navigation('取消导航！'))

    def test_speech_commands_are_not_cancel(self):
        # “停止说话 / 安静”只中止 LLM/TTS，绝不能取消导航。
        for text in ('停止说话', '安静', '别说话了', '停止'):
            with self.subTest(text=text):
                self.assertFalse(is_cancel_navigation(text), f'误判为取消: {text!r}')
                self.assertIsNone(parse_cancel_navigation(text))


class ParserNodeWiringTests(unittest.TestCase):
    """源码静态断言：parse() 最前面必须先判取消、再判导航。"""

    def test_parser_node_routes_navigation_first(self):
        source = (
            PACKAGE_ROOT / 'llm_parser' / 'llm_command_parser_node.py'
        ).read_text()
        self.assertIn('from llm_parser.navigation_intent import', source)
        # 锚定到 parse() 内的调用点（而非 import 列表），确认取消先于导航。
        cancel_call = source.index('parse_cancel_navigation(')
        nav_call = source.index('parse_navigation(')
        self.assertLess(
            cancel_call, nav_call,
            '取消判断必须在导航判断之前（取消优先级更高）',
        )


if __name__ == '__main__':
    unittest.main()
