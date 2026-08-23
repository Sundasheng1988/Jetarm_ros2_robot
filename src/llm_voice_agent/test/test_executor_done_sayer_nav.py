#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""executor_done_sayer 导航结果映射测试（纯静态方法，不需要 ROS）。"""

import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from llm_voice_agent.executor_done_sayer import ExecutorDoneSayer  # noqa: E402


class DoneSayerNavTextTests(unittest.TestCase):
    def test_succeeded(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'SUCCEEDED', 'success': True, 'place_name': '客厅点1'},
                'navigate_to_place',
            ),
            '已经到达客厅点1。',
        )

    def test_place_not_found(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'FAILED', 'error_code': 'PLACE_NOT_FOUND',
                 'place_name': '阳台'},
                'navigate_to_place',
            ),
            '没有找到名为阳台的地点。',
        )

    def test_nav2_failed(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'FAILED', 'error_code': 'NAV2_FAILED'}, 'navigate_to_place',
            ),
            '导航失败，请检查道路。',
        )

    def test_cancelled(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'CANCELLED', 'error_code': 'CANCELLED',
                 'place_name': '客厅点1'},
                'navigate_to_place',
            ),
            '已停止移动。',
        )

    def test_busy(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'REJECTED', 'error_code': 'BUSY', 'place_name': '卧室'},
                'navigate_to_place',
            ),
            '当前正在移动，请先停止当前导航。',
        )

    def test_cancel_action_idle(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'CANCELLED', 'success': True, 'place_name': ''},
                'cancel_navigation',
            ),
            '已停止移动。',
        )

    def test_fallbacks(self):
        # error_detail 旧字段回退。
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'FAILED', 'error_detail': 'PLACE_NOT_FOUND',
                 'place_name': '厨房'},
                'navigate_to_place',
            ),
            '没有找到名为厨房的地点。',
        )
        # evidence.place_name 回退。
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'SUCCEEDED', 'success': True,
                 'evidence': {'place_name': '门口'}},
                'navigate_to_place',
            ),
            '已经到达门口。',
        )


class PauseResumeTextTests(unittest.TestCase):
    """Patch 4C.3：PAUSED / resume 结果话术。"""

    def test_paused_with_place(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'PAUSED', 'place_name': '餐厅'},
                'navigate_to_place',
            ),
            '已暂停前往餐厅的导航。',
        )

    def test_paused_without_place(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'PAUSED', 'place_name': ''},
                'pause_navigation',
            ),
            '已暂停导航。',
        )

    def test_resume_without_paused_target(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'REJECTED', 'error_code': 'NO_PAUSED_NAVIGATION'},
                'resume_navigation',
            ),
            '当前没有可以恢复的导航任务。',
        )

    def test_resume_busy(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'REJECTED', 'error_code': 'BUSY'},
                'resume_navigation',
            ),
            '当前导航还没有进入可恢复状态。',
        )

    def test_resume_succeeded_reaches_arrival_text(self):
        self.assertEqual(
            ExecutorDoneSayer._format_nav_text(
                {'status': 'SUCCEEDED', 'success': True, 'place_name': '餐厅'},
                'resume_navigation',
            ),
            '已经到达餐厅。',
        )


if __name__ == '__main__':
    unittest.main()
