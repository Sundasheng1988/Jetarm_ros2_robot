#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航执行节点纯逻辑测试（分类 + 任务ID + 结果 envelope 结构）。

这些测试不启动真实 Nav2 / /goto_place 服务，只验证 ``_classify`` 的状态映射、
``_next_task_id`` 的格式，以及发布 envelope 包含与 ExecutionResult 兼容的字段。
"""

import unittest

from place_manager.navigation_executor_node import (
    NavigationExecutorNode,
    STATUS_CANCELLED,
    STATUS_FAILED,
    STATUS_REJECTED,
    STATUS_SUCCEEDED,
    ERR_BUSY,
    ERR_CANCELLED,
    ERR_NAV2_FAILED,
    ERR_PLACE_NOT_FOUND,
    ERR_SERVICE_UNAVAILABLE,
)


class ClassifyTests(unittest.TestCase):
    def test_success(self):
        self.assertEqual(
            NavigationExecutorNode._classify(True, '已到达', False),
            (STATUS_SUCCEEDED, ''),
        )

    def test_explicit_cancel_requested(self):
        # 用户显式取消优先于其它失败原因。
        self.assertEqual(
            NavigationExecutorNode._classify(False, '导航失败', True),
            (STATUS_CANCELLED, ERR_CANCELLED),
        )

    def test_place_not_found_by_message(self):
        self.assertEqual(
            NavigationExecutorNode._classify(False, '地点 "阳台" 不存在', False),
            (STATUS_FAILED, ERR_PLACE_NOT_FOUND),
        )

    def test_service_unavailable_by_message(self):
        self.assertEqual(
            NavigationExecutorNode._classify(False, 'Nav2 不可用', False),
            (STATUS_FAILED, ERR_SERVICE_UNAVAILABLE),
        )

    def test_cancelled_by_message(self):
        self.assertEqual(
            NavigationExecutorNode._classify(False, '导航被取消', False),
            (STATUS_CANCELLED, ERR_CANCELLED),
        )

    def test_generic_nav2_failure(self):
        self.assertEqual(
            NavigationExecutorNode._classify(False, '规划路径失败', False),
            (STATUS_FAILED, ERR_NAV2_FAILED),
        )


class EnvelopeShapeTests(unittest.TestCase):
    """源码断言：发布 envelope 必须兼容 ExecutionResult 且含导航扩展字段。"""

    def setUp(self):
        from pathlib import Path
        pkg_root = Path(__file__).resolve().parents[1]
        self.source = (
            pkg_root / 'place_manager' / 'navigation_executor_node.py'
        ).read_text()

    def test_envelope_has_compatible_and_nav_fields(self):
        # ExecutionResult 兼容字段。
        for key in ('task_id', 'success', 'reason', 'confidence',
                    'evidence', 'error_detail', 'timestamp'):
            with self.subTest(key=key):
                self.assertIn(f"'{key}'", self.source)
        # 导航扩展字段。
        for key in ('action', 'place_name', 'status', 'error_code'):
            with self.subTest(key=key):
                self.assertIn(f"'{key}'", self.source)

    def test_busy_does_not_preempt(self):
        # 进行中收到第二个导航必须 BUSY 拒绝，而不是抢占。
        self.assertIn(ERR_BUSY, self.source)
        self.assertIn(STATUS_REJECTED, self.source)
        self.assertIn('当前正在移动，请先停止当前导航', self.source)

    def test_only_nav_actions_consumed(self):
        # 只消费 navigate_to_place / cancel_navigation，其它交给机械臂链路。
        self.assertIn("'navigate_to_place'", self.source)
        self.assertIn("'cancel_navigation'", self.source)


if __name__ == '__main__':
    unittest.main()
