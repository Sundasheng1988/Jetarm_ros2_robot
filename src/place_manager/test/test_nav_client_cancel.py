#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NavClient 外部取消支持测试（/cancel_navigation 依赖的接口）。

不需要真实 Nav2 action server：``cancel_active_goal`` 在没有活动目标时是幂等
的，仅依赖内部锁状态，不接触网络。这里验证该幂等行为与 ``has_active_goal``。
"""

import os
import unittest

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from place_manager.nav_client import NavClient

# 哨兵：区分“CYCLONEDDS_URI 原本未设置”与“原值为空串”。
_UNSET = object()


class NavClientCancelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # 运行环境的 CYCLONEDDS_URI 通常绑定到与 Orin 直连的 eno1；离线
        # 测试机上该接口不存在，会导致 Cyclone 建域失败、节点无法创建。这里
        # 仅在测试进程内临时解除该绑定（不改任何系统/网络配置），让 DDS 自动
        # 选择可用接口，以便能构造 NavClient。tearDownClass 会完整恢复原值，
        # 避免污染同一 pytest 进程里的其它测试。
        cls._saved_cyclone_uri = os.environ.get('CYCLONEDDS_URI', _UNSET)
        os.environ.pop('CYCLONEDDS_URI', None)
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
        # 完整恢复 CYCLONEDDS_URI：原本没有就删除，原本有就写回原值。
        if cls._saved_cyclone_uri is _UNSET:
            os.environ.pop('CYCLONEDDS_URI', None)
        else:
            os.environ['CYCLONEDDS_URI'] = cls._saved_cyclone_uri

    def setUp(self):
        self.node = rclpy.create_node('test_nav_client_cancel')

    def tearDown(self):
        self.node.destroy_node()

    def _make_client(self):
        return NavClient(
            self.node,
            callback_group=MutuallyExclusiveCallbackGroup(),
            wait_for_server_timeout=0.1,
            action_timeout=1.0,
            cancel_timeout=1.0,
            feedback_log_period=0.0,
        )

    def test_no_active_goal_reports_idle(self):
        client = self._make_client()
        self.assertFalse(client.has_active_goal())
        ok, message = client.cancel_active_goal()
        self.assertFalse(ok)
        self.assertIn('没有', message)

    def test_set_clear_active_goal(self):
        client = self._make_client()
        # 模拟 goto() 设置活动目标后由结果 Future 清除。
        client._set_active_goal('fake-handle', 'fake-future')
        self.assertTrue(client.has_active_goal())
        client._clear_active_goal()
        self.assertFalse(client.has_active_goal())

    def test_cancel_returns_true_when_goal_set(self):
        client = self._make_client()

        class _FakeCancelResponse:
            # 非空表示 Nav2 已接受取消请求。
            goals_canceling = [object()]

        class _FakeCancelFuture:
            def done(self):
                return True

            def result(self):
                return _FakeCancelResponse()

        class _FakeHandle:
            cancelled = False

            def cancel_goal_async(self):
                self.cancelled = True
                return _FakeCancelFuture()

        handle = _FakeHandle()
        client._set_active_goal(handle, 'fake-result-future')

        ok, message = client.cancel_active_goal()

        self.assertTrue(ok)
        self.assertTrue(handle.cancelled)
        self.assertIn('接受', message)


if __name__ == '__main__':
    unittest.main()
