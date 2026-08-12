#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""取消语义的 mock/fake 测试。

核心不变量（本文件逐条验证）：

1. **不得仅因为 ``cancel_goal_async`` 已发出就宣布“已停止移动”。** “已停止移动”
   只能由 ``/runtime/execution_result`` 的 ``CANCELLED`` 触发，而该结果只在
   导航 Future 真正终止（``_finalize_navigation``）或本就无活动导航时才发布。
2. **必须依据原导航任务的最终状态**：即便发起过取消，若 Nav2 实际到达
   （SUCCEEDED），应播报“已经到达”，而不是“已停止移动”。
3. **无活动目标取消幂等**；**每个导航任务只发布一次 execution_result**；
   **done_sayer 对同一 task_id 只播报一次**，且机械臂结果不在导航通道播报。

这些测试用 fake future / fake response 驱动节点内部状态，不启动真实 Nav2，
也不依赖 /goto_place、/cancel_navigation 服务端。
"""

import json
import os
import unittest

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from place_manager.navigation_executor_node import (
    NavigationExecutorNode,
    STATUS_CANCELLED,
    STATUS_SUCCEEDED,
)
from place_manager.srv import GotoPlace

_UNSET = object()


class _FakeFuture:
    """最小 Future 替身：可控的 done() / result()。"""

    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        if self._done:
            cb(self)


def _goto_response(success: bool, message: str) -> GotoPlace.Response:
    resp = GotoPlace.Response()
    resp.success = success
    resp.message = message
    return resp


class CancelSemanticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._saved_cyclone_uri = os.environ.get('CYCLONEDDS_URI', _UNSET)
        os.environ.pop('CYCLONEDDS_URI', None)
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
        if cls._saved_cyclone_uri is _UNSET:
            os.environ.pop('CYCLONEDDS_URI', None)
        else:
            os.environ['CYCLONEDDS_URI'] = cls._saved_cyclone_uri

    def setUp(self):
        self.node = NavigationExecutorNode()
        self.published = []
        # 捕获真正会发到 /runtime/execution_result 的 envelope。
        self.node._pub_result.publish = (
            lambda msg: self.published.append(json.loads(msg.data))
        )
        # /cancel_navigation 无真实服务端：让可用性检查与 call_async 返回受控假象。
        self.node._cancel_client.service_is_ready = lambda: True
        self.node._cancel_client.call_async = lambda req: _FakeFuture(done=False)

    def tearDown(self):
        self.node.destroy_node()

    def _arm_active_nav(self, place: str, response, cancel_accepted: bool):
        """把节点置为“有一个导航 Future 已终止”的状态。"""
        self.node._goto_future = _FakeFuture(result=response, done=True)
        self.node._nav_meta = {
            'task_id': 'nav_test_1',
            'place_name': place,
            'raw_text': f'去{place}',
        }
        self.node._cancel_accepted = cancel_accepted

    # ── 无活动目标：幂等，立即一次 CANCELLED ──────────────────────────────
    def test_no_active_nav_cancel_is_idempotent(self):
        self.node._goto_future = None
        self.node._request_cancel('停止移动')
        self.assertEqual(len(self.published), 1)
        env = self.published[0]
        self.assertEqual(env['status'], STATUS_CANCELLED)
        self.assertTrue(env['success'])
        self.assertEqual(env['action'], 'cancel_navigation')

    # ── 有活动目标但 Future 未终止：不得提前发布结果 ─────────────────────
    def test_active_nav_cancel_does_not_publish_prematurely(self):
        self.node._goto_future = _FakeFuture(done=False)  # 导航仍在进行
        self.node._nav_meta = {
            'task_id': 'nav_test_1',
            'place_name': '客厅',
            'raw_text': '去客厅',
        }

        before = len(self.published)
        self.node._request_cancel('停止移动')

        self.assertEqual(
            len(self.published),
            before,
            '导航未终止前不得发布结果',
        )
        self.assertIsNotNone(self.node._cancel_future)
        self.assertFalse(self.node._cancel_accepted)

    def test_cancel_acceptance_still_waits_for_navigation_terminal_state(self):
        self.node._goto_future = _FakeFuture(done=False)
        self.node._nav_meta = {
            'task_id': 'nav_test_1',
            'place_name': '客厅',
            'raw_text': '去客厅',
        }

        cancel_response = Trigger.Response()
        cancel_response.success = True
        cancel_response.message = 'Nav2 已接受取消请求，正在等待导航终止'

        self.node._cancel_client.call_async = (
            lambda request: _FakeFuture(
                result=cancel_response,
                done=True,
            )
        )

        before = len(self.published)
        self.node._request_cancel('停止移动')
        self.node._tick()

        self.assertTrue(self.node._cancel_accepted)
        self.assertIsNone(self.node._cancel_future)
        self.assertEqual(
            len(self.published),
            before,
            'Nav2 仅接受取消时仍不得提前宣布已经停止',
        )

    # ── 取消后导航真正被取消 → CANCELLED（→“已停止移动”）────────────────
    def test_cancel_then_canceled_announces_stopped(self):
        self._arm_active_nav(
            '客厅', _goto_response(False, '导航被取消'), cancel_accepted=True,
        )
        self.node._finalize_navigation()
        self.assertEqual(len(self.published), 1)
        env = self.published[0]
        self.assertEqual(env['status'], STATUS_CANCELLED)
        self.assertFalse(env['success'])
        self.assertEqual(env['action'], 'navigate_to_place')

    # ── 取消已发但 Nav2 实际到达 → 必须 SUCCEEDED（→“已经到达”）─────────
    def test_cancel_requested_but_arrived_is_success_not_cancelled(self):
        self._arm_active_nav(
            '客厅', _goto_response(True, '已到达'), cancel_accepted=True,
        )
        self.node._finalize_navigation()
        self.assertEqual(len(self.published), 1)
        env = self.published[0]
        self.assertEqual(env['status'], STATUS_SUCCEEDED)
        self.assertTrue(env['success'])  # 关键：不能因发起过取消而谎称已停止

    # ── 每个导航任务只发布一次 execution_result ──────────────────────────
    def test_single_result_no_duplicate(self):
        self._arm_active_nav(
            '客厅', _goto_response(False, '导航被取消'), cancel_accepted=True,
        )
        self.node._finalize_navigation()
        self.node._finalize_navigation()  # Future 已清空，不应再发布
        self.assertEqual(len(self.published), 1)

    # ── _tick 在 Future 终止时驱动 finalize ───────────────────────────────
    def test_tick_finalizes_when_done(self):
        self._arm_active_nav(
            '客厅', _goto_response(False, '导航被取消'), cancel_accepted=True,
        )
        self.node._tick()
        self.assertEqual(len(self.published), 1)
        self.assertEqual(self.published[0]['status'], STATUS_CANCELLED)


class DoneSayerDedupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._saved_cyclone_uri = os.environ.get('CYCLONEDDS_URI', _UNSET)
        os.environ.pop('CYCLONEDDS_URI', None)
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
        if cls._saved_cyclone_uri is _UNSET:
            os.environ.pop('CYCLONEDDS_URI', None)
        else:
            os.environ['CYCLONEDDS_URI'] = cls._saved_cyclone_uri

    def setUp(self):
        from llm_voice_agent.executor_done_sayer import ExecutorDoneSayer
        self.node = ExecutorDoneSayer()
        self.said = []
        self.node.pub_reply.publish = lambda msg: self.said.append(msg.data)

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def _msg(envelope: dict) -> String:
        return String(data=json.dumps(envelope, ensure_ascii=False))

    def test_same_task_id_announced_once(self):
        env = {'task_id': 'nav_dup_1', 'action': 'cancel_navigation',
               'status': STATUS_CANCELLED, 'success': True, 'place_name': ''}
        self.node._on_execution_result(self._msg(env))
        self.node._on_execution_result(self._msg(env))  # 重复
        self.assertEqual(len(self.said), 1)
        self.assertEqual(self.said[0], '已停止移动。')

    def test_arm_result_not_announced_on_nav_channel(self):
        # 机械臂结果（非导航 action）不应在导航通道播报（由 /executor/done 负责）。
        env = {'task_id': 'arm_1', 'action': 'pick', 'status': 'SUCCEEDED',
               'success': True, 'place_name': ''}
        self.node._on_execution_result(self._msg(env))
        self.assertEqual(len(self.said), 0)

    def test_malformed_json_does_not_crash(self):
        self.node._on_execution_result(String(data='not json {'))
        self.node._on_execution_result(String(data='123'))
        self.assertEqual(len(self.said), 0)  # 异常一律跳过，不崩溃也不播报


if __name__ == '__main__':
    unittest.main()
