#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航执行节点纯逻辑测试（分类 + 任务ID + 结果 envelope 结构 + PAUSED 状态机）。

这些测试不启动真实 Nav2 / /goto_place 服务，只验证 ``_classify`` 的状态映射、
``_next_task_id`` 的格式、发布 envelope 包含与 ExecutionResult 兼容的字段，
以及 Patch 4C.2 引入的 pause / resume 状态机（通过无 ROS 的最小实例驱动）。
"""

import unittest
from types import SimpleNamespace

from place_manager.navigation_executor_node import (
    NavigationExecutorNode,
    NAV_ACTIONS,
    STATUS_CANCELLED,
    STATUS_FAILED,
    STATUS_PAUSED,
    STATUS_REJECTED,
    STATUS_SUCCEEDED,
    ERR_BUSY,
    ERR_CANCELLED,
    ERR_NAV2_FAILED,
    ERR_NO_PAUSED_NAVIGATION,
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
        # 只消费四个导航 action，其它交给机械臂链路。
        self.assertIn("'navigate_to_place'", self.source)
        self.assertIn("'pause_navigation'", self.source)
        self.assertIn("'resume_navigation'", self.source)
        self.assertIn("'cancel_navigation'", self.source)


# ---------------------------------------------------------------------------
# Patch 4C.2：无 ROS 状态机 harness（不启动真实 Nav2）
# ---------------------------------------------------------------------------
class _FakeFuture:
    def __init__(self):
        self._result = None
        self._done = False

    def done(self):
        return self._done

    def result(self):
        return self._result


class _FakeClient:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, req):
        self.requests.append(req)
        return _FakeFuture()


class _FakeLogger:
    def info(self, msg):
        pass

    def warn(self, msg):
        pass

    def error(self, msg):
        pass


def make_executor():
    """跳过 Node.__init__，只驱动状态机方法所需的最小属性集。"""
    node = NavigationExecutorNode.__new__(NavigationExecutorNode)
    node._goto_future = None
    node._nav_meta = None
    node._cancel_future = None
    node._cancel_meta = None
    node._cancel_accepted = False
    node._cancel_action = None
    node._paused_nav_meta = None
    node._nav_seq = 0
    node._goto_service = '/goto_place'
    node._cancel_service = '/cancel_navigation'
    node._goto_client = _FakeClient()
    node._cancel_client = _FakeClient()
    node.get_logger = lambda: _FakeLogger()
    node._published = []
    node._publish_result = lambda **kw: node._published.append(kw)
    return node


def start_nav(node, place='餐厅', task_id='nav_1'):
    node._request_navigation(place, f'导航到{place}', task_id)


def finish_cancel(node, accepted, message='已取消'):
    fut = node._cancel_future
    fut._result = SimpleNamespace(success=accepted, message=message)
    fut._done = True
    node._tick()


def finish_goto(node, success, message=''):
    fut = node._goto_future
    fut._result = SimpleNamespace(success=success, message=message)
    fut._done = True
    node._tick()


class NavActionContractTests(unittest.TestCase):
    """Patch 4C.2 契约常量。"""

    def test_nav_actions_contains_four_actions(self):
        self.assertEqual(
            NAV_ACTIONS,
            ('navigate_to_place', 'pause_navigation',
             'resume_navigation', 'cancel_navigation'),
        )

    def test_status_paused_constant(self):
        self.assertEqual(STATUS_PAUSED, 'PAUSED')


class PauseStateMachineTests(unittest.TestCase):
    def test_pause_saves_target_only_after_cancel_accepted_and_goto_terminated(self):
        # 行为 3：只有 cancel accepted + 原 goto Future 真正结束才保存 target。
        ex = make_executor()
        start_nav(ex, '餐厅')
        ex._request_cancel('停止导航', action='pause_navigation')
        finish_cancel(ex, accepted=True)
        # cancel 已接受但 goto 未结束：尚未保存。
        self.assertIsNone(ex._paused_nav_meta)
        finish_goto(ex, success=False, message='导航被取消')
        self.assertEqual(ex._paused_nav_meta['place_name'], '餐厅')
        self.assertEqual(ex._paused_nav_meta['source_task_id'], 'nav_1')
        self.assertEqual(ex._published[-1]['action'], 'navigate_to_place')
        self.assertEqual(ex._published[-1]['status'], STATUS_PAUSED)
        self.assertFalse(ex._published[-1]['success'])
        self.assertEqual(ex._published[-1]['error_code'], '')

    def test_cancel_accepted_does_not_save_target(self):
        # 行为 4：cancel accepted 不保存 paused target。
        ex = make_executor()
        start_nav(ex, '餐厅')
        ex._request_cancel('取消导航', action='cancel_navigation')
        finish_cancel(ex, accepted=True)
        finish_goto(ex, success=False, message='导航被取消')
        self.assertIsNone(ex._paused_nav_meta)
        self.assertEqual(ex._published[-1]['status'], STATUS_CANCELLED)
        self.assertEqual(ex._published[-1]['error_code'], ERR_CANCELLED)

    def test_goto_success_never_saves_target(self):
        # Case A：成功到达不得保存 paused target。
        ex = make_executor()
        start_nav(ex, '餐厅')
        ex._request_cancel('停止导航', action='pause_navigation')
        finish_cancel(ex, accepted=True)
        finish_goto(ex, success=True, message='已到达')
        self.assertIsNone(ex._paused_nav_meta)
        self.assertEqual(ex._published[-1]['status'], STATUS_SUCCEEDED)


class PauseNoActiveNavTests(unittest.TestCase):
    def test_pause_without_active_nav_never_creates_target(self):
        ex = make_executor()
        ex._request_cancel('暂停导航', action='pause_navigation')
        self.assertIsNone(ex._paused_nav_meta)
        self.assertEqual(ex._published[-1]['action'], 'pause_navigation')
        self.assertEqual(ex._published[-1]['status'], STATUS_PAUSED)
        self.assertTrue(ex._published[-1]['success'])
        self.assertEqual(ex._published[-1]['reason'], '当前无活动导航，无需暂停')

    def test_pause_when_already_paused_reports_paused_place(self):
        ex = make_executor()
        ex._paused_nav_meta = {
            'place_name': '餐厅', 'source_task_id': 'nav_1', 'raw_text': '去餐厅',
        }
        ex._request_cancel('暂停导航', action='pause_navigation')
        self.assertEqual(ex._paused_nav_meta['place_name'], '餐厅')
        self.assertEqual(ex._published[-1]['place_name'], '餐厅')
        self.assertEqual(ex._published[-1]['reason'], '导航已处于暂停状态')

    def test_cancel_without_active_nav_clears_target(self):
        ex = make_executor()
        ex._paused_nav_meta = {
            'place_name': '餐厅', 'source_task_id': 'nav_1', 'raw_text': '去餐厅',
        }
        ex._request_cancel('取消导航', action='cancel_navigation')
        self.assertIsNone(ex._paused_nav_meta)


class ResumeTests(unittest.TestCase):
    def test_resume_without_paused_target_rejected(self):
        # 行为 5。
        ex = make_executor()
        ex._request_resume('继续导航')
        self.assertFalse(ex._published[-1]['success'])
        self.assertEqual(ex._published[-1]['status'], STATUS_REJECTED)
        self.assertEqual(
            ex._published[-1]['error_code'], ERR_NO_PAUSED_NAVIGATION
        )

    def test_resume_uses_paused_place_and_new_task_id(self):
        # 行为 6 / 7：使用 paused 地点 + 新 task_id 重新 goto_place。
        ex = make_executor()
        ex._paused_nav_meta = {
            'place_name': '餐厅', 'source_task_id': 'nav_1', 'raw_text': '去餐厅',
        }
        ex._request_resume('继续导航')
        self.assertIsNotNone(ex._goto_future)
        self.assertEqual(ex._nav_meta['place_name'], '餐厅')
        self.assertNotEqual(ex._nav_meta['task_id'], 'nav_1')
        self.assertTrue(ex._nav_meta['task_id'].startswith('nav_'))
        self.assertEqual(ex._nav_meta['action'], 'resume_navigation')
        self.assertEqual(ex._goto_client.requests[-1].name, '餐厅')

    def test_resume_rejected_while_goto_active(self):
        ex = make_executor()
        start_nav(ex, '卧室')
        ex._request_resume('继续导航')
        self.assertEqual(ex._published[-1]['error_code'], ERR_BUSY)
        self.assertEqual(ex._published[-1]['status'], STATUS_REJECTED)


class NewNavigationClearsPausedTests(unittest.TestCase):
    def test_new_navigation_clears_old_paused_target(self):
        # 行为 8：PAUSED(餐厅) 后成功启动 navigate_to_place(卧室) 清除餐厅。
        ex = make_executor()
        ex._paused_nav_meta = {
            'place_name': '餐厅', 'source_task_id': 'nav_1', 'raw_text': '去餐厅',
        }
        ex._request_navigation('卧室', '去卧室', 'nav_2')
        self.assertIsNotNone(ex._goto_future)
        self.assertIsNone(ex._paused_nav_meta)


class CancelUpgradeTests(unittest.TestCase):
    def _pending_pause(self):
        ex = make_executor()
        start_nav(ex, '餐厅')
        ex._request_cancel('停止导航', action='pause_navigation')
        return ex

    def test_pending_pause_can_upgrade_to_cancel(self):
        # 行为 9：pause pending 时允许升级为 cancel 并清除 target。
        ex = self._pending_pause()
        ex._paused_nav_meta = {
            'place_name': '餐厅', 'source_task_id': 'nav_0', 'raw_text': '',
        }
        before = len(ex._published)
        ex._request_cancel('取消导航', action='cancel_navigation')
        self.assertEqual(ex._cancel_action, 'cancel_navigation')
        self.assertIsNone(ex._paused_nav_meta)
        self.assertEqual(ex._cancel_meta['action'], 'cancel_navigation')
        self.assertEqual(len(ex._published), before)

    def test_pending_cancel_cannot_downgrade_to_pause(self):
        # 行为 10：cancel -> pause 不允许降级。
        ex = self._pending_pause()
        ex._cancel_action = 'cancel_navigation'
        ex._cancel_meta = {
            'task_id': 'nav_9', 'place_name': '餐厅',
            'raw_text': '', 'action': 'cancel_navigation',
        }
        ex._request_cancel('暂停导航', action='pause_navigation')
        self.assertEqual(ex._cancel_action, 'cancel_navigation')
        self.assertEqual(ex._cancel_meta['task_id'], 'nav_9')


class FullLifecycleTests(unittest.TestCase):
    """Patch 4C.5：PC-only pause / resume 完整生命周期。"""

    def test_full_pause_resume_lifecycle(self):
        # CASE 6：NAVIGATING -> pause -> PAUSED -> resume confirm ->
        # 新 task_id 的 /goto_place(餐厅)。
        ex = make_executor()

        start_nav(
            ex,
            '餐厅',
            'nav_1',
        )

        # 正在导航
        self.assertIsNotNone(ex._goto_future)

        ex._request_cancel(
            '停止导航',
            action='pause_navigation',
        )

        finish_cancel(
            ex,
            accepted=True,
        )

        # cancel accepted 但 goto 未终止：
        # 仍不能 resume
        self.assertIsNone(
            ex._paused_nav_meta
        )

        finish_goto(
            ex,
            success=False,
            message='导航被取消',
        )

        # 真正 PAUSED
        self.assertEqual(
            ex._paused_nav_meta['place_name'],
            '餐厅',
        )

        old_task_id = (
            ex._paused_nav_meta['source_task_id']
        )

        ex._request_resume(
            '恢复导航'
        )

        # 从 paused place 重新 goto
        self.assertIsNotNone(
            ex._goto_future
        )

        self.assertEqual(
            ex._nav_meta['place_name'],
            '餐厅',
        )

        self.assertEqual(
            ex._nav_meta['action'],
            'resume_navigation',
        )

        self.assertNotEqual(
            ex._nav_meta['task_id'],
            old_task_id,
        )

        # resume 已真正启动后 paused target 清除
        self.assertIsNone(
            ex._paused_nav_meta
        )

    def test_full_pause_cancel_forgets_resume_target(self):
        # CASE 7：PAUSED 后彻底取消，resume 必须被拒绝。
        ex = make_executor()

        start_nav(
            ex,
            '餐厅',
            'nav_1',
        )

        ex._request_cancel(
            '停止导航',
            action='pause_navigation',
        )

        finish_cancel(
            ex,
            accepted=True,
        )

        finish_goto(
            ex,
            success=False,
            message='导航被取消',
        )

        self.assertIsNotNone(
            ex._paused_nav_meta
        )

        # 已暂停时彻底取消
        ex._request_cancel(
            '取消导航',
            action='cancel_navigation',
        )

        self.assertIsNone(
            ex._paused_nav_meta
        )

        before = len(ex._published)

        ex._request_resume(
            '恢复导航'
        )

        self.assertEqual(
            len(ex._published),
            before + 1,
        )

        result = ex._published[-1]

        self.assertEqual(
            result['action'],
            'resume_navigation',
        )

        self.assertEqual(
            result['error_code'],
            ERR_NO_PAUSED_NAVIGATION,
        )

        self.assertIsNone(
            ex._goto_future
        )

    def test_pause_pending_upgraded_to_cancel_saves_no_target(self):
        # CASE 8：pause 尚未完成时说“取消导航”升级为 cancel，
        # 最终不得保存 paused target。
        ex = make_executor()

        start_nav(
            ex,
            '餐厅',
            'nav_1',
        )

        ex._request_cancel(
            '停止导航',
            action='pause_navigation',
        )

        ex._request_cancel(
            '取消导航',
            action='cancel_navigation',
        )

        finish_cancel(
            ex,
            accepted=True,
        )

        finish_goto(
            ex,
            success=False,
            message='导航被取消',
        )

        self.assertIsNone(
            ex._paused_nav_meta
        )

        self.assertEqual(
            ex._published[-1]['status'],
            STATUS_CANCELLED,
        )

        self.assertEqual(
            ex._published[-1]['error_code'],
            ERR_CANCELLED,
        )


class FutureOrderingRaceTests(unittest.TestCase):
    """FIX 2：goto terminal 先于 cancel response 到达时不得提前 finalize。"""

    def test_goto_done_before_cancel_response_waits_for_cancel(self):
        # pause 请求已发出，goto Future 先 done、cancel response 未返回：
        # 必须暂缓 finalize，等 cancel accepted 后才判定 PAUSED。
        ex = make_executor()

        start_nav(
            ex,
            '餐厅',
            'nav_1',
        )

        ex._request_cancel(
            '停止导航',
            action='pause_navigation',
        )

        # goto Future 先进入 done，cancel Future 尚未 done
        goto_fut = ex._goto_future
        goto_fut._result = SimpleNamespace(
            success=False, message='导航被取消'
        )
        goto_fut._done = True

        ex._tick()

        # 暂缓 finalize：不得产生终态，也不得保存 paused target
        self.assertIsNotNone(ex._goto_future)
        self.assertIsNone(ex._paused_nav_meta)
        for result in ex._published:
            self.assertNotEqual(result['status'], STATUS_PAUSED)

        # cancel response 返回 accepted
        ex._cancel_future._result = SimpleNamespace(
            success=True, message='已取消'
        )
        ex._cancel_future._done = True

        # 同一个 tick：先 finalize cancel（清 _cancel_future），
        # 再 finalize goto → PAUSED
        ex._tick()

        self.assertIsNone(ex._goto_future)
        self.assertIsNotNone(ex._paused_nav_meta)
        self.assertEqual(ex._paused_nav_meta['place_name'], '餐厅')
        self.assertEqual(ex._published[-1]['status'], STATUS_PAUSED)

    def test_goto_done_before_cancel_reject_waits_and_never_pauses(self):
        # goto 先 done、cancel response 后返回 rejected：
        # 同样必须先等 cancel response；最终不得创建 paused target。
        ex = make_executor()

        start_nav(
            ex,
            '餐厅',
            'nav_1',
        )

        ex._request_cancel(
            '停止导航',
            action='pause_navigation',
        )

        goto_fut = ex._goto_future
        goto_fut._result = SimpleNamespace(
            success=False, message='导航被取消'
        )
        goto_fut._done = True

        ex._tick()
        self.assertIsNotNone(ex._goto_future)
        self.assertIsNone(ex._paused_nav_meta)

        # cancel response 返回 rejected
        ex._cancel_future._result = SimpleNamespace(
            success=False, message='取消被拒绝'
        )
        ex._cancel_future._done = True

        ex._tick()

        self.assertIsNone(ex._goto_future)
        self.assertIsNone(ex._paused_nav_meta)
        self.assertNotEqual(ex._published[-1]['status'], STATUS_PAUSED)


class NavSeqStabilityTests(unittest.TestCase):
    """task_id 已存在时不得 eager 调用 _next_task_id（幽灵递增）。"""

    def test_finalize_with_existing_task_id_does_not_advance_nav_seq(self):
        ex = make_executor()

        start_nav(
            ex,
            '餐厅',
            'nav_existing',
        )

        seq_before = ex._nav_seq

        finish_goto(
            ex,
            success=True,
            message='已到达',
        )

        self.assertEqual(
            ex._nav_seq,
            seq_before,
        )

        self.assertEqual(
            ex._published[-1]['task_id'],
            'nav_existing',
        )


if __name__ == '__main__':
    unittest.main()
