#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航任务执行节点：把 /parsed_command 中的导航动作路由到 /goto_place。

职责边界（与机械臂 Runtime 严格分离）：

- 只消费 ``navigate_to_place`` 与 ``cancel_navigation`` 两个 action；其它
  action（pick/place 等）一律忽略，仍由 Grounding + real_grounded_runtime_node
  处理。
- 导航动作不进入需要 ``target_object`` 的机械臂 Grounding 流程。
- 同一时刻只允许一个导航任务；导航进行中收到第二个导航任务时，以明确的
  ``BUSY`` 拒绝，绝不自动抢占。
- 异步调用现有 ``/goto_place`` 服务（``call_async``），不在订阅回调里做会
  阻塞整个 executor 的同步等待；用一个低频定时器轮询 Future 终态。
- 取消通过 ``/cancel_navigation`` 服务完成，该服务在 ``goto_place_node``
  的独立 Reentrant 回调组中运行，保证导航进行中仍可响应。

结果复用现有 ``/runtime/execution_result`` 协议（与
``sketch_runtime.execution_result.ExecutionResult.to_dict()`` 同形），
并在其中扩展导航字段 ``action / place_name / status / error_code``。
"""

from __future__ import annotations

import json
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from place_manager.config import (
    CANCEL_NAVIGATION_SERVICE,
    GOTO_PLACE_SERVICE,
)
from place_manager.srv import GotoPlace


# ── 结果状态与错误码 ──────────────────────────────────────────────────────
STATUS_SUCCEEDED = 'SUCCEEDED'
STATUS_FAILED = 'FAILED'
STATUS_CANCELLED = 'CANCELLED'
STATUS_REJECTED = 'REJECTED'  # 例如 BUSY

ERR_PLACE_NOT_FOUND = 'PLACE_NOT_FOUND'
ERR_NAV2_FAILED = 'NAV2_FAILED'
ERR_CANCELLED = 'CANCELLED'
ERR_BUSY = 'BUSY'
ERR_SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE'
ERR_CANCEL_REJECTED = 'CANCEL_REJECTED'

# 处理这些 action；其它 action 交给机械臂链路。
NAV_ACTIONS = ('navigate_to_place', 'cancel_navigation')


class NavigationExecutorNode(Node):
    """单一导航任务编排器。"""

    def __init__(self):
        super().__init__('navigation_executor_node')

        self.declare_parameter('parsed_command_topic', '/parsed_command')
        self.declare_parameter('execution_result_topic', '/runtime/execution_result')
        self.declare_parameter('tick_period', 0.1)
        self.declare_parameter('goto_place_service', GOTO_PLACE_SERVICE)
        self.declare_parameter('cancel_navigation_service', CANCEL_NAVIGATION_SERVICE)

        self._parsed_topic = str(self.get_parameter('parsed_command_topic').value)
        self._result_topic = str(self.get_parameter('execution_result_topic').value)
        self._tick_period = float(self.get_parameter('tick_period').value)
        self._goto_service = str(self.get_parameter('goto_place_service').value)
        self._cancel_service = str(self.get_parameter('cancel_navigation_service').value)

        # ── IO ──
        self.create_subscription(String, self._parsed_topic, self._on_parsed_command, 10)
        self._pub_result = self.create_publisher(String, self._result_topic, 10)

        self._goto_client = self.create_client(GotoPlace, self._goto_service)
        self._cancel_client = self.create_client(Trigger, self._cancel_service)

        # ── 导航任务状态（单线程 executor 串行访问，无需锁）──
        self._goto_future = None
        self._nav_meta: Optional[dict] = None      # 当前导航的 task_id/place_name/raw_text
        self._cancel_future = None
        self._cancel_meta: Optional[dict] = None
        # 只有 /cancel_navigation 明确返回 success=True 后才置 True。
        self._cancel_accepted = False
        self._nav_seq = 0

        self.create_timer(self._tick_period, self._tick)

        self.get_logger().info(
            f'NavigationExecutorNode 启动: sub={self._parsed_topic}, '
            f'pub={self._result_topic}, goto={self._goto_service}, '
            f'cancel={self._cancel_service}'
        )

    # ── 订阅 /parsed_command ──────────────────────────────────────────────

    def _on_parsed_command(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(f'解析 /parsed_command JSON 失败: {exc}')
            return

        action = (data.get('action') or '').strip()
        if action not in NAV_ACTIONS:
            return  # 非导航动作：交给机械臂链路，忽略

        raw_text = data.get('raw_text') or data.get('raw') or ''
        if action == 'cancel_navigation':
            self._request_cancel(raw_text)
            return

        # navigate_to_place
        place_name = (data.get('place_name') or '').strip()
        task_id = data.get('task_id') or self._next_task_id()
        if not place_name:
            # Parser 已保证不产生空地点；这里防御性拒绝。
            self._publish_result(
                task_id=task_id, action=action, place_name='',
                success=False, status=STATUS_FAILED,
                error_code=ERR_PLACE_NOT_FOUND,
                reason='导航地点名称为空',
                raw_text=raw_text,
            )
            return

        self._request_navigation(place_name, raw_text, task_id)

    # ── 发起导航 / 取消 ───────────────────────────────────────────────────

    def _request_navigation(self, place_name: str, raw_text: str, task_id: str) -> None:
        # 单一导航：进行中直接 BUSY 拒绝，不抢占。
        if self._goto_future is not None:
            self.get_logger().warn(f'BUSY：已有导航进行中，拒绝到 "{place_name}"')
            self._publish_result(
                task_id=task_id, action='navigate_to_place',
                place_name=place_name, success=False, status=STATUS_REJECTED,
                error_code=ERR_BUSY,
                reason='当前正在移动，请先停止当前导航',
                raw_text=raw_text,
            )
            return

        if not self._goto_client.service_is_ready():
            self.get_logger().error(f'/goto_place 服务不可用，无法导航到 "{place_name}"')
            self._publish_result(
                task_id=task_id, action='navigate_to_place',
                place_name=place_name, success=False, status=STATUS_FAILED,
                error_code=ERR_SERVICE_UNAVAILABLE,
                reason=f'{self._goto_service} 服务不可用',
                raw_text=raw_text,
            )
            return

        req = GotoPlace.Request()
        req.name = place_name
        self._goto_future = self._goto_client.call_async(req)
        self._nav_meta = {
            'task_id': task_id,
            'place_name': place_name,
            'raw_text': raw_text,
        }
        self._cancel_accepted = False
        self._cancel_future = None
        self._cancel_meta = None
        self.get_logger().info(f'开始导航到 "{place_name}" (task_id={task_id})')

    def _request_cancel(self, raw_text: str) -> None:
        cancel_task_id = self._next_task_id()

        # 没有活动导航：幂等成功。
        if self._goto_future is None:
            self.get_logger().info(
                '取消导航：当前无活动导航，幂等返回 CANCELLED'
            )
            self._publish_result(
                task_id=cancel_task_id,
                action='cancel_navigation',
                place_name='',
                success=True,
                status=STATUS_CANCELLED,
                error_code=ERR_CANCELLED,
                reason='已停止移动（无活动导航）',
                raw_text=raw_text,
            )
            return

        # 取消请求正在等待响应，或已经被 Nav2 接受。
        if self._cancel_future is not None or self._cancel_accepted:
            self.get_logger().info(
                '取消导航：取消请求正在处理，忽略重复请求'
            )
            return

        place_name = (
            self._nav_meta.get('place_name', '')
            if self._nav_meta else ''
        )

        if not self._cancel_client.service_is_ready():
            self.get_logger().error(
                f'{self._cancel_service} 服务不可用，无法取消'
            )
            self._publish_result(
                # 取消动作必须使用自己的 task_id，不能复用原导航 task_id。
                task_id=cancel_task_id,
                action='cancel_navigation',
                place_name=place_name,
                success=False,
                status=STATUS_FAILED,
                error_code=ERR_SERVICE_UNAVAILABLE,
                reason=f'{self._cancel_service} 服务不可用，未能取消',
                raw_text=raw_text,
            )
            return

        try:
            self._cancel_future = self._cancel_client.call_async(
                Trigger.Request()
            )
        except Exception as exc:
            self._cancel_future = None
            self._publish_result(
                task_id=cancel_task_id,
                action='cancel_navigation',
                place_name=place_name,
                success=False,
                status=STATUS_FAILED,
                error_code=ERR_CANCEL_REJECTED,
                reason=f'发送取消服务请求失败: {exc}',
                raw_text=raw_text,
            )
            return

        self._cancel_meta = {
            'task_id': cancel_task_id,
            'place_name': place_name,
            'raw_text': raw_text,
        }
        self.get_logger().info(
            '已发送取消服务请求，等待 Nav2 接受或拒绝'
        )

    # ── 定时器：轮询 Future 终态 ──────────────────────────────────────────

    def _tick(self) -> None:
        # 必须先处理取消服务响应，再处理导航终态。
        # 如果两个 Future 在同一个 tick 内完成，分类时才能知道取消是否被接受。
        if (
            self._cancel_future is not None
            and self._cancel_future.done()
        ):
            self._finalize_cancel_request()

        if (
            self._goto_future is not None
            and self._goto_future.done()
        ):
            self._finalize_navigation()

    def _finalize_cancel_request(self) -> None:
        """处理 /cancel_navigation 的明确接受、拒绝或异常响应。"""
        if self._cancel_future is None:
            return

        future = self._cancel_future
        meta = self._cancel_meta or {}

        self._cancel_future = None
        self._cancel_meta = None

        try:
            response = future.result()
            accepted = bool(
                response is not None and response.success
            )
            message = (
                response.message
                if response is not None
                else '取消服务返回空响应'
            )
        except Exception as exc:
            accepted = False
            message = f'读取取消服务响应失败: {exc}'

        if accepted:
            self._cancel_accepted = True
            self.get_logger().info(
                f'/cancel_navigation 已接受: {message}'
            )
            # 此处不发布“已停止”；必须等待原导航 Future 的最终结果。
            return

        self._cancel_accepted = False
        self.get_logger().error(
            f'/cancel_navigation 未接受: {message}'
        )

        # 取消失败使用独立 task_id 发布，不能占用原导航 task_id。
        if self._goto_future is not None:
            self._publish_result(
                task_id=meta.get('task_id', self._next_task_id()),
                action='cancel_navigation',
                place_name=meta.get('place_name', ''),
                success=False,
                status=STATUS_FAILED,
                error_code=ERR_CANCEL_REJECTED,
                reason=message,
                raw_text=meta.get('raw_text', ''),
            )

    def _finalize_navigation(self) -> None:
        # _tick 保证只在 Future 终止且未被处理时调用一次；这里再防御一层，
        # 确保即便被重复调用，也绝不为同一导航发布第二次 execution_result。
        if self._goto_future is None:
            return
        future = self._goto_future
        meta = self._nav_meta or {}
        self._goto_future = None
        self._nav_meta = None
        cancel_accepted = self._cancel_accepted
        self._cancel_accepted = False

        # 原导航已经终止，不再处理迟到的取消响应。
        self._cancel_future = None
        self._cancel_meta = None

        place_name = meta.get('place_name', '')
        raw_text = meta.get('raw_text', '')
        task_id = meta.get('task_id', self._next_task_id())

        try:
            response: GotoPlace.Response = future.result()
        except Exception as exc:
            self.get_logger().error(f'/goto_place 调用异常: {exc}')
            self._publish_result(
                task_id=task_id, action='navigate_to_place', place_name=place_name,
                success=False, status=STATUS_FAILED, error_code=ERR_NAV2_FAILED,
                reason=f'/goto_place 调用异常: {exc}', raw_text=raw_text,
            )
            return

        if response is None:
            self._publish_result(
                task_id=task_id, action='navigate_to_place', place_name=place_name,
                success=False, status=STATUS_FAILED, error_code=ERR_NAV2_FAILED,
                reason='/goto_place 返回空响应', raw_text=raw_text,
            )
            return

        success = bool(response.success)
        message = response.message or ''
        status, error_code = self._classify(
            success, message, cancel_accepted
        )

        self.get_logger().info(
            f'导航结束: place="{place_name}" success={success} '
            f'status={status} error_code={error_code} msg={message}'
        )
        self._publish_result(
            task_id=task_id, action='navigate_to_place', place_name=place_name,
            success=success and status == STATUS_SUCCEEDED,
            status=status, error_code=error_code,
            reason=message, raw_text=raw_text,
        )

    # ── 分类 / 发布 ───────────────────────────────────────────────────────

    @staticmethod
    def _classify(
        success: bool,
        message: str,
        cancel_accepted: bool,
    ) -> Tuple[str, str]:
        """根据导航结果和已确认的取消状态进行分类。"""
        # 即使曾请求取消，Nav2 最终成功到达仍必须报告成功。
        if success:
            return STATUS_SUCCEEDED, ''

        # 只有 Nav2 明确接受取消，才能用取消标志分类。
        if cancel_accepted:
            return STATUS_CANCELLED, ERR_CANCELLED

        if '不存在' in message:
            return STATUS_FAILED, ERR_PLACE_NOT_FOUND
        if '不可用' in message:
            return STATUS_FAILED, ERR_SERVICE_UNAVAILABLE
        if '取消' in message:
            return STATUS_CANCELLED, ERR_CANCELLED

        return STATUS_FAILED, ERR_NAV2_FAILED

    def _publish_result(
        self,
        task_id: str,
        action: str,
        place_name: str,
        success: bool,
        status: str,
        error_code: str,
        reason: str,
        raw_text: str,
    ) -> None:
        envelope = {
            # 与 ExecutionResult.to_dict() 同形（兼容 RobotOps / done_sayer）。
            'task_id': task_id,
            'success': success,
            'reason': reason,
            'confidence': 1.0 if success else 0.0,
            'evidence': {
                'domain': 'navigation',
                'place_name': place_name,
                'raw_text': raw_text,
            },
            'error_detail': error_code or None,
            'timestamp': time.time(),
            # 导航扩展字段。
            'action': action,
            'place_name': place_name,
            'status': status,
            'error_code': error_code,
        }
        self._pub_result.publish(String(data=json.dumps(envelope, ensure_ascii=False)))
        self.get_logger().info(
            f'发布 /runtime/execution_result: action={action} status={status} '
            f'place="{place_name}" task_id={task_id}'
        )

    def _next_task_id(self) -> str:
        self._nav_seq += 1
        return f'nav_{int(time.time())}_{self._nav_seq}'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
