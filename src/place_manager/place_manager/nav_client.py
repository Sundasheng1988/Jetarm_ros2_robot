"""Nav2 NavigateToPose Action 客户端。

该版本不在回调中调用 spin。同步等待由 threading.Event 完成，Action
Future 则由 GotoPlaceNode 的 MultiThreadedExecutor 在其他线程处理。
"""

import threading
import time
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from place_manager.config import (
    MAP_FRAME,
    NAV2_ACTION_TIMEOUT,
    NAV2_CANCEL_TIMEOUT,
    NAV2_FEEDBACK_LOG_PERIOD,
    NAV2_NAVIGATE_TO_POSE_ACTION,
    NAV2_WAIT_FOR_SERVER_TIMEOUT,
)
from place_manager.pose_utils import yaw_to_quaternion


class NavClient:
    """一次只执行一个 NavigateToPose 目标的阻塞式外观封装。"""

    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
        wait_for_server_timeout: float = NAV2_WAIT_FOR_SERVER_TIMEOUT,
        action_timeout: float = NAV2_ACTION_TIMEOUT,
        cancel_timeout: float = NAV2_CANCEL_TIMEOUT,
        feedback_log_period: float = NAV2_FEEDBACK_LOG_PERIOD,
    ):
        self._node = node
        self._log = node.get_logger()
        self._wait_for_server_timeout = float(wait_for_server_timeout)
        self._action_timeout = float(action_timeout)
        self._cancel_timeout = float(cancel_timeout)
        self._feedback_log_period = float(feedback_log_period)

        self._action_client = ActionClient(
            node,
            NavigateToPose,
            NAV2_NAVIGATE_TO_POSE_ACTION,
            callback_group=callback_group,
        )

        self._feedback_lock = threading.Lock()
        self._distance_remaining: Optional[float] = None
        self._number_of_recoveries = 0
        self._last_feedback_log = 0.0

        # Active-goal tracking so an external caller (/cancel_navigation) can
        # cancel the in-flight Nav2 goal without owning the goal handle. The
        # goal handle is cleared via the result future's done-callback, so it
        # is released no matter how ``goto`` returns.
        self._active_lock = threading.Lock()
        self._active_goal_handle = None
        self._active_result_future = None

    @staticmethod
    def _wait_future(future, timeout: float) -> bool:
        """等待 Future 完成，但不调用 rclpy.spin*。"""
        if future.done():
            return True
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        return event.wait(max(0.0, timeout)) or future.done()

    def _reset_feedback(self) -> None:
        with self._feedback_lock:
            self._distance_remaining = None
            self._number_of_recoveries = 0
            self._last_feedback_log = 0.0

    def _feedback_callback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        now = time.monotonic()
        with self._feedback_lock:
            self._distance_remaining = float(feedback.distance_remaining)
            self._number_of_recoveries = int(feedback.number_of_recoveries)
            should_log = (
                self._feedback_log_period > 0.0
                and now - self._last_feedback_log >= self._feedback_log_period
            )
            if should_log:
                self._last_feedback_log = now
                distance = self._distance_remaining
                recoveries = self._number_of_recoveries

        if should_log:
            self._log.info(
                f'导航反馈: 剩余距离={distance:.2f} m, 恢复次数={recoveries}'
            )

    def _feedback_summary(self, elapsed: float) -> str:
        with self._feedback_lock:
            distance = self._distance_remaining
            recoveries = self._number_of_recoveries
        distance_text = '未知' if distance is None else f'{distance:.2f} m'
        return (
            f'用时={elapsed:.1f}s, 剩余距离={distance_text}, '
            f'恢复次数={recoveries}'
        )

    def _cancel_goal(self, goal_handle, result_future) -> str:
        """请求取消并等待 Nav2 确认。"""
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:  # ROS Future/transport error
            return f'取消请求发送失败: {exc}'

        if not self._wait_future(cancel_future, self._cancel_timeout):
            return f'取消请求确认超时（{self._cancel_timeout:.1f}s）'

        try:
            cancel_response = cancel_future.result()
        except Exception as exc:
            return f'取消请求失败: {exc}'

        if not cancel_response.goals_canceling:
            # 目标也可能恰好在取消请求到达前结束。
            if result_future.done():
                try:
                    status = result_future.result().status
                    return f'取消未受理，目标已结束（status={status}）'
                except Exception:
                    pass
            return 'Nav2 未接受取消请求'

        if self._wait_future(result_future, self._cancel_timeout):
            try:
                status = result_future.result().status
                if status == GoalStatus.STATUS_CANCELED:
                    return '导航目标已取消'
                return f'取消后目标结束（status={status}）'
            except Exception as exc:
                return f'取消已受理，但读取终态失败: {exc}'

        return '取消已受理，等待终态超时'

    def _cancel_late_goal(self, send_future) -> None:
        """发送响应本身超时时，取消之后才被接受的目标。"""
        try:
            goal_handle = send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                self._log.warning('迟到的导航目标响应已被接受，立即请求取消')
                goal_handle.cancel_goal_async()
        except Exception as exc:
            self._log.error(f'处理迟到导航目标响应失败: {exc}')

    # ── 外部取消支持（/cancel_navigation）─────────────────────────────────

    def _set_active_goal(self, goal_handle, result_future) -> None:
        with self._active_lock:
            self._active_goal_handle = goal_handle
            self._active_result_future = result_future

    def _clear_active_goal(self) -> None:
        with self._active_lock:
            self._active_goal_handle = None
            self._active_result_future = None

    def has_active_goal(self) -> bool:
        with self._active_lock:
            return self._active_goal_handle is not None

    def cancel_active_goal(self) -> Tuple[bool, str]:
        """请求取消当前目标，并等待 Nav2 明确接受或拒绝。

        返回 True 只表示 Nav2 已接受取消请求，不代表机器人已经完成停车。
        最终是否进入取消终态仍由 ``goto`` 的结果 Future 决定。
        """
        with self._active_lock:
            goal_handle = self._active_goal_handle

        if goal_handle is None:
            return False, '没有正在进行的导航目标'

        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:
            return False, f'取消请求发送失败: {exc}'

        if not self._wait_future(cancel_future, self._cancel_timeout):
            return False, '等待 Nav2 确认取消请求超时'

        try:
            cancel_response = cancel_future.result()
        except Exception as exc:
            return False, f'读取 Nav2 取消响应失败: {exc}'

        if cancel_response is None:
            return False, 'Nav2 返回了空的取消响应'

        goals_canceling = getattr(cancel_response, 'goals_canceling', [])
        if not goals_canceling:
            return False, 'Nav2 拒绝取消当前导航目标'

        return True, 'Nav2 已接受取消请求，正在等待导航终止'

    def goto(self, x: float, y: float, yaw: float) -> Tuple[bool, str]:
        """导航到 map 坐标；返回 ``(success, message)``。"""
        if not self._action_client.wait_for_server(
            timeout_sec=self._wait_for_server_timeout,
        ):
            message = (
                f'Nav2 action server 不可用 '
                f'（{NAV2_NAVIGATE_TO_POSE_ACTION}，'
                f'等待 {self._wait_for_server_timeout:.1f}s）'
            )
            self._log.error(message)
            return False, message

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._reset_feedback()
        started = time.monotonic()
        self._log.info(f'发送导航目标: ({x:.3f}, {y:.3f}, {yaw:.3f})')

        try:
            send_future = self._action_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback,
            )
        except Exception as exc:
            return False, f'发送导航目标失败: {exc}'

        if not self._wait_future(send_future, self._wait_for_server_timeout):
            send_future.add_done_callback(self._cancel_late_goal)
            return False, '发送导航目标响应超时；若目标迟到被接受，将自动取消'

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return False, f'发送导航目标失败: {exc}'

        if goal_handle is None or not goal_handle.accepted:
            return False, '导航目标被 Nav2 拒绝'

        self._log.info('导航目标已被 Nav2 接受')
        result_future = goal_handle.get_result_async()
        self._set_active_goal(goal_handle, result_future)
        # 无论 goto 如何返回（成功/失败/取消/异常），结果 Future 完成后都
        # 释放 active goal，确保 /cancel_navigation 不会作用到已结束的目标。
        result_future.add_done_callback(lambda _f: self._clear_active_goal())

        if not self._wait_future(result_future, self._action_timeout):
            cancel_message = self._cancel_goal(goal_handle, result_future)
            elapsed = time.monotonic() - started
            summary = self._feedback_summary(elapsed)
            message = (
                f'本地导航超时（{self._action_timeout:.1f}s）；'
                f'{cancel_message}；{summary}'
            )
            self._log.error(message)
            return False, message

        elapsed = time.monotonic() - started
        summary = self._feedback_summary(elapsed)
        try:
            wrapped_result = result_future.result()
        except Exception as exc:
            return False, f'读取导航结果失败: {exc}；{summary}'

        status = wrapped_result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True, f'Nav2 导航成功；{summary}'
        if status == GoalStatus.STATUS_CANCELED:
            return False, f'导航被取消；{summary}'
        if status == GoalStatus.STATUS_ABORTED:
            return False, f'导航被 Nav2 中止；{summary}'
        return False, f'导航异常（status={status}）；{summary}'
