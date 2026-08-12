#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""结果播报节点：把执行结果翻译成语音反馈。

两条播报通道，互不重复：

1. 机械臂完成 → 订阅 ``/executor/done``（Bool）。True 时播报
   ``success_text``（默认“任务已完成。需要继续吗？”）。
   机械臂的结果**也**会出现在 ``/runtime/execution_result`` 上，但本节点
   只把 ``/runtime/execution_result`` 用于**导航**播报，因此不会对机械臂
   任务重复播报。

2. 导航结果 → 订阅 ``/runtime/execution_result``（String JSON）。仅当
   ``action`` 属于导航动作（``navigate_to_place`` / ``cancel_navigation``）
   时才播报，按 ``status`` / ``error_code`` 映射到对应话术：

   - SUCCEEDED              → “已经到达{place}。”
   - PLACE_NOT_FOUND        → “没有找到名为{place}的地点。”
   - NAV2_FAILED            → “导航失败，请检查道路。”
   - CANCELLED              → “已停止移动。”
   - BUSY / REJECTED        → “当前正在移动，请先停止当前导航。”

   导航执行器不会发布 ``/executor/done``，因此导航结果只会被播报一次。
   额外地，用最近 ``task_id`` 做短时去重，防止任何重复发布导致重播。

所有 JSON 解析都包裹在 ``try/except`` 中：异常消息一律跳过，绝不让节点崩溃。
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# ── 导航动作与状态（与 navigation_executor_node 保持一致）──────────────────
_NAV_ACTIONS = ("navigate_to_place", "cancel_navigation")
_STATUS_SUCCEEDED = "SUCCEEDED"
_STATUS_FAILED = "FAILED"
_STATUS_CANCELLED = "CANCELLED"
_STATUS_REJECTED = "REJECTED"

_ERR_PLACE_NOT_FOUND = "PLACE_NOT_FOUND"
_ERR_NAV2_FAILED = "NAV2_FAILED"
_ERR_CANCELLED = "CANCELLED"
_ERR_BUSY = "BUSY"

# task_id 去重保留时长（秒）。
_DEDUP_TTL_S = 60.0


class ExecutorDoneSayer(Node):
    """机械臂完成 + 导航结果的双通道语音播报。"""

    def __init__(self):
        super().__init__("executor_done_sayer")

        self.done_topic = self.declare_parameter(
            "done_topic", "/executor/done"
        ).get_parameter_value().string_value
        self.reply_topic = self.declare_parameter(
            "reply_topic", "/speech_reply"
        ).get_parameter_value().string_value
        self.success_text = self.declare_parameter(
            "success_text", "任务已完成。需要继续吗？"
        ).get_parameter_value().string_value
        self.execution_result_topic = self.declare_parameter(
            "execution_result_topic", "/runtime/execution_result"
        ).get_parameter_value().string_value

        self.pub_reply = self.create_publisher(String, self.reply_topic, 10)
        # 机械臂完成（保持原有行为不变）。
        self.sub_done = self.create_subscription(
            Bool, self.done_topic, self._on_done, 10
        )
        # 导航结果（仅播报导航动作）。
        self.sub_result = self.create_subscription(
            String, self.execution_result_topic, self._on_execution_result, 10
        )

        # task_id -> 最近播报时间戳，用于短时去重。
        self._announced: dict[str, float] = {}

        self.get_logger().info(
            f"✅ executor_done_sayer 启动: arm_done={self.done_topic}, "
            f"nav_result={self.execution_result_topic} -> {self.reply_topic}"
        )

    # ── 机械臂完成播报（原有行为）────────────────────────────────────────
    def _on_done(self, msg: Bool):
        if msg.data:
            self.pub_reply.publish(String(data=self.success_text))

    # ── 导航结果播报 ─────────────────────────────────────────────────────
    def _on_execution_result(self, msg: String):
        # JSON 异常绝不能让节点崩溃。
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"忽略无法解析的 execution_result: {exc}")
            return
        if not isinstance(data, dict):
            self.get_logger().warn("忽略非对象 execution_result")
            return

        action = str(data.get("action") or "").strip()
        if action not in _NAV_ACTIONS:
            # 非导航（例如机械臂）结果：交给 /executor/done 通道，这里不播报，
            # 避免对同一任务重复播报。
            return

        # 短时去重：同一 task_id 只播报一次。
        task_id = str(data.get("task_id") or "")
        now = time.time()
        if task_id and task_id in self._announced:
            self.get_logger().info(f"跳过重复 execution_result task_id={task_id}")
            self._prune_dedup(now)
            return
        if task_id:
            self._announced[task_id] = now
        self._prune_dedup(now)

        text = self._format_nav_text(data, action)
        if not text:
            return
        self.pub_reply.publish(String(data=text))
        self.get_logger().info(f"导航播报: {text}")

    @staticmethod
    def _format_nav_text(data: dict, action: str) -> str:
        """把导航结果 envelope 映射成一句话。返回空串表示不播报。"""
        status = str(data.get("status") or "").strip()
        # error_code 优先，回退到 error_detail（兼容旧 envelope）。
        error_code = str(
            data.get("error_code") or data.get("error_detail") or ""
        ).strip()
        success = bool(data.get("success"))
        place = str(
            data.get("place_name")
            or (data.get("evidence") or {}).get("place_name")
            or ""
        ).strip()

        if action == "cancel_navigation":
            # 取消动作：成功/已取消都播报“已停止移动”；取消服务本身失败才提示失败。
            if success or status == _STATUS_CANCELLED:
                return "已停止移动。"
            return "导航失败，请检查道路。"

        # navigate_to_place
        if status == _STATUS_SUCCEEDED or success:
            return f"已经到达{place}。" if place else "已经到达。"
        if error_code == _ERR_PLACE_NOT_FOUND:
            return f"没有找到名为{place}的地点。" if place else "没有找到该地点。"
        if error_code == _ERR_BUSY or status == _STATUS_REJECTED:
            return "当前正在移动，请先停止当前导航。"
        if status == _STATUS_CANCELLED or error_code == _ERR_CANCELLED:
            return "已停止移动。"
        # 其余失败统一提示检查道路。
        return "导航失败，请检查道路。"

    def _prune_dedup(self, now: float) -> None:
        """清理过期的去重记录，避免字典无限增长。"""
        stale = [tid for tid, t in self._announced.items() if now - t > _DEDUP_TTL_S]
        for tid in stale:
            self._announced.pop(tid, None)


def main(args=None):
    rclpy.init(args=args)
    node = ExecutorDoneSayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
