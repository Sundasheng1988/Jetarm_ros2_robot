#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import time
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class Case:
    text: str
    exp_action: Optional[str] = None
    exp_from: Optional[str] = None
    exp_to: Optional[str] = None
    note: str = ""  # 备注


class Tester(Node):
    def __init__(self, timeout_s: float = 5.0):
        super().__init__("llm_parser_regression_tester")
        self.pub = self.create_publisher(String, "/text_input", 10)
        self.sub = self.create_subscription(String, "/parsed_command", self._on_msg, 10)
        self.timeout_s = timeout_s
        self._last_json: Optional[Dict] = None

        # 等 ROS 2 话题发现完成，避免首条消息丢失
        self._wait_for_connections(timeout=8.0)

    def _wait_for_connections(self, timeout: float = 8.0):
        deadline = time.time() + timeout

        # 等待 /text_input 至少有一个订阅者（被测节点的订阅）
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 等待 /parsed_command 至少有一个发布者（被测节点的发布）
        while len(self.get_publishers_info_by_topic("/parsed_command")) == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        time.sleep(0.2)  # 保险歇一下

    def _on_msg(self, msg: String):
        try:
            obj = json.loads(msg.data)
            self._last_json = obj
        except Exception as e:
            self.get_logger().error(f"解析 /parsed_command JSON 失败: {e}\n原文: {msg.data}")

    def send_and_wait(self, text: str) -> Optional[Dict]:
        # 发送
        self._last_json = None
        self.pub.publish(String(data=text))
        self.get_logger().info(f"📤 发送: {text}")

        # 等待回包
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < self.timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_json is not None:
                return self._last_json
        return None


def assert_match(case: Case, got: Dict) -> Tuple[bool, str]:
    """仅校验 action/from/to（允许多余字段）"""
    exp: Dict[str, str] = {}
    if case.exp_action is not None:
        exp["action"] = case.exp_action
    if case.exp_from is not None:
        exp["from"] = case.exp_from
    if case.exp_to is not None:
        exp["to"] = case.exp_to

    for k, v in exp.items():
        gv = got.get(k)
        if gv != v:
            return False, f"{k} 期望 {v}，实际 {gv}"
    return True, ""


def main():
    rclpy.init()
    node = Tester(timeout_s=5.0)

    # 覆盖：指代记忆 + 基础解析 + 英文方位/容器
    cases: List[Case] = [
        Case("拿起黄色的杯子", exp_action="hold", exp_from="yellow_cup", exp_to="none", note="建立 last_object=yellow_cup"),
        Case("把这个放右边", exp_action="pick", exp_from="yellow_cup", exp_to="right_side"),
        Case("拿起蓝色的球", exp_action="hold", exp_from="blue_ball", exp_to="none", note="切换 last_object=blue_ball"),
        Case("把这个放左侧", exp_action="pick", exp_from="blue_ball", exp_to="left_side"),
        Case("移动到 front 的位置", exp_action="move", exp_from="unknown_object", exp_to="front_side"),
        Case("把这个放后面", exp_action="pick", exp_from="blue_ball", exp_to="back_side"),
        Case("把杯子搬到 bin a", exp_action="pick", exp_from="cup", exp_to="bin_a"),
        Case("把蓝色的球放到右边，谢谢～！", exp_action="pick", exp_from="blue_ball", exp_to="right_side"),
        Case("把杯子放到 middle", exp_action="pick", exp_from="cup", exp_to="center"),
        Case("移动到 back", exp_action="move", exp_from="unknown_object", exp_to="back_side"),
    ]

    total = len(cases)
    passed = 0
    fails = []

    for idx, c in enumerate(cases, 1):
        got = node.send_and_wait(c.text)

        # 首条用例若超时，可能是 DDS 发现尚未完全同步——重试 1 次
        if got is None and idx == 1:
            node.get_logger().warn("首条用例超时，可能是发现延迟，重试一次…")
            time.sleep(0.3)
            got = node.send_and_wait(c.text)

        if got is None:
            fails.append((idx, c, "超时未收到 /parsed_command"))
            node.get_logger().error(f"❌ 超时：{c.text}")
            continue

        ok, why = assert_match(c, got)
        if ok:
            passed += 1
            node.get_logger().info(f"✅ [{idx}/{total}] 通过：{c.text}")
        else:
            fails.append((idx, c, why))
            node.get_logger().error(
                f"❌ [{idx}/{total}] 失败：{c.text}\n  原因: {why}\n  实际: {json.dumps(got, ensure_ascii=False)}"
            )

        time.sleep(0.2)  # 节流

    print("\n================ 回归结果 ================")
    print(f"通过 {passed}/{total}")
    if fails:
        print("失败明细：")
        for idx, c, why in fails:
            print(f"- #{idx}『{c.text}』→ {why}")

    rclpy.shutdown()
    import sys
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()

