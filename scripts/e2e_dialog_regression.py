#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, time, threading
from typing import Tuple, Callable, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

PARSED_TOPIC   = "/parsed_command"
GROUNDED_TOPIC = "/grounded_goal"
INPUT_TOPIC    = "/text_input"

TIMEOUT_S = 6.0  # 单步超时更宽松
SLEEP_S   = 0.05

def pretty(o): return json.dumps(o, ensure_ascii=False)

class E2ETester(Node):
    def __init__(self):
        super().__init__("llm_parser_regression_tester")
        self.pub = self.create_publisher(String, INPUT_TOPIC, 10)

        self._last_parsed: Optional[Dict[str, Any]] = None
        self._last_goal:   Optional[Dict[str, Any]] = None
        self._lock = threading.Lock()

        self.sub_parsed = self.create_subscription(String, PARSED_TOPIC, self._on_parsed, 10)
        self.sub_goal   = self.create_subscription(String, GROUNDED_TOPIC, self._on_goal, 10)

        self.cli_clear_ground = self.create_client(Trigger, "/grounding/clear_memory")
        self.cli_clear_llm    = self.create_client(Trigger, "/llm_parser/clear_memory")  # 可能不存在，容错

        self.get_logger().info("🧪 启动：端到端对话回归测试")

    # ---------- topics ----------
    def _on_parsed(self, msg: String):
        try:
            d = json.loads(msg.data)
            with self._lock:
                self._last_parsed = d
        except Exception:
            pass

    def _on_goal(self, msg: String):
        try:
            d = json.loads(msg.data)
            with self._lock:
                self._last_goal = d
        except Exception:
            pass

    def flush(self):
        with self._lock:
            self._last_parsed = None
            self._last_goal = None

    def send_and_wait(self, text: str) -> Tuple[Optional[Dict[str,Any]], Optional[Dict[str,Any]]]:
        self.flush()
        self.get_logger().info(f"📤 发送: {text}")
        self.pub.publish(String(data=text))
        t0 = time.time()
        seen_parsed = seen_goal = None
        while time.time() - t0 < TIMEOUT_S:
            with self._lock:
                seen_parsed = self._last_parsed
                seen_goal   = self._last_goal
            if seen_parsed is not None and seen_goal is not None:
                return seen_parsed, seen_goal
            time.sleep(SLEEP_S)
        return seen_parsed, seen_goal

    # ---------- services ----------
    def _wait_service(self, cli, name: str, timeout=2.0):
        if not cli:
            return False
        if cli.service_is_ready():
            return True
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"{name} 不可用，跳过。")
            return False
        return True

    def clear_grounding(self):
        if self._wait_service(self.cli_clear_ground, "grounding/clear_memory"):
            fut = self.cli_clear_ground.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def clear_llm(self):
        if self._wait_service(self.cli_clear_llm, "llm_parser/clear_memory"):
            fut = self.cli_clear_llm.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

# ---------- 断言工具 ----------
def ok(cond, msg="ok"): return True, msg
def fail(msg): return False, msg

def expect_illegal_place(parsed, goal):
    # 非法地点：parser 应该给 to=none；grounding 报 no_target
    if not parsed or not goal:
        return fail("缺少 parsed/goal")
    if parsed.get("action") not in ("pick","hold","pick"):
        return fail(f"action 异常: {parsed.get('action')}")
    if parsed.get("to") != "none":
        return fail(f"非法地点应 to=none，实际 {parsed.get('to')}")
    if goal.get("status") != "no_target":
        return fail(f"goal.status 应 no_target，实际 {goal.get('status')}")
    return ok("illegal place OK")

def expect_unknown_object(parsed, goal):
    # 世界模型没有该物体：应 no_match，object_id=-1
    if not parsed or not goal:
        return fail("缺少 parsed/goal")
    if parsed.get("from") != "green_ball":
        return fail(f"from 应 green_ball，实际 {parsed.get('from')}")
    if goal.get("status") != "no_match":
        return fail(f"goal.status 应 no_match，实际 {goal.get('status')}")
    if goal.get("object_id") not in (-1, None):
        return fail(f"object_id 应 -1/None，实际 {goal.get('object_id')}")
    return ok("unknown object OK")

def expect_pick_to(parsed, goal, from_tok, to_tok):
    if not parsed or not goal:
        return fail("缺少 parsed/goal")
    if parsed.get("action") != "pick":
        return fail(f"action 应 pick，实际 {parsed.get('action')}")
    if parsed.get("from") != from_tok:
        return fail(f"from 应 {from_tok}，实际 {parsed.get('from')}")
    if parsed.get("to") != to_tok:
        return fail(f"to 应 {to_tok}，实际 {parsed.get('to')}")
    if goal.get("intent") != "pick":
        return fail(f"goal.intent 应 pick，实际 {goal.get('intent')}")
    if not goal.get("target_pose"):
        return fail("goal.target_pose 缺失")
    return ok("pick_to OK")

def expect_hold_from(parsed, goal, from_tok):
    if not parsed or not goal:
        return fail("缺少 parsed/goal")
    if parsed.get("action") != "hold":
        return fail(f"action 应 hold，实际 {parsed.get('action')}")
    if parsed.get("from") != from_tok:
        return fail(f"from 应 {from_tok}，实际 {parsed.get('from')}")
    if goal.get("intent") != "hold":
        return fail(f"goal.intent 应 hold，实际 {goal.get('intent')}")
    return ok("hold_from OK")

def expect_move_to(parsed, goal, to_tok):
    if not parsed or not goal:
        return fail("缺少 parsed/goal")
    if parsed.get("action") != "move":
        return fail(f"action 应 move，实际 {parsed.get('action')}")
    if parsed.get("to") != to_tok:
        return fail(f"to 应 {to_tok}，实际 {parsed.get('to')}")
    if goal.get("intent") != "move":
        return fail(f"goal.intent 应 move，实际 {goal.get('intent')}")
    if not goal.get("target_pose"):
        return fail("goal.target_pose 缺失")
    return ok("move_to OK")

def run():
    rclpy.init()
    node = E2ETester()

    # 开始前清内存（grounding 与 llm_parser 都清）
    node.clear_grounding()
    node.clear_llm()

    tests = [
        # 1 hold yellow cup
        ("拿起黄色的杯子",
         lambda p,g: expect_hold_from(p,g,"yellow_cup"),
         "hold_yellow_cup"),

        # 2 pronoun: place right
        ("把它放右边",
         lambda p,g: expect_pick_to(p,g,"yellow_cup","right_side"),
         "pronoun_place_right"),

        # 3 预热：拿起蓝色的球（为了后续“它”指代）
        ("拿起蓝色的球",
         lambda p,g: expect_hold_from(p,g,"blue_ball"),
         "preheat_blue_ball"),

        # 4 pronoun: place left
        ("把它放到左侧",
         lambda p,g: expect_pick_to(p,g,"blue_ball","left_side"),
         "blue_ball_left"),

        # 5 move front
        ("移动到 front 的位置",
         lambda p,g: expect_move_to(p,g,"front_side"),
         "move_front"),

        # 6 cup -> bin a
        ("把杯子搬到 bin a",
         lambda p,g: expect_pick_to(p,g,"cup","bin_a"),
         "cup_to_bina"),

        # 7 polite blue ball -> right
        ("把蓝色的球放到右边，谢谢～！",
         lambda p,g: expect_pick_to(p,g,"blue_ball","right_side"),
         "blue_ball_right_polite"),

        # 8 cup -> middle(center)
        ("把杯子放到 middle",
         lambda p,g: expect_pick_to(p,g,"cup","center"),
         "cup_to_middle"),

        # 9 move back
        ("移动到 back",
         lambda p,g: expect_move_to(p,g,"back_side"),
         "move_back"),

        # 10 非法地点（应 no_target）
        ("把红色的杯子放到月球",
         expect_illegal_place,
         "illegal_place"),

        # 11 未知物体（应 no_match）
        ("拿起绿色的球",
         expect_unknown_object,
         "unknown_object"),

        # 12 预热后，代词放中间（可选：若想与你的旧 11 步保持一致可注释掉本条）
        ("拿起黄色的杯子",
         lambda p,g: expect_hold_from(p,g,"yellow_cup"),
         "preheat_yellow"),
        ("把这个放中间",
         lambda p,g: expect_pick_to(p,g,"yellow_cup","center"),
         "pronoun_center"),
    ]

    passed = 0; total = len(tests)
    for i,(text,check,name) in enumerate(tests, start=1):
        parsed, goal = node.send_and_wait(text)
        if parsed is None or goal is None:
            node.get_logger().error(f"❌ 超时或缺失话题：parsed={parsed is not None}, goal={goal is not None}")
            continue
        ok, msg = check(parsed, goal)
        if ok:
            node.get_logger().info(f"✅ 通过：{name}")
            passed += 1
        else:
            node.get_logger().error(f"❌ 失败：{name}\n 解析: {pretty(parsed)}\n 对齐: {pretty(goal)}\n 原因: {msg}")

    node.get_logger().info(f"\n================ 回归结果 ================\n通过 {passed}/{total}\n")
    rclpy.shutdown()

if __name__ == "__main__":
    run()

