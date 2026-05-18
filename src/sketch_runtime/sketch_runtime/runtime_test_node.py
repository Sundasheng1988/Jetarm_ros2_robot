#!/usr/bin/env python3
import json
import time
import asyncio

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from sketch_runtime.task_context import TaskState, TaskContext
from sketch_runtime.execution_result import ExecutionResult
from sketch_runtime.skill_registry import SkillRegistry, SkillManager
from sketch_runtime.runtime_adapter import RuntimeAdapter
from sketch_runtime.runtime_task_builder import TaskBuilder
from sketch_runtime.skills.pick_skill import PickSkill


class RuntimeTestNode(Node):
    def __init__(self):
        super().__init__("runtime_test_node")

        self.declare_parameter("test_command", "pick red cup to right side")
        self.declare_parameter("hover_height", 0.08)
        self.declare_parameter("approach_z", 0.015)
        self.declare_parameter("lift_height", 0.08)
        self.declare_parameter("auto_confirm", True)

        self.test_command = str(
            self.get_parameter("test_command").value
        )
        self.hover_height = float(self.get_parameter("hover_height").value)
        self.approach_z = float(self.get_parameter("approach_z").value)
        self.lift_height = float(self.get_parameter("lift_height").value)
        self.auto_confirm = bool(self.get_parameter("auto_confirm").value)

        # Runtime outputs
        self.pub_state = self.create_publisher(String, "/runtime/state", 10)
        self.pub_log = self.create_publisher(String, "/runtime/log", 10)
        self.pub_result = self.create_publisher(
            String, "/runtime/execution_result", 10
        )
        self.done_pub = self.create_publisher(Bool, "/executor/done", 10)

        # Adapter + SkillManager
        self.adapter = RuntimeAdapter(node=self, dry_run=True)
        self.skill_mgr = SkillManager(adapter=self.adapter)
        SkillRegistry.register(PickSkill)

        self.get_logger().info("=" * 60)
        self.get_logger().info(" RuntimeTestNode started — DRY_RUN mode")
        self.get_logger().info(f" test_command  = {self.test_command}")
        self.get_logger().info(f" hover_height = {self.hover_height}")
        self.get_logger().info(f" approach_z   = {self.approach_z}")
        self.get_logger().info(f" lift_height  = {self.lift_height}")
        self.get_logger().info(" Registered skills: " + str(SkillRegistry.list_all()))
        self.get_logger().info("=" * 60)

        self.create_timer(1.0, self.run_test)

    def _emit_state(self, ctx: TaskContext):
        self.pub_state.publish(
            String(data=json.dumps(ctx.to_dict(), ensure_ascii=False))
        )
        self.get_logger().info(f"[state] {ctx.state.value}")

    def _emit_log(self, ctx: TaskContext, event: str, data: dict = None):
        msg = {
            "task_id": ctx.task_id,
            "event": event,
            "data": data or {},
            "timestamp": time.time(),
        }
        self.pub_log.publish(String(data=json.dumps(msg, ensure_ascii=False)))
        self.get_logger().info(f"[log] {event} -> {json.dumps(data, ensure_ascii=False)[:120]}")

    def run_test(self):
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(" BEGIN FULL RUNTIME CHAIN TEST (dry_run)")
        self.get_logger().info("=" * 60)

        # ---- Step 1: Simulate parsed_command ----
        self.get_logger().info("\n[1/7] Simulating parsed_command ...")
        parsed = {
            "action": "pick",
            "from": "red_cup",
            "to": "right_side",
            "raw": self.test_command,
            "source": "test",
        }
        ctx = TaskBuilder.from_parsed_command(parsed)
        self._emit_state(ctx)
        self._emit_log(ctx, "parsed_command", {"parsed": parsed})
        self.get_logger().info(f"       task_id = {ctx.task_id}")
        self.get_logger().info(f"       state   = {ctx.state.value}")

        # ---- Step 2: Simulate grounded_goal ----
        self.get_logger().info("\n[2/7] Simulating grounded_goal ...")
        grounded = {
            "intent": "pick",
            "object_hints": {"class": "cup", "color": "red"},
            "object_id": 42,
            "source_pose": {
                "frame": "table",
                "xyz": [0.15, -0.10, 0.03],
                "rpy": [0.0, 0.0, 1.57],
            },
            "target_pose": {
                "frame": "table",
                "xyz": [0.20, -0.15, 0.02],
                "rpy": [0.0, 0.0, 1.57],
            },
            "status": "ok",
        }
        ctx = TaskBuilder.from_grounded_goal(grounded, ctx)
        ctx.skill_params = {
            "hover_height": self.hover_height,
            "approach_z": self.approach_z,
            "lift_height": self.lift_height,
        }
        self._emit_state(ctx)
        self._emit_log(ctx, "grounded_goal", {
            "object_hints": grounded["object_hints"],
            "object_id": grounded["object_id"],
        })
        self.get_logger().info(f"       state       = {ctx.state.value}")
        self.get_logger().info(f"       target_obj  = {ctx.target_object.get('class_name','?')} {ctx.target_object.get('color','?')}")
        self.get_logger().info(f"       target_pose = {ctx.target_pose}")

        # ---- Step 3: SkillManager select ----
        self.get_logger().info("\n[3/7] SkillManager.select ...")
        skill_name = self.skill_mgr.select(ctx)
        self._emit_log(ctx, "skill_selection", {"selected": skill_name})
        self.get_logger().info(f"       intent      = pick")
        self.get_logger().info(f"       skill_name  = {skill_name}")

        if skill_name == "unknown_skill":
            self.get_logger().error("FAIL: no skill found for intent")
            return

        ctx.selected_skill = skill_name
        ctx.transition(TaskState.SKILL_SELECTED)
        self._emit_state(ctx)

        # ---- Step 4: SkillManager instantiate ----
        self.get_logger().info("\n[4/7] SkillManager.instantiate ...")
        skill = self.skill_mgr.instantiate(skill_name)
        if skill is None:
            self.get_logger().error("FAIL: skill instantiation returned None")
            return
        self.get_logger().info(f"       skill class = {skill.__class__.__name__}")
        self.get_logger().info(f"       skill name  = {skill.name}")

        # ---- Step 5: precheck ----
        self.get_logger().info("\n[5/7] skill.precheck ...")
        pre = skill.precheck(ctx)
        if pre is not None:
            self.get_logger().error(f"FAIL: precheck blocked — {pre.reason}")
            self._emit_log(ctx, "precheck_failed", pre.to_dict())
            return
        self.get_logger().info("       precheck OK (passed)")

        # ---- Step 6: execute ----
        self.get_logger().info("\n[6/7] skill.execute ...")
        self.get_logger().info("       (all servo/IK calls are dry_run — no hardware)")
        ctx.transition(TaskState.EXECUTING)
        self._emit_state(ctx)
        self._emit_log(ctx, "execution_started", {"skill": skill_name})

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(skill.execute(ctx))
        except Exception as e:
            result = ExecutionResult(task_id=ctx.task_id, success=False,
                                     reason="exception", error_detail=str(e))
        finally:
            loop.close()

        self.get_logger().info(f"       result.success   = {result.success}")
        self.get_logger().info(f"       result.reason    = {result.reason}")
        self.get_logger().info(f"       evidence         = {json.dumps(result.evidence, ensure_ascii=False)}")

        # ---- Step 7: postcheck + done signal ----
        self.get_logger().info("\n[7/7] postcheck + publish /executor/done ...")
        result = skill.postcheck(ctx, result)
        skill.cleanup(ctx)

        if result.success:
            ctx.transition(TaskState.DONE)
        else:
            ctx.transition(TaskState.FAILED)
        ctx.result = result.to_dict()
        self._emit_state(ctx)
        self._emit_log(ctx, "execution_result", result.to_dict())
        self.pub_result.publish(String(data=json.dumps(result.to_dict(), ensure_ascii=False)))
        self.done_pub.publish(Bool(data=True))

        # ---- Summary ----
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(" RUNTIME TEST COMPLETE")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  task_id    = {ctx.task_id}")
        self.get_logger().info(f"  state      = {ctx.state.value}")
        self.get_logger().info(f"  success    = {result.success}")
        self.get_logger().info(f"  reason     = {result.reason}")
        self.get_logger().info(f"  elapsed_ms = {ctx.elapsed_ms:.1f}")
        self.get_logger().info("")
        self.get_logger().info(" Adapter call log:")
        for i, call in enumerate(self.adapter._call_log):
            self.get_logger().info(
                f"  [{i+1}] {call['method']}: {json.dumps(call['args'], ensure_ascii=False)}"
            )
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
