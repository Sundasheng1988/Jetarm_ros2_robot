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


class RealGroundedRuntimeNode(Node):
    def __init__(self):
        super().__init__("real_grounded_runtime_node")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("run_once", True)
        self.declare_parameter("input_topic", "/grounded_task_context")

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.run_once = bool(self.get_parameter("run_once").value)
        self.input_topic = str(self.get_parameter("input_topic").value)

        self._has_run = False
        self._busy = False

        self.pub_state = self.create_publisher(String, "/runtime/state", 10)
        self.pub_log = self.create_publisher(String, "/runtime/log", 10)
        self.pub_result = self.create_publisher(
            String, "/runtime/execution_result", 10
        )
        self.done_pub = self.create_publisher(Bool, "/executor/done", 10)

        self.adapter = RuntimeAdapter(node=self, dry_run=self.dry_run)
        self.skill_mgr = SkillManager(adapter=self.adapter)
        SkillRegistry.register(PickSkill)

        self.create_subscription(
            String, self.input_topic, self._on_grounded_task, 10
        )

        mode_label = "ONCE" if self.run_once else "CONTINUOUS"
        dry_label = "DRY_RUN" if self.dry_run else "REAL_HARDWARE"
        self.get_logger().info("=" * 60)
        self.get_logger().info(f" RealGroundedRuntimeNode — {dry_label} — {mode_label}")
        self.get_logger().info(f" input_topic   = {self.input_topic}")
        self.get_logger().info(f" dry_run       = {self.dry_run}")
        self.get_logger().info(f" run_once      = {self.run_once}")
        self.get_logger().info(" Registered skills: " + str(SkillRegistry.list_all()))
        if not self.dry_run:
            self.get_logger().warn(
                "=" * 60 + "\n"
                " WARNING: dry_run=false — REAL HARDWARE may be controlled.\n"
                " Confirm servo_controller and IK service are available and safe.\n"
                + "=" * 60
            )
        self.get_logger().info("=" * 60)

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

    def _on_grounded_task(self, msg: String):
        if self.run_once and self._has_run:
            return

        if self._busy:
            self.get_logger().warn("busy — skipping incoming grounded_task_context")
            return

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"JSON parse failed: {e}")
            return

        self._busy = True
        try:
            self._execute_task(data)
        except Exception as e:
            self.get_logger().error(f"Task execution exception: {e}")
        finally:
            self._busy = False

    def _execute_task(self, data: dict):
        status = data.get("status", "")
        if status not in ("ok", ""):
            self.get_logger().info(f"skip — status={status}")
            return

        intent = data.get("intent", "")
        parsed = data.get("parsed_command") or {}
        tgt_obj = data.get("target_object")
        tgt_pose = data.get("target_pose")

        if not tgt_obj:
            self.get_logger().info("skip — no target_object in grounded_task_context")
            return

        self.get_logger().info(f"\n[>] received intent={intent}")

        # Build TaskContext
        ctx = TaskBuilder.from_parsed_command(
            parsed, source="grounding"
        )
        grounded = {
            "intent": intent,
            "object_hints": {
                "class": tgt_obj.get("class_name", ""),
                "color": tgt_obj.get("color", ""),
            },
            "source_pose": {
                "frame": tgt_obj.get("world_frame", "base"),
                "xyz": [
                    tgt_obj.get("world_x", 0.0),
                    tgt_obj.get("world_y", 0.0),
                    tgt_obj.get("world_z", 0.0),
                ],
                "rpy": [0.0, 0.0, 0.0],
            },
            "target_pose": tgt_pose,
            "status": "ok",
        }
        ctx = TaskBuilder.from_grounded_goal(grounded, ctx)
        self._emit_state(ctx)
        self._emit_log(ctx, "grounded_task_received", {"intent": intent})

        # Skill selection
        skill_name = self.skill_mgr.select(ctx)
        self.get_logger().info(f"  skill = {skill_name}")
        if skill_name == "unknown_skill":
            self._emit_log(ctx, "no_skill", {"intent": intent})
            return
        ctx.selected_skill = skill_name
        ctx.transition(TaskState.SKILL_SELECTED)
        self._emit_state(ctx)

        # Instantiate
        skill = self.skill_mgr.instantiate(skill_name)
        if skill is None:
            self.get_logger().error("skill instantiation returned None")
            return

        # Precheck
        pre = skill.precheck(ctx)
        if pre is not None:
            self.get_logger().warn(f"precheck blocked: {pre.reason}")
            self._emit_log(ctx, "precheck_failed", pre.to_dict())
            return

        # Execute
        ctx.transition(TaskState.EXECUTING)
        self._emit_state(ctx)
        self._emit_log(ctx, "execution_started", {"skill": skill_name})

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(skill.execute(ctx))
        except Exception as e:
            result = ExecutionResult(
                task_id=ctx.task_id,
                success=False,
                reason="exception",
                error_detail=str(e),
            )
        finally:
            loop.close()

        # Postcheck + cleanup
        result = skill.postcheck(ctx, result)
        skill.cleanup(ctx)

        if result.success:
            ctx.transition(TaskState.DONE)
        else:
            ctx.transition(TaskState.FAILED)
        ctx.result = result.to_dict()
        self._emit_state(ctx)
        self._emit_log(ctx, "execution_result", result.to_dict())
        self.pub_result.publish(
            String(data=json.dumps(result.to_dict(), ensure_ascii=False))
        )
        self.done_pub.publish(Bool(data=result.success))

        self.get_logger().info(
            f"[done] success={result.success} reason={result.reason} "
            f"elapsed_ms={ctx.elapsed_ms:.1f}"
        )

        # Adapter call log
        for i, call in enumerate(self.adapter._call_log[-8:]):
            self.get_logger().info(
                f"  adapter[{i+1}] {call['method']}: "
                f"{json.dumps(call['args'], ensure_ascii=False)}"
            )

        self._has_run = True
        self.adapter._call_log.clear()


def main(args=None):
    rclpy.init(args=args)
    node = RealGroundedRuntimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
