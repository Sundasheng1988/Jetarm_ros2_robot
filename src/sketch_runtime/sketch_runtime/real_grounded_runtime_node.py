#!/usr/bin/env python3
import json
import time
import asyncio
from typing import Optional

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
        self.declare_parameter("require_confirm", True)
        self.declare_parameter("confirm_timeout_sec", 300.0)
        self.declare_parameter("enable_real_ik", False)
        self.declare_parameter("enable_real_servo", False)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.run_once = bool(self.get_parameter("run_once").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.require_confirm = bool(self.get_parameter("require_confirm").value)
        self.confirm_timeout_sec = float(
            self.get_parameter("confirm_timeout_sec").value
        )
        self.enable_real_ik = bool(self.get_parameter("enable_real_ik").value)
        self.enable_real_servo = bool(self.get_parameter("enable_real_servo").value)

        self._has_run = False
        self._busy = False
        self._pending_ctx = None
        self._pending_skill = None
        self._pending_deadline = 0.0
        self._active_ctx: Optional[TaskContext] = None

        self.pub_state = self.create_publisher(String, "/runtime/state", 10)
        self.pub_log = self.create_publisher(String, "/runtime/log", 10)
        self.pub_result = self.create_publisher(
            String, "/runtime/execution_result", 10
        )
        self.done_pub = self.create_publisher(Bool, "/executor/done", 10)
        self.preview_pub = self.create_publisher(
            String, "/runtime/preview", 10
        )

        self.adapter = RuntimeAdapter(
            node=self, dry_run=self.dry_run,
            enable_real_ik=self.enable_real_ik,
            enable_real_servo=self.enable_real_servo,
        )
        self.skill_mgr = SkillManager(adapter=self.adapter)
        SkillRegistry.register(PickSkill)

        self.create_subscription(
            String, self.input_topic, self._on_grounded_task, 10
        )
        self.create_subscription(
            String, "/runtime/confirm", self._on_confirm, 10
        )
        self.create_subscription(
            String, "/runtime/verification_result", self._on_verification_result, 10
        )

        self._confirm_timer = self.create_timer(0.5, self._check_confirm_timeout)

        mode_label = "ONCE" if self.run_once else "CONTINUOUS"
        dry_label = "DRY_RUN" if self.dry_run else "REAL_HARDWARE"
        confirm_label = "CONFIRM_REQUIRED" if self.require_confirm else "AUTO_EXECUTE"
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f" RealGroundedRuntimeNode — {dry_label} — {mode_label} — {confirm_label}"
        )
        self.get_logger().info(f" input_topic         = {self.input_topic}")
        self.get_logger().info(f" dry_run             = {self.dry_run}")
        self.get_logger().info(f" enable_real_ik      = {self.enable_real_ik}")
        self.get_logger().info(f" enable_real_servo   = {self.enable_real_servo}")
        self.get_logger().info(f" run_once            = {self.run_once}")
        self.get_logger().info(f" require_confirm     = {self.require_confirm}")
        self.get_logger().info(f" confirm_timeout_sec = {self.confirm_timeout_sec}")
        self.get_logger().info(f" adapter mode        = {self.adapter.mode_summary()}")
        self.get_logger().info(
            " Registered skills: " + str(SkillRegistry.list_all())
        )
        if not self.dry_run:
            self.get_logger().warn(
                "=" * 60 + "\n"
                " WARNING: dry_run=false — REAL HARDWARE may be controlled.\n"
                " Confirm servo_controller and IK service are available and safe.\n"
                + "=" * 60
            )
        if self.require_confirm:
            self.get_logger().info(
                " Confirm via: ros2 topic pub --once /runtime/confirm "
                "std_msgs/msg/String 'data: "
                "'{\"task_id\":\"<ID>\",\"confirm\":true}' '"
            )
        self.get_logger().info("=" * 60)

    # ── publishers ──

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

    def _publish_preview(self, ctx: TaskContext):
        preview = {
            "task_id": ctx.task_id,
            "intent": (ctx.parsed_command or {}).get("action", ""),
            "selected_skill": ctx.selected_skill,
            "target_object": ctx.target_object,
            "target_pose": ctx.target_pose,
            "dry_run": self.dry_run,
            "require_confirm": self.require_confirm,
            "summary": (
                f"{ctx.selected_skill}: "
                f"{ctx.target_object.get('class_name','?') if ctx.target_object else '?'}"
                f" ({ctx.target_object.get('color','?') if ctx.target_object else '?'})"
            ),
            "status": "waiting_confirm",
            "timestamp": time.time(),
        }
        self.preview_pub.publish(
            String(data=json.dumps(preview, ensure_ascii=False))
        )
        self.get_logger().info(
            f"[preview] task_id={ctx.task_id} skill={ctx.selected_skill} "
            f"waiting for /runtime/confirm"
        )

    # ── incoming grounded_task_context ──

    def _on_grounded_task(self, msg: String):
        if self.run_once and self._has_run:
            return

        if self._busy or self._pending_ctx is not None:
            self.get_logger().warn(
                "busy or pending confirm — skipping incoming grounded_task_context"
            )
            return

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"JSON parse failed: {e}")
            return

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

        ctx = TaskBuilder.from_parsed_command(parsed, source="grounding")
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
                "rpy": [0.0, 0.0, tgt_obj.get("world_yaw", 0.0)],
            },
            "target_pose": tgt_pose,
            "status": "ok",
        }
        ctx = TaskBuilder.from_grounded_goal(grounded, ctx)
        self._emit_state(ctx)
        self._emit_log(ctx, "grounded_task_received", {"intent": intent})

        skill_name = self.skill_mgr.select(ctx)
        self.get_logger().info(f"  skill = {skill_name}")
        if skill_name == "unknown_skill":
            self._emit_log(ctx, "no_skill", {"intent": intent})
            return
        ctx.selected_skill = skill_name
        ctx.transition(TaskState.SKILL_SELECTED)
        self._emit_state(ctx)

        skill = self.skill_mgr.instantiate(skill_name)
        if skill is None:
            self.get_logger().error("skill instantiation returned None")
            return

        pre = skill.precheck(ctx)
        if pre is not None:
            self.get_logger().warn(f"precheck blocked: {pre.reason}")
            self._emit_log(ctx, "precheck_failed", pre.to_dict())
            return

        if self.require_confirm:
            self._publish_preview(ctx)
            ctx.transition(TaskState.WAITING_CONFIRM)
            self._emit_state(ctx)
            self._pending_ctx = ctx
            self._pending_skill = skill
            self._pending_deadline = time.time() + self.confirm_timeout_sec
            self.get_logger().info(
                f"[confirm] waiting for /runtime/confirm "
                f"(timeout={self.confirm_timeout_sec}s)"
            )
        else:
            self._publish_preview(ctx)
            self._execute_skill(ctx, skill)

    # ── confirm ──

    def _on_confirm(self, msg: String):
        if self._pending_ctx is None:
            return

        raw = (msg.data or "").strip()

        # 1) Try JSON confirm with task_id
        try:
            data = json.loads(raw)
        except Exception:
            data = None

        if data is not None and isinstance(data, dict) and "task_id" in data:
            confirm_id = data.get("task_id", "")
            if confirm_id != self._pending_ctx.task_id:
                return
            confirmed = bool(data.get("confirm", False))
        else:
            # 2) Simple string confirm — applies to current pending task only
            lower = raw.lower()
            CONFIRM = {"yes", "y", "true", "ok", "confirm", "go"}
            CANCEL  = {"no", "n", "false", "cancel", "stop"}
            if lower in CONFIRM:
                confirmed = True
            elif lower in CANCEL:
                confirmed = False
            else:
                self.get_logger().warn(
                    f"[confirm] unrecognized input: '{raw}'"
                )
                return

        ctx = self._pending_ctx
        skill = self._pending_skill
        self._pending_ctx = None
        self._pending_skill = None

        if confirmed:
            self.get_logger().info(
                f"[confirm] task_id={ctx.task_id} CONFIRMED — executing"
            )
            self._emit_log(ctx, "user_confirmed")
            self._execute_skill(ctx, skill)
        else:
            self.get_logger().info(
                f"[confirm] task_id={ctx.task_id} CANCELLED by user"
            )
            self._cancel_task(ctx)

    def _check_confirm_timeout(self):
        if self._pending_ctx is None:
            return
        if time.time() < self._pending_deadline:
            return

        ctx = self._pending_ctx
        self._pending_ctx = None
        self._pending_skill = None
        self.get_logger().warn(
            f"[confirm] task_id={ctx.task_id} TIMEOUT after "
            f"{self.confirm_timeout_sec}s"
        )
        self._cancel_task(ctx)

    def _cancel_task(self, ctx: TaskContext):
        ctx.transition(TaskState.CANCELLED)
        ctx.result = {
            "success": False,
            "reason": "cancelled_by_user",
            "evidence": {},
        }
        self._emit_state(ctx)
        self._emit_log(ctx, "execution_cancelled", {"reason": "user_cancelled_or_timeout"})
        self.done_pub.publish(Bool(data=False))
        self.get_logger().info(f"[cancel] task_id={ctx.task_id}")
        self._has_run = True
        self._active_ctx = None

    # ── execution ──

    def _execute_skill(self, ctx: TaskContext, skill):
        self._busy = True
        try:
            ctx.transition(TaskState.EXECUTING)
            self._emit_state(ctx)
            self._emit_log(ctx, "execution_started", {"skill": ctx.selected_skill})

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

            result = skill.postcheck(ctx, result)
            skill.cleanup(ctx)

            if result.success:
                ctx.transition(TaskState.VERIFYING)
                self._emit_state(ctx)
                self._emit_log(ctx, "verification_started", {"skill": ctx.selected_skill})
                self._active_ctx = ctx
                self.done_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f"[verify] task_id={ctx.task_id} — postcheck triggered, "
                    f"waiting for verification_result"
                )
                self._has_run = True
                self.adapter._call_log.clear()
                return
            else:
                ctx.transition(TaskState.FAILED)
                ctx.result = result.to_dict()
                self._emit_state(ctx)
                self._emit_log(ctx, "execution_result", result.to_dict())
                self.pub_result.publish(
                    String(data=json.dumps(result.to_dict(), ensure_ascii=False))
                )
                self.done_pub.publish(Bool(data=False))
                self.get_logger().info(
                    f"[done] success=False reason={result.reason}"
                )
                self._has_run = True
                self.adapter._call_log.clear()

            for i, call in enumerate(self.adapter._call_log[-8:]):
                self.get_logger().info(
                    f"  adapter[{i+1}] {call['method']}: "
                    f"{json.dumps(call['args'], ensure_ascii=False)}"
                )

        finally:
            self._busy = False

    # ── verification result ──

    def _on_verification_result(self, msg: String):
        if self._active_ctx is None:
            return

        try:
            data = json.loads(msg.data)
        except Exception:
            return

        stage = data.get("stage", "")
        if stage != "postcheck":
            return

        ctx = self._active_ctx
        if ctx.state != TaskState.VERIFYING:
            return

        success = bool(data.get("success", False))
        self.get_logger().info(
            f"[verify] task_id={ctx.task_id} postcheck success={success}"
        )

        if ctx.result is None:
            ctx.result = {}

        ctx.result["verification"] = data

        if success:
            ctx.transition(TaskState.VERIFIED)
        else:
            ctx.transition(TaskState.VERIFICATION_FAILED)

        self._emit_state(ctx)
        self._emit_log(ctx, "verification_complete", {"success": success})
        self._active_ctx = None


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
