import json
import time
from typing import Optional, List, Dict, Any

from sketch_runtime.task_context import TaskState, TaskContext
from sketch_runtime.target_object import TargetObject


class TaskBuilder:
    @staticmethod
    def _safe_str(val: Any, default: str = "") -> str:
        return str(val) if val is not None else default

    @staticmethod
    def _safe_float(val: Any, default: float = 0.0) -> float:
        try:
            return float(val)
        except (TypeError, ValueError):
            return default

    @classmethod
    def from_parsed_command(cls, parsed: dict, source: str = "runtime") -> TaskContext:
        ctx = TaskContext(
            user_command=cls._safe_str(parsed.get("raw") or parsed.get("text")),
            source=parsed.get("source", source),
            parsed_command=parsed,
        )
        ctx.transition(TaskState.PARSED)
        return ctx

    @classmethod
    def from_grounded_goal(cls, grounded: dict,
                           ctx: Optional[TaskContext] = None) -> TaskContext:
        if ctx is None:
            ctx = TaskContext(
                user_command=cls._safe_str(grounded.get("detail")),
                source="grounding",
            )

        intent = cls._safe_str(grounded.get("intent"))
        src_pose = grounded.get("source_pose")
        tgt_pose = grounded.get("target_pose")
        hints = grounded.get("object_hints") or {}

        to = TargetObject(
            class_name=cls._safe_str(hints.get("class")),
            color=cls._safe_str(hints.get("color")),
            source="grounding",
        )
        if src_pose:
            to.world_frame = cls._safe_str(
                src_pose.get("frame"), "base"
            )
            xyz = src_pose.get("xyz") or [0, 0, 0]
            rpy = src_pose.get("rpy") or [0, 0, 0]
            to.world_x = cls._safe_float(xyz[0]) if len(xyz) > 0 else 0.0
            to.world_y = cls._safe_float(xyz[1]) if len(xyz) > 1 else 0.0
            to.world_z = cls._safe_float(xyz[2]) if len(xyz) > 2 else 0.0
            to.world_roll = cls._safe_float(rpy[0]) if len(rpy) > 0 else 0.0
            to.world_pitch = cls._safe_float(rpy[1]) if len(rpy) > 1 else 0.0
            to.world_yaw = cls._safe_float(rpy[2]) if len(rpy) > 2 else 0.0

        object_id = grounded.get("object_id", -1)
        if object_id and object_id != -1:
            to.object_id = str(object_id)
        to.name = to.name or f"{to.color}{to.class_name}".strip()

        ctx.parsed_command = ctx.parsed_command or {"action": intent}
        ctx.target_object = to.to_skill_dict()
        ctx.target_pose = tgt_pose
        ctx.transition(TaskState.GROUNDED)

        return ctx

    @classmethod
    def target_objects_from_world_model(cls,
                                         wm_data_or_list) -> List[TargetObject]:
        if isinstance(wm_data_or_list, dict):
            objects = wm_data_or_list.get("objects") or []
        elif isinstance(wm_data_or_list, list):
            objects = wm_data_or_list
        else:
            objects = []

        results = []
        for obj in objects:
            to = TargetObject.from_world_model(obj)
            results.append(to)
        return results

    @classmethod
    def target_objects_from_detection_result(cls,
                                              det,
                                              world_objects: list = None) -> List[TargetObject]:
        if isinstance(det, dict):
            class_names = det.get("class_name") or []
        else:
            class_names = getattr(det, "class_name", []) or []
        results = []
        for idx in range(len(class_names)):
            to = TargetObject.from_detection_result(det, idx, world_objects)
            results.append(to)
        return results

    @classmethod
    def build_full_task(cls,
                        parsed: dict,
                        grounded: dict,
                        wm_objects: Optional[list] = None) -> TaskContext:
        ctx = cls.from_parsed_command(parsed)
        ctx = cls.from_grounded_goal(grounded, ctx)

        if wm_objects:
            targets = cls.target_objects_from_world_model(wm_objects)
            intent_cls = (ctx.parsed_command or {}).get("action", "")
            object_hints = grounded.get("object_hints", {})
            hint_cls = object_hints.get("class", "")
            hint_color = object_hints.get("color", "")

            best = None
            for t in targets:
                cls_ok = (not hint_cls) or (t.class_name == hint_cls)
                color_ok = (not hint_color) or (t.color == hint_color)
                if cls_ok and color_ok:
                    if best is None or t.confidence > best.confidence:
                        best = t

            if best:
                ctx.target_object = best.to_skill_dict()
                if best.to_pose_dict():
                    ctx.target_pose = ctx.target_pose or best.to_pose_dict()

        return ctx
