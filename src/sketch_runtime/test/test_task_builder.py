import pytest
import time

from sketch_runtime.task_context import TaskState, TaskContext
from sketch_runtime.target_object import TargetObject
from sketch_runtime.runtime_task_builder import TaskBuilder


class TestTaskBuilderParsedCommand:
    def test_from_parsed_command_basic(self):
        parsed = {
            "action": "pick",
            "from": "red_cup",
            "to": "right_side",
            "raw": "pick the red cup",
        }
        ctx = TaskBuilder.from_parsed_command(parsed)
        assert ctx.state == TaskState.PARSED
        assert ctx.user_command == "pick the red cup"
        assert ctx.parsed_command == parsed

    def test_from_parsed_command_no_raw(self):
        parsed = {"action": "move", "from": "blue_ball"}
        ctx = TaskBuilder.from_parsed_command(parsed)
        assert ctx.state == TaskState.PARSED
        assert ctx.user_command == ""

    def test_from_parsed_command_preserves_source(self):
        parsed = {"action": "pick", "raw": "hello", "source": "voice"}
        ctx = TaskBuilder.from_parsed_command(parsed)
        assert ctx.source == "voice"


class TestTaskBuilderGroundedGoal:
    def test_from_grounded_goal_basic(self):
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
            "detail": "",
        }
        ctx = TaskBuilder.from_grounded_goal(grounded)
        assert ctx.state == TaskState.GROUNDED
        assert ctx.target_object is not None
        to = ctx.target_object
        assert to["class_name"] == "cup"
        assert to["color"] == "red"
        assert to["source_pose"]["frame"] == "table"
        assert ctx.target_pose is not None

    def test_from_grounded_goal_no_target_pose(self):
        grounded = {
            "intent": "hold",
            "object_hints": {"class": "ball"},
            "object_id": 7,
            "source_pose": {"frame": "base", "xyz": [0, 0, 0], "rpy": [0, 0, 0]},
        }
        ctx = TaskBuilder.from_grounded_goal(grounded)
        assert ctx.target_pose is None
        assert ctx.target_object is not None

    def test_from_grounded_goal_no_object(self):
        grounded = {
            "intent": "move",
            "target_pose": {"frame": "base", "xyz": [0.5, 0, 0], "rpy": [0, 0, 0]},
            "status": "no_match",
        }
        ctx = TaskBuilder.from_grounded_goal(grounded)
        assert ctx.state == TaskState.GROUNDED
        assert ctx.target_pose is not None

    def test_from_grounded_goal_with_existing_ctx(self):
        parsed = {"action": "pick", "from": "red_cup", "raw": "pick red cup"}
        ctx = TaskBuilder.from_parsed_command(parsed)
        grounded = {
            "intent": "pick",
            "object_hints": {"class": "cup", "color": "red"},
            "object_id": 5,
            "source_pose": {"frame": "base", "xyz": [1, 2, 3], "rpy": [0, 0, 1]},
        }
        ctx2 = TaskBuilder.from_grounded_goal(grounded, ctx)
        assert ctx2 is ctx
        assert ctx2.state == TaskState.GROUNDED
        assert len(ctx2.state_history) == 2
        assert ctx2.user_command == "pick red cup"


class TestTaskBuilderWorldModel:
    def test_target_objects_from_dict(self):
        wm = {
            "objects": [
                {
                    "id": 1,
                    "class_name": "cup",
                    "color": "red",
                    "pose": {"frame": "base", "xyz": [0.1, 0.2, 0.05], "rpy": [0, 0, 0]},
                    "confidence": 0.9,
                },
                {
                    "id": 2,
                    "class_name": "ball",
                    "color": "blue",
                    "pose": {"frame": "base", "xyz": [0.3, 0.1, 0.04], "rpy": [0, 0, 0]},
                    "confidence": 0.8,
                },
            ]
        }
        targets = TaskBuilder.target_objects_from_world_model(wm)
        assert len(targets) == 2
        assert targets[0].class_name == "cup"
        assert targets[0].color == "red"
        assert targets[1].class_name == "ball"

    def test_target_objects_from_list(self):
        objs = [
            {"id": 1, "class_name": "box", "pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
        ]
        targets = TaskBuilder.target_objects_from_world_model(objs)
        assert len(targets) == 1
        assert targets[0].class_name == "box"

    def test_target_objects_empty(self):
        assert TaskBuilder.target_objects_from_world_model({}) == []
        assert TaskBuilder.target_objects_from_world_model([]) == []


class TestTaskBuilderDetectionResult:
    def test_from_detection_result_dict(self):
        det = {
            "class_name": ["cup", "ball"],
            "confidence": [0.95, 0.82],
            "center_x": [320, 400],
            "center_y": [240, 300],
            "center_z": [0.5, 0.6],
            "image_width": [640],
            "image_height": [480],
        }
        targets = TaskBuilder.target_objects_from_detection_result(det)
        assert len(targets) == 2
        assert targets[0].class_name == "cup"
        assert targets[0].confidence == 0.95
        assert targets[1].class_name == "ball"

    def test_from_detection_result_with_world_objects(self):
        det = {
            "class_name": ["cup"],
            "confidence": [0.88],
            "center_x": [150],
            "center_y": [200],
            "center_z": [-1.0],
        }
        wm_objects = [
            {"pose": {"frame": "base", "xyz": [0.11, -0.05, 0.03], "rpy": [0, 0, 1.57]}}
        ]
        targets = TaskBuilder.target_objects_from_detection_result(det, wm_objects)
        assert len(targets) == 1
        assert targets[0].world_x == 0.11
        assert targets[0].world_yaw == 1.57

    def test_from_detection_result_empty(self):
        det = {"class_name": [], "confidence": [], "center_x": [], "center_y": []}
        assert TaskBuilder.target_objects_from_detection_result(det) == []


class TestTaskBuilderBuildFullTask:
    def test_build_full_task_complete(self):
        parsed = {
            "action": "pick",
            "from": "red_cup",
            "to": "right_side",
            "raw": "pick the red cup",
        }
        grounded = {
            "intent": "pick",
            "object_hints": {"class": "cup", "color": "red"},
            "object_id": 1,
            "source_pose": {"frame": "table", "xyz": [0.15, -0.10, 0.03], "rpy": [0, 0, 1.57]},
            "target_pose": {"frame": "table", "xyz": [0.20, -0.15, 0.02], "rpy": [0, 0, 1.57]},
            "status": "ok",
        }
        wm = {
            "objects": [
                {
                    "id": 1,
                    "class_name": "cup",
                    "color": "red",
                    "pose": {"frame": "table", "xyz": [0.15, -0.10, 0.03], "rpy": [0, 0, 1.57]},
                    "confidence": 0.92,
                }
            ]
        }
        ctx = TaskBuilder.build_full_task(parsed, grounded, wm)
        assert ctx.state == TaskState.GROUNDED
        assert ctx.user_command == "pick the red cup"
        assert ctx.target_object is not None
        assert ctx.target_object["class_name"] == "cup"
        assert ctx.target_object["color"] == "red"
        assert ctx.target_pose is not None
        assert len(ctx.state_history) == 2

    def test_build_full_task_no_wm(self):
        parsed = {"action": "pick", "raw": "test"}
        grounded = {
            "intent": "pick",
            "object_hints": {},
            "source_pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
        }
        ctx = TaskBuilder.build_full_task(parsed, grounded)
        assert ctx.state == TaskState.GROUNDED

    def test_build_full_task_skill_selected(self):
        parsed = {"action": "pick", "raw": "pick blue ball"}
        grounded = {
            "intent": "pick",
            "object_hints": {"class": "ball", "color": "blue"},
            "source_pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
            "status": "ok",
        }
        wm = {
            "objects": [
                {
                    "id": 3,
                    "class_name": "ball",
                    "color": "blue",
                    "pose": {"frame": "base", "xyz": [0.3, 0.0, 0.04], "rpy": [0, 0, 0]},
                    "confidence": 0.85,
                }
            ]
        }
        ctx = TaskBuilder.build_full_task(parsed, grounded, wm)
        assert ctx.target_object["class_name"] == "ball"
        assert ctx.target_object["color"] == "blue"
