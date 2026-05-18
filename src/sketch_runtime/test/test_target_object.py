import pytest
import time

from sketch_runtime.target_object import TargetObject


class TestTargetObject:
    def test_defaults(self):
        obj = TargetObject()
        assert obj.class_name == ""
        assert obj.color == ""
        assert obj.world_frame == "base"
        assert obj.confidence == 0.0
        assert obj.center_z == -1.0

    def test_generates_object_id_when_empty(self):
        obj = TargetObject(source="test")
        assert obj.object_id.startswith("test_")

    def test_accepts_custom_object_id(self):
        obj = TargetObject(object_id="my_object")
        assert obj.object_id == "my_object"

    def test_auto_generates_name(self):
        obj = TargetObject(color="red", class_name="cup")
        assert obj.name == "redcup"

    def test_to_pose_dict(self):
        obj = TargetObject(
            world_frame="table",
            world_x=0.1234,
            world_y=-0.0501,
            world_z=0.0300,
            world_yaw=1.57,
        )
        pose = obj.to_pose_dict()
        assert pose["frame"] == "table"
        assert pose["xyz"][0] == round(0.1234, 4)
        assert pose["xyz"][1] == round(-0.0501, 4)
        assert pose["xyz"][2] == round(0.0300, 4)
        assert pose["rpy"][0] == 0.0
        assert pose["rpy"][2] == 1.57

    def test_to_skill_dict(self):
        obj = TargetObject(
            object_id="obj_001",
            class_name="ball",
            color="blue",
            world_x=0.1,
            world_y=0.2,
            world_z=0.05,
            confidence=0.88,
            source="yolo_v8",
        )
        sd = obj.to_skill_dict()
        assert sd["object_id"] == "obj_001"
        assert sd["class_name"] == "ball"
        assert sd["color"] == "blue"
        assert sd["confidence"] == 0.88
        assert sd["source"] == "yolo_v8"
        assert "source_pose" in sd

    def test_from_world_model_basic(self):
        wm = {
            "id": 5,
            "class_name": "cup",
            "color": "yellow",
            "pose": {
                "frame": "table",
                "xyz": [0.15, -0.10, 0.03],
                "rpy": [0.0, 0.0, 1.57],
            },
            "confidence": 0.92,
            "updated_at": 1715900000.0,
        }
        obj = TargetObject.from_world_model(wm, source="dummy")
        assert obj.class_name == "cup"
        assert obj.color == "yellow"
        assert obj.world_x == 0.15
        assert obj.world_y == -0.10
        assert obj.world_z == 0.03
        assert obj.world_yaw == 1.57
        assert obj.confidence == 0.92
        assert obj.source == "dummy"

    def test_from_world_model_missing_fields(self):
        wm = {"id": 1, "class_name": "ball"}
        obj = TargetObject.from_world_model(wm)
        assert obj.class_name == "ball"
        assert obj.color == ""
        assert obj.world_x == 0.0
        assert obj.confidence == 0.0

    def test_metadata_is_preserved(self):
        obj = TargetObject(metadata={"key": "value"})
        assert obj.metadata["key"] == "value"
