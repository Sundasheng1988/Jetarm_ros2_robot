import pytest
import time

from sketch_runtime.verification_result import VerificationResult, extract_source_xyz


class TestVerificationResultDataclass:
    def test_default_values(self):
        vr = VerificationResult()
        assert vr.task_id == "verification"
        assert vr.stage == ""
        assert vr.success is False
        assert vr.confidence == 0.0
        assert vr.reason == ""
        assert vr.evidence == {}
        assert isinstance(vr.timestamp, float)

    def test_custom_values(self):
        vr = VerificationResult(
            task_id="test_001",
            stage="precheck",
            success=True,
            confidence=0.95,
            reason="object_found",
            evidence={"class_name": "cup"},
            timestamp=1717100000.0,
        )
        assert vr.task_id == "test_001"
        assert vr.stage == "precheck"
        assert vr.success is True
        assert vr.confidence == 0.95
        assert vr.reason == "object_found"
        assert vr.evidence == {"class_name": "cup"}
        assert vr.timestamp == 1717100000.0

    def test_to_dict(self):
        vr = VerificationResult(
            task_id="test_002",
            stage="postcheck",
            success=False,
            confidence=0.0,
            reason="object_still_at_source",
            evidence={"objects_near_source": 1},
        )
        d = vr.to_dict()
        assert d["task_id"] == "test_002"
        assert d["stage"] == "postcheck"
        assert d["success"] is False
        assert d["confidence"] == 0.0
        assert d["reason"] == "object_still_at_source"
        assert d["evidence"] == {"objects_near_source": 1}
        assert isinstance(d["timestamp"], float)

    def test_to_dict_rounds_confidence(self):
        vr = VerificationResult(confidence=0.9531)
        d = vr.to_dict()
        assert d["confidence"] == 0.953

    def test_evidence_dict_default_is_empty(self):
        vr = VerificationResult()
        vr.evidence["key"] = "value"
        assert vr.evidence == {"key": "value"}

    def test_timestamp_is_time_float(self):
        before = time.time()
        vr = VerificationResult()
        after = time.time()
        assert before <= vr.timestamp <= after

    def test_success_confidence_binary(self):
        vr = VerificationResult(success=True)
        assert vr.confidence == 0.0
        vr.confidence = 1.0 if vr.success else 0.0
        assert vr.confidence == 1.0

        vr2 = VerificationResult(success=False)
        vr2.confidence = 1.0 if vr2.success else 0.0
        assert vr2.confidence == 0.0


class TestExtractSourceXYZ:
    """Test extract_source_xyz from verification_result module."""

    def test_from_world_xyz_primary(self):
        target = {"world_x": 0.1, "world_y": -0.2, "world_z": 0.02}
        result = extract_source_xyz(target)
        assert result == [0.1, -0.2, 0.02]

    def test_from_world_xyz_with_strings(self):
        target = {"world_x": "0.5", "world_y": "-0.3", "world_z": "0.01"}
        result = extract_source_xyz(target)
        assert result == [0.5, -0.3, 0.01]

    def test_from_pose_xyz_fallback(self):
        target = {"pose": {"xyz": [0.3, 0.4, 0.05]}}
        result = extract_source_xyz(target)
        assert result == [0.3, 0.4, 0.05]

    def test_world_xyz_takes_precedence_over_pose_xyz(self):
        target = {
            "world_x": 0.1,
            "world_y": 0.2,
            "world_z": 0.03,
            "pose": {"xyz": [9.9, 9.9, 9.9]},
        }
        result = extract_source_xyz(target)
        assert result == [0.1, 0.2, 0.03]

    def test_empty_target_returns_zeros(self):
        result = extract_source_xyz({})
        assert result == [0.0, 0.0, 0.0]


class TestVerificationResultMatching:
    """Test matching logic directly (without ROS2)."""

    @staticmethod
    def _euclidean_distance(a, b):
        return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5

    def test_match_by_class_near_position(self):
        stable = [
            {"class_name": "cup", "color": "blue", "pose": {"xyz": [0.12, -0.15, 0.02]}},
            {"class_name": "block", "color": "red", "pose": {"xyz": [0.5, 0.5, 0.0]}},
        ]
        matches = [
            o for o in stable
            if o["class_name"] == "cup"
            and TestVerificationResultMatching._euclidean_distance(
                o["pose"]["xyz"], [0.12, -0.15, 0.02]
            ) < 0.1
        ]
        assert len(matches) == 1
        assert matches[0]["class_name"] == "cup"

    def test_match_by_class_far_position_no_match(self):
        stable = [
            {"class_name": "cup", "color": "blue", "pose": {"xyz": [0.5, 0.5, 0.0]}},
        ]
        matches = [
            o for o in stable
            if o["class_name"] == "cup"
            and TestVerificationResultMatching._euclidean_distance(
                o["pose"]["xyz"], [0.12, -0.15, 0.02]
            ) < 0.1
        ]
        assert len(matches) == 0

    def test_match_with_color_unknown_ignored(self):
        stable = [
            {"class_name": "cup", "color": "unknown", "pose": {"xyz": [0.12, -0.15, 0.02]}},
        ]
        matches = [
            o for o in stable
            if o["class_name"] == "cup"
            and (
                o["color"].lower() == "unknown"
                or o["color"].lower() == "unknown"
            )
            and TestVerificationResultMatching._euclidean_distance(
                o["pose"]["xyz"], [0.12, -0.15, 0.02]
            ) < 0.1
        ]
        assert len(matches) == 1

    def test_match_color_not_ignored_when_present(self):
        stable = [
            {"class_name": "cup", "color": "blue", "pose": {"xyz": [0.12, -0.15, 0.02]}},
            {"class_name": "cup", "color": "red", "pose": {"xyz": [0.12, -0.15, 0.02]}},
        ]
        matches = [
            o for o in stable
            if o["class_name"] == "cup"
            and o["color"].lower() == "blue"
        ]
        assert len(matches) == 1
        assert matches[0]["color"] == "blue"

    def test_position_only_match_when_class_missing(self):
        stable = [
            {"class_name": "", "color": "blue", "pose": {"xyz": [0.12, -0.15, 0.02]}},
        ]
        matches = [
            o for o in stable
            if TestVerificationResultMatching._euclidean_distance(
                o["pose"]["xyz"], [0.12, -0.15, 0.02]
            ) < 0.1
        ]
        assert len(matches) == 1

    def test_multiple_candidates_near_source(self):
        stable = [
            {"class_name": "cup", "color": "blue", "pose": {"xyz": [0.12, -0.15, 0.02]}},
            {"class_name": "cup", "color": "blue", "pose": {"xyz": [0.12, -0.14, 0.02]}},
            {"class_name": "block", "color": "red", "pose": {"xyz": [0.5, 0.5, 0.0]}},
        ]
        matches = [
            o for o in stable
            if o["class_name"] == "cup"
            and TestVerificationResultMatching._euclidean_distance(
                o["pose"]["xyz"], [0.12, -0.15, 0.02]
            ) < 0.1
        ]
        assert len(matches) == 2

    def test_distance_threshold_filtering(self):
        stable = [
            {"class_name": "cup", "color": "blue", "pose": {"xyz": [0.12, -0.15, 0.02]}},
            {"class_name": "cup", "color": "blue", "pose": {"xyz": [0.3, 0.1, 0.05]}},
        ]
        tight = [
            o for o in stable
            if o["class_name"] == "cup"
            and TestVerificationResultMatching._euclidean_distance(
                o["pose"]["xyz"], [0.12, -0.15, 0.02]
            ) < 0.05
        ]
        assert len(tight) == 1

        loose = [
            o for o in stable
            if o["class_name"] == "cup"
            and TestVerificationResultMatching._euclidean_distance(
                o["pose"]["xyz"], [0.12, -0.15, 0.02]
            ) < 0.5
        ]
        assert len(loose) == 2
