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


# ── ROS2 mocks for VerificationResultNode tests ──
import sys
import types
import threading
from unittest.mock import MagicMock

_rclpy_mod = types.ModuleType('rclpy')
_rclpy_node_mod = types.ModuleType('rclpy.node')
_rclpy_node_mod.Node = object
_rclpy_mod.node = _rclpy_node_mod

_rclpy_qos_mod = types.ModuleType('rclpy.qos')
_rclpy_qos_mod.qos_profile_sensor_data = None
_rclpy_qos_mod.ReliabilityPolicy = MagicMock()
_rclpy_qos_mod.QoSProfile = MagicMock
_rclpy_mod.qos = _rclpy_qos_mod

sys.modules['rclpy'] = _rclpy_mod
sys.modules['rclpy.node'] = _rclpy_node_mod
sys.modules['rclpy.qos'] = _rclpy_qos_mod


class _FakeString:
    def __init__(self, data=""):
        self.data = data


class _FakeBool:
    def __init__(self, data=False):
        self.data = data


_std_msgs_mod = types.ModuleType('std_msgs')
_std_msgs_msg_mod = types.ModuleType('std_msgs.msg')
_std_msgs_msg_mod.String = _FakeString
_std_msgs_msg_mod.Bool = _FakeBool
_std_msgs_mod.msg = _std_msgs_msg_mod
sys.modules['std_msgs'] = _std_msgs_mod
sys.modules['std_msgs.msg'] = _std_msgs_msg_mod

import json
from sketch_runtime.verification_result_node import VerificationResultNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_verification_node():
    node = object.__new__(VerificationResultNode)
    node._pending_precheck = None
    node._pending_place_check = None
    node._last_stable_objects = None
    node._postcheck_timer = None
    node._lock = threading.Lock()
    node.publisher = FakePublisher()
    node.distance_threshold = 0.1
    node.postcheck_delay_sec = 1.0
    node.enable_precheck = True
    node.enable_postcheck = True
    node.get_logger = MagicMock()
    node.create_timer = MagicMock(return_value=MagicMock())
    node.destroy_timer = MagicMock()
    return node


def latest_payload(node):
    assert node.publisher.messages
    return json.loads(node.publisher.messages[-1].data)


class TestPostPlaceVerification:
    """Test post_place verification via real VerificationResultNode methods."""

    @staticmethod
    def _place_context(target_xyz=(0.20, -0.15, 0.02)):
        return json.dumps({
            "intent": "place",
            "status": "ok",
            "target_object": {"class_name": "cup", "color": "blue"},
            "target_pose": {"frame": "table", "xyz": list(target_xyz),
                            "rpy": [0.0, 0.0, 1.57]},
        })

    @staticmethod
    def _pick_context(source_xyz=(0.12, -0.15, 0.02)):
        return json.dumps({
            "intent": "pick",
            "status": "ok",
            "target_object": {"class_name": "cup", "color": "blue",
                              "world_x": source_xyz[0],
                              "world_y": source_xyz[1],
                              "world_z": source_xyz[2]},
        })

    @staticmethod
    def _stable_objects():
        return json.dumps([
            {"class_name": "cup", "color": "blue",
             "pose": {"xyz": [0.20, -0.15, 0.02]}},
            {"class_name": "block", "color": "red",
             "pose": {"xyz": [0.5, 0.5, 0.0]}},
        ])

    # ── task_context_callback tests ──

    def test_place_context_sets_pending_place_check(self):
        node = make_verification_node()
        msg = _FakeString(data=self._place_context())

        node.task_context_callback(msg)

        assert node._pending_place_check is not None
        assert node._pending_place_check["target_class"] == "cup"
        assert node._pending_place_check["target_xyz"] == [0.20, -0.15, 0.02]

    def test_place_context_clears_pick_pending(self):
        node = make_verification_node()
        node._pending_precheck = {"old": "pick_data"}
        msg = _FakeString(data=self._place_context())

        node.task_context_callback(msg)

        assert node._pending_precheck is None
        assert node._pending_place_check is not None

    def test_pick_context_clears_place_pending(self):
        node = make_verification_node()
        node._pending_place_check = {"old": "place_data"}
        node._last_stable_objects = self._stable_objects()
        msg = _FakeString(data=self._pick_context())

        node.task_context_callback(msg)

        assert node._pending_place_check is None
        assert node._pending_precheck is not None

    def test_place_context_no_precheck_published(self):
        node = make_verification_node()
        msg = _FakeString(data=self._place_context())

        node.task_context_callback(msg)

        assert len(node.publisher.messages) == 0

    # ── executor_done_callback tests ──

    def test_executor_done_triggers_post_place_timer(self):
        node = make_verification_node()
        node._pending_place_check = {"target_class": "cup", "target_xyz": [0.2, -0.15, 0.02]}
        done_msg = _FakeBool(data=True)

        node.executor_done_callback(done_msg)

        node.create_timer.assert_called_once()
        args, _ = node.create_timer.call_args
        assert args[1] == node._run_post_place_check

    def test_executor_done_ignored_for_place_when_false(self):
        node = make_verification_node()
        node._pending_place_check = {"target_class": "cup", "target_xyz": [0.2, -0.15, 0.02]}
        done_msg = _FakeBool(data=False)

        node.executor_done_callback(done_msg)

        node.create_timer.assert_not_called()
        assert node._pending_place_check is not None

    # ── _run_post_place_check tests ──

    def test_post_place_check_success(self):
        node = make_verification_node()
        node._pending_place_check = {
            "target_class": "cup",
            "target_color": "blue",
            "target_xyz": [0.20, -0.15, 0.02],
        }
        node._last_stable_objects = self._stable_objects()

        node._run_post_place_check()

        payload = latest_payload(node)
        assert payload["stage"] == "post_place"
        assert payload["success"] is True
        assert payload["reason"] == "object_found_at_target"
        assert payload["evidence"]["object_found_at_target"] is True

    def test_post_place_check_failure_no_match(self):
        node = make_verification_node()
        node._pending_place_check = {
            "target_class": "cup",
            "target_color": "blue",
            "target_xyz": [0.9, 0.9, 0.9],
        }
        node._last_stable_objects = self._stable_objects()

        node._run_post_place_check()

        payload = latest_payload(node)
        assert payload["stage"] == "post_place"
        assert payload["success"] is False
        assert payload["reason"] == "object_not_found_at_target"
        assert payload["evidence"]["object_found_at_target"] is False

    def test_post_place_check_pending_cleared_after_run(self):
        node = make_verification_node()
        node._pending_place_check = {
            "target_class": "cup",
            "target_xyz": [0.20, -0.15, 0.02],
        }
        node._last_stable_objects = self._stable_objects()

        node._run_post_place_check()

        assert node._pending_place_check is None

    # ── _find_matching_objects direct tests ──

    def test_find_matching_objects_class_match_near_target(self):
        node = make_verification_node()
        stable = [
            {"class_name": "cup", "color": "blue",
             "pose": {"xyz": [0.20, -0.15, 0.02]}},
            {"class_name": "block", "color": "red",
             "pose": {"xyz": [0.5, 0.5, 0.0]}},
        ]

        matches = node._find_matching_objects(
            stable, "cup", "", [0.20, -0.15, 0.02], 0.1
        )

        assert len(matches) == 1
        assert matches[0]["class_name"] == "cup"

    def test_find_matching_objects_no_match_far_away(self):
        node = make_verification_node()
        stable = [
            {"class_name": "cup", "color": "blue",
             "pose": {"xyz": [0.5, 0.5, 0.0]}},
        ]

        matches = node._find_matching_objects(
            stable, "cup", "", [0.20, -0.15, 0.02], 0.1
        )

        assert len(matches) == 0

    def test_find_matching_objects_unknown_color_ignored(self):
        node = make_verification_node()
        stable = [
            {"class_name": "cup", "color": "unknown",
             "pose": {"xyz": [0.20, -0.15, 0.02]}},
        ]

        matches = node._find_matching_objects(
            stable, "cup", "unknown", [0.20, -0.15, 0.02], 0.1
        )

        assert len(matches) == 1

    def test_find_matching_objects_wrong_class_no_match(self):
        node = make_verification_node()
        stable = [
            {"class_name": "block", "color": "red",
             "pose": {"xyz": [0.20, -0.15, 0.02]}},
        ]

        matches = node._find_matching_objects(
            stable, "cup", "", [0.20, -0.15, 0.02], 0.1
        )

        assert len(matches) == 0

    # ── _build_post_place_evidence direct test ──

    def test_build_post_place_evidence_structure(self):
        node = make_verification_node()
        candidates = [
            {"class_name": "cup", "color": "blue", "distance": 0.01,
             "track_id": None, "xyz": [0.20, -0.15, 0.02]},
        ]
        stable = [
            {"class_name": "cup", "color": "blue",
             "pose": {"xyz": [0.20, -0.15, 0.02]}},
        ]

        evidence = node._build_post_place_evidence(
            candidates, "cup", "blue", [0.20, -0.15, 0.02], stable
        )

        assert evidence["target_class"] == "cup"
        assert evidence["target_color"] == "blue"
        assert evidence["target_xyz"] == [0.20, -0.15, 0.02]
        assert evidence["object_found_at_target"] is True
        assert len(evidence["objects_near_target"]) == 1
        assert evidence["total_stable_objects"] == 1
