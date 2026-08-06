import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from app.audit_utils import (
    assign_track,
    compute_track_metrics,
    euclidean_distance,
    extract_xyz,
    find_dominant,
    parse_roi_message,
)


def test_parse_roi_message_valid():
    result = parse_roi_message(json.dumps({"objects": [{"class_name": "cup", "color": "red"}]}))
    assert result["objects"][0]["class_name"] == "cup"
    assert result["objects"][0]["color"] == "red"


def test_parse_roi_message_empty_objects():
    result = parse_roi_message(json.dumps({"objects": []}))
    assert result["objects"] == []


def test_parse_roi_message_no_objects_key():
    result = parse_roi_message(json.dumps({}))
    assert result["objects"] == []


def test_extract_xyz_valid():
    obj = {"pose": {"frame": "base", "xyz": [0.186, -0.018, 0.030]}}
    result = extract_xyz(obj)
    assert result == (0.186, -0.018, 0.030)


def test_extract_xyz_missing_pose():
    obj = {"class_name": "cup"}
    result = extract_xyz(obj)
    assert result is None


def test_extract_xyz_none_xyz():
    obj = {"pose": {"xyz": None}}
    result = extract_xyz(obj)
    assert result is None


def test_euclidean_distance():
    d = euclidean_distance((0, 0, 0), (3, 4, 0))
    assert abs(d - 5.0) < 0.001


def test_euclidean_distance_same():
    d = euclidean_distance((1.0, 2.0, 3.0), (1.0, 2.0, 3.0))
    assert d == 0.0


def test_find_dominant():
    assert find_dominant({"a": 5, "b": 3, "c": 1}) == "a"


def test_find_dominant_tie():
    result = find_dominant({"a": 3, "b": 3})
    assert result in ("a", "b")


def test_find_dominant_empty():
    assert find_dominant({}) is None


def test_assign_track_new():
    tracks = {}
    tid = assign_track(tracks, (0.186, -0.018, 0.030), 0.05)
    assert tid == "track_001"
    assert tid in tracks
    assert tracks[tid]["xyz_mean"] == [0.186, -0.018, 0.030]


def test_assign_track_match():
    tracks = {}
    tid1 = assign_track(tracks, (0.186, -0.018, 0.030), 0.05)
    tid2 = assign_track(tracks, (0.187, -0.017, 0.031), 0.05)
    assert tid1 == tid2  # matched


def test_assign_track_far():
    tracks = {}
    tid1 = assign_track(tracks, (0.186, -0.018, 0.030), 0.05)
    tid2 = assign_track(tracks, (0.500, -0.200, 0.050), 0.05)
    assert tid1 != tid2  # too far


def test_compute_track_metrics_stable():
    track = {
        "track_id": "track_001",
        "frames": [
            {"class_name": "cup", "color": "red", "confidence": 0.85, "xyz": [0.186, -0.018, 0.030]},
            {"class_name": "cup", "color": "red", "confidence": 0.88, "xyz": [0.187, -0.017, 0.031]},
            {"class_name": "cup", "color": "red", "confidence": 0.82, "xyz": [0.185, -0.019, 0.029]},
        ],
    }
    m = compute_track_metrics(track)
    assert m["frames_seen"] == 3
    assert m["dominant_class"] == "cup"
    assert m["dominant_color"] == "red"
    assert m["dominant_label"] == "cup red"
    assert m["class_stability_ratio"] == 1.0
    assert m["color_stability_ratio"] == 1.0
    assert m["label_stability_ratio"] == 1.0
    assert m["label_switch_count"] == 0
    assert m["unstable"] is False


def test_compute_track_metrics_unstable():
    track = {
        "track_id": "track_003",
        "frames": [
            {"class_name": "cup", "color": "red", "confidence": 0.80, "xyz": [0.186, -0.018, 0.030]},
            {"class_name": "cup", "color": "black", "confidence": 0.75, "xyz": [0.186, -0.018, 0.030]},
            {"class_name": "cylinder", "color": "red", "confidence": 0.70, "xyz": [0.186, -0.018, 0.030]},
            {"class_name": "cup", "color": "red", "confidence": 0.82, "xyz": [0.186, -0.018, 0.030]},
        ],
    }
    m = compute_track_metrics(track)
    assert m["frames_seen"] == 4
    assert m["dominant_class"] == "cup"
    assert m["dominant_color"] == "red"
    assert m["class_stability_ratio"] == 0.75
    assert m["label_switch_count"] == 3  # cup red->cup black->cylinder red->cup red
    assert m["unstable"] is True


def test_compute_track_metrics_empty():
    m = compute_track_metrics({"track_id": "empty", "frames": []})
    assert m == {}


def test_compute_track_metrics_confidence_range():
    track = {
        "track_id": "t",
        "frames": [
            {"class_name": "cup", "color": "red", "confidence": 0.52, "xyz": [0.0, 0.0, 0.0]},
            {"class_name": "cup", "color": "red", "confidence": 0.95, "xyz": [0.0, 0.0, 0.0]},
        ],
    }
    m = compute_track_metrics(track)
    assert m["confidence_min"] == 0.52
    assert m["confidence_max"] == 0.95
    assert abs(m["confidence_mean"] - 0.735) < 0.001
