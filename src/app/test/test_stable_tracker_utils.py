import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from app.stable_tracker_utils import (
    build_stable_object,
    build_votes,
    compute_label_stability,
    ema_update,
    extract_frame_data,
    extract_rpy,
    is_track_expired,
    majority_vote,
    match_or_create_track,
    prune_expired_tracks,
)


# ============================================================
# match_or_create_track
# ============================================================

def test_match_or_create_track_new():
    tracks = {}
    tid, t = match_or_create_track(tracks, (0.186, -0.018, 0.030), 0.05, 20)
    assert tid == "track_001"
    assert tid in tracks
    assert tracks[tid]["xyz_latest"] == [0.186, -0.018, 0.030]
    assert tracks[tid]["total_frames"] == 0


def test_match_or_create_track_match():
    tracks = {}
    tid1, _ = match_or_create_track(tracks, (0.186, -0.018, 0.030), 0.05, 20)
    tid2, _ = match_or_create_track(tracks, (0.187, -0.017, 0.031), 0.05, 20)
    assert tid1 == tid2


def test_match_or_create_track_far():
    tracks = {}
    tid1, _ = match_or_create_track(tracks, (0.186, -0.018, 0.030), 0.05, 20)
    tid2, _ = match_or_create_track(tracks, (0.500, -0.200, 0.050), 0.05, 20)
    assert tid1 != tid2
    assert len(tracks) == 2


# ============================================================
# majority_vote
# ============================================================

def test_majority_vote_single():
    track = {
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "red"},
        ]),
    }
    assert majority_vote(track, "class_name") == "cup"
    assert majority_vote(track, "color") == "red"


def test_majority_vote_mixed():
    track = {
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "black"},
        ] * 15 + [
            {"class_name": "cylinder", "color": "red"},
        ] * 5),
    }
    assert majority_vote(track, "class_name") == "cup"
    assert majority_vote(track, "color") == "black"


def test_majority_vote_empty():
    track = {"frames": deque(maxlen=20)}
    assert majority_vote(track, "class_name") == "unknown"


# ============================================================
# build_votes
# ============================================================

def test_build_votes():
    track = {
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "black"},
        ] * 12 + [
            {"class_name": "cylinder", "color": "red"},
        ] * 8),
    }
    v = build_votes(track, "class_name")
    assert v == {"cup": 12, "cylinder": 8}


# ============================================================
# ema_update
# ============================================================

def test_ema_update_initial():
    result = ema_update(None, 0.80, 0.2)
    assert result == 0.80


def test_ema_update_steady():
    s = 0.80
    for _ in range(3):
        s = ema_update(s, 0.80, 0.2)
    assert abs(s - 0.80) < 0.001


def test_ema_update_trend():
    s = 0.50
    for _ in range(5):
        s = ema_update(s, 0.90, 0.2)
    assert s > 0.70


# ============================================================
# is_track_expired
# ============================================================

def test_is_track_expired():
    t = {"last_seen": 1000.0}
    assert is_track_expired(t, 1003.5, 3.0) is True


def test_is_track_expired_not():
    t = {"last_seen": 1000.0}
    assert is_track_expired(t, 1002.0, 3.0) is False


def test_is_track_expired_exact_boundary():
    t = {"last_seen": 1000.0}
    assert is_track_expired(t, 1003.0, 3.0) is False


# ============================================================
# prune_expired_tracks
# ============================================================

def test_prune_expired_tracks():
    tracks = {
        "a": {"last_seen": 1000.0},
        "b": {"last_seen": 1005.0},
        "c": {"last_seen": 1000.5},
    }
    expired = prune_expired_tracks(tracks, 1004.0, 3.0)
    assert len(expired) == 2
    assert "a" in expired
    assert "c" in expired
    assert "b" not in expired
    assert "a" not in tracks
    assert "c" not in tracks
    assert "b" in tracks


# ============================================================
# extract_rpy
# ============================================================

def test_extract_rpy_valid():
    obj = {"pose": {"frame": "base", "rpy": [0.0, 0.0, -1.4]}}
    assert extract_rpy(obj) == (0.0, 0.0, -1.4)


def test_extract_rpy_missing():
    obj = {"class_name": "cup"}
    assert extract_rpy(obj) is None


def test_extract_rpy_none():
    obj = {"pose": {"rpy": None}}
    assert extract_rpy(obj) is None


# ============================================================
# extract_frame_data
# ============================================================

def test_extract_frame_data_full():
    obj = {
        "class_name": "cup",
        "color": "black",
        "confidence": 0.83,
        "pose": {"frame": "base", "xyz": [0.182, 0.004, 0.034], "rpy": [0.0, 0.0, -1.4]},
    }
    f = extract_frame_data(obj)
    assert f["class_name"] == "cup"
    assert f["color"] == "black"
    assert f["confidence"] == 0.83
    assert f["xyz"] == [0.182, 0.004, 0.034]
    assert f["rpy"] == [0.0, 0.0, -1.4]


def test_extract_frame_data_with_fused_source():
    obj = {
        "class_name": "cup",
        "color": "red",
        "confidence": 0.82,
        "pose": {"frame": "base", "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5]},
        "source": "yolo_roi_fused",
        "yolo_class": "cup",
        "roi_class": "cube",
        "match_distance": 0.023,
    }
    f = extract_frame_data(obj)
    assert f["class_name"] == "cup"
    assert f["color"] == "red"
    assert f["source"] == "yolo_roi_fused"
    assert f["yolo_class"] == "cup"
    assert f["roi_class"] == "cube"
    assert f["match_distance"] == 0.023


def test_extract_frame_data_no_pose():
    obj = {"class_name": "cylinder", "color": "red", "confidence": 0.60}
    f = extract_frame_data(obj)
    assert f["class_name"] == "cylinder"
    assert f["xyz"] is None
    assert f["rpy"] == [0.0, 0.0, 0.0]


# ============================================================
# compute_label_stability
# ============================================================

def test_compute_label_stability_all_same():
    track = {
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "black"},
        ] * 10),
    }
    assert compute_label_stability(track) == 1.0


def test_compute_label_stability_mixed():
    track = {
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "black"},
        ] * 13 + [
            {"class_name": "cylinder", "color": "red"},
        ] * 7),
    }
    s = compute_label_stability(track)
    assert s == 0.65


def test_compute_label_stability_empty():
    track = {"frames": deque(maxlen=20)}
    assert compute_label_stability(track) == 0.0


# ============================================================
# build_stable_object
# ============================================================

def test_build_stable_object_below_min():
    track = {
        "track_id": "t1",
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "black", "confidence": 0.80, "xyz": [0.1, 0.0, 0.03], "rpy": [0, 0, -1.4]},
        ] * 3),
        "xyz_latest": [0.1, 0.0, 0.03],
        "rpy_latest": [0.0, 0.0, -1.4],
        "confidence_smooth": 0.80,
        "last_seen": 1000.0,
        "total_frames": 3,
    }
    assert build_stable_object(track, 5) is None


def test_build_stable_object_valid():
    track = {
        "track_id": "track_001",
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "black", "confidence": 0.83, "xyz": [0.182, 0.004, 0.034], "rpy": [0, 0, -1.4]},
        ] * 15 + [
            {"class_name": "cylinder", "color": "red", "confidence": 0.70, "xyz": [0.182, 0.004, 0.034], "rpy": [0, 0, -1.4]},
        ] * 5),
        "xyz_latest": [0.182, 0.004, 0.034],
        "rpy_latest": [0.0, 0.0, -1.4],
        "confidence_smooth": 0.79,
        "last_seen": 1000.0,
        "total_frames": 93,
    }
    obj = build_stable_object(track, 5)
    assert obj is not None
    assert obj["track_id"] == "track_001"
    assert obj["class_name"] == "cup"
    assert obj["color"] == "black"
    assert obj["confidence_smooth"] == 0.79
    assert obj["frames_tracked"] == 93
    assert obj["source"] == "stable"
    assert "source_votes" in obj
    assert obj["class_votes"] == {"cup": 15, "cylinder": 5}
    assert obj["color_votes"] == {"black": 15, "red": 5}
    assert abs(obj["label_stability_ratio"] - 0.75) < 0.001


def test_build_stable_object_source_votes():
    track = {
        "track_id": "track_002",
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "red", "confidence": 0.80, "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5], "source": "yolo_roi_fused"},
        ] * 12 + [
            {"class_name": "cube", "color": "red", "confidence": 0.60, "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5], "source": "roi_only"},
        ] * 3),
        "xyz_latest": [0.2, 0.0, 0.03],
        "rpy_latest": [0.0, 0.0, -1.5],
        "confidence_smooth": 0.75,
        "last_seen": 1000.0,
        "total_frames": 15,
    }
    obj = build_stable_object(track, 5)
    assert obj is not None
    assert obj["source"] == "stable"
    assert obj["source_votes"] == {"yolo_roi_fused": 12, "roi_only": 3}


# ============================================================
# majority_vote with ignore_values
# ============================================================

def test_majority_vote_ignores_unknown():
    track = {
        "frames": deque(maxlen=20, iterable=[
            {"color": "blue"},
        ] * 15 + [
            {"color": "unknown"},
        ] * 10),
    }
    assert majority_vote(track, "color", ignore_values={"unknown"}) == "blue"


def test_majority_vote_all_unknown():
    track = {
        "frames": deque(maxlen=20, iterable=[
            {"color": "unknown"},
        ] * 10),
    }
    assert majority_vote(track, "color", ignore_values={"unknown"}) == "unknown"


def test_majority_vote_ignore_values_none():
    track = {
        "frames": deque(maxlen=20, iterable=[
            {"color": "blue"},
        ] * 10 + [
            {"color": "unknown"},
        ] * 15),
    }
    assert majority_vote(track, "color") == "unknown"


# ============================================================
# build_stable_object roi_only gating + filtered color
# ============================================================

def test_build_stable_object_roi_only_rejects_below_min():
    track = {
        "track_id": "t_roi",
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cube", "color": "red", "confidence": 0.60, "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5], "source": "roi_only"},
        ] * 6),
        "xyz_latest": [0.2, 0.0, 0.03],
        "rpy_latest": [0.0, 0.0, -1.5],
        "confidence_smooth": 0.60,
        "last_seen": 1000.0,
        "total_frames": 6,
    }
    assert build_stable_object(track, 5, min_frames_roi_only=8) is None


def test_build_stable_object_roi_only_accepts_above_min():
    track = {
        "track_id": "t_roi",
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cube", "color": "red", "confidence": 0.60, "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5], "source": "roi_only"},
        ] * 10),
        "xyz_latest": [0.2, 0.0, 0.03],
        "rpy_latest": [0.0, 0.0, -1.5],
        "confidence_smooth": 0.65,
        "last_seen": 1000.0,
        "total_frames": 10,
    }
    obj = build_stable_object(track, 5, min_frames_roi_only=8)
    assert obj is not None
    assert obj["source"] == "stable"
    assert obj["source_votes"] == {"roi_only": 10}


def test_build_stable_object_fused_not_subject_to_roi_min():
    track = {
        "track_id": "t_fused",
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "blue", "confidence": 0.80, "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5], "source": "yolo_roi_fused"},
        ] * 5),
        "xyz_latest": [0.2, 0.0, 0.03],
        "rpy_latest": [0.0, 0.0, -1.5],
        "confidence_smooth": 0.80,
        "last_seen": 1000.0,
        "total_frames": 5,
    }
    obj = build_stable_object(track, 5, min_frames_roi_only=8)
    assert obj is not None
    assert obj["source_votes"] == {"yolo_roi_fused": 5}


def test_build_stable_object_unknown_color_filtered():
    track = {
        "track_id": "t_mixed",
        "frames": deque(maxlen=20, iterable=[
            {"class_name": "cup", "color": "blue", "confidence": 0.80, "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5], "source": "yolo_roi_fused"},
        ] * 8 + [
            {"class_name": "cup", "color": "unknown", "confidence": 0.70, "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, 1.57], "source": "yolo_only"},
        ] * 12),
        "xyz_latest": [0.2, 0.0, 0.03],
        "rpy_latest": [0.0, 0.0, 1.57],
        "confidence_smooth": 0.75,
        "last_seen": 1000.0,
        "total_frames": 20,
    }
    obj = build_stable_object(track, 5)
    assert obj is not None
    assert obj["class_name"] == "cup"
    assert obj["color"] == "blue"
    assert obj["color_votes"] == {"blue": 8, "unknown": 12}
    assert obj["source_votes"] == {"yolo_roi_fused": 8, "yolo_only": 12}
    assert "source_votes" in obj
