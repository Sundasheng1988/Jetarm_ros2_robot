import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from app.perception_fusion_utils import (
    build_cache_entry,
    build_fused_object,
    build_roi_only_object,
    build_yolo_only_object,
    match_objects,
    prune_cache,
)


def _make_cache_entry(overrides=None):
    base = {
        "cache_id": 0,
        "class_name": "cup",
        "color": "red",
        "pose": {"frame": "base", "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5]},
        "confidence": 0.8,
        "updated_at": 1000.0,
        "source": "roi",
        "xyz": (0.2, 0.0, 0.03),
    }
    if overrides:
        base.update(overrides)
    return base


# ============================================================
# build_cache_entry
# ============================================================

def test_build_cache_entry_yolo():
    obj = {
        "class_name": "cup",
        "confidence": 0.85,
        "pose": {"frame": "base", "xyz": [0.182, 0.004, 0.034], "rpy": [0, 0, 1.57]},
    }
    e = build_cache_entry(obj, "yolo", 1, 1000.0)
    assert e is not None
    assert e["class_name"] == "cup"
    assert e["color"] is None
    assert e["source"] == "yolo"
    assert e["confidence"] == 0.85


def test_build_cache_entry_roi():
    obj = {
        "class_name": "cube",
        "color": "red",
        "confidence": 0.78,
        "pose": {"frame": "base", "xyz": [0.182, 0.004, 0.034], "rpy": [0, 0, -1.4]},
    }
    e = build_cache_entry(obj, "roi", 2, 1000.0)
    assert e is not None
    assert e["class_name"] == "cube"
    assert e["color"] == "red"
    assert e["source"] == "roi"


def test_build_cache_entry_no_xyz():
    obj = {
        "class_name": "cup",
        "pose": {"frame": "base"},
    }
    e = build_cache_entry(obj, "yolo", 3, 1000.0)
    assert e is None


# ============================================================
# prune_cache
# ============================================================

def test_prune_cache_expired():
    cache = [
        _make_cache_entry({"cache_id": 1, "updated_at": 1000.0}),
        _make_cache_entry({"cache_id": 2, "updated_at": 1003.0}),
        _make_cache_entry({"cache_id": 3, "updated_at": 1005.0}),
    ]
    prune_cache(cache, 1006.0, 3.0)
    ids = [e["cache_id"] for e in cache]
    assert 1 not in ids  # 6s old > 3s ttl
    assert 2 in ids      # 3s old = ttl (not expired)
    assert 3 in ids      # 1s old


def test_prune_cache_keep_recent():
    cache = [
        _make_cache_entry({"cache_id": 1, "updated_at": 1004.0}),
        _make_cache_entry({"cache_id": 2, "updated_at": 1005.0}),
    ]
    prune_cache(cache, 1006.0, 5.0)
    assert len(cache) == 2


# ============================================================
# match_objects
# ============================================================

def test_match_objects_exact():
    yolo = [_make_cache_entry({"cache_id": 1, "source": "yolo", "color": None})]
    roi = [_make_cache_entry({"cache_id": 2, "source": "roi"})]
    fused, roi_only, yolo_only = match_objects(roi, yolo, 0.06)
    assert len(fused) == 1
    assert len(roi_only) == 0
    assert len(yolo_only) == 0
    assert fused[0][2] == 0.0  # match_distance = 0


def test_match_objects_too_far():
    yolo = [
        _make_cache_entry({"cache_id": 1, "source": "yolo", "color": None,
                           "xyz": (0.5, 0.0, 0.03), "pose": {"frame": "base", "xyz": [0.5, 0.0, 0.03], "rpy": [0, 0, 1.57]}})
    ]
    roi = [
        _make_cache_entry({"cache_id": 2, "source": "roi",
                           "xyz": (0.2, 0.0, 0.03), "pose": {"frame": "base", "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5]}})
    ]
    fused, roi_only, yolo_only = match_objects(roi, yolo, 0.06)
    assert len(fused) == 0
    assert len(roi_only) == 1
    assert len(yolo_only) == 1


def test_match_objects_greedy():
    yolo = [
        _make_cache_entry({"cache_id": 10, "source": "yolo", "color": None,
                           "xyz": (0.200, 0.000, 0.030), "class_name": "cup",
                           "pose": {"frame": "base", "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, 1.57]}}),
        _make_cache_entry({"cache_id": 11, "source": "yolo", "color": None,
                           "xyz": (0.195, 0.003, 0.030), "class_name": "bottle",
                           "pose": {"frame": "base", "xyz": [0.195, 0.003, 0.03], "rpy": [0, 0, 1.57]}}),
    ]
    roi = [
        _make_cache_entry({"cache_id": 20, "source": "roi", "confidence": 0.9, "color": "red",
                           "xyz": (0.198, 0.002, 0.030),
                           "pose": {"frame": "base", "xyz": [0.198, 0.002, 0.03], "rpy": [0, 0, -1.5]}}),
        _make_cache_entry({"cache_id": 21, "source": "roi", "confidence": 0.6, "color": "blue",
                           "xyz": (0.201, 0.001, 0.031),
                           "pose": {"frame": "base", "xyz": [0.201, 0.001, 0.031], "rpy": [0, 0, -1.5]}}),
    ]
    fused, roi_only, yolo_only = match_objects(roi, yolo, 0.06)
    assert len(fused) == 2
    assert len(roi_only) == 0
    assert len(yolo_only) == 0
    ids = [(f[0]["cache_id"], f[1]["cache_id"]) for f in fused]
    assert (10, 20) in ids or (11, 20) in ids


# ============================================================
# build_fused_object
# ============================================================

def test_build_fused_object():
    yolo = _make_cache_entry({"class_name": "cup", "confidence": 0.9, "color": None, "source": "yolo"})
    roi = _make_cache_entry({"class_name": "cube", "confidence": 0.5, "color": "red", "source": "roi",
                             "pose": {"frame": "base", "xyz": [0.2, 0.0, 0.03], "rpy": [0, 0, -1.5]}})
    obj = build_fused_object(yolo, roi, 0.023, 1000.0)
    assert obj["class_name"] == "cup"       # YOLO class wins
    assert obj["color"] == "red"             # ROI color
    assert obj["confidence"] == 0.5          # min(0.9, 0.5)
    assert obj["source"] == "yolo_roi_fused"
    assert obj["yolo_class"] == "cup"
    assert obj["roi_class"] == "cube"
    assert obj["match_distance"] == 0.023
    assert obj["pose"]["rpy"] == [0, 0, -1.5]


# ============================================================
# build_roi_only_object
# ============================================================

def test_build_roi_only_object():
    roi = _make_cache_entry({"class_name": "cylinder", "color": "blue", "source": "roi"})
    obj = build_roi_only_object(roi, 1000.0)
    assert obj["class_name"] == "cylinder"
    assert obj["color"] == "blue"
    assert obj["source"] == "roi_only"
    assert "yolo_class" not in obj
    assert "match_distance" not in obj


# ============================================================
# build_yolo_only_object
# ============================================================

def test_build_yolo_only_object():
    yolo = _make_cache_entry({"class_name": "bottle", "color": None, "source": "yolo"})
    obj = build_yolo_only_object(yolo, 1000.0)
    assert obj["class_name"] == "bottle"
    assert obj["color"] == "unknown"
    assert obj["source"] == "yolo_only"
    assert "roi_class" not in obj
    assert "match_distance" not in obj


# ============================================================
# confidence edge case
# ============================================================

def test_confidence_min():
    yolo = _make_cache_entry({"confidence": 0.92, "color": None, "source": "yolo"})
    roi = _make_cache_entry({"confidence": 0.45, "source": "roi"})
    obj = build_fused_object(yolo, roi, 0.01, 1000.0)
    assert obj["confidence"] == 0.45
