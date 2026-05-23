#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import Counter, deque
from typing import Any, Dict, List, Optional, Tuple

from app.audit_utils import euclidean_distance


def match_or_create_track(
    tracks: Dict[str, Dict[str, Any]],
    xyz: Tuple[float, float, float],
    threshold: float,
    voting_window: int,
) -> Tuple[str, Dict[str, Any]]:
    # Using latest xyz for responsiveness; future version may use xyz_mean/Kalman.
    best_id = None
    best_dist = float("inf")
    for track_id, t in tracks.items():
        latest = t.get("xyz_latest")
        if latest is None:
            continue
        dist = euclidean_distance(xyz, latest)
        if dist < best_dist:
            best_dist = dist
            best_id = track_id
    if best_id is not None and best_dist < threshold:
        return best_id, tracks[best_id]
    new_id = f"track_{len(tracks) + 1:03d}"
    tracks[new_id] = {
        "track_id": new_id,
        "frames": deque(maxlen=voting_window),
        "xyz_latest": list(xyz),
        "rpy_latest": [0.0, 0.0, 0.0],
        "confidence_smooth": None,
        "last_seen": 0.0,
        "total_frames": 0,
    }
    return new_id, tracks[new_id]


def majority_vote(track: Dict[str, Any], key: str) -> str:
    frames = track.get("frames", [])
    if not frames:
        return "unknown"
    counter = Counter()
    for f in frames:
        val = f.get(key, "unknown")
        counter[val] += 1
    dominant = counter.most_common(1)
    if dominant:
        return dominant[0][0]
    return "unknown"


def build_votes(track: Dict[str, Any], key: str) -> Dict[str, int]:
    frames = track.get("frames", [])
    counter = Counter()
    for f in frames:
        val = f.get(key, "unknown")
        counter[val] += 1
    return dict(counter)


def ema_update(current_smooth: Optional[float], new_value: float, alpha: float) -> float:
    if current_smooth is None:
        return new_value
    return alpha * new_value + (1.0 - alpha) * current_smooth


def is_track_expired(track: Dict[str, Any], now: float, ttl_sec: float) -> bool:
    return (now - track.get("last_seen", 0.0)) > ttl_sec


def prune_expired_tracks(
    tracks: Dict[str, Dict[str, Any]], now: float, ttl_sec: float
) -> List[str]:
    expired = [
        tid for tid, t in tracks.items() if is_track_expired(t, now, ttl_sec)
    ]
    for tid in expired:
        del tracks[tid]
    return expired


def extract_rpy(obj: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    pose = obj.get("pose", {})
    if not isinstance(pose, dict):
        return None
    rpy = pose.get("rpy")
    if isinstance(rpy, (list, tuple)) and len(rpy) >= 3:
        return (float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return None


def extract_frame_data(obj: Dict[str, Any]) -> Dict[str, Any]:
    from app.audit_utils import extract_xyz

    xyz = extract_xyz(obj)
    rpy = extract_rpy(obj)
    frame = {
        "class_name": str(obj.get("class_name", "unknown")),
        "color": str(obj.get("color", "unknown")),
        "confidence": float(obj.get("confidence", 0.5)),
        "xyz": [xyz[0], xyz[1], xyz[2]] if xyz else None,
        "rpy": [rpy[0], rpy[1], rpy[2]] if rpy else [0.0, 0.0, 0.0],
    }
    for extra in ("source", "yolo_class", "roi_class", "match_distance"):
        if extra in obj:
            frame[extra] = obj[extra]
    return frame


def compute_label_stability(track: Dict[str, Any]) -> float:
    frames = track.get("frames", [])
    n = len(frames)
    if n == 0:
        return 0.0
    dominant_label = majority_vote(track, "class_name") + " " + majority_vote(track, "color")
    count = sum(
        1
        for f in frames
        if f"{f.get('class_name', '')} {f.get('color', '')}" == dominant_label
    )
    return round(count / n, 4)


def build_stable_object(
    track: Dict[str, Any], min_frames: int
) -> Optional[Dict[str, Any]]:
    n = len(track.get("frames", []))
    if n < min_frames:
        return None

    class_votes = build_votes(track, "class_name")
    color_votes = build_votes(track, "color")
    class_name = majority_vote(track, "class_name")
    color = majority_vote(track, "color")
    label_stab = compute_label_stability(track)
    conf = track.get("confidence_smooth")
    latest_conf = 0.5
    if track["frames"]:
        latest_conf = float(track["frames"][-1].get("confidence", 0.5))

    return {
        "track_id": track.get("track_id", "?"),
        "class_name": class_name,
        "color": color,
        "pose": {
            "frame": "base",
            "xyz": [round(v, 4) for v in (track.get("xyz_latest", [0, 0, 0]))],
            "rpy": [round(v, 4) for v in (track.get("rpy_latest", [0, 0, 0]))],
        },
        "confidence": round(latest_conf, 4),
        "confidence_smooth": round(conf, 4) if conf is not None else round(latest_conf, 4),
        "frames_tracked": track.get("total_frames", 0),
        "class_votes": class_votes,
        "color_votes": color_votes,
        "label_stability_ratio": label_stab,
        "last_seen": track.get("last_seen", 0.0),
        "source": "stable",
        "source_votes": build_votes(track, "source"),
    }
