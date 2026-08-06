#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple


def parse_roi_message(msg_data: str) -> Dict[str, Any]:
    data = json.loads(msg_data)
    if not isinstance(data, dict):
        raise ValueError(f"expected dict, got {type(data).__name__}")
    data.setdefault("objects", [])
    return data


def extract_xyz(obj: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    pose = obj.get("pose", {})
    if not isinstance(pose, dict):
        return None
    xyz = pose.get("xyz")
    if isinstance(xyz, (list, tuple)) and len(xyz) >= 3:
        return (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    return None


def euclidean_distance(a: Tuple[float, ...], b: Tuple[float, ...]) -> float:
    if len(a) != len(b):
        raise ValueError(f"dimension mismatch: {len(a)} vs {len(b)}")
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def find_dominant(counter: Dict[str, int]) -> Optional[str]:
    if not counter:
        return None
    return max(counter, key=counter.get)


def assign_track(
    tracks: Dict[str, Dict[str, Any]],
    xyz: Tuple[float, float, float],
    threshold: float,
) -> str:
    best_id = None
    best_dist = float("inf")
    for track_id, t in tracks.items():
        if t.get("xyz_mean") is None:
            continue
        dist = euclidean_distance(xyz, t["xyz_mean"])
        if dist < best_dist:
            best_dist = dist
            best_id = track_id
    if best_id is not None and best_dist < threshold:
        return best_id
    new_id = f"track_{len(tracks) + 1:03d}"
    tracks[new_id] = {
        "track_id": new_id,
        "frames": [],
        "xyz_sum": [0.0, 0.0, 0.0],
        "xyz_count": 0,
        "xyz_mean": list(xyz),
    }
    return new_id


def compute_track_metrics(track: Dict[str, Any]) -> Dict[str, Any]:
    frames = track.get("frames", [])
    n = len(frames)
    if n == 0:
        return {}

    class_counter = Counter()
    color_counter = Counter()
    label_counter = Counter()
    confidences = []
    x_values = []
    y_values = []
    z_values = []

    prev_label = None
    label_switches = 0

    for f in frames:
        cls = f.get("class_name", "unknown")
        col = f.get("color", "unknown")
        label = f"{cls} {col}"
        conf = float(f.get("confidence", 0.5))

        class_counter[cls] += 1
        color_counter[col] += 1
        label_counter[label] += 1
        confidences.append(conf)

        xyz = f.get("xyz")
        if xyz and len(xyz) >= 3:
            x_values.append(float(xyz[0]))
            y_values.append(float(xyz[1]))
            z_values.append(float(xyz[2]))

        if prev_label is not None and label != prev_label:
            label_switches += 1
        prev_label = label

    dominant_class = find_dominant(dict(class_counter))
    dominant_color = find_dominant(dict(color_counter))
    dominant_label = find_dominant(dict(label_counter))

    dc = int(class_counter.get(dominant_class, 0)) if dominant_class else 0
    dcol = int(color_counter.get(dominant_color, 0)) if dominant_color else 0
    dl = int(label_counter.get(dominant_label, 0)) if dominant_label else 0

    xyz_mean = [0.0, 0.0, 0.0]
    xyz_std = [0.0, 0.0, 0.0]
    if x_values:
        mx = sum(x_values) / len(x_values)
        my = sum(y_values) / len(y_values)
        mz = sum(z_values) / len(z_values)
        xyz_mean = [round(mx, 4), round(my, 4), round(mz, 4)]
        vx = sum((v - mx) ** 2 for v in x_values) / len(x_values)
        vy = sum((v - my) ** 2 for v in y_values) / len(y_values)
        vz = sum((v - mz) ** 2 for v in z_values) / len(z_values)
        xyz_std = [round(math.sqrt(vx), 4), round(math.sqrt(vy), 4), round(math.sqrt(vz), 4)]

    return {
        "track_id": track.get("track_id", "?"),
        "frames_seen": n,
        "class_counts": dict(class_counter),
        "color_counts": dict(color_counter),
        "label_counts": dict(label_counter),
        "dominant_class": dominant_class,
        "dominant_color": dominant_color,
        "dominant_label": dominant_label,
        "class_stability_ratio": round(dc / n, 4) if n else 0.0,
        "color_stability_ratio": round(dcol / n, 4) if n else 0.0,
        "label_stability_ratio": round(dl / n, 4) if n else 0.0,
        "confidence_min": round(min(confidences), 4) if confidences else 0.0,
        "confidence_max": round(max(confidences), 4) if confidences else 0.0,
        "confidence_mean": round(sum(confidences) / len(confidences), 4) if confidences else 0.0,
        "xyz_mean": xyz_mean,
        "xyz_std": xyz_std,
        "label_switch_count": label_switches,
        "unstable": (dl / n) < 0.75 if n else True,
    }
