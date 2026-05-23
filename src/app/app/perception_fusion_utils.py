#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Any, Dict, List, Optional, Tuple

from app.audit_utils import euclidean_distance


def build_cache_entry(
    obj: Dict[str, Any],
    source: str,
    counter: int,
    now: float,
) -> Optional[Dict[str, Any]]:
    from app.audit_utils import extract_xyz

    xyz = extract_xyz(obj)
    if xyz is None:
        return None

    pose = obj.get("pose", {}) if isinstance(obj.get("pose"), dict) else {}
    rpy = [0.0, 0.0, 0.0]
    raw_rpy = pose.get("rpy")
    if isinstance(raw_rpy, (list, tuple)) and len(raw_rpy) >= 3:
        rpy = [float(raw_rpy[0]), float(raw_rpy[1]), float(raw_rpy[2])]

    return {
        "cache_id": counter,
        "class_name": str(obj.get("class_name", "unknown")),
        "color": str(obj.get("color", "")) if obj.get("color") is not None else None,
        "pose": {
            "frame": str(pose.get("frame", "base")),
            "xyz": [round(xyz[0], 4), round(xyz[1], 4), round(xyz[2], 4)],
            "rpy": rpy,
        },
        "confidence": float(obj.get("confidence", 0.5)),
        "updated_at": now,
        "source": source,
        "xyz": (xyz[0], xyz[1], xyz[2]),
    }


def prune_cache(cache: List[Dict[str, Any]], now: float, ttl_sec: float) -> None:
    cache[:] = [e for e in cache if (now - e.get("updated_at", 0.0)) <= ttl_sec]


def match_objects(
    roi_cache: List[Dict[str, Any]],
    yolo_cache: List[Dict[str, Any]],
    threshold: float,
) -> Tuple[
    List[Tuple[Dict[str, Any], Dict[str, Any], float]],
    List[Dict[str, Any]],
    List[Dict[str, Any]],
]:
    roi_sorted = sorted(roi_cache, key=lambda e: -e.get("confidence", 0))
    used_yolo_ids = set()
    fused = []
    roi_only = []

    for roi_entry in roi_sorted:
        best_yolo = None
        best_dist = float("inf")
        for yolo_entry in yolo_cache:
            if yolo_entry["cache_id"] in used_yolo_ids:
                continue
            dist = euclidean_distance(roi_entry["xyz"], yolo_entry["xyz"])
            if dist < threshold and dist < best_dist:
                best_dist = dist
                best_yolo = yolo_entry
        if best_yolo is not None:
            used_yolo_ids.add(best_yolo["cache_id"])
            fused.append((best_yolo, roi_entry, round(float(best_dist), 4)))
        else:
            roi_only.append(roi_entry)

    yolo_only = [y for y in yolo_cache if y["cache_id"] not in used_yolo_ids]
    return fused, roi_only, yolo_only


def build_fused_object(
    yolo_entry: Dict[str, Any], roi_entry: Dict[str, Any], match_distance: float, now: float
) -> Dict[str, Any]:
    return {
        "class_name": yolo_entry["class_name"],
        "color": roi_entry.get("color", "unknown"),
        "pose": roi_entry.get("pose", {}),
        "confidence": round(min(yolo_entry.get("confidence", 0.5), roi_entry.get("confidence", 0.5)), 4),
        "source": "yolo_roi_fused",
        "yolo_class": yolo_entry["class_name"],
        "roi_class": roi_entry["class_name"],
        "match_distance": match_distance,
        "updated_at": now,
    }


def build_roi_only_object(roi_entry: Dict[str, Any], now: float) -> Dict[str, Any]:
    return {
        "class_name": roi_entry["class_name"],
        "color": roi_entry.get("color", "unknown"),
        "pose": roi_entry.get("pose", {}),
        "confidence": round(float(roi_entry.get("confidence", 0.5)), 4),
        "source": "roi_only",
        "updated_at": now,
    }


def build_yolo_only_object(yolo_entry: Dict[str, Any], now: float) -> Dict[str, Any]:
    return {
        "class_name": yolo_entry["class_name"],
        "color": "unknown",
        "pose": yolo_entry.get("pose", {}),
        "confidence": round(float(yolo_entry.get("confidence", 0.5)), 4),
        "source": "yolo_only",
        "updated_at": now,
    }
