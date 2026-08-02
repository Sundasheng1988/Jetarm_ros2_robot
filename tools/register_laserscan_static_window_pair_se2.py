#!/usr/bin/env python3
"""E01 Phase 2A: independent S0 -> S4 LaserScan SE(2) registration.

Purpose
-------
Estimate T_S0_S4 from Jetson-local LaserScan data only:

    p_S0 = T_S0_S4 @ p_S4

The main registration does NOT use Odom or IMU as an initial guess.  Static
TF is used only to express LaserScan points in base_footprint.

Workflow
--------
--step inspect:
    Validate the bag, confirmed windows, dependencies, scan availability, and
    base_footprint<->laser static TF. No registration is run.

--step register:
    Build representative S0/S4 clouds from multiple static scans, run
    coarse-to-fine SE(2) search followed by robust point-to-point ICP in both
    directions, repeat for all/odd/even scan subsets, check inverse consistency
    and subset stability, and write a conservative validity decision.

This script does not modify the bag, robot code, TF, parameters, or Phase-1
outputs. It does not run live ROS and does not assert the historical ~90-degree
SLAM root cause.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Basic math
# ---------------------------------------------------------------------------

def ns_to_s(ns: int) -> float:
    return ns * 1e-9


def wrap_angle(rad: float) -> float:
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def angle_difference(a: float, b: float) -> float:
    """Return wrapped a-b."""
    return wrap_angle(a - b)


def transform_matrix_2d(dx: float, dy: float, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array(
        [[c, -s, dx],
         [s,  c, dy],
         [0.0, 0.0, 1.0]],
        dtype=float,
    )


def invert_transform_2d(T: np.ndarray) -> np.ndarray:
    R = T[:2, :2]
    t = T[:2, 2]
    out = np.eye(3, dtype=float)
    out[:2, :2] = R.T
    out[:2, 2] = -(R.T @ t)
    return out


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return points.copy()
    return points @ T[:2, :2].T + T[:2, 2]


def pose_from_transform(T: np.ndarray) -> Tuple[float, float, float]:
    return (
        float(T[0, 2]),
        float(T[1, 2]),
        math.atan2(float(T[1, 0]), float(T[0, 0])),
    )


def compose_transform(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Composition A @ B."""
    return A @ B


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = float(np.linalg.norm(q))
    if n == 0.0:
        raise ValueError("zero-norm quaternion")
    q /= n
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w),     2 * (x*z + y*w)],
            [2 * (x*y + z*w),     1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w),     2 * (y*z + x*w),     1 - 2 * (x*x + y*y)],
        ],
        dtype=float,
    )


def static_transform_to_matrix(tf_msg) -> np.ndarray:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    R = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = [float(t.x), float(t.y), float(t.z)]
    return T


# ---------------------------------------------------------------------------
# Lazy runtime dependencies
# ---------------------------------------------------------------------------

def import_runtime_dependencies():
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from scipy.spatial import cKDTree
    except Exception as exc:
        raise RuntimeError(
            "Missing runtime dependency. Source ROS 2 and the workspace, and "
            "ensure scipy is installed. Original error: " + repr(exc)
        ) from exc
    return rosbag2_py, deserialize_message, get_message, cKDTree


# ---------------------------------------------------------------------------
# Input models
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Window:
    stage: str
    start_bag_ns: int
    end_bag_ns: int

    @property
    def duration_s(self) -> float:
        return ns_to_s(self.end_bag_ns - self.start_bag_ns)


@dataclass
class ScanRecord:
    bag_ns: int
    header_ns: int
    msg: object


@dataclass
class CloudStats:
    stage: str
    subset: str
    scans_available: int
    scans_used: int
    first_scan_rel_s: float
    last_scan_rel_s: float
    raw_valid_points: int
    downsampled_points: int
    voxel_m: float
    range_cap_m: float


@dataclass
class SearchCandidate:
    dx: float
    dy: float
    yaw: float
    score: float
    overlap: float
    rmse_inlier: float
    inliers: int


@dataclass
class RegistrationResult:
    subset: str
    direction: str
    transform_convention: str
    dx: float
    dy: float
    translation_magnitude: float
    dyaw_deg: float
    coarse_best_score: float
    coarse_second_score: float
    score_margin: float
    ambiguity_ratio: float
    coarse_best_dx: float
    coarse_best_dy: float
    coarse_best_dyaw_deg: float
    coarse_second_dx: float
    coarse_second_dy: float
    coarse_second_dyaw_deg: float
    icp_rmse: float
    overlap_ratio: float
    inlier_count: int
    source_points: int
    target_points: int
    icp_iterations: int
    converged: bool
    valid: bool
    confidence: str
    invalid_reason: str


# ---------------------------------------------------------------------------
# Confirmed windows
# ---------------------------------------------------------------------------

def parse_int_exact(text: str) -> int:
    text = text.strip()
    if not text:
        raise ValueError("empty integer field")
    # CSV values are expected to be integer nanoseconds. Do not round through float.
    return int(text)


def load_confirmed_windows(path: Path) -> Dict[str, Window]:
    windows: Dict[str, Window] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            stage = row.get("stage", "").strip()
            if stage not in {"S0", "S4"}:
                continue
            windows[stage] = Window(
                stage=stage,
                start_bag_ns=parse_int_exact(row["start_bag_time"]),
                end_bag_ns=parse_int_exact(row["end_bag_time"]),
            )
    missing = [s for s in ("S0", "S4") if s not in windows]
    if missing:
        raise ValueError(f"confirmed windows missing: {missing}")
    for w in windows.values():
        if w.start_bag_ns >= w.end_bag_ns:
            raise ValueError(f"invalid window {w.stage}: start >= end")
    return windows


# ---------------------------------------------------------------------------
# Bag loading and static-TF graph
# ---------------------------------------------------------------------------

def load_scans_and_static_tf(bag_uri: Path) -> Tuple[List[ScanRecord], List[object], int, int]:
    rosbag2_py, deserialize_message, get_message, _ = import_runtime_dependencies()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    required = {"/scan", "/tf_static"}
    missing = sorted(required - set(topic_types))
    if missing:
        raise RuntimeError(f"bag missing required topics: {missing}")

    msg_classes = {name: get_message(type_name) for name, type_name in topic_types.items()}
    scans: List[ScanRecord] = []
    static_transforms: List[object] = []
    bag_start: Optional[int] = None
    bag_end: Optional[int] = None

    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if bag_start is None:
            bag_start = bag_ns
        bag_end = bag_ns
        if topic not in required:
            continue
        msg = deserialize_message(raw, msg_classes[topic])
        if topic == "/scan":
            header_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            scans.append(ScanRecord(bag_ns=bag_ns, header_ns=header_ns, msg=msg))
        else:
            static_transforms.extend(list(msg.transforms))

    if bag_start is None or bag_end is None:
        raise RuntimeError("bag contains no messages")
    return scans, static_transforms, bag_start, bag_end


def build_static_tf_graph(static_transforms: Sequence[object]) -> Dict[str, List[Tuple[str, np.ndarray]]]:
    """Graph edge matrix maps coordinates from current frame to neighbor frame."""
    graph: Dict[str, List[Tuple[str, np.ndarray]]] = {}
    for tf in static_transforms:
        parent = tf.header.frame_id.strip().lstrip("/")
        child = tf.child_frame_id.strip().lstrip("/")
        if not parent or not child:
            continue
        T_parent_child = static_transform_to_matrix(tf)  # p_parent = T_parent_child p_child
        graph.setdefault(child, []).append((parent, T_parent_child))
        graph.setdefault(parent, []).append((child, np.linalg.inv(T_parent_child)))
    return graph


def find_static_transform(
    graph: Dict[str, List[Tuple[str, np.ndarray]]],
    source_frame: str,
    target_frame: str,
) -> Tuple[np.ndarray, List[str]]:
    """Return T_target_source and frame path."""
    source = source_frame.strip().lstrip("/")
    target = target_frame.strip().lstrip("/")
    if source == target:
        return np.eye(4, dtype=float), [source]

    queue: List[Tuple[str, np.ndarray, List[str]]] = [(source, np.eye(4, dtype=float), [source])]
    seen = {source}
    while queue:
        frame, T_frame_source, path = queue.pop(0)
        for neighbor, T_neighbor_frame in graph.get(frame, []):
            if neighbor in seen:
                continue
            T_neighbor_source = T_neighbor_frame @ T_frame_source
            next_path = path + [neighbor]
            if neighbor == target:
                return T_neighbor_source, next_path
            seen.add(neighbor)
            queue.append((neighbor, T_neighbor_source, next_path))
    raise RuntimeError(f"no static TF path from {source} to {target}")


# ---------------------------------------------------------------------------
# Cloud construction
# ---------------------------------------------------------------------------

def choose_scan_subset(scans: Sequence[ScanRecord], subset: str) -> List[ScanRecord]:
    if subset == "all":
        return list(scans)
    if subset == "odd":
        return [s for i, s in enumerate(scans) if i % 2 == 1]
    if subset == "even":
        return [s for i, s in enumerate(scans) if i % 2 == 0]
    raise ValueError(f"unknown subset {subset}")


def evenly_limit(items: Sequence[ScanRecord], max_count: int) -> List[ScanRecord]:
    if max_count <= 0 or len(items) <= max_count:
        return list(items)
    idx = np.linspace(0, len(items) - 1, max_count).round().astype(int)
    return [items[int(i)] for i in sorted(set(idx.tolist()))]


def scan_to_base_points(scan_msg, T_base_laser: np.ndarray, max_range_m: float) -> np.ndarray:
    ranges = np.asarray(scan_msg.ranges, dtype=float)
    if ranges.size == 0:
        return np.empty((0, 2), dtype=float)

    angles = float(scan_msg.angle_min) + np.arange(ranges.size, dtype=float) * float(scan_msg.angle_increment)
    lower = max(float(scan_msg.range_min), 0.02)
    upper = min(float(scan_msg.range_max), max_range_m)
    valid = np.isfinite(ranges) & (ranges >= lower) & (ranges <= upper)
    if not np.any(valid):
        return np.empty((0, 2), dtype=float)

    r = ranges[valid]
    a = angles[valid]
    p_laser = np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r), np.ones_like(r)))
    p_base = (T_base_laser @ p_laser.T).T
    return p_base[:, :2]


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    if len(points) == 0:
        return points.copy()
    if voxel_m <= 0:
        return points.copy()
    keys = np.floor(points / voxel_m).astype(np.int64)
    unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((len(unique_keys), 2), dtype=float)
    counts = np.zeros(len(unique_keys), dtype=np.int64)
    np.add.at(sums, inverse, points)
    np.add.at(counts, inverse, 1)
    return sums / counts[:, None]


def build_representative_cloud(
    stage: str,
    subset: str,
    scans_all: Sequence[ScanRecord],
    window: Window,
    trim_s: float,
    T_base_laser: np.ndarray,
    voxel_m: float,
    max_range_m: float,
    max_scans: int,
    bag_start_ns: int,
) -> Tuple[np.ndarray, CloudStats]:
    trim_ns = int(trim_s * 1e9)
    lo = window.start_bag_ns + trim_ns
    hi = window.end_bag_ns - trim_ns
    if lo >= hi:
        raise ValueError(f"{stage} window too short after {trim_s}s edge trim")

    available = [s for s in scans_all if lo <= s.bag_ns <= hi]
    selected = evenly_limit(choose_scan_subset(available, subset), max_scans)
    if not selected:
        raise RuntimeError(f"no scans selected for {stage}/{subset}")

    clouds: List[np.ndarray] = []
    raw_count = 0
    for record in selected:
        pts = scan_to_base_points(record.msg, T_base_laser, max_range_m=max_range_m)
        raw_count += len(pts)
        if len(pts):
            clouds.append(pts)
    if not clouds:
        raise RuntimeError(f"all selected scans invalid for {stage}/{subset}")

    cloud = voxel_downsample(np.vstack(clouds), voxel_m=voxel_m)
    stats = CloudStats(
        stage=stage,
        subset=subset,
        scans_available=len(available),
        scans_used=len(selected),
        first_scan_rel_s=ns_to_s(selected[0].bag_ns - bag_start_ns),
        last_scan_rel_s=ns_to_s(selected[-1].bag_ns - bag_start_ns),
        raw_valid_points=raw_count,
        downsampled_points=len(cloud),
        voxel_m=voxel_m,
        range_cap_m=max_range_m,
    )
    return cloud, stats


# ---------------------------------------------------------------------------
# Registration scoring
# ---------------------------------------------------------------------------

def deterministic_sample(points: np.ndarray, max_points: int) -> np.ndarray:
    if len(points) <= max_points:
        return points
    idx = np.linspace(0, len(points) - 1, max_points).round().astype(int)
    return points[idx]


def score_transform(
    source: np.ndarray,
    target_tree,
    T_target_source: np.ndarray,
    inlier_threshold_m: float,
    truncation_m: float,
) -> Tuple[float, float, float, int]:
    transformed = transform_points(source, T_target_source)
    distances, _ = target_tree.query(transformed, k=1, workers=-1)
    distances = np.asarray(distances, dtype=float)
    inlier_mask = distances <= inlier_threshold_m
    inliers = int(np.count_nonzero(inlier_mask))
    overlap = inliers / max(len(distances), 1)
    rmse = (
        float(np.sqrt(np.mean(distances[inlier_mask] ** 2)))
        if inliers > 0 else float("inf")
    )
    truncated = np.minimum(distances, truncation_m)
    robust_rmse = float(np.sqrt(np.mean(truncated ** 2)))
    # Penalize low overlap so a tiny accidental patch cannot win.
    score = robust_rmse + truncation_m * (1.0 - overlap)
    return score, overlap, rmse, inliers


def grid_values(center: float, half_width: float, step: float) -> np.ndarray:
    count = int(round((2.0 * half_width) / step))
    return center + np.arange(count + 1, dtype=float) * step - half_width


def search_grid(
    source_sample: np.ndarray,
    target_tree,
    x_values: Sequence[float],
    y_values: Sequence[float],
    yaw_values: Sequence[float],
    inlier_threshold_m: float,
    truncation_m: float,
    keep: int = 30,
) -> List[SearchCandidate]:
    best: List[SearchCandidate] = []
    for yaw in yaw_values:
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        rotated = source_sample @ np.array([[c, s], [-s, c]], dtype=float)
        for dx in x_values:
            for dy in y_values:
                moved = rotated + np.array([dx, dy], dtype=float)
                distances, _ = target_tree.query(moved, k=1, workers=-1)
                distances = np.asarray(distances, dtype=float)
                mask = distances <= inlier_threshold_m
                inliers = int(np.count_nonzero(mask))
                overlap = inliers / max(len(distances), 1)
                rmse = float(np.sqrt(np.mean(distances[mask] ** 2))) if inliers else float("inf")
                robust_rmse = float(np.sqrt(np.mean(np.minimum(distances, truncation_m) ** 2)))
                score = robust_rmse + truncation_m * (1.0 - overlap)
                candidate = SearchCandidate(
                    dx=float(dx), dy=float(dy), yaw=float(yaw),
                    score=score, overlap=overlap, rmse_inlier=rmse, inliers=inliers,
                )
                if len(best) < keep:
                    best.append(candidate)
                    best.sort(key=lambda c_: c_.score)
                elif score < best[-1].score:
                    best[-1] = candidate
                    best.sort(key=lambda c_: c_.score)
    return best


def choose_distinct_second(
    candidates: Sequence[SearchCandidate],
    best: SearchCandidate,
    min_translation_sep_m: float,
    min_yaw_sep_deg: float,
) -> Optional[SearchCandidate]:
    min_yaw_sep = math.radians(min_yaw_sep_deg)
    for c in sorted(candidates, key=lambda x: x.score):
        trans_sep = math.hypot(c.dx - best.dx, c.dy - best.dy)
        yaw_sep = abs(angle_difference(c.yaw, best.yaw))
        if trans_sep >= min_translation_sep_m or yaw_sep >= min_yaw_sep:
            return c
    return None


# ---------------------------------------------------------------------------
# Robust point-to-point ICP
# ---------------------------------------------------------------------------

def best_fit_transform_2d(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    if len(source) != len(target) or len(source) < 3:
        raise ValueError("best_fit_transform_2d requires >=3 paired points")
    cs = source.mean(axis=0)
    ct = target.mean(axis=0)
    X = source - cs
    Y = target - ct
    H = X.T @ Y
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = ct - R @ cs
    T = np.eye(3, dtype=float)
    T[:2, :2] = R
    T[:2, 2] = t
    return T


def robust_icp(
    source: np.ndarray,
    target: np.ndarray,
    initial_T: np.ndarray,
    cKDTree,
    max_iterations: int,
    correspondence_m: float,
    trim_fraction: float,
    translation_tol_m: float,
    yaw_tol_rad: float,
) -> Tuple[np.ndarray, int, bool]:
    tree = cKDTree(target)
    T = initial_T.copy()
    converged = False

    for iteration in range(1, max_iterations + 1):
        current = transform_points(source, T)
        distances, indices = tree.query(current, k=1, workers=-1)
        distances = np.asarray(distances)
        indices = np.asarray(indices)
        mask = distances <= correspondence_m
        valid_idx = np.flatnonzero(mask)
        if len(valid_idx) < 20:
            break

        # Retain the nearest trim_fraction correspondences.
        order = valid_idx[np.argsort(distances[valid_idx])]
        keep_n = max(20, int(math.ceil(len(order) * trim_fraction)))
        keep_idx = order[:keep_n]

        matched_source = current[keep_idx]
        matched_target = target[indices[keep_idx]]
        delta = best_fit_transform_2d(matched_source, matched_target)
        T_new = delta @ T

        ddx, ddy, dyaw = pose_from_transform(delta)
        T = T_new
        if math.hypot(ddx, ddy) <= translation_tol_m and abs(dyaw) <= yaw_tol_rad:
            converged = True
            return T, iteration, converged

    return T, max_iterations, converged


# ---------------------------------------------------------------------------
# One-direction registration
# ---------------------------------------------------------------------------

def register_direction(
    subset: str,
    direction: str,
    source: np.ndarray,
    target: np.ndarray,
    args,
    cKDTree,
) -> RegistrationResult:
    source_search = deterministic_sample(source, args.coarse_sample_points)
    target_tree = cKDTree(target)

    # Level 1: global independent search.
    level1 = search_grid(
        source_search,
        target_tree,
        grid_values(0.0, args.xy_search_m, args.level1_xy_step_m),
        grid_values(0.0, args.xy_search_m, args.level1_xy_step_m),
        np.radians(grid_values(0.0, args.yaw_search_deg, args.level1_yaw_step_deg)),
        args.coarse_inlier_m,
        args.coarse_truncation_m,
        keep=args.keep_candidates,
    )
    if not level1:
        raise RuntimeError("level-1 search produced no candidates")
    b1 = level1[0]

    # Level 2: local refinement around best.
    level2 = search_grid(
        source_search,
        target_tree,
        grid_values(b1.dx, args.level2_xy_half_m, args.level2_xy_step_m),
        grid_values(b1.dy, args.level2_xy_half_m, args.level2_xy_step_m),
        b1.yaw + np.radians(
            grid_values(0.0, args.level2_yaw_half_deg, args.level2_yaw_step_deg)
        ),
        args.coarse_inlier_m,
        args.coarse_truncation_m,
        keep=args.keep_candidates,
    )
    b2 = level2[0]

    # Level 3: fine local grid before ICP.
    level3 = search_grid(
        source_search,
        target_tree,
        grid_values(b2.dx, args.level3_xy_half_m, args.level3_xy_step_m),
        grid_values(b2.dy, args.level3_xy_half_m, args.level3_xy_step_m),
        b2.yaw + np.radians(
            grid_values(0.0, args.level3_yaw_half_deg, args.level3_yaw_step_deg)
        ),
        args.coarse_inlier_m,
        args.coarse_truncation_m,
        keep=args.keep_candidates,
    )
    best = level3[0]

    all_candidates = level1 + level2 + level3
    second = choose_distinct_second(
        all_candidates,
        best,
        min_translation_sep_m=args.second_min_translation_m,
        min_yaw_sep_deg=args.second_min_yaw_deg,
    )

    initial_T = transform_matrix_2d(best.dx, best.dy, best.yaw)
    refined_T, iterations, converged = robust_icp(
        source=source,
        target=target,
        initial_T=initial_T,
        cKDTree=cKDTree,
        max_iterations=args.icp_max_iterations,
        correspondence_m=args.icp_correspondence_m,
        trim_fraction=args.icp_trim_fraction,
        translation_tol_m=args.icp_translation_tol_m,
        yaw_tol_rad=math.radians(args.icp_yaw_tol_deg),
    )
    final_score, overlap, rmse, inliers = score_transform(
        deterministic_sample(source, args.final_metric_sample_points),
        target_tree,
        refined_T,
        inlier_threshold_m=args.final_inlier_m,
        truncation_m=args.coarse_truncation_m,
    )
    dx, dy, yaw = pose_from_transform(refined_T)

    second_score = second.score if second is not None else float("nan")
    margin = second_score - best.score if second is not None else float("nan")
    ambiguity_ratio = best.score / second_score if second is not None and second_score > 0 else float("nan")

    reasons: List[str] = []
    if len(source) < args.min_cloud_points or len(target) < args.min_cloud_points:
        reasons.append("insufficient_cloud_points")
    if not converged:
        reasons.append("icp_not_converged")
    if overlap < args.min_overlap:
        reasons.append("low_overlap")
    if not math.isfinite(rmse) or rmse > args.max_rmse_m:
        reasons.append("high_rmse")
    if math.isfinite(ambiguity_ratio) and ambiguity_ratio > args.max_ambiguity_ratio:
        reasons.append("coarse_solution_ambiguous")

    valid = not reasons
    confidence = "high" if valid and overlap >= 0.55 and rmse <= 0.07 else ("medium" if valid else "low")

    return RegistrationResult(
        subset=subset,
        direction=direction,
        transform_convention=(
            "p_target = T_target_source * p_source; "
            + ("p_S0 = T_S0_S4 * p_S4" if direction == "S4_to_S0" else "p_S4 = T_S4_S0 * p_S0")
        ),
        dx=dx,
        dy=dy,
        translation_magnitude=math.hypot(dx, dy),
        dyaw_deg=math.degrees(yaw),
        coarse_best_score=best.score,
        coarse_second_score=second_score,
        score_margin=margin,
        ambiguity_ratio=ambiguity_ratio,
        coarse_best_dx=best.dx,
        coarse_best_dy=best.dy,
        coarse_best_dyaw_deg=math.degrees(best.yaw),
        coarse_second_dx=(second.dx if second is not None else float("nan")),
        coarse_second_dy=(second.dy if second is not None else float("nan")),
        coarse_second_dyaw_deg=(math.degrees(second.yaw) if second is not None else float("nan")),
        icp_rmse=rmse,
        overlap_ratio=overlap,
        inlier_count=inliers,
        source_points=len(source),
        target_points=len(target),
        icp_iterations=iterations,
        converged=converged,
        valid=valid,
        confidence=confidence,
        invalid_reason=";".join(reasons),
    )


# ---------------------------------------------------------------------------
# Bidirectional and subset checks
# ---------------------------------------------------------------------------

def registration_to_transform(result: RegistrationResult) -> np.ndarray:
    return transform_matrix_2d(result.dx, result.dy, math.radians(result.dyaw_deg))


def bidirectional_subset_registration(
    subset: str,
    s0_cloud: np.ndarray,
    s4_cloud: np.ndarray,
    args,
    cKDTree,
) -> Tuple[RegistrationResult, RegistrationResult, Dict[str, float]]:
    forward = register_direction(
        subset=subset,
        direction="S4_to_S0",
        source=s4_cloud,
        target=s0_cloud,
        args=args,
        cKDTree=cKDTree,
    )
    reverse = register_direction(
        subset=subset,
        direction="S0_to_S4",
        source=s0_cloud,
        target=s4_cloud,
        args=args,
        cKDTree=cKDTree,
    )

    T_forward = registration_to_transform(forward)
    T_reverse_inverted = invert_transform_2d(registration_to_transform(reverse))
    dx1, dy1, yaw1 = pose_from_transform(T_forward)
    dx2, dy2, yaw2 = pose_from_transform(T_reverse_inverted)

    errors = {
        "inverse_dx_error_m": abs(dx1 - dx2),
        "inverse_dy_error_m": abs(dy1 - dy2),
        "inverse_translation_error_m": math.hypot(dx1 - dx2, dy1 - dy2),
        "inverse_yaw_error_deg": abs(math.degrees(angle_difference(yaw1, yaw2))),
    }

    inverse_ok = (
        errors["inverse_translation_error_m"] <= args.max_inverse_translation_error_m
        and errors["inverse_yaw_error_deg"] <= args.max_inverse_yaw_error_deg
    )
    if not inverse_ok:
        forward.valid = False
        reverse.valid = False
        forward.confidence = "low"
        reverse.confidence = "low"
        reason = "inverse_inconsistency"
        forward.invalid_reason = ";".join(filter(None, [forward.invalid_reason, reason]))
        reverse.invalid_reason = ";".join(filter(None, [reverse.invalid_reason, reason]))

    return forward, reverse, errors


def summarize_subsets(
    forward_results: Sequence[RegistrationResult],
    inverse_errors: Sequence[Dict[str, float]],
    args,
) -> Dict[str, object]:
    valid = [r for r in forward_results if r.valid]
    summary: Dict[str, object] = {
        "subsets_total": len(forward_results),
        "subsets_valid": len(valid),
        "aggregate_valid": False,
        "aggregate_confidence": "low",
        "invalid_reason": "",
    }
    if len(valid) < 2:
        summary["invalid_reason"] = "fewer_than_two_valid_subsets"
        return summary

    dx = np.array([r.dx for r in valid], dtype=float)
    dy = np.array([r.dy for r in valid], dtype=float)
    mag = np.array([r.translation_magnitude for r in valid], dtype=float)
    yaw = np.array([r.dyaw_deg for r in valid], dtype=float)

    # Yaw range is safe here because search range is only +/-20 degrees.
    summary.update(
        {
            "dx_median": float(np.median(dx)),
            "dy_median": float(np.median(dy)),
            "translation_magnitude_median": float(np.median(mag)),
            "dyaw_deg_median": float(np.median(yaw)),
            "dx_spread": float(np.max(dx) - np.min(dx)),
            "dy_spread": float(np.max(dy) - np.min(dy)),
            "translation_magnitude_spread": float(np.max(mag) - np.min(mag)),
            "dyaw_deg_spread": float(np.max(yaw) - np.min(yaw)),
            "rmse_median": float(np.median([r.icp_rmse for r in valid])),
            "overlap_median": float(np.median([r.overlap_ratio for r in valid])),
            "max_inverse_translation_error_m": float(
                max(e["inverse_translation_error_m"] for e in inverse_errors)
            ),
            "max_inverse_yaw_error_deg": float(
                max(e["inverse_yaw_error_deg"] for e in inverse_errors)
            ),
        }
    )

    reasons: List[str] = []
    if summary["translation_magnitude_spread"] > args.max_subset_translation_spread_m:
        reasons.append("subset_translation_spread")
    if summary["dyaw_deg_spread"] > args.max_subset_yaw_spread_deg:
        reasons.append("subset_yaw_spread")
    if reasons:
        summary["invalid_reason"] = ";".join(reasons)
        return summary

    summary["aggregate_valid"] = True
    high_count = sum(1 for r in valid if r.confidence == "high")
    summary["aggregate_confidence"] = "high" if high_count >= 2 else "medium"
    return summary


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_rows_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: List[str] = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_cloud_csv(path: Path, cloud: np.ndarray) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x_m", "y_m"])
        writer.writerows(cloud.tolist())


def maybe_write_plots(
    out_dir: Path,
    s0_cloud: np.ndarray,
    s4_cloud: np.ndarray,
    T_S0_S4: Optional[np.ndarray],
) -> List[str]:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return []

    written: List[str] = []

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111)
    ax.scatter(s0_cloud[:, 0], s0_cloud[:, 1], s=2, label="S0")
    ax.scatter(s4_cloud[:, 0], s4_cloud[:, 1], s=2, label="S4 (unregistered)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_title("S0 and S4 representative clouds before registration")
    p = out_dir / "s0_s4_before.png"
    fig.savefig(p, dpi=160, bbox_inches="tight")
    plt.close(fig)
    written.append(p.name)

    if T_S0_S4 is not None:
        aligned = transform_points(s4_cloud, T_S0_S4)
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111)
        ax.scatter(s0_cloud[:, 0], s0_cloud[:, 1], s=2, label="S0")
        ax.scatter(aligned[:, 0], aligned[:, 1], s=2, label="T_S0_S4 * S4")
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x in S0 (m)")
        ax.set_ylabel("y in S0 (m)")
        ax.legend()
        ax.set_title("S0 and transformed S4 representative clouds")
        p = out_dir / "s0_s4_after.png"
        fig.savefig(p, dpi=160, bbox_inches="tight")
        plt.close(fig)
        written.append(p.name)

    return written


def interval_distance(value: float, lo: float, hi: float) -> float:
    if lo <= value <= hi:
        return 0.0
    return min(abs(value - lo), abs(value - hi))


def write_review(
    path: Path,
    windows: Dict[str, Window],
    tf_path: Sequence[str],
    cloud_stats: Sequence[CloudStats],
    forward_results: Sequence[RegistrationResult],
    reverse_results: Sequence[RegistrationResult],
    inverse_errors: Sequence[Dict[str, float]],
    summary: Dict[str, object],
    plot_files: Sequence[str],
) -> None:
    lines: List[str] = []
    lines.append("# E01 Phase 2A — S0→S4 LaserScan Registration")
    lines.append("")
    lines.append("## A. Scope and transform convention")
    lines.append("")
    lines.append("- Main result: `T_S0_S4`, defined by `p_S0 = T_S0_S4 * p_S4`.")
    lines.append("- Main coarse search does not use Odom or IMU as an initial guess.")
    lines.append("- Static TF is used only to express LaserScan points in `base_footprint`.")
    lines.append("- No S0→S1 90° registration, no live ROS, no parameter changes, and no historical root-cause claim.")
    lines.append("")
    lines.append("## B. Confirmed windows and static TF")
    lines.append("")
    for stage in ("S0", "S4"):
        w = windows[stage]
        lines.append(
            f"- {stage}: start={w.start_bag_ns}, end={w.end_bag_ns}, duration={w.duration_s:.3f}s"
        )
    lines.append(f"- Static TF path used: `{' -> '.join(tf_path)}`.")
    lines.append("")
    lines.append("## C. Representative clouds")
    lines.append("")
    lines.append("| stage | subset | scans used | raw points | voxel points | scan range (relative s) |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for s in cloud_stats:
        lines.append(
            f"| {s.stage} | {s.subset} | {s.scans_used} | {s.raw_valid_points} | "
            f"{s.downsampled_points} | {s.first_scan_rel_s:.3f}→{s.last_scan_rel_s:.3f} |"
        )
    lines.append("")
    lines.append("## D. Bidirectional registration by subset")
    lines.append("")
    lines.append("| subset | dx m | dy m | magnitude m | dyaw deg | RMSE m | overlap | ambiguity ratio | inverse trans err m | inverse yaw err deg | valid |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    for fwd, err in zip(forward_results, inverse_errors):
        lines.append(
            f"| {fwd.subset} | {fwd.dx:.4f} | {fwd.dy:.4f} | {fwd.translation_magnitude:.4f} | "
            f"{fwd.dyaw_deg:.3f} | {fwd.icp_rmse:.4f} | {fwd.overlap_ratio:.3f} | "
            f"{fwd.ambiguity_ratio:.4f} | {err['inverse_translation_error_m']:.4f} | "
            f"{err['inverse_yaw_error_deg']:.3f} | {fwd.valid} |"
        )
    lines.append("")
    lines.append("## E. Aggregate result")
    lines.append("")
    lines.append(f"- aggregate_valid: **{summary.get('aggregate_valid')}**")
    lines.append(f"- aggregate_confidence: **{summary.get('aggregate_confidence')}**")
    lines.append(f"- invalid_reason: `{summary.get('invalid_reason', '')}`")
    if summary.get("aggregate_valid"):
        lines.append(f"- dx median: {summary['dx_median']:.4f} m")
        lines.append(f"- dy median: {summary['dy_median']:.4f} m")
        lines.append(
            f"- translation magnitude median: {summary['translation_magnitude_median']:.4f} m"
        )
        lines.append(f"- dyaw median: {summary['dyaw_deg_median']:.3f}°")
        lines.append(
            f"- subset magnitude spread: {summary['translation_magnitude_spread']:.4f} m"
        )
        lines.append(f"- subset yaw spread: {summary['dyaw_deg_spread']:.3f}°")
    lines.append("")
    lines.append("## F. Comparison with Phase 1 and physical measurement")
    lines.append("")
    lines.append("- Physical final displacement magnitude: approximately 0.666 m.")
    lines.append("- Physical final yaw residual: approximately −2° to −1° (after ROS sign conversion).")
    lines.append("- Physical x/y direction comparison remains conditional on the user-reported axis convention.")
    lines.append("- Odom/TF final displacement: dx=+0.0061 m, dy=+0.0011 m, magnitude=0.0062 m.")
    lines.append("- IMU/Odom/TF final yaw residual: +6.81°.")
    if summary.get("aggregate_valid"):
        mag = float(summary["translation_magnitude_median"])
        yaw = float(summary["dyaw_deg_median"])
        physical_mag_error = abs(mag - 0.666)
        odom_mag_error = abs(mag - 0.0062)
        physical_yaw_error = interval_distance(yaw, -2.0, -1.0)
        odom_yaw_error = abs(yaw - 6.81)
        lines.append(
            f"- Scan magnitude error vs physical: {physical_mag_error:.4f} m; "
            f"vs Odom: {odom_mag_error:.4f} m."
        )
        lines.append(
            f"- Scan yaw distance to physical interval: {physical_yaw_error:.3f}°; "
            f"to IMU/Odom/TF: {odom_yaw_error:.3f}°."
        )
        lines.append(
            "- Scan translation is closer to: "
            + ("physical measurement." if physical_mag_error < odom_mag_error else "Odom/TF.")
        )
        lines.append(
            "- Scan yaw is closer to: "
            + ("physical measurement." if physical_yaw_error < odom_yaw_error else "IMU/Odom/TF.")
        )
    else:
        lines.append("- Registration is not valid enough for a physical-vs-Odom conclusion.")
    lines.append("")
    lines.append("## G. Confirmed facts")
    lines.append("")
    lines.append("- Facts are limited to the recorded scan counts, static-TF path, numerical registration metrics, and validity checks above.")
    lines.append("")
    lines.append("## H. Inferences")
    lines.append("")
    if summary.get("aggregate_valid"):
        lines.append("- Any comparison in Section F is an inference supported by a valid geometric registration, not a direct physical measurement.")
    else:
        lines.append("- No geometric inference is made because aggregate registration validity failed.")
    lines.append("")
    lines.append("## I. Unknowns")
    lines.append("")
    lines.append("- Environmental symmetry or unmodeled scan distortion may still influence registration.")
    lines.append("- The user-reported x/y axes have not been independently aligned to `base_footprint`.")
    lines.append("- This phase does not establish the cause of the historical ~90° map/scan rotation.")
    if plot_files:
        lines.append("")
        lines.append("## J. Plots")
        lines.append("")
        for name in plot_files:
            lines.append(f"- `{name}`")
    lines.append("")
    lines.append("No original bag or Phase-1 file was modified.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Inspect and run
# ---------------------------------------------------------------------------

def preflight(args) -> Tuple[Dict[str, Window], List[ScanRecord], np.ndarray, List[str], int, int]:
    bag_uri = Path(args.bag).resolve()
    windows_path = Path(args.windows).resolve()
    if not bag_uri.is_dir():
        raise FileNotFoundError(f"bag directory not found: {bag_uri}")
    if not windows_path.is_file():
        raise FileNotFoundError(f"confirmed windows not found: {windows_path}")

    # Dependency check before reading.
    import_runtime_dependencies()
    windows = load_confirmed_windows(windows_path)
    scans, static_transforms, bag_start, bag_end = load_scans_and_static_tf(bag_uri)
    graph = build_static_tf_graph(static_transforms)

    # LaserScan frame is taken from the first scan.
    if not scans:
        raise RuntimeError("bag contains no /scan messages")
    laser_frame = scans[0].msg.header.frame_id.strip().lstrip("/")
    T_base_laser, tf_path = find_static_transform(
        graph,
        source_frame=laser_frame,
        target_frame=args.base_frame,
    )
    return windows, scans, T_base_laser, tf_path, bag_start, bag_end


def inspect(args) -> int:
    windows, scans, T_base_laser, tf_path, bag_start, bag_end = preflight(args)
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    trim_ns = int(args.trim_s * 1e9)
    report = {
        "bag": str(Path(args.bag).resolve()),
        "confirmed_windows": str(Path(args.windows).resolve()),
        "bag_duration_s": ns_to_s(bag_end - bag_start),
        "scan_count_total": len(scans),
        "base_frame": args.base_frame,
        "laser_frame": scans[0].msg.header.frame_id,
        "static_tf_path": tf_path,
        "T_base_laser": T_base_laser.tolist(),
        "windows": {},
        "algorithm": {
            "transform_convention": "p_S0 = T_S0_S4 * p_S4",
            "main_initialization": "independent global coarse SE(2) search; no Odom/IMU initial guess",
            "subsets": ["all", "odd", "even"],
            "bidirectional": True,
            "refinement": "robust trimmed point-to-point ICP",
        },
    }
    for stage, window in windows.items():
        lo = window.start_bag_ns + trim_ns
        hi = window.end_bag_ns - trim_ns
        count = sum(1 for s in scans if lo <= s.bag_ns <= hi)
        report["windows"][stage] = {
            "start_bag_ns": window.start_bag_ns,
            "end_bag_ns": window.end_bag_ns,
            "duration_s": window.duration_s,
            "trimmed_scan_count": count,
        }
        if count < 4:
            raise RuntimeError(f"{stage} has only {count} scans after trim")

    (out_dir / "PHASE2A_INSPECT.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    lines = [
        "# E01 Phase 2A Inspect",
        "",
        f"- Bag: `{report['bag']}`",
        f"- Confirmed windows: `{report['confirmed_windows']}`",
        f"- Bag duration: {report['bag_duration_s']:.3f}s",
        f"- Total scans: {report['scan_count_total']}",
        f"- Static TF path: `{' -> '.join(tf_path)}`",
        "- Transform convention: `p_S0 = T_S0_S4 * p_S4`",
        "- Main initialization: independent coarse-to-fine SE(2) search; no Odom/IMU initial guess.",
        "- Subset repetitions: all, odd, even.",
        "- Bidirectional registration and inverse consistency: enabled.",
        "- ICP: robust trimmed point-to-point.",
        "",
        "## Window scan counts",
        "",
    ]
    for stage in ("S0", "S4"):
        item = report["windows"][stage]
        lines.append(
            f"- {stage}: duration={item['duration_s']:.3f}s, "
            f"scans after edge trim={item['trimmed_scan_count']}"
        )
    lines += [
        "",
        "Inspect only: no point-cloud registration was run.",
    ]
    (out_dir / "PHASE2A_INSPECT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"INSPECT_OK -> {out_dir}")
    return 0


def register(args) -> int:
    windows, scans, T_base_laser, tf_path, bag_start, bag_end = preflight(args)
    _, _, _, cKDTree = import_runtime_dependencies()

    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    subsets = ["all", "odd", "even"]
    clouds: Dict[Tuple[str, str], np.ndarray] = {}
    stats: List[CloudStats] = []

    for subset in subsets:
        for stage in ("S0", "S4"):
            cloud, cloud_stats = build_representative_cloud(
                stage=stage,
                subset=subset,
                scans_all=scans,
                window=windows[stage],
                trim_s=args.trim_s,
                T_base_laser=T_base_laser,
                voxel_m=args.voxel_m,
                max_range_m=args.max_range_m,
                max_scans=args.max_scans_per_subset,
                bag_start_ns=bag_start,
            )
            clouds[(stage, subset)] = cloud
            stats.append(cloud_stats)

    # Persist all-subset representative clouds.
    write_cloud_csv(out_dir / "s0_cloud.csv", clouds[("S0", "all")])
    write_cloud_csv(out_dir / "s4_cloud.csv", clouds[("S4", "all")])

    forward_results: List[RegistrationResult] = []
    reverse_results: List[RegistrationResult] = []
    inverse_errors: List[Dict[str, float]] = []

    for subset in subsets:
        forward, reverse, errors = bidirectional_subset_registration(
            subset=subset,
            s0_cloud=clouds[("S0", subset)],
            s4_cloud=clouds[("S4", subset)],
            args=args,
            cKDTree=cKDTree,
        )
        forward_results.append(forward)
        reverse_results.append(reverse)
        inverse_errors.append(errors)

    summary = summarize_subsets(forward_results, inverse_errors, args)

    rows: List[Dict[str, object]] = []
    for forward, reverse, errors in zip(forward_results, reverse_results, inverse_errors):
        frow = asdict(forward)
        frow.update(errors)
        rows.append(frow)
        rrow = asdict(reverse)
        rrow.update(errors)
        rows.append(rrow)

    aggregate_row = {
        "subset": "aggregate",
        "direction": "S4_to_S0",
        "transform_convention": "p_S0 = T_S0_S4 * p_S4",
        **summary,
    }
    rows.append(aggregate_row)
    write_rows_csv(out_dir / "s0_s4_scan_registration.csv", rows)
    write_rows_csv(out_dir / "cloud_stats.csv", [asdict(s) for s in stats])

    aggregate_T: Optional[np.ndarray] = None
    if summary.get("aggregate_valid"):
        aggregate_T = transform_matrix_2d(
            float(summary["dx_median"]),
            float(summary["dy_median"]),
            math.radians(float(summary["dyaw_deg_median"])),
        )
    plot_files = maybe_write_plots(
        out_dir,
        clouds[("S0", "all")],
        clouds[("S4", "all")],
        aggregate_T,
    )

    write_review(
        out_dir / "S0_S4_SCAN_REVIEW.md",
        windows=windows,
        tf_path=tf_path,
        cloud_stats=stats,
        forward_results=forward_results,
        reverse_results=reverse_results,
        inverse_errors=inverse_errors,
        summary=summary,
        plot_files=plot_files,
    )
    (out_dir / "phase2a_summary.json").write_text(
        json.dumps(
            {
                "summary": summary,
                "forward_results": [asdict(r) for r in forward_results],
                "reverse_results": [asdict(r) for r in reverse_results],
                "inverse_errors": inverse_errors,
                "cloud_stats": [asdict(s) for s in stats],
                "static_tf_path": tf_path,
                "transform_convention": "p_S0 = T_S0_S4 * p_S4",
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"REGISTER_OK -> {out_dir}")
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True, help="rosbag2 directory")
    ap.add_argument("--windows", required=True, help="human-confirmed e01_event_windows_confirmed.csv")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--step", choices=["inspect", "register"], default="inspect")
    ap.add_argument("--base-frame", default="base_footprint")

    # Cloud construction.
    ap.add_argument("--trim-s", type=float, default=1.0)
    ap.add_argument("--voxel-m", type=float, default=0.025)
    ap.add_argument("--max-range-m", type=float, default=8.0)
    ap.add_argument("--max-scans-per-subset", type=int, default=40)
    ap.add_argument("--min-cloud-points", type=int, default=250)

    # Global search.
    ap.add_argument("--xy-search-m", type=float, default=1.2)
    ap.add_argument("--yaw-search-deg", type=float, default=20.0)
    ap.add_argument("--level1-xy-step-m", type=float, default=0.12)
    ap.add_argument("--level1-yaw-step-deg", type=float, default=2.0)
    ap.add_argument("--level2-xy-half-m", type=float, default=0.18)
    ap.add_argument("--level2-xy-step-m", type=float, default=0.03)
    ap.add_argument("--level2-yaw-half-deg", type=float, default=3.0)
    ap.add_argument("--level2-yaw-step-deg", type=float, default=0.5)
    ap.add_argument("--level3-xy-half-m", type=float, default=0.04)
    ap.add_argument("--level3-xy-step-m", type=float, default=0.01)
    ap.add_argument("--level3-yaw-half-deg", type=float, default=0.6)
    ap.add_argument("--level3-yaw-step-deg", type=float, default=0.2)
    ap.add_argument("--coarse-sample-points", type=int, default=350)
    ap.add_argument("--keep-candidates", type=int, default=40)
    ap.add_argument("--coarse-inlier-m", type=float, default=0.18)
    ap.add_argument("--coarse-truncation-m", type=float, default=0.30)
    ap.add_argument("--second-min-translation-m", type=float, default=0.20)
    ap.add_argument("--second-min-yaw-deg", type=float, default=4.0)

    # ICP and final metrics.
    ap.add_argument("--icp-max-iterations", type=int, default=60)
    ap.add_argument("--icp-correspondence-m", type=float, default=0.20)
    ap.add_argument("--icp-trim-fraction", type=float, default=0.80)
    ap.add_argument("--icp-translation-tol-m", type=float, default=0.0002)
    ap.add_argument("--icp-yaw-tol-deg", type=float, default=0.01)
    ap.add_argument("--final-metric-sample-points", type=int, default=3000)
    ap.add_argument("--final-inlier-m", type=float, default=0.15)

    # Conservative validity gates.
    ap.add_argument("--min-overlap", type=float, default=0.35)
    ap.add_argument("--max-rmse-m", type=float, default=0.12)
    ap.add_argument("--max-ambiguity-ratio", type=float, default=0.98)
    ap.add_argument("--max-inverse-translation-error-m", type=float, default=0.15)
    ap.add_argument("--max-inverse-yaw-error-deg", type=float, default=3.0)
    ap.add_argument("--max-subset-translation-spread-m", type=float, default=0.15)
    ap.add_argument("--max-subset-yaw-spread-deg", type=float, default=3.0)
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.step == "inspect":
            return inspect(args)
        return register(args)
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())