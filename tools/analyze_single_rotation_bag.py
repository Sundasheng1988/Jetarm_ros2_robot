#!/usr/bin/env python3
"""Offline analyzer for one single-rotation ROS 2 bag: S0 -> R1 -> S1.

This tool is intentionally separate from ``analyze_e01_dynamic_yaw.py``.
The E01 tool is preserved for its original four-rotation experiment, while this
analyzer supports one bounded CW or CCW rotation per bag.

It performs no scan registration and never publishes ROS topics. It reads the
bag directly through rosbag2_py and writes:

- summary.json
- metrics.csv
- event_windows.csv
- scan_windows.csv (S1 is mapped to S4 for analyze_e01_scan.py compatibility)
- report.md

Evidence boundary
-----------------
IMU orientation, Odom pose yaw and odom_combined->base_footprint TF yaw may be
part of the same copy/publication chain. Agreement among them is therefore a
consistency check, not three independent physical measurements.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


NS_PER_S = 1_000_000_000
TARGET_TF_PARENT = "odom_combined"
TARGET_TF_CHILD = "base_footprint"
STATIC_TF_PARENT = "base_footprint"
STATIC_TF_CHILD = "laser"


# ---------------------------------------------------------------------------
# Generic helpers
# ---------------------------------------------------------------------------
def ns_to_s(value_ns: int) -> float:
    return value_ns / NS_PER_S


def header_stamp_ns(header: Any) -> int:
    return int(header.stamp.sec) * NS_PER_S + int(header.stamp.nanosec)


def quaternion_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_angles(angles: Sequence[float]) -> List[float]:
    if not angles:
        return []
    out = [float(angles[0])]
    for index in range(1, len(angles)):
        out.append(out[-1] + wrap_pi(float(angles[index]) - float(angles[index - 1])))
    return out


def sign_with_threshold(value: float, threshold: float) -> int:
    if value > threshold:
        return 1
    if value < -threshold:
        return -1
    return 0


def median_mad(values: Sequence[float]) -> Tuple[float, float]:
    if not values:
        return float("nan"), float("nan")
    arr = np.asarray(values, dtype=float)
    median = float(np.median(arr))
    mad = float(np.median(np.abs(arr - median)))
    return median, mad


def interval_stats(stamps_ns: Sequence[int]) -> Dict[str, Any]:
    if len(stamps_ns) < 2:
        return {
            "count": len(stamps_ns),
            "median_interval_s": float("nan"),
            "p95_interval_s": float("nan"),
            "max_gap_s": float("nan"),
            "timestamp_monotonic": True,
            "timestamp_backwards_count": 0,
            "duplicate_timestamp_count": 0,
        }
    arr = np.asarray(stamps_ns, dtype=np.int64)
    diffs = np.diff(arr).astype(float) / NS_PER_S
    return {
        "count": len(stamps_ns),
        "median_interval_s": float(np.median(diffs)),
        "p95_interval_s": float(np.percentile(diffs, 95)),
        "max_gap_s": float(np.max(diffs)),
        "timestamp_monotonic": bool(np.all(diffs >= 0.0)),
        "timestamp_backwards_count": int(np.sum(diffs < 0.0)),
        "duplicate_timestamp_count": int(np.sum(diffs == 0.0)),
    }


def finite(value: Any) -> bool:
    return isinstance(value, (int, float, np.integer, np.floating)) and math.isfinite(float(value))


def json_safe(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return value


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(json_safe(payload), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: List[str] = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen:
                fields.append(key)
                seen.add(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json_safe(row.get(key, "")) for key in fields})


def nearest_index(sorted_values: Sequence[int], query: int) -> Optional[int]:
    if not sorted_values:
        return None
    index = bisect.bisect_left(sorted_values, query)
    candidates: List[int] = []
    if index < len(sorted_values):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    return min(candidates, key=lambda item: abs(sorted_values[item] - query))


def robust_command_threshold(values: Sequence[float]) -> float:
    nonzero = [abs(float(value)) for value in values if abs(float(value)) > 1e-6]
    if not nonzero:
        return 0.05
    return max(0.03, 0.5 * float(np.median(nonzero)))


def parse_field_observation(run_dir: Path) -> Dict[str, str]:
    path = run_dir / "field_observation.txt"
    if not path.is_file():
        return {}
    parsed: Dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if ":" not in raw_line:
            continue
        key, value = raw_line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if key and key not in parsed:
            parsed[key] = value
        elif key and value:
            # Preserve repeated Observer notes without losing either entry.
            parsed[key] = (parsed.get(key, "") + " | " + value).strip(" |")
    return parsed


# ---------------------------------------------------------------------------
# ROS bag loading
# ---------------------------------------------------------------------------
@dataclass
class StampedRecord:
    bag_ns: int
    header_ns: int
    msg: Any


@dataclass
class BagRecord:
    bag_ns: int
    msg: Any


@dataclass
class ExpandedTF:
    bag_ns: int
    header_ns: int
    parent: str
    child: str
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass
class LoadedBag:
    uri: Path
    topic_types: Dict[str, str]
    qos_profiles: Dict[str, str]
    odom: List[StampedRecord] = field(default_factory=list)
    imu: List[StampedRecord] = field(default_factory=list)
    scan: List[StampedRecord] = field(default_factory=list)
    robotvel: List[BagRecord] = field(default_factory=list)
    tf: List[BagRecord] = field(default_factory=list)
    tf_static: List[BagRecord] = field(default_factory=list)
    cmd_vel: List[BagRecord] = field(default_factory=list)
    bag_start_ns: int = 0
    bag_end_ns: int = 0

    @property
    def duration_s(self) -> float:
        return ns_to_s(self.bag_end_ns - self.bag_start_ns)


def load_bag(uri: Path) -> LoadedBag:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:  # pragma: no cover - ROS runtime dependency
        raise RuntimeError(
            "ROS 2 Python dependencies are unavailable. Source "
            "/opt/ros/humble/setup.bash and the workspace install/setup.bash."
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_info = reader.get_all_topics_and_types()
    topic_types = {item.name: item.type for item in topic_info}
    qos_profiles = {item.name: item.offered_qos_profiles for item in topic_info}
    message_classes = {name: get_message(type_name) for name, type_name in topic_types.items()}

    bag = LoadedBag(uri=uri, topic_types=topic_types, qos_profiles=qos_profiles)
    all_bag_stamps: List[int] = []

    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        bag_ns = int(bag_ns)
        all_bag_stamps.append(bag_ns)
        msg = deserialize_message(raw, message_classes[topic])
        if topic == "/odom_combined":
            bag.odom.append(StampedRecord(bag_ns, header_stamp_ns(msg.header), msg))
        elif topic == "/mobile_base/sensors/imu_data":
            bag.imu.append(StampedRecord(bag_ns, header_stamp_ns(msg.header), msg))
        elif topic == "/scan":
            bag.scan.append(StampedRecord(bag_ns, header_stamp_ns(msg.header), msg))
        elif topic == "/robotvel":
            bag.robotvel.append(BagRecord(bag_ns, msg))
        elif topic == "/tf":
            bag.tf.append(BagRecord(bag_ns, msg))
        elif topic == "/tf_static":
            bag.tf_static.append(BagRecord(bag_ns, msg))
        elif topic == "/cmd_vel":
            bag.cmd_vel.append(BagRecord(bag_ns, msg))

    if not all_bag_stamps:
        raise RuntimeError(f"Bag contains no messages: {uri}")
    bag.bag_start_ns = min(all_bag_stamps)
    bag.bag_end_ns = max(all_bag_stamps)
    return bag


def expand_tf(records: Sequence[BagRecord]) -> List[ExpandedTF]:
    expanded: List[ExpandedTF] = []
    for record in records:
        for transform in record.msg.transforms:
            t = transform.transform.translation
            q = transform.transform.rotation
            expanded.append(
                ExpandedTF(
                    bag_ns=record.bag_ns,
                    header_ns=header_stamp_ns(transform.header),
                    parent=str(transform.header.frame_id),
                    child=str(transform.child_frame_id),
                    tx=float(t.x),
                    ty=float(t.y),
                    tz=float(t.z),
                    qx=float(q.x),
                    qy=float(q.y),
                    qz=float(q.z),
                    qw=float(q.w),
                )
            )
    return expanded


# ---------------------------------------------------------------------------
# Single-rotation event detection
# ---------------------------------------------------------------------------
@dataclass
class CommandInterval:
    start_bag_ns: int
    end_bag_ns: int
    sign: int
    sample_count: int
    peak_abs: float
    median: float
    mean: float
    sample_span_s: float
    median_period_s: float
    estimated_hold_s: float


def detect_command_intervals(
    records: Sequence[BagRecord],
    *,
    max_sample_gap_s: float,
) -> Tuple[List[CommandInterval], float, int]:
    values = [float(record.msg.angular.z) for record in records]
    threshold = robust_command_threshold(values)
    max_gap_ns = int(max_sample_gap_s * NS_PER_S)

    groups: List[List[Tuple[int, float]]] = []
    current: List[Tuple[int, float]] = []
    previous_stamp: Optional[int] = None
    previous_sign = 0

    for record, value in zip(records, values):
        sign = sign_with_threshold(value, threshold)
        if sign == 0:
            if current:
                groups.append(current)
                current = []
            previous_stamp = record.bag_ns
            previous_sign = 0
            continue

        split = False
        if current and previous_stamp is not None:
            split = (record.bag_ns - previous_stamp > max_gap_ns) or (sign != previous_sign)
        if split:
            groups.append(current)
            current = []
        current.append((record.bag_ns, value))
        previous_stamp = record.bag_ns
        previous_sign = sign

    if current:
        groups.append(current)

    intervals: List[CommandInterval] = []
    for group in groups:
        stamps = [stamp for stamp, _ in group]
        group_values = [value for _, value in group]
        periods = np.diff(np.asarray(stamps, dtype=np.int64)).astype(float) / NS_PER_S
        median_period = float(np.median(periods)) if len(periods) else float("nan")
        sample_span = ns_to_s(stamps[-1] - stamps[0])
        estimated_hold = sample_span + median_period if math.isfinite(median_period) else sample_span
        median_value = float(np.median(group_values))
        intervals.append(
            CommandInterval(
                start_bag_ns=stamps[0],
                end_bag_ns=stamps[-1],
                sign=1 if median_value > 0.0 else -1,
                sample_count=len(group),
                peak_abs=max(abs(value) for value in group_values),
                median=median_value,
                mean=float(np.mean(group_values)),
                sample_span_s=sample_span,
                median_period_s=median_period,
                estimated_hold_s=estimated_hold,
            )
        )

    zero_count = sum(1 for value in values if sign_with_threshold(value, threshold) == 0)
    return intervals, threshold, zero_count


def select_single_rotation(
    intervals: Sequence[CommandInterval],
    *,
    min_duration_s: float,
    min_peak: float,
) -> Tuple[CommandInterval, List[CommandInterval]]:
    candidates = [
        interval
        for interval in intervals
        if interval.sample_span_s >= min_duration_s and interval.peak_abs >= min_peak
    ]
    if len(candidates) != 1:
        details = ", ".join(
            f"sign={item.sign}, span={item.sample_span_s:.3f}s, n={item.sample_count}, peak={item.peak_abs:.3f}"
            for item in intervals
        ) or "none"
        raise RuntimeError(
            f"Expected exactly one main rotation interval, found {len(candidates)}. "
            f"Detected intervals: {details}"
        )
    main = candidates[0]
    extras = [item for item in intervals if item is not main]
    return main, extras


def choose_static_window(
    raw_start_ns: int,
    raw_end_ns: int,
    *,
    settle_s: float,
    representative_window_s: float,
) -> Tuple[int, int]:
    settle_ns = int(settle_s * NS_PER_S)
    start = raw_start_ns + settle_ns
    end = raw_end_ns - settle_ns
    if end <= start:
        start, end = raw_start_ns, raw_end_ns
    if end <= start:
        raise RuntimeError("Static gap is empty")
    desired_ns = int(representative_window_s * NS_PER_S)
    available_ns = end - start
    if available_ns <= desired_ns:
        return start, end
    center = (start + end) // 2
    half = desired_ns // 2
    return center - half, center + half


def bag_to_odom_header(bag: LoadedBag, query_bag_ns: int) -> int:
    ordered = sorted(bag.odom, key=lambda item: item.bag_ns)
    bag_stamps = [item.bag_ns for item in ordered]
    index = nearest_index(bag_stamps, query_bag_ns)
    if index is None:
        raise RuntimeError("Cannot map bag time to header time because Odom is empty")
    return ordered[index].header_ns


def build_event_windows(
    bag: LoadedBag,
    rotation: CommandInterval,
    *,
    settle_s: float,
    static_window_s: float,
) -> List[Dict[str, Any]]:
    s0_start, s0_end = choose_static_window(
        bag.bag_start_ns,
        rotation.start_bag_ns,
        settle_s=settle_s,
        representative_window_s=static_window_s,
    )
    s1_start, s1_end = choose_static_window(
        rotation.end_bag_ns,
        bag.bag_end_ns,
        settle_s=settle_s,
        representative_window_s=static_window_s,
    )

    rows = [
        {
            "stage": "S0",
            "kind": "static",
            "start_bag_time": s0_start,
            "end_bag_time": s0_end,
            "representative_header_start": bag_to_odom_header(bag, s0_start),
            "representative_header_end": bag_to_odom_header(bag, s0_end),
            "duration_s": ns_to_s(s0_end - s0_start),
            "selection_basis": "centered stable pre-command gap after settle exclusion",
        },
        {
            "stage": "R1",
            "kind": "rotation",
            "start_bag_time": rotation.start_bag_ns,
            "end_bag_time": rotation.end_bag_ns,
            "representative_header_start": bag_to_odom_header(bag, rotation.start_bag_ns),
            "representative_header_end": bag_to_odom_header(bag, rotation.end_bag_ns),
            "duration_s": rotation.sample_span_s,
            "selection_basis": "unique non-zero /cmd_vel angular.z interval",
        },
        {
            "stage": "S1",
            "kind": "static",
            "start_bag_time": s1_start,
            "end_bag_time": s1_end,
            "representative_header_start": bag_to_odom_header(bag, s1_start),
            "representative_header_end": bag_to_odom_header(bag, s1_end),
            "duration_s": ns_to_s(s1_end - s1_start),
            "selection_basis": "centered stable post-command gap after settle exclusion",
        },
    ]
    return rows


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def integrate_trapezoid(
    stamps_ns: Sequence[int],
    values: Sequence[float],
    start_ns: int,
    end_ns: int,
    *,
    bias: float = 0.0,
) -> Dict[str, Any]:
    samples = [(stamp, value) for stamp, value in zip(stamps_ns, values) if start_ns <= stamp <= end_ns]
    if len(samples) < 2:
        return {"integral": float("nan"), "sample_count": len(samples), "max_gap_s": float("nan")}
    integral = 0.0
    max_gap = 0.0
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        dt = ns_to_s(t1 - t0)
        if dt <= 0.0:
            continue
        max_gap = max(max_gap, dt)
        integral += 0.5 * ((v0 - bias) + (v1 - bias)) * dt
    return {"integral": integral, "sample_count": len(samples), "max_gap_s": max_gap}


def window_values(stamps_ns: Sequence[int], values: Sequence[float], start_ns: int, end_ns: int) -> List[float]:
    return [value for stamp, value in zip(stamps_ns, values) if start_ns <= stamp <= end_ns]


def median_angle_window(
    stamps_ns: Sequence[int],
    unwrapped_angles: Sequence[float],
    start_ns: int,
    end_ns: int,
) -> Tuple[float, float, int]:
    values = window_values(stamps_ns, unwrapped_angles, start_ns, end_ns)
    median, mad = median_mad(values)
    return median, mad, len(values)


def median_xy_window(
    stamps_ns: Sequence[int],
    xs: Sequence[float],
    ys: Sequence[float],
    start_ns: int,
    end_ns: int,
) -> Tuple[float, float, int]:
    points = [
        (x, y)
        for stamp, x, y in zip(stamps_ns, xs, ys)
        if start_ns <= stamp <= end_ns
    ]
    if not points:
        return float("nan"), float("nan"), 0
    return (
        float(np.median([point[0] for point in points])),
        float(np.median([point[1] for point in points])),
        len(points),
    )


def build_yaw_series(bag: LoadedBag, expanded_tf: Sequence[ExpandedTF]) -> Dict[str, Dict[str, List[float]]]:
    imu_records = sorted(bag.imu, key=lambda item: item.header_ns)
    odom_records = sorted(bag.odom, key=lambda item: item.header_ns)
    target_tf = sorted(
        [
            item
            for item in expanded_tf
            if item.parent == TARGET_TF_PARENT and item.child == TARGET_TF_CHILD
        ],
        key=lambda item: item.header_ns,
    )

    imu_angles = [
        quaternion_yaw(item.msg.orientation.x, item.msg.orientation.y, item.msg.orientation.z, item.msg.orientation.w)
        for item in imu_records
    ]
    odom_angles = [
        quaternion_yaw(
            item.msg.pose.pose.orientation.x,
            item.msg.pose.pose.orientation.y,
            item.msg.pose.pose.orientation.z,
            item.msg.pose.pose.orientation.w,
        )
        for item in odom_records
    ]
    tf_angles = [quaternion_yaw(item.qx, item.qy, item.qz, item.qw) for item in target_tf]

    return {
        "imu_orientation": {
            "stamps": [item.header_ns for item in imu_records],
            "angles": unwrap_angles(imu_angles),
        },
        "odom_pose": {
            "stamps": [item.header_ns for item in odom_records],
            "angles": unwrap_angles(odom_angles),
        },
        "tf_yaw": {
            "stamps": [item.header_ns for item in target_tf],
            "angles": unwrap_angles(tf_angles),
        },
    }


def static_tf_check(
    bag: LoadedBag,
    *,
    expected_xyz: Tuple[float, float, float],
    tolerance: float,
) -> Dict[str, Any]:
    expanded = expand_tf(bag.tf_static)
    matches = [
        item
        for item in expanded
        if item.parent == STATIC_TF_PARENT and item.child == STATIC_TF_CHILD
    ]
    if not matches:
        return {
            "found": False,
            "match_expected": False,
            "count": 0,
            "reason": "base_footprint->laser not found",
        }

    canonical = {
        (
            round(item.tx, 12), round(item.ty, 12), round(item.tz, 12),
            round(item.qx, 12), round(item.qy, 12), round(item.qz, 12), round(item.qw, 12),
        )
        for item in matches
    }
    first = matches[0]
    q_norm = math.sqrt(first.qx**2 + first.qy**2 + first.qz**2 + first.qw**2)
    identity_rotation = (
        abs(first.qx) <= tolerance
        and abs(first.qy) <= tolerance
        and abs(first.qz) <= tolerance
        and abs(abs(first.qw) - 1.0) <= tolerance
    )
    xyz_match = all(
        abs(actual - expected) <= tolerance
        for actual, expected in zip((first.tx, first.ty, first.tz), expected_xyz)
    )
    return {
        "found": True,
        "match_expected": bool(xyz_match and identity_rotation),
        "count": len(matches),
        "unique_transform_count": len(canonical),
        "conflicting_transform": len(canonical) > 1,
        "translation": [first.tx, first.ty, first.tz],
        "rotation": [first.qx, first.qy, first.qz, first.qw],
        "quaternion_norm": q_norm,
        "translation_match": xyz_match,
        "identity_rotation": identity_rotation,
        "expected_translation": list(expected_xyz),
        "tolerance": tolerance,
    }


def analyze(
    bag: LoadedBag,
    windows: Sequence[Dict[str, Any]],
    rotation: CommandInterval,
    intervals: Sequence[CommandInterval],
    zero_cmd_count: int,
    command_threshold: float,
    args: argparse.Namespace,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    by_stage = {row["stage"]: row for row in windows}
    s0 = by_stage["S0"]
    r1 = by_stage["R1"]
    s1 = by_stage["S1"]

    expanded_tf = expand_tf(bag.tf)
    target_tf = sorted(
        [
            item
            for item in expanded_tf
            if item.parent == TARGET_TF_PARENT and item.child == TARGET_TF_CHILD
        ],
        key=lambda item: item.header_ns,
    )
    yaw_series = build_yaw_series(bag, expanded_tf)

    imu_records = sorted(bag.imu, key=lambda item: item.header_ns)
    imu_header = [item.header_ns for item in imu_records]
    imu_gyro = [float(item.msg.angular_velocity.z) for item in imu_records]
    s0_gyro = window_values(
        imu_header,
        imu_gyro,
        int(s0["representative_header_start"]),
        int(s0["representative_header_end"]),
    )
    gyro_bias, gyro_bias_mad = median_mad(s0_gyro)
    r1_gyro = window_values(
        imu_header,
        imu_gyro,
        int(r1["representative_header_start"]),
        int(r1["representative_header_end"]),
    )
    gyro_median, gyro_mad = median_mad(r1_gyro)
    raw_integral = integrate_trapezoid(
        imu_header,
        imu_gyro,
        int(r1["representative_header_start"]),
        int(r1["representative_header_end"]),
        bias=0.0,
    )
    corrected_integral = integrate_trapezoid(
        imu_header,
        imu_gyro,
        int(r1["representative_header_start"]),
        int(r1["representative_header_end"]),
        bias=gyro_bias if math.isfinite(gyro_bias) else 0.0,
    )

    static_yaw: Dict[str, Dict[str, Any]] = {}
    for source, series in yaw_series.items():
        s0_med, s0_mad, s0_count = median_angle_window(
            series["stamps"],
            series["angles"],
            int(s0["representative_header_start"]),
            int(s0["representative_header_end"]),
        )
        s1_med, s1_mad, s1_count = median_angle_window(
            series["stamps"],
            series["angles"],
            int(s1["representative_header_start"]),
            int(s1["representative_header_end"]),
        )
        delta = s1_med - s0_med if finite(s0_med) and finite(s1_med) else float("nan")
        static_yaw[source] = {
            "s0_median_deg": math.degrees(s0_med) if finite(s0_med) else float("nan"),
            "s0_mad_deg": math.degrees(s0_mad) if finite(s0_mad) else float("nan"),
            "s0_count": s0_count,
            "s1_median_deg": math.degrees(s1_med) if finite(s1_med) else float("nan"),
            "s1_mad_deg": math.degrees(s1_mad) if finite(s1_mad) else float("nan"),
            "s1_count": s1_count,
            "delta_yaw_deg": math.degrees(delta) if finite(delta) else float("nan"),
        }

    odom_records = sorted(bag.odom, key=lambda item: item.header_ns)
    odom_header = [item.header_ns for item in odom_records]
    odom_x = [float(item.msg.pose.pose.position.x) for item in odom_records]
    odom_y = [float(item.msg.pose.pose.position.y) for item in odom_records]
    odom_s0_x, odom_s0_y, odom_s0_count = median_xy_window(
        odom_header,
        odom_x,
        odom_y,
        int(s0["representative_header_start"]),
        int(s0["representative_header_end"]),
    )
    odom_s1_x, odom_s1_y, odom_s1_count = median_xy_window(
        odom_header,
        odom_x,
        odom_y,
        int(s1["representative_header_start"]),
        int(s1["representative_header_end"]),
    )
    odom_dx = odom_s1_x - odom_s0_x
    odom_dy = odom_s1_y - odom_s0_y

    tf_header = [item.header_ns for item in target_tf]
    tf_x = [item.tx for item in target_tf]
    tf_y = [item.ty for item in target_tf]
    tf_s0_x, tf_s0_y, tf_s0_count = median_xy_window(
        tf_header,
        tf_x,
        tf_y,
        int(s0["representative_header_start"]),
        int(s0["representative_header_end"]),
    )
    tf_s1_x, tf_s1_y, tf_s1_count = median_xy_window(
        tf_header,
        tf_x,
        tf_y,
        int(s1["representative_header_start"]),
        int(s1["representative_header_end"]),
    )
    tf_dx = tf_s1_x - tf_s0_x
    tf_dy = tf_s1_y - tf_s0_y

    cmd_nonzero_count = sum(item.sample_count for item in intervals)
    cmd_rate = (
        (rotation.sample_count - 1) / rotation.sample_span_s
        if rotation.sample_count >= 2 and rotation.sample_span_s > 0.0
        else float("nan")
    )
    direction = "CCW" if rotation.sign > 0 else "CW"

    odom_stats = interval_stats([item.header_ns for item in bag.odom])
    imu_stats = interval_stats([item.header_ns for item in bag.imu])
    scan_stats = interval_stats([item.header_ns for item in bag.scan])
    tf_stats = interval_stats(tf_header)
    cmd_stats = interval_stats([item.bag_ns for item in bag.cmd_vel])

    odom_stamp_set = {item.header_ns for item in bag.odom}
    tf_stamp_set = set(tf_header)
    target_tf_audit = {
        "count": len(target_tf),
        "exact_unique_stamp_match_vs_odom": len(odom_stamp_set & tf_stamp_set),
        "odom_missing_tf": len(odom_stamp_set - tf_stamp_set),
        "tf_no_odom": len(tf_stamp_set - odom_stamp_set),
        **tf_stats,
    }

    static_tf = static_tf_check(
        bag,
        expected_xyz=(args.expected_laser_x, args.expected_laser_y, args.expected_laser_z),
        tolerance=args.static_tf_tolerance,
    )

    odom_frame_ids = sorted({str(item.msg.header.frame_id) for item in bag.odom})
    odom_child_ids = sorted({str(item.msg.child_frame_id) for item in bag.odom})
    scan_frame_ids = sorted({str(item.msg.header.frame_id) for item in bag.scan})

    odom_yaw_delta = static_yaw["odom_pose"]["delta_yaw_deg"]
    tf_yaw_delta = static_yaw["tf_yaw"]["delta_yaw_deg"]
    imu_yaw_delta = static_yaw["imu_orientation"]["delta_yaw_deg"]

    odom_tf_consistency = {
        "delta_x_difference_m": tf_dx - odom_dx,
        "delta_y_difference_m": tf_dy - odom_dy,
        "translation_difference_m": math.hypot(tf_dx - odom_dx, tf_dy - odom_dy),
        "delta_yaw_difference_deg": tf_yaw_delta - odom_yaw_delta,
        "consistent_within_tolerance": bool(
            math.hypot(tf_dx - odom_dx, tf_dy - odom_dy) <= args.odom_tf_translation_tolerance
            and abs(tf_yaw_delta - odom_yaw_delta) <= args.odom_tf_yaw_tolerance_deg
        ),
        "translation_tolerance_m": args.odom_tf_translation_tolerance,
        "yaw_tolerance_deg": args.odom_tf_yaw_tolerance_deg,
    }

    required_topics = [
        "/cmd_vel",
        "/mobile_base/sensors/imu_data",
        "/odom_combined",
        "/scan",
        "/tf",
        "/tf_static",
    ]
    missing_topics = [topic for topic in required_topics if topic not in bag.topic_types]

    summary: Dict[str, Any] = {
        "schema_version": 1,
        "run_id": args.run_id,
        "bag": str(bag.uri),
        "run_dir": str(bag.uri.parent),
        "bag_duration_s": bag.duration_s,
        "topic_types": bag.topic_types,
        "missing_required_topics": missing_topics,
        "analysis_valid": len(missing_topics) == 0,
        "event_model": "S0->R1->S1 single rotation",
        "command": {
            "direction": direction,
            "sign": rotation.sign,
            "angular_z_median_rad_s": rotation.median,
            "angular_z_mean_rad_s": rotation.mean,
            "peak_abs_rad_s": rotation.peak_abs,
            "nonzero_count": cmd_nonzero_count,
            "zero_count": zero_cmd_count,
            "total_count": len(bag.cmd_vel),
            "sample_span_s": rotation.sample_span_s,
            "estimated_hold_s": rotation.estimated_hold_s,
            "median_period_s": rotation.median_period_s,
            "actual_rate_hz": cmd_rate,
            "detection_threshold_rad_s": command_threshold,
            "extra_nonzero_intervals": max(0, len(intervals) - 1),
        },
        "imu": {
            "gyro_bias_s0_rad_s": gyro_bias,
            "gyro_bias_s0_mad_rad_s": gyro_bias_mad,
            "gyro_r1_median_rad_s": gyro_median,
            "gyro_r1_mad_rad_s": gyro_mad,
            "gyro_integral_raw_deg": math.degrees(raw_integral["integral"]) if finite(raw_integral["integral"]) else float("nan"),
            "gyro_integral_bias_corrected_deg": math.degrees(corrected_integral["integral"]) if finite(corrected_integral["integral"]) else float("nan"),
            "gyro_integral_sample_count": corrected_integral["sample_count"],
            "gyro_integral_max_gap_s": corrected_integral["max_gap_s"],
            "orientation_s0_to_s1_yaw_deg": imu_yaw_delta,
            "timing": imu_stats,
        },
        "odom": {
            "frame_ids": odom_frame_ids,
            "child_frame_ids": odom_child_ids,
            "s0_median_x_m": odom_s0_x,
            "s0_median_y_m": odom_s0_y,
            "s0_count": odom_s0_count,
            "s1_median_x_m": odom_s1_x,
            "s1_median_y_m": odom_s1_y,
            "s1_count": odom_s1_count,
            "delta_x_m": odom_dx,
            "delta_y_m": odom_dy,
            "translation_m": math.hypot(odom_dx, odom_dy),
            "delta_yaw_deg": odom_yaw_delta,
            "timing": odom_stats,
        },
        "tf": {
            "edge": f"{TARGET_TF_PARENT}->{TARGET_TF_CHILD}",
            "s0_median_x_m": tf_s0_x,
            "s0_median_y_m": tf_s0_y,
            "s0_count": tf_s0_count,
            "s1_median_x_m": tf_s1_x,
            "s1_median_y_m": tf_s1_y,
            "s1_count": tf_s1_count,
            "delta_x_m": tf_dx,
            "delta_y_m": tf_dy,
            "translation_m": math.hypot(tf_dx, tf_dy),
            "delta_yaw_deg": tf_yaw_delta,
            "audit": target_tf_audit,
        },
        "static_tf": static_tf,
        "scan": {
            "frame_ids": scan_frame_ids,
            "count": len(bag.scan),
            "s0_count": sum(
                1 for item in bag.scan
                if int(s0["representative_header_start"]) <= item.header_ns <= int(s0["representative_header_end"])
            ),
            "s1_count": sum(
                1 for item in bag.scan
                if int(s1["representative_header_start"]) <= item.header_ns <= int(s1["representative_header_end"])
            ),
            "timing": scan_stats,
            "registration": "not performed by this tool; use analyze_e01_scan.py with scan_windows.csv",
        },
        "yaw_chain": {
            "imu_orientation_delta_deg": imu_yaw_delta,
            "odom_pose_delta_deg": odom_yaw_delta,
            "tf_yaw_delta_deg": tf_yaw_delta,
            "imu_minus_odom_deg": imu_yaw_delta - odom_yaw_delta,
            "odom_minus_tf_deg": odom_yaw_delta - tf_yaw_delta,
            "evidence_boundary": (
                "IMU orientation, Odom pose yaw and target TF yaw may be one copy/publication chain; "
                "agreement is not independent physical validation."
            ),
        },
        "odom_tf_consistency": odom_tf_consistency,
        "timing": {
            "cmd_vel": cmd_stats,
            "imu": imu_stats,
            "odom": odom_stats,
            "scan": scan_stats,
            "target_tf": tf_stats,
        },
        "field_observation": parse_field_observation(bag.uri.parent),
    }

    metrics_rows = [
        {"category": "command", "metric": "direction", "value": direction, "unit": ""},
        {"category": "command", "metric": "angular_z_median", "value": rotation.median, "unit": "rad/s"},
        {"category": "command", "metric": "nonzero_count", "value": cmd_nonzero_count, "unit": "messages"},
        {"category": "command", "metric": "zero_count", "value": zero_cmd_count, "unit": "messages"},
        {"category": "command", "metric": "sample_span", "value": rotation.sample_span_s, "unit": "s"},
        {"category": "command", "metric": "estimated_hold", "value": rotation.estimated_hold_s, "unit": "s"},
        {"category": "command", "metric": "actual_rate", "value": cmd_rate, "unit": "Hz"},
        {"category": "imu", "metric": "gyro_bias_s0", "value": gyro_bias, "unit": "rad/s"},
        {"category": "imu", "metric": "gyro_r1_median", "value": gyro_median, "unit": "rad/s"},
        {"category": "imu", "metric": "gyro_integral_raw", "value": summary["imu"]["gyro_integral_raw_deg"], "unit": "deg"},
        {"category": "imu", "metric": "gyro_integral_bias_corrected", "value": summary["imu"]["gyro_integral_bias_corrected_deg"], "unit": "deg"},
        {"category": "yaw", "metric": "imu_orientation_s0_to_s1", "value": imu_yaw_delta, "unit": "deg"},
        {"category": "yaw", "metric": "odom_pose_s0_to_s1", "value": odom_yaw_delta, "unit": "deg"},
        {"category": "yaw", "metric": "tf_s0_to_s1", "value": tf_yaw_delta, "unit": "deg"},
        {"category": "odom", "metric": "delta_x", "value": odom_dx, "unit": "m"},
        {"category": "odom", "metric": "delta_y", "value": odom_dy, "unit": "m"},
        {"category": "odom", "metric": "translation", "value": math.hypot(odom_dx, odom_dy), "unit": "m"},
        {"category": "tf", "metric": "delta_x", "value": tf_dx, "unit": "m"},
        {"category": "tf", "metric": "delta_y", "value": tf_dy, "unit": "m"},
        {"category": "tf", "metric": "translation", "value": math.hypot(tf_dx, tf_dy), "unit": "m"},
        {"category": "tf", "metric": "max_gap", "value": tf_stats["max_gap_s"], "unit": "s"},
        {"category": "static_tf", "metric": "base_footprint_to_laser_match_expected", "value": static_tf["match_expected"], "unit": "bool"},
    ]
    return summary, metrics_rows


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def format_num(value: Any, digits: int = 4) -> str:
    if not finite(value):
        return "Unknown"
    return f"{float(value):.{digits}f}"


def write_report(path: Path, summary: Dict[str, Any], windows: Sequence[Dict[str, Any]]) -> None:
    command = summary["command"]
    imu = summary["imu"]
    odom = summary["odom"]
    tf = summary["tf"]
    static_tf = summary["static_tf"]
    yaw_chain = summary["yaw_chain"]
    consistency = summary["odom_tf_consistency"]
    field = summary.get("field_observation", {})

    lines = [
        f"# Single Rotation Analysis — {summary['run_id']}",
        "",
        "## Confirmed Fact",
        "",
        f"- [CF] Bag duration: {format_num(summary['bag_duration_s'], 3)} s.",
        f"- [CF] Command direction: {command['direction']}; angular.z median={format_num(command['angular_z_median_rad_s'], 4)} rad/s.",
        f"- [CF] /cmd_vel: non-zero={command['nonzero_count']}, zero={command['zero_count']}, total={command['total_count']}.",
        f"- [CF] Command sample span={format_num(command['sample_span_s'], 3)} s; estimated hold={format_num(command['estimated_hold_s'], 3)} s; rate={format_num(command['actual_rate_hz'], 3)} Hz.",
        f"- [CF] Static TF base_footprint->laser found={static_tf['found']}; expected transform match={static_tf['match_expected']}.",
        f"- [CF] Odom S0->S1: dx={format_num(odom['delta_x_m'])} m, dy={format_num(odom['delta_y_m'])} m, translation={format_num(odom['translation_m'])} m, yaw={format_num(odom['delta_yaw_deg'], 3)}°.",
        f"- [CF] TF S0->S1: dx={format_num(tf['delta_x_m'])} m, dy={format_num(tf['delta_y_m'])} m, translation={format_num(tf['translation_m'])} m, yaw={format_num(tf['delta_yaw_deg'], 3)}°.",
        f"- [CF] IMU gyro integral: raw={format_num(imu['gyro_integral_raw_deg'], 3)}°, bias-corrected={format_num(imu['gyro_integral_bias_corrected_deg'], 3)}°.",
        f"- [CF] IMU orientation S0->S1 yaw={format_num(imu['orientation_s0_to_s1_yaw_deg'], 3)}°.",
        "",
        "## Direct Observation",
        "",
    ]
    if field:
        for key in ("Actual rotation angle", "Center net displacement", "Displacement direction", "Observer notes"):
            if field.get(key):
                lines.append(f"- [DO] {key}: {field[key]}")
    else:
        lines.append("- [UNK] field_observation.txt was not found or contained no parsed fields.")

    lines.extend([
        "",
        "## Inference",
        "",
        f"- [INF] Odom and target TF are consistent within configured tolerance: {consistency['consistent_within_tolerance']}.",
        "- [INF] Similar IMU-orientation, Odom-yaw and TF-yaw values indicate copy/publication consistency, not three independent physical measurements.",
        "",
        "## Unknown",
        "",
        "- [UNK] This tool does not perform LaserScan registration. Use analyze_e01_scan.py with scan_windows.csv.",
        "- [UNK] A physical center-displacement conclusion requires a completed field observation or an independent Scan result.",
        "",
        "## Event windows",
        "",
        "| Stage | Kind | Start bag ns | End bag ns | Duration s |",
        "|---|---|---:|---:|---:|",
    ])
    for row in windows:
        lines.append(
            f"| {row['stage']} | {row['kind']} | {row['start_bag_time']} | {row['end_bag_time']} | {float(row['duration_s']):.3f} |"
        )

    lines.extend([
        "",
        "No robot command was published, no bag was played, and no runtime code was modified.",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="rosbag2 directory")
    parser.add_argument("--out-dir", required=True, help="analysis output directory")
    parser.add_argument("--run-id", default=None, help="run identifier; defaults to bag parent directory name")
    parser.add_argument("--cmd-sample-gap-s", type=float, default=0.5)
    parser.add_argument("--main-duration-s", type=float, default=3.0)
    parser.add_argument("--main-peak", type=float, default=0.08)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--static-window-s", type=float, default=10.0)
    parser.add_argument("--expected-laser-x", type=float, default=0.105)
    parser.add_argument("--expected-laser-y", type=float, default=0.0)
    parser.add_argument("--expected-laser-z", type=float, default=0.210)
    parser.add_argument("--static-tf-tolerance", type=float, default=1e-6)
    parser.add_argument("--odom-tf-translation-tolerance", type=float, default=1e-6)
    parser.add_argument("--odom-tf-yaw-tolerance-deg", type=float, default=1e-4)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    bag_path = Path(args.bag).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    if not bag_path.is_dir():
        print(f"ERROR: bag directory not found: {bag_path}", file=sys.stderr)
        return 2
    args.run_id = args.run_id or bag_path.parent.name

    try:
        bag = load_bag(bag_path)
        if not bag.cmd_vel:
            raise RuntimeError("/cmd_vel is missing or empty")
        if not bag.odom:
            raise RuntimeError("/odom_combined is missing or empty")
        if not bag.imu:
            raise RuntimeError("/mobile_base/sensors/imu_data is missing or empty")

        intervals, threshold, zero_count = detect_command_intervals(
            bag.cmd_vel,
            max_sample_gap_s=args.cmd_sample_gap_s,
        )
        rotation, extras = select_single_rotation(
            intervals,
            min_duration_s=args.main_duration_s,
            min_peak=args.main_peak,
        )
        if extras:
            details = ", ".join(
                f"sign={item.sign}, span={item.sample_span_s:.3f}s, n={item.sample_count}"
                for item in extras
            )
            raise RuntimeError(
                "Additional non-zero command intervals were detected; refusing to silently merge them: " + details
            )

        windows = build_event_windows(
            bag,
            rotation,
            settle_s=args.settle_s,
            static_window_s=args.static_window_s,
        )
        summary, metrics_rows = analyze(
            bag,
            windows,
            rotation,
            intervals,
            zero_count,
            threshold,
            args,
        )

        out_dir.mkdir(parents=True, exist_ok=True)
        write_json(out_dir / "summary.json", summary)
        write_csv(out_dir / "metrics.csv", metrics_rows)
        write_csv(out_dir / "event_windows.csv", windows)

        by_stage = {row["stage"]: row for row in windows}
        scan_windows = [
            {
                "stage": "S0",
                "start_bag_time": by_stage["S0"]["start_bag_time"],
                "end_bag_time": by_stage["S0"]["end_bag_time"],
            },
            {
                "stage": "S4",
                "start_bag_time": by_stage["S1"]["start_bag_time"],
                "end_bag_time": by_stage["S1"]["end_bag_time"],
            },
        ]
        write_csv(out_dir / "scan_windows.csv", scan_windows)
        write_report(out_dir / "report.md", summary, windows)

        print(
            "SINGLE_ROTATION_OK "
            f"run_id={args.run_id} direction={summary['command']['direction']} "
            f"cmd_nonzero={summary['command']['nonzero_count']} "
            f"odom_yaw_deg={summary['odom']['delta_yaw_deg']:.3f} "
            f"imu_gyro_deg={summary['imu']['gyro_integral_bias_corrected_deg']:.3f} "
            f"out_dir={out_dir}"
        )
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 10


if __name__ == "__main__":
    raise SystemExit(main())
