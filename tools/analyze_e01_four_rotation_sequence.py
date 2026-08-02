#!/usr/bin/env python3
"""E01_dynamic_yaw_90_01 Phase-1 offline analyzer.

Workflow
--------
Step A (audit):
    * Read the Jetson-local rosbag.
    * Audit topic timing, duplicate stamps, target TF edge and static TF.
    * Detect raw non-zero /cmd_vel intervals.
    * Identify four candidate main rotations, attach short correction nudges,
      validate each rotation against IMU gyro and odom yaw, and build S0..S4
      representative static windows.
    * Write only preview outputs. It does NOT compute final yaw/translation
      metrics when the event-window gate fails.

Step B (metrics):
    * Requires a human-confirmed windows CSV.
    * Computes yaw-chain deltas, raw/bias-corrected gyro integration,
      static-platform yaw medians, odom translation, and a wheel-velocity
      reference trajectory.
    * No scan registration and no root-cause declaration.

Important time-base rules
-------------------------
* /cmd_vel and /robotvel have no header: use rosbag timestamps.
* Header-time boundaries are mapped by nearest Odom bag timestamp -> that
  message's header stamp.
* TF missing-location classification uses header-stamp boundaries only.
* Wheel-velocity trajectory uses IMU yaw matched by bag timestamp and expressed
  relative to S0 yaw.
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
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


NS_PER_S = 1_000_000_000
TARGET_TF_PARENT = "odom_combined"
TARGET_TF_CHILD = "base_footprint"
STATIC_TF_PARENT = "base_footprint"
STATIC_TF_CHILD = "laser"
EXPECTED_STAGES = ["S0", "R1", "S1", "R2", "S2", "R3", "S3", "R4", "S4"]

PHYSICAL_FINAL_X_M = -0.66
PHYSICAL_FINAL_Y_M = 0.09
PHYSICAL_FINAL_YAW_MIN_DEG = -2.0
PHYSICAL_FINAL_YAW_MAX_DEG = -1.0


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


def sign_with_threshold(value: float, threshold: float) -> int:
    if value > threshold:
        return 1
    if value < -threshold:
        return -1
    return 0


def unwrap_angles(angles: Sequence[float]) -> List[float]:
    if not angles:
        return []
    out = [float(angles[0])]
    for i in range(1, len(angles)):
        out.append(out[-1] + wrap_pi(float(angles[i]) - float(angles[i - 1])))
    return out


def median_mad(values: Sequence[float]) -> Tuple[float, float]:
    if not values:
        return float("nan"), float("nan")
    arr = np.asarray(values, dtype=float)
    median = float(np.median(arr))
    mad = float(np.median(np.abs(arr - median)))
    return median, mad


def interval_stats(stamps_ns: Sequence[int]) -> Dict[str, float]:
    if len(stamps_ns) < 2:
        return {
            "median_interval_s": float("nan"),
            "mad_interval_s": float("nan"),
            "p95_interval_s": float("nan"),
            "p99_interval_s": float("nan"),
            "max_gap_s": float("nan"),
        }
    diffs = np.diff(np.asarray(stamps_ns, dtype=np.int64)).astype(float) / NS_PER_S
    med = float(np.median(diffs))
    return {
        "median_interval_s": med,
        "mad_interval_s": float(np.median(np.abs(diffs - med))),
        "p95_interval_s": float(np.percentile(diffs, 95)),
        "p99_interval_s": float(np.percentile(diffs, 99)),
        "max_gap_s": float(np.max(diffs)),
    }


def robust_threshold(values: Sequence[float], *, floor: float, multiplier: float = 8.0) -> float:
    """Threshold from the low-motion half of |values|, with a safe floor."""
    if not values:
        return floor
    abs_values = np.sort(np.abs(np.asarray(values, dtype=float)))
    low = abs_values[: max(10, len(abs_values) // 2)]
    med = float(np.median(low))
    mad = float(np.median(np.abs(low - med)))
    return max(floor, med + multiplier * max(mad, 1e-9))


def canonical_transform(values: Sequence[float]) -> Tuple[float, ...]:
    t = np.asarray(values[:3], dtype=float)
    q = np.asarray(values[3:], dtype=float)
    norm = float(np.linalg.norm(q))
    if norm > 0.0:
        q /= norm
    nz = np.flatnonzero(np.abs(q) > 1e-15)
    if nz.size and q[nz[0]] < 0.0:
        q = -q
    return tuple(np.round(np.concatenate([t, q]), 12))


def finite_or_blank(value: Any, digits: int = 6) -> Any:
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.{digits}f}"
    return value


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
            writer.writerow({key: row.get(key, "") for key in fields})


def nearest_index(sorted_values: Sequence[int], query: int) -> Optional[int]:
    if not sorted_values:
        return None
    i = bisect.bisect_left(sorted_values, query)
    candidates: List[int] = []
    if i < len(sorted_values):
        candidates.append(i)
    if i > 0:
        candidates.append(i - 1)
    return min(candidates, key=lambda idx: abs(sorted_values[idx] - query))


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
    except Exception as exc:  # pragma: no cover - depends on ROS environment
        raise RuntimeError(
            "ROS 2 Python dependencies are unavailable. Source /opt/ros/humble/setup.bash "
            "and the workspace install/setup.bash before running."
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
        all_bag_stamps.append(int(bag_ns))
        msg = deserialize_message(raw, message_classes[topic])
        if topic == "/odom_combined":
            bag.odom.append(StampedRecord(int(bag_ns), header_stamp_ns(msg.header), msg))
        elif topic == "/mobile_base/sensors/imu_data":
            bag.imu.append(StampedRecord(int(bag_ns), header_stamp_ns(msg.header), msg))
        elif topic == "/scan":
            bag.scan.append(StampedRecord(int(bag_ns), header_stamp_ns(msg.header), msg))
        elif topic == "/robotvel":
            bag.robotvel.append(BagRecord(int(bag_ns), msg))
        elif topic == "/tf":
            bag.tf.append(BagRecord(int(bag_ns), msg))
        elif topic == "/tf_static":
            bag.tf_static.append(BagRecord(int(bag_ns), msg))
        elif topic == "/cmd_vel":
            bag.cmd_vel.append(BagRecord(int(bag_ns), msg))

    if not all_bag_stamps:
        raise RuntimeError(f"Bag contains no messages: {uri}")
    bag.bag_start_ns = min(all_bag_stamps)
    bag.bag_end_ns = max(all_bag_stamps)
    return bag


# ---------------------------------------------------------------------------
# Raw-data audit
# ---------------------------------------------------------------------------
def message_content_key(msg: Any) -> str:
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        p = msg.pose.pose
        q = p.orientation
        return (
            f"p={p.position.x:.9f},{p.position.y:.9f},{p.position.z:.9f};"
            f"q={q.x:.9f},{q.y:.9f},{q.z:.9f},{q.w:.9f}"
        )
    if hasattr(msg, "orientation") and hasattr(msg, "angular_velocity"):
        q = msg.orientation
        return (
            f"q={q.x:.9f},{q.y:.9f},{q.z:.9f},{q.w:.9f};"
            f"gyro_z={msg.angular_velocity.z:.9f}"
        )
    if hasattr(msg, "ranges"):
        return f"scan={len(msg.ranges)};amin={msg.angle_min:.9f};amax={msg.angle_max:.9f}"
    return repr(msg)[:200]


def audit_stamped_topic(name: str, records: Sequence[StampedRecord], msg_type: str) -> Dict[str, Any]:
    header_stamps = [record.header_ns for record in records]
    grouped: Dict[int, set] = defaultdict(set)
    for record in records:
        grouped[record.header_ns].add(message_content_key(record.msg))
    row: Dict[str, Any] = {
        "source": name,
        "msg_type": msg_type,
        "count": len(records),
        "time_basis": "header_stamp",
        "header_monotonic": all(a <= b for a, b in zip(header_stamps, header_stamps[1:])),
        "duplicate_stamps": len(header_stamps) - len(set(header_stamps)),
        "conflicting_duplicate_stamps": sum(1 for values in grouped.values() if len(values) > 1),
    }
    row.update(interval_stats(header_stamps))
    return row


def audit_no_header_topic(name: str, records: Sequence[BagRecord], msg_type: str) -> Dict[str, Any]:
    bag_stamps = [record.bag_ns for record in records]
    row: Dict[str, Any] = {
        "source": name,
        "msg_type": msg_type,
        "count": len(records),
        "time_basis": "bag_timestamp",
        "header_monotonic": "",
        "duplicate_stamps": "",
        "conflicting_duplicate_stamps": "",
        "note": "no header; alignment_limitation=no_header",
    }
    row.update(interval_stats(bag_stamps))
    return row


@dataclass
class ExpandedTF:
    bag_ns: int
    header_ns: int
    parent: str
    child: str
    values: Tuple[float, float, float, float, float, float, float]
    tf_message_size: int


def expand_tf(records: Sequence[BagRecord]) -> List[ExpandedTF]:
    expanded: List[ExpandedTF] = []
    for record in records:
        size = len(record.msg.transforms)
        for transform in record.msg.transforms:
            t = transform.transform.translation
            q = transform.transform.rotation
            expanded.append(
                ExpandedTF(
                    bag_ns=record.bag_ns,
                    header_ns=header_stamp_ns(transform.header),
                    parent=transform.header.frame_id,
                    child=transform.child_frame_id,
                    values=(float(t.x), float(t.y), float(t.z), float(q.x), float(q.y), float(q.z), float(q.w)),
                    tf_message_size=size,
                )
            )
    return expanded


def locate_header_stamps(stamps: Sequence[int], header_start: int, header_end: int, edge_s: float = 10.0) -> str:
    if not stamps:
        return "none"
    edge_ns = int(edge_s * NS_PER_S)
    near_start = sum(1 for stamp in stamps if stamp - header_start <= edge_ns)
    near_end = sum(1 for stamp in stamps if header_end - stamp <= edge_ns)
    middle = len(stamps) - near_start - near_end
    return f"near_start_{edge_s:g}s={near_start} near_end_{edge_s:g}s={near_end} middle={middle}"


def audit_tf_stream(records: Sequence[BagRecord], expanded: Sequence[ExpandedTF]) -> Dict[str, Any]:
    bag_stamps = [record.bag_ns for record in records]
    row: Dict[str, Any] = {
        "source": "/tf TFMessage stream",
        "msg_type": "tf2_msgs/msg/TFMessage",
        "count": len(records),
        "time_basis": "bag_timestamp",
        "all_transform_count": len(expanded),
        "tf_message_size_distribution": json.dumps(Counter(len(record.msg.transforms) for record in records), sort_keys=True),
        "edge_counts": json.dumps(Counter(f"{item.parent}->{item.child}" for item in expanded), sort_keys=True),
    }
    row.update(interval_stats(bag_stamps))
    return row


def audit_target_tf(expanded: Sequence[ExpandedTF], odom: Sequence[StampedRecord]) -> Dict[str, Any]:
    target = [
        item for item in expanded
        if item.parent == TARGET_TF_PARENT and item.child == TARGET_TF_CHILD
    ]
    tf_stamps = [item.header_ns for item in target]
    odom_stamps = [record.header_ns for record in odom]
    tf_set = set(tf_stamps)
    odom_set = set(odom_stamps)
    grouped: Dict[int, set] = defaultdict(set)
    for item in target:
        grouped[item.header_ns].add(canonical_transform(item.values))

    if odom_stamps:
        header_start = min(odom_stamps)
        header_end = max(odom_stamps)
    elif tf_stamps:
        header_start = min(tf_stamps)
        header_end = max(tf_stamps)
    else:
        header_start = header_end = 0

    row: Dict[str, Any] = {
        "source": f"tf_edge:{TARGET_TF_PARENT}->{TARGET_TF_CHILD}",
        "msg_type": "geometry_msgs/msg/TransformStamped",
        "count": len(target),
        "time_basis": "header_stamp",
        "header_monotonic": all(a <= b for a, b in zip(tf_stamps, tf_stamps[1:])),
        "duplicate_stamps": len(tf_stamps) - len(tf_set),
        "conflicting_duplicate_stamps": sum(1 for values in grouped.values() if len(values) > 1),
        "exact_unique_stamp_match_vs_odom": len(tf_set & odom_set),
        "odom_missing_tf": len(odom_set - tf_set),
        "tf_no_odom": len(tf_set - odom_set),
        "tf_no_odom_location": locate_header_stamps(sorted(tf_set - odom_set), header_start, header_end),
    }
    row.update(interval_stats(tf_stamps))
    return row


def audit_static_tf(records: Sequence[BagRecord]) -> Dict[str, Any]:
    transforms: List[Any] = []
    for record in records:
        transforms.extend(record.msg.transforms)
    grouped: Dict[str, set] = defaultdict(set)
    laser_description = ""
    for transform in transforms:
        t = transform.transform.translation
        q = transform.transform.rotation
        key = f"{transform.header.frame_id}->{transform.child_frame_id}"
        values = (float(t.x), float(t.y), float(t.z), float(q.x), float(q.y), float(q.z), float(q.w))
        grouped[key].add(canonical_transform(values))
        if transform.header.frame_id == STATIC_TF_PARENT and transform.child_frame_id == STATIC_TF_CHILD:
            laser_description = (
                f"translation=[{t.x:.6f},{t.y:.6f},{t.z:.6f}] "
                f"rotation=[{q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f}]"
            )
    return {
        "source": "/tf_static",
        "msg_type": "tf2_msgs/msg/TFMessage",
        "count": len(transforms),
        "time_basis": "latched_static",
        "edges": ";".join(sorted(grouped)),
        "conflicting_static_transforms": sum(1 for values in grouped.values() if len(values) > 1),
        "base_footprint_to_laser": laser_description,
        "note": "latched static transform; no rate computed",
    }


# ---------------------------------------------------------------------------
# Event detection and validation
# ---------------------------------------------------------------------------
@dataclass
class CommandInterval:
    start_bag_ns: int
    end_bag_ns: int
    sign: int
    sample_count: int
    peak_abs: float
    mean: float
    duration_s: float
    is_nudge: bool
    attached_to: Optional[str] = None
    attachment_role: Optional[str] = None


@dataclass
class RotationCandidate:
    stage: str
    cmd_sign: int
    main_start_bag_ns: int
    main_end_bag_ns: int
    peak_abs: float
    mean_cmd: float
    corrections: List[CommandInterval] = field(default_factory=list)
    effective_start_bag_ns: int = 0
    effective_end_bag_ns: int = 0

    def refresh_effective_bounds(self) -> None:
        starts = [self.main_start_bag_ns] + [item.start_bag_ns for item in self.corrections]
        ends = [self.main_end_bag_ns] + [item.end_bag_ns for item in self.corrections]
        self.effective_start_bag_ns = min(starts)
        self.effective_end_bag_ns = max(ends)


def detect_cmd_intervals(
    records: Sequence[BagRecord],
    *,
    max_sample_gap_s: float = 0.5,
    main_duration_s: float = 3.0,
    main_peak: float = 0.08,
) -> Tuple[List[CommandInterval], float]:
    if not records:
        return [], float("nan")

    nonzero_magnitudes = [abs(float(record.msg.angular.z)) for record in records if abs(float(record.msg.angular.z)) > 1e-6]
    command_threshold = max(0.03, 0.5 * float(np.median(nonzero_magnitudes))) if nonzero_magnitudes else 0.05
    max_gap_ns = int(max_sample_gap_s * NS_PER_S)

    groups: List[List[Tuple[int, float]]] = []
    current: List[Tuple[int, float]] = []
    previous_bag_ns: Optional[int] = None
    previous_sign: Optional[int] = None

    for record in records:
        value = float(record.msg.angular.z)
        current_sign = sign_with_threshold(value, command_threshold)
        if current_sign == 0:
            if current:
                groups.append(current)
                current = []
            previous_bag_ns = record.bag_ns
            previous_sign = None
            continue

        split = False
        if current and previous_bag_ns is not None:
            split = (record.bag_ns - previous_bag_ns > max_gap_ns) or (current_sign != previous_sign)
        if split:
            groups.append(current)
            current = []
        current.append((record.bag_ns, value))
        previous_bag_ns = record.bag_ns
        previous_sign = current_sign

    if current:
        groups.append(current)

    intervals: List[CommandInterval] = []
    for group in groups:
        values = [value for _, value in group]
        start = group[0][0]
        end = group[-1][0]
        duration_s = ns_to_s(end - start)
        peak_abs = max(abs(value) for value in values)
        mean = float(np.mean(values))
        sign = 1 if mean > 0 else -1
        is_nudge = duration_s < main_duration_s or peak_abs < main_peak
        intervals.append(
            CommandInterval(
                start_bag_ns=start,
                end_bag_ns=end,
                sign=sign,
                sample_count=len(group),
                peak_abs=peak_abs,
                mean=mean,
                duration_s=duration_s,
                is_nudge=is_nudge,
            )
        )
    return intervals, command_threshold


def attach_nudges(
    intervals: Sequence[CommandInterval],
    *,
    attach_gap_s: float = 5.0,
) -> Tuple[List[RotationCandidate], List[CommandInterval], bool]:
    main_intervals = [item for item in intervals if not item.is_nudge]
    main_intervals.sort(key=lambda item: item.start_bag_ns)
    rotations: List[RotationCandidate] = []
    for index, item in enumerate(main_intervals, start=1):
        candidate = RotationCandidate(
            stage=f"R{index}",
            cmd_sign=item.sign,
            main_start_bag_ns=item.start_bag_ns,
            main_end_bag_ns=item.end_bag_ns,
            peak_abs=item.peak_abs,
            mean_cmd=item.mean,
        )
        candidate.refresh_effective_bounds()
        rotations.append(candidate)

    standalone: List[CommandInterval] = []
    attach_gap_ns = int(attach_gap_s * NS_PER_S)
    for nudge in [item for item in intervals if item.is_nudge]:
        best: Optional[RotationCandidate] = None
        best_gap: Optional[int] = None
        for rotation in rotations:
            if nudge.start_bag_ns >= rotation.main_end_bag_ns:
                gap = nudge.start_bag_ns - rotation.main_end_bag_ns
            elif nudge.end_bag_ns <= rotation.main_start_bag_ns:
                gap = rotation.main_start_bag_ns - nudge.end_bag_ns
            else:
                gap = 0
            if best_gap is None or gap < best_gap:
                best = rotation
                best_gap = gap
        if best is not None and best_gap is not None and best_gap <= attach_gap_ns:
            nudge.attached_to = best.stage
            nudge.attachment_role = "correction_nudge"
            best.corrections.append(nudge)
            best.refresh_effective_bounds()
        else:
            nudge.attachment_role = "standalone"
            standalone.append(nudge)

    # A substantial unassigned command interval blocks automatic acceptance.
    ambiguous = any(item.duration_s >= 0.5 and item.peak_abs >= 0.05 for item in standalone)
    return rotations, standalone, ambiguous


def odom_yaw_by_bag(odom: Sequence[StampedRecord]) -> List[Tuple[int, float]]:
    ordered = sorted(odom, key=lambda item: item.bag_ns)
    angles = [
        quaternion_yaw(
            item.msg.pose.pose.orientation.x,
            item.msg.pose.pose.orientation.y,
            item.msg.pose.pose.orientation.z,
            item.msg.pose.pose.orientation.w,
        )
        for item in ordered
    ]
    unwrapped = unwrap_angles(angles)
    return [(item.bag_ns, unwrapped[index]) for index, item in enumerate(ordered)]


def derivative_by_bag(samples: Sequence[Tuple[int, float]]) -> List[Tuple[int, float]]:
    out: List[Tuple[int, float]] = []
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        dt = ns_to_s(t1 - t0)
        if dt > 0.0:
            out.append((t1, (v1 - v0) / dt))
    return out


def values_in_bag_window(samples: Sequence[Tuple[int, float]], start_ns: int, end_ns: int) -> List[float]:
    return [value for stamp, value in samples if start_ns <= stamp <= end_ns]


def validate_rotation(
    rotation: RotationCandidate,
    bag: LoadedBag,
    odom_yaw_samples: Sequence[Tuple[int, float]],
    gyro_threshold: float,
    odom_rate_threshold: float,
    min_odom_delta_deg: float = 20.0,
) -> Dict[str, Any]:
    start = rotation.effective_start_bag_ns
    end = rotation.effective_end_bag_ns
    gyro_values = [float(item.msg.angular_velocity.z) for item in bag.imu if start <= item.bag_ns <= end]
    robotvel_values = [float(item.msg.z) for item in bag.robotvel if start <= item.bag_ns <= end]
    odom_values = [(stamp, yaw) for stamp, yaw in odom_yaw_samples if start <= stamp <= end]

    gyro_median = float(np.median(gyro_values)) if gyro_values else float("nan")
    robotvel_median = float(np.median(robotvel_values)) if robotvel_values else float("nan")
    odom_delta = odom_values[-1][1] - odom_values[0][1] if len(odom_values) >= 2 else float("nan")
    odom_delta_deg = math.degrees(odom_delta) if math.isfinite(odom_delta) else float("nan")

    cmd_sign = rotation.cmd_sign
    gyro_sign = sign_with_threshold(gyro_median, gyro_threshold) if math.isfinite(gyro_median) else 0
    odom_sign = sign_with_threshold(odom_delta, math.radians(min_odom_delta_deg)) if math.isfinite(odom_delta) else 0
    robotvel_sign = sign_with_threshold(robotvel_median, gyro_threshold) if math.isfinite(robotvel_median) else 0

    gyro_motion = bool(gyro_values) and abs(gyro_median) > gyro_threshold
    odom_motion = len(odom_values) >= 2 and abs(odom_delta_deg) >= min_odom_delta_deg
    cmd_gyro_match = gyro_sign != 0 and gyro_sign == cmd_sign
    cmd_odom_match = odom_sign != 0 and odom_sign == cmd_sign

    if gyro_motion and odom_motion and cmd_gyro_match and cmd_odom_match:
        confidence = "high"
    elif (cmd_gyro_match and odom_sign == 0) or (cmd_odom_match and gyro_sign == 0):
        confidence = "medium"
    else:
        confidence = "low"

    return {
        "cmd_sign": cmd_sign,
        "gyro_median_rad_s": gyro_median,
        "gyro_sign": gyro_sign,
        "gyro_motion_detected": gyro_motion,
        "robotvel_z_median": robotvel_median,
        "robotvel_sign_report_only": robotvel_sign,
        "odom_yaw_delta_deg": odom_delta_deg,
        "odom_sign": odom_sign,
        "odom_motion_detected": odom_motion,
        "cmd_gyro_match": cmd_gyro_match,
        "cmd_odom_match": cmd_odom_match,
        "confidence": confidence,
        "gyro_threshold_rad_s": gyro_threshold,
        "odom_rate_threshold_rad_s": odom_rate_threshold,
        "note": "robotvel sign is reported but does not gate the event window because its sign convention is unverified",
    }


def odom_bag_to_header_index(odom: Sequence[StampedRecord]) -> Tuple[List[int], List[int]]:
    ordered = sorted(odom, key=lambda item: item.bag_ns)
    return [item.bag_ns for item in ordered], [item.header_ns for item in ordered]


def bag_time_to_odom_header(bag_stamps: Sequence[int], header_stamps: Sequence[int], query_bag_ns: int) -> int:
    index = nearest_index(bag_stamps, query_bag_ns)
    if index is None:
        raise RuntimeError("Cannot map bag time to header time: Odom is empty")
    return int(header_stamps[index])


def choose_static_window(
    raw_start_ns: int,
    raw_end_ns: int,
    *,
    settle_s: float,
    representative_window_s: float,
) -> Tuple[int, int]:
    start = raw_start_ns + int(settle_s * NS_PER_S)
    end = raw_end_ns - int(settle_s * NS_PER_S)
    if end <= start:
        return raw_start_ns, raw_end_ns
    available = end - start
    desired = int(representative_window_s * NS_PER_S)
    if available <= desired:
        return start, end
    center = (start + end) // 2
    half = desired // 2
    return center - half, center + half


def build_event_windows(
    bag: LoadedBag,
    rotations: Sequence[RotationCandidate],
    standalone_nudges: Sequence[CommandInterval],
    ambiguous: bool,
    *,
    settle_s: float = 2.0,
    static_window_s: float = 10.0,
) -> Tuple[List[Dict[str, Any]], bool, Dict[str, Dict[str, Any]]]:
    bag_stamps, header_stamps = odom_bag_to_header_index(bag.odom)
    yaw_samples = odom_yaw_by_bag(bag.odom)
    odom_rates = derivative_by_bag(yaw_samples)
    gyro_threshold = robust_threshold([float(item.msg.angular_velocity.z) for item in bag.imu], floor=0.01)
    odom_rate_threshold = robust_threshold([value for _, value in odom_rates], floor=0.01)

    validation: Dict[str, Dict[str, Any]] = {}
    windows: List[Dict[str, Any]] = []
    for rotation in rotations:
        result = validate_rotation(rotation, bag, yaw_samples, gyro_threshold, odom_rate_threshold)
        validation[rotation.stage] = result
        windows.append({
            "stage": rotation.stage,
            "kind": "rotation",
            "main_start_bag_time": rotation.main_start_bag_ns,
            "main_end_bag_time": rotation.main_end_bag_ns,
            "start_bag_time": rotation.effective_start_bag_ns,
            "end_bag_time": rotation.effective_end_bag_ns,
            "representative_header_start": bag_time_to_odom_header(bag_stamps, header_stamps, rotation.effective_start_bag_ns),
            "representative_header_end": bag_time_to_odom_header(bag_stamps, header_stamps, rotation.effective_end_bag_ns),
            "duration_s": ns_to_s(rotation.effective_end_bag_ns - rotation.effective_start_bag_ns),
            "cmd_sign": rotation.cmd_sign,
            "cmd_peak_abs": rotation.peak_abs,
            "cmd_mean": rotation.mean_cmd,
            "correction_count": len(rotation.corrections),
            **result,
        })
        for index, correction in enumerate(rotation.corrections, start=1):
            windows.append({
                "stage": f"{rotation.stage}.correction_{index}",
                "kind": "correction_nudge",
                "start_bag_time": correction.start_bag_ns,
                "end_bag_time": correction.end_bag_ns,
                "representative_header_start": bag_time_to_odom_header(bag_stamps, header_stamps, correction.start_bag_ns),
                "representative_header_end": bag_time_to_odom_header(bag_stamps, header_stamps, correction.end_bag_ns),
                "duration_s": correction.duration_s,
                "cmd_sign": correction.sign,
                "cmd_peak_abs": correction.peak_abs,
                "cmd_mean": correction.mean,
                "attached_to": rotation.stage,
            })

    for index, nudge in enumerate(standalone_nudges, start=1):
        windows.append({
            "stage": f"nudge_{index}",
            "kind": "standalone_nudge",
            "start_bag_time": nudge.start_bag_ns,
            "end_bag_time": nudge.end_bag_ns,
            "representative_header_start": bag_time_to_odom_header(bag_stamps, header_stamps, nudge.start_bag_ns),
            "representative_header_end": bag_time_to_odom_header(bag_stamps, header_stamps, nudge.end_bag_ns),
            "duration_s": nudge.duration_s,
            "cmd_sign": nudge.sign,
            "cmd_peak_abs": nudge.peak_abs,
            "cmd_mean": nudge.mean,
        })

    rotations_sorted = sorted(rotations, key=lambda item: item.effective_start_bag_ns)
    if rotations_sorted:
        static_bounds: List[Tuple[str, int, int]] = [("S0", bag.bag_start_ns, rotations_sorted[0].effective_start_bag_ns)]
        for index in range(len(rotations_sorted) - 1):
            static_bounds.append((
                f"S{index + 1}",
                rotations_sorted[index].effective_end_bag_ns,
                rotations_sorted[index + 1].effective_start_bag_ns,
            ))
        static_bounds.append((f"S{len(rotations_sorted)}", rotations_sorted[-1].effective_end_bag_ns, bag.bag_end_ns))
        for stage, raw_start, raw_end in static_bounds:
            start, end = choose_static_window(
                raw_start,
                raw_end,
                settle_s=settle_s,
                representative_window_s=static_window_s,
            )
            windows.append({
                "stage": stage,
                "kind": "static",
                "raw_gap_start_bag_time": raw_start,
                "raw_gap_end_bag_time": raw_end,
                "start_bag_time": start,
                "end_bag_time": end,
                "representative_header_start": bag_time_to_odom_header(bag_stamps, header_stamps, start),
                "representative_header_end": bag_time_to_odom_header(bag_stamps, header_stamps, end),
                "duration_s": ns_to_s(end - start),
                "settle_excluded_s": settle_s,
            })

    windows.sort(key=lambda row: (int(row["start_bag_time"]), row["stage"]))

    stage_rows = {row["stage"]: row for row in windows if row["stage"] in EXPECTED_STAGES}
    correct_count = len(rotations_sorted) == 4
    all_expected = all(stage in stage_rows for stage in EXPECTED_STAGES)
    validation_ok = all(
        validation.get(f"R{index}", {}).get("confidence") in {"high", "medium"}
        and validation.get(f"R{index}", {}).get("gyro_motion_detected")
        and validation.get(f"R{index}", {}).get("odom_motion_detected")
        and validation.get(f"R{index}", {}).get("cmd_gyro_match")
        and validation.get(f"R{index}", {}).get("cmd_odom_match")
        for index in range(1, 5)
    ) if correct_count else False

    ordered = True
    if all_expected:
        for left, right in zip(EXPECTED_STAGES, EXPECTED_STAGES[1:]):
            if int(stage_rows[left]["end_bag_time"]) > int(stage_rows[right]["start_bag_time"]):
                ordered = False
                break
    no_static_overlap = True
    static_rows = [row for row in windows if row.get("kind") == "static"]
    motion_rows = [row for row in windows if row.get("kind") in {"rotation", "correction_nudge", "standalone_nudge"}]
    for static in static_rows:
        for motion in motion_rows:
            if max(int(static["start_bag_time"]), int(motion["start_bag_time"])) < min(int(static["end_bag_time"]), int(motion["end_bag_time"])):
                no_static_overlap = False
                break

    event_window_valid = (
        not ambiguous
        and correct_count
        and all_expected
        and validation_ok
        and ordered
        and no_static_overlap
    )
    for row in windows:
        row["event_window_valid"] = event_window_valid
        row["event_window_gate_reason"] = (
            f"ambiguous={ambiguous};four_rotations={correct_count};all_expected={all_expected};"
            f"validation_ok={validation_ok};ordered={ordered};no_static_overlap={no_static_overlap}"
        )
    return windows, event_window_valid, validation


# ---------------------------------------------------------------------------
# Step B metrics
# ---------------------------------------------------------------------------
def integrate_trapezoid(
    stamps_ns: Sequence[int],
    values: Sequence[float],
    start_ns: int,
    end_ns: int,
    *,
    bias: float = 0.0,
) -> Tuple[float, float, int]:
    samples = [(stamp, value) for stamp, value in zip(stamps_ns, values) if start_ns <= stamp <= end_ns]
    if len(samples) < 2:
        return float("nan"), float("nan"), len(samples)
    integral = 0.0
    max_gap = 0.0
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        dt = ns_to_s(t1 - t0)
        max_gap = max(max_gap, dt)
        integral += 0.5 * ((v0 - bias) + (v1 - bias)) * dt
    return integral, max_gap, len(samples)


def angle_window_metrics(stamps_ns: Sequence[int], unwrapped: Sequence[float], start_ns: int, end_ns: int) -> Tuple[float, float, float]:
    values = [value for stamp, value in zip(stamps_ns, unwrapped) if start_ns <= stamp <= end_ns]
    if len(values) < 2:
        return float("nan"), float("nan"), float("nan")
    return values[0], values[-1], values[-1] - values[0]


def median_angle_window(stamps_ns: Sequence[int], unwrapped: Sequence[float], start_ns: int, end_ns: int) -> Tuple[float, float]:
    values = [value for stamp, value in zip(stamps_ns, unwrapped) if start_ns <= stamp <= end_ns]
    return median_mad(values)


def median_pose_window(odom: Sequence[StampedRecord], start_header_ns: int, end_header_ns: int) -> Tuple[float, float]:
    points = [
        (float(item.msg.pose.pose.position.x), float(item.msg.pose.pose.position.y))
        for item in odom
        if start_header_ns <= item.header_ns <= end_header_ns
    ]
    if not points:
        return float("nan"), float("nan")
    return float(np.median([point[0] for point in points])), float(np.median([point[1] for point in points]))


def load_confirmed_windows(path: Path) -> Dict[str, Dict[str, str]]:
    with path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    by_stage = {row["stage"]: row for row in rows if row.get("stage") in EXPECTED_STAGES}
    missing = [stage for stage in EXPECTED_STAGES if stage not in by_stage]
    if missing:
        raise RuntimeError(f"Confirmed windows missing required stages: {missing}")
    return by_stage


def int_field(row: Dict[str, str], key: str) -> int:
    value = row.get(key, "")
    if value == "":
        raise RuntimeError(f"Confirmed window field is empty: {row.get('stage')}:{key}")
    return int(value)


def build_yaw_sources(bag: LoadedBag, expanded_tf: Sequence[ExpandedTF]) -> Dict[str, Tuple[List[int], List[float]]]:
    imu_header = [item.header_ns for item in bag.imu]
    imu_orientation = [
        quaternion_yaw(item.msg.orientation.x, item.msg.orientation.y, item.msg.orientation.z, item.msg.orientation.w)
        for item in bag.imu
    ]
    odom_header = [item.header_ns for item in bag.odom]
    odom_orientation = [
        quaternion_yaw(
            item.msg.pose.pose.orientation.x,
            item.msg.pose.pose.orientation.y,
            item.msg.pose.pose.orientation.z,
            item.msg.pose.pose.orientation.w,
        )
        for item in bag.odom
    ]
    target = [
        item for item in expanded_tf
        if item.parent == TARGET_TF_PARENT and item.child == TARGET_TF_CHILD
    ]
    target.sort(key=lambda item: item.header_ns)
    tf_header = [item.header_ns for item in target]
    tf_orientation = [quaternion_yaw(*item.values[3:]) for item in target]
    return {
        "imu_orientation": (imu_header, unwrap_angles(imu_orientation)),
        "odom_pose": (odom_header, unwrap_angles(odom_orientation)),
        "tf_yaw": (tf_header, unwrap_angles(tf_orientation)),
    }


def nearest_imu_yaw_by_bag(bag: LoadedBag) -> Tuple[List[int], List[float]]:
    ordered = sorted(bag.imu, key=lambda item: item.bag_ns)
    angles = [
        quaternion_yaw(item.msg.orientation.x, item.msg.orientation.y, item.msg.orientation.z, item.msg.orientation.w)
        for item in ordered
    ]
    return [item.bag_ns for item in ordered], unwrap_angles(angles)


def integrate_robotvel_reference(
    bag: LoadedBag,
    start_bag_ns: int,
    end_bag_ns: int,
    s0_yaw_rad: float,
) -> Tuple[float, float, float]:
    imu_bag_ns, imu_yaw = nearest_imu_yaw_by_bag(bag)
    samples = [item for item in bag.robotvel if start_bag_ns <= item.bag_ns <= end_bag_ns]
    if len(samples) < 2:
        return float("nan"), float("nan"), float("nan")

    x = y = 0.0
    max_alignment_s = 0.0
    for previous, current in zip(samples, samples[1:]):
        index = nearest_index(imu_bag_ns, previous.bag_ns)
        if index is None:
            continue
        max_alignment_s = max(max_alignment_s, abs(imu_bag_ns[index] - previous.bag_ns) / NS_PER_S)
        yaw_relative = imu_yaw[index] - s0_yaw_rad
        vx = float(previous.msg.x)
        vy = float(previous.msg.y)
        dt = ns_to_s(current.bag_ns - previous.bag_ns)
        x += (vx * math.cos(yaw_relative) - vy * math.sin(yaw_relative)) * dt
        y += (vx * math.sin(yaw_relative) + vy * math.cos(yaw_relative)) * dt
    return x, y, max_alignment_s


def physical_ground_truth_text() -> str:
    physical_disp = math.hypot(PHYSICAL_FINAL_X_M, PHYSICAL_FINAL_Y_M)
    return f"""# E01 Physical Ground Truth

The following values were measured on site by the user after the complete
0° -> +90° -> 0° -> -90° -> 0° sequence.

- final physical x displacement: {PHYSICAL_FINAL_X_M:+.3f} m
- final physical y displacement: {PHYSICAL_FINAL_Y_M:+.3f} m
- final physical planar displacement magnitude: {physical_disp:.3f} m
- final physical yaw residual: clockwise approximately 1° to 2°
- ROS counter-clockwise-positive representation: approximately {PHYSICAL_FINAL_YAW_MIN_DEG:.1f}° to {PHYSICAL_FINAL_YAW_MAX_DEG:.1f}°

Limitations:

- x/y axis convention is user-reported and has not been independently aligned
  with odom_combined.
- The measurement is valid only for the final S0 -> S4 comparison; it is not a
  per-segment ground truth.
- The yaw is an interval, not an exact -1.5° measurement.
"""


def ensure_physical_ground_truth(out_dir: Path) -> Path:
    path = out_dir / "physical_ground_truth.md"
    expected = physical_ground_truth_text()
    if not path.exists():
        path.write_text(expected, encoding="utf-8")
    return path


def run_metrics(bag: LoadedBag, windows_csv: Path, out_dir: Path) -> None:
    confirmed = load_confirmed_windows(windows_csv)
    expanded = expand_tf(bag.tf)
    yaw_sources = build_yaw_sources(bag, expanded)

    imu_header = [item.header_ns for item in bag.imu]
    imu_gyro = [float(item.msg.angular_velocity.z) for item in bag.imu]
    odom_header = [item.header_ns for item in bag.odom]
    odom_twist = [float(item.msg.twist.twist.angular.z) for item in bag.odom]
    robotvel_bag = [item.bag_ns for item in bag.robotvel]
    robotvel_z = [float(item.msg.z) for item in bag.robotvel]

    s0 = confirmed["S0"]
    s0_header_start = int_field(s0, "representative_header_start")
    s0_header_end = int_field(s0, "representative_header_end")
    s0_gyro_values = [value for stamp, value in zip(imu_header, imu_gyro) if s0_header_start <= stamp <= s0_header_end]
    gyro_bias, gyro_bias_mad = median_mad(s0_gyro_values)
    if not math.isfinite(gyro_bias):
        raise RuntimeError("Cannot estimate gyro bias from S0")

    metrics: List[Dict[str, Any]] = []
    angular_velocity_source = "imu.angular_velocity.z"

    for stage in ["R1", "R2", "R3", "R4"]:
        row = confirmed[stage]
        header_start = int_field(row, "representative_header_start")
        header_end = int_field(row, "representative_header_end")
        bag_start = int_field(row, "start_bag_time")
        bag_end = int_field(row, "end_bag_time")
        gyro_window = [value for stamp, value in zip(imu_header, imu_gyro) if header_start <= stamp <= header_end]
        gyro_peak = float(np.max(np.abs(gyro_window))) if gyro_window else float("nan")
        gyro_mean = float(np.mean(gyro_window)) if gyro_window else float("nan")

        for source_name, (stamps, angles) in yaw_sources.items():
            start_yaw, end_yaw, delta = angle_window_metrics(stamps, angles, header_start, header_end)
            metrics.append({
                "stage": stage,
                "metric_scope": "effective_rotation",
                "source": source_name,
                "start_yaw_deg": math.degrees(start_yaw) if math.isfinite(start_yaw) else "",
                "end_yaw_deg": math.degrees(end_yaw) if math.isfinite(end_yaw) else "",
                "delta_yaw_deg": math.degrees(delta) if math.isfinite(delta) else "",
                "peak_angular_velocity_rad_s": gyro_peak,
                "mean_angular_velocity_rad_s": gyro_mean,
                "angular_velocity_source": angular_velocity_source,
                "time_basis": "header_stamp",
            })

        raw, raw_gap, raw_count = integrate_trapezoid(imu_header, imu_gyro, header_start, header_end, bias=0.0)
        corrected, corrected_gap, corrected_count = integrate_trapezoid(
            imu_header, imu_gyro, header_start, header_end, bias=gyro_bias
        )
        odom_twist_integral, odom_gap, odom_count = integrate_trapezoid(
            odom_header, odom_twist, header_start, header_end, bias=0.0
        )
        wheel_integral, wheel_gap, wheel_count = integrate_trapezoid(
            robotvel_bag, robotvel_z, bag_start, bag_end, bias=0.0
        )
        for source, value, gap, count, basis in [
            ("imu_gyro_integral_raw", raw, raw_gap, raw_count, "header_stamp"),
            ("imu_gyro_integral_bias_corrected", corrected, corrected_gap, corrected_count, "header_stamp"),
            ("odom_twist_integral", odom_twist_integral, odom_gap, odom_count, "header_stamp"),
            ("robotvel_z_integral", wheel_integral, wheel_gap, wheel_count, "bag_timestamp; no_header"),
        ]:
            metrics.append({
                "stage": stage,
                "metric_scope": "effective_rotation",
                "source": source,
                "delta_yaw_deg": math.degrees(value) if math.isfinite(value) else "",
                "integration_max_gap_s": gap,
                "sample_count": count,
                "time_basis": basis,
            })

        odom_points = [
            (float(item.msg.pose.pose.position.x), float(item.msg.pose.pose.position.y))
            for item in bag.odom
            if header_start <= item.header_ns <= header_end
        ]
        if len(odom_points) >= 2:
            dx = odom_points[-1][0] - odom_points[0][0]
            dy = odom_points[-1][1] - odom_points[0][1]
            metrics.append({
                "stage": stage,
                "metric_scope": "effective_rotation",
                "source": "odom_translation",
                "delta_x_m": dx,
                "delta_y_m": dy,
                "planar_displacement_m": math.hypot(dx, dy),
                "direction_deg_in_odom": math.degrees(math.atan2(dy, dx)),
                "frame": "odom_combined",
                "reference": "stage_start",
            })

    # Static platform medians and static-to-static deltas.
    static_medians: Dict[str, Dict[str, float]] = defaultdict(dict)
    for stage in ["S0", "S1", "S2", "S3", "S4"]:
        row = confirmed[stage]
        header_start = int_field(row, "representative_header_start")
        header_end = int_field(row, "representative_header_end")
        for source_name, (stamps, angles) in yaw_sources.items():
            med, mad = median_angle_window(stamps, angles, header_start, header_end)
            static_medians[stage][source_name] = med
            metrics.append({
                "stage": stage,
                "metric_scope": "static_platform",
                "source": source_name,
                "median_yaw_deg": math.degrees(med) if math.isfinite(med) else "",
                "mad_yaw_deg": math.degrees(mad) if math.isfinite(mad) else "",
                "time_basis": "header_stamp",
            })

    comparisons = [
        ("S0", "S1", "+90 nominal"),
        ("S1", "S2", "-90 nominal"),
        ("S2", "S3", "-90 nominal"),
        ("S3", "S4", "+90 nominal"),
        ("S0", "S2", "first return residual"),
        ("S0", "S4", "final residual"),
    ]
    for start_stage, end_stage, label in comparisons:
        for source_name in yaw_sources:
            start_value = static_medians[start_stage][source_name]
            end_value = static_medians[end_stage][source_name]
            metrics.append({
                "stage": f"{start_stage}->{end_stage}",
                "metric_scope": "static_to_static",
                "source": source_name,
                "start_yaw_deg": math.degrees(start_value) if math.isfinite(start_value) else "",
                "end_yaw_deg": math.degrees(end_value) if math.isfinite(end_value) else "",
                "delta_yaw_deg": math.degrees(end_value - start_value) if math.isfinite(start_value) and math.isfinite(end_value) else "",
                "nominal_or_role": label,
            })

    s0_bag_start = int_field(confirmed["S0"], "start_bag_time")
    s4_bag_end = int_field(confirmed["S4"], "end_bag_time")
    s4_header_end = int_field(confirmed["S4"], "representative_header_end")
    gyro_raw_total, gyro_raw_gap, _ = integrate_trapezoid(
        imu_header, imu_gyro, s0_header_start, s4_header_end, bias=0.0
    )
    gyro_corrected_total, gyro_corrected_gap, _ = integrate_trapezoid(
        imu_header, imu_gyro, s0_header_start, s4_header_end, bias=gyro_bias
    )

    s0_x, s0_y = median_pose_window(bag.odom, s0_header_start, s0_header_end)
    s4_x, s4_y = median_pose_window(
        bag.odom,
        int_field(confirmed["S4"], "representative_header_start"),
        s4_header_end,
    )
    odom_dx = s4_x - s0_x
    odom_dy = s4_y - s0_y
    odom_disp = math.hypot(odom_dx, odom_dy)
    physical_disp = math.hypot(PHYSICAL_FINAL_X_M, PHYSICAL_FINAL_Y_M)

    s0_imu_yaw = static_medians["S0"]["imu_orientation"]
    wheel_dx, wheel_dy, wheel_alignment = integrate_robotvel_reference(
        bag, s0_bag_start, s4_bag_end, s0_imu_yaw
    )

    metrics.extend([
        {
            "stage": "S0->S4",
            "metric_scope": "total",
            "source": "imu_gyro_integral_raw_total",
            "delta_yaw_deg": math.degrees(gyro_raw_total),
            "integration_max_gap_s": gyro_raw_gap,
            "gyro_bias_rad_s": gyro_bias,
            "gyro_bias_mad_rad_s": gyro_bias_mad,
        },
        {
            "stage": "S0->S4",
            "metric_scope": "total",
            "source": "imu_gyro_integral_bias_corrected_total",
            "delta_yaw_deg": math.degrees(gyro_corrected_total),
            "integration_max_gap_s": gyro_corrected_gap,
            "gyro_bias_rad_s": gyro_bias,
            "gyro_bias_mad_rad_s": gyro_bias_mad,
        },
        {
            "stage": "S0->S4",
            "metric_scope": "final_translation",
            "source": "odom_translation",
            "delta_x_m": odom_dx,
            "delta_y_m": odom_dy,
            "planar_displacement_m": odom_disp,
            "direction_deg_in_odom": math.degrees(math.atan2(odom_dy, odom_dx)),
            "frame": "odom_combined",
            "reference": "S0 median pose",
            "physical_displacement_m": physical_disp,
            "direction_comparison": "conditional on unverified user-vs-odom axis convention",
        },
        {
            "stage": "S0->S4",
            "metric_scope": "final_translation",
            "source": "robotvel_reference_trajectory",
            "delta_x_m": wheel_dx,
            "delta_y_m": wheel_dy,
            "planar_displacement_m": math.hypot(wheel_dx, wheel_dy),
            "frame": "S0-relative yaw frame",
            "reference": "robotvel x/y integrated with nearest IMU yaw by bag timestamp",
            "max_imu_alignment_error_s": wheel_alignment,
            "alignment_limitation": "robotvel has no header",
        },
    ])

    write_csv(out_dir / "e01_segment_metrics.csv", metrics)

    final_yaw = {
        source: math.degrees(static_medians["S4"][source] - static_medians["S0"][source])
        for source in yaw_sources
    }
    facts = [
        f"Gyro S0 bias median={gyro_bias:.6f} rad/s, MAD={gyro_bias_mad:.6f} rad/s.",
        f"Odom final translation relative to S0: dx={odom_dx:.4f} m, dy={odom_dy:.4f} m, magnitude={odom_disp:.4f} m.",
        f"Physical final translation magnitude={physical_disp:.4f} m; x/y direction comparison remains conditional on axis alignment.",
        f"Final yaw residuals: {', '.join(f'{name}={value:.3f}°' for name, value in final_yaw.items())}.",
        f"Gyro total raw={math.degrees(gyro_raw_total):.3f}°, bias-corrected={math.degrees(gyro_corrected_total):.3f}°.",
    ]

    inferences: List[str] = []
    if physical_disp > 0:
        ratio = odom_disp / physical_disp
        if 0.7 <= ratio <= 1.3:
            inferences.append(
                f"Inference: Odom displacement magnitude is broadly consistent with the physical magnitude (ratio={ratio:.2f}); direction still requires axis alignment."
            )
        elif ratio < 0.3:
            inferences.append(
                f"Inference: Odom records much less translation than the measured physical displacement (ratio={ratio:.2f}); unobserved translation is a high-priority candidate."
            )
        else:
            inferences.append(
                f"Inference: Odom and physical displacement magnitudes differ materially (ratio={ratio:.2f}); scan geometry is needed before attributing the discrepancy."
            )
    imu_final = final_yaw.get("imu_orientation", float("nan"))
    if PHYSICAL_FINAL_YAW_MIN_DEG <= imu_final <= PHYSICAL_FINAL_YAW_MAX_DEG:
        inferences.append("Inference: IMU-orientation final yaw residual falls inside the user-measured -2° to -1° interval.")
    else:
        inferences.append("Inference: IMU-orientation final yaw residual does not fall inside the user-measured -2° to -1° interval.")

    review = [
        "# E01 Phase-1 Review",
        "",
        "## A. Confirmed facts",
        *[f"- {item}" for item in facts],
        "",
        "## B. Physical ground truth",
        f"- final x={PHYSICAL_FINAL_X_M:+.3f} m, y={PHYSICAL_FINAL_Y_M:+.3f} m",
        f"- final yaw residual approximately {PHYSICAL_FINAL_YAW_MIN_DEG:.1f}° to {PHYSICAL_FINAL_YAW_MAX_DEG:.1f}° in ROS sign convention",
        "- x/y axis convention is user-reported and unverified; only S0->S4 is measured.",
        "",
        "## C. Yaw-chain results",
        "- See e01_segment_metrics.csv for per-stage IMU orientation, Odom pose and TF yaw.",
        "- Lineage: IMU gyro_z -> Mahony orientation -> Odom pose yaw -> target TF yaw.",
        "- Odom/TF agreement proves copying/publication consistency, not independent physical correctness.",
        "",
        "## D. Translation results",
        f"- Odom S0->S4: dx={odom_dx:.4f} m, dy={odom_dy:.4f} m, magnitude={odom_disp:.4f} m.",
        f"- Robotvel reference: dx={wheel_dx:.4f} m, dy={wheel_dy:.4f} m, magnitude={math.hypot(wheel_dx, wheel_dy):.4f} m.",
        "",
        "## E. Inferences",
        *[f"- {item}" for item in inferences],
        "",
        "## F. Unknowns",
        "- User x/y axes have not been aligned with odom_combined.",
        "- LaserScan SE(2) registration has not yet independently validated the final translation or yaw.",
        "- This Phase-1 analysis does not establish the cause of the historical ~90° map/scan rotation.",
        "",
        "## G. Next experiment",
        "- Phase 2: multi-start coarse SE(2) scan matching followed by ICP refinement for S0/S1/S2/S3/S4 representative scans.",
        "- Report best and second-best coarse scores, ambiguity, overlap/inliers, refined fitness and transform convention.",
        "",
        "No robot code, TF parameters, bag contents or SLAM configuration were modified.",
    ]
    (out_dir / "E01_PHASE1_REVIEW.md").write_text("\n".join(review) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Preview output
# ---------------------------------------------------------------------------
def write_preview(
    path: Path,
    bag: LoadedBag,
    audit_rows: Sequence[Dict[str, Any]],
    intervals: Sequence[CommandInterval],
    rotations: Sequence[RotationCandidate],
    standalone: Sequence[CommandInterval],
    windows: Sequence[Dict[str, Any]],
    valid: bool,
) -> None:
    lines = [
        "# E01 Phase-1 Preview",
        "",
        f"- bag: `{bag.uri}`",
        f"- duration: {bag.duration_s:.3f} s",
        f"- event_window_valid: **{valid}**",
        "",
        "## Raw-data audit",
        "",
        "| source | count | basis | median interval (s) | max gap (s) |",
        "|---|---:|---|---:|---:|",
    ]
    for row in audit_rows:
        lines.append(
            f"| {row.get('source','')} | {row.get('count','')} | {row.get('time_basis','')} | "
            f"{finite_or_blank(row.get('median_interval_s',''))} | {finite_or_blank(row.get('max_gap_s',''))} |"
        )

    lines.extend([
        "",
        "## Raw non-zero cmd_vel intervals",
        "",
        "| idx | start rel (s) | end rel (s) | duration (s) | sign | peak | nudge | attachment |",
        "|---:|---:|---:|---:|---|---:|---|---|",
    ])
    for index, item in enumerate(intervals, start=1):
        lines.append(
            f"| {index} | {ns_to_s(item.start_bag_ns-bag.bag_start_ns):.3f} | "
            f"{ns_to_s(item.end_bag_ns-bag.bag_start_ns):.3f} | {item.duration_s:.3f} | "
            f"{'CCW' if item.sign > 0 else 'CW'} | {item.peak_abs:.3f} | {item.is_nudge} | "
            f"{item.attached_to or item.attachment_role or ''} |"
        )

    lines.extend([
        "",
        "## Candidate rotations",
        "",
        "| stage | effective start rel (s) | effective end rel (s) | gyro sign | odom sign | gyro motion | odom motion | confidence | corrections |",
        "|---|---:|---:|---:|---:|---|---|---|---:|",
    ])
    window_by_stage = {row["stage"]: row for row in windows}
    for rotation in rotations:
        row = window_by_stage.get(rotation.stage, {})
        lines.append(
            f"| {rotation.stage} | {ns_to_s(rotation.effective_start_bag_ns-bag.bag_start_ns):.3f} | "
            f"{ns_to_s(rotation.effective_end_bag_ns-bag.bag_start_ns):.3f} | {row.get('gyro_sign','')} | "
            f"{row.get('odom_sign','')} | {row.get('gyro_motion_detected','')} | {row.get('odom_motion_detected','')} | "
            f"{row.get('confidence','')} | {len(rotation.corrections)} |"
        )

    lines.extend([
        "",
        "## Static windows",
        "",
        "| stage | start rel (s) | end rel (s) | duration (s) |",
        "|---|---:|---:|---:|",
    ])
    for row in windows:
        if row.get("kind") == "static":
            lines.append(
                f"| {row['stage']} | {ns_to_s(int(row['start_bag_time'])-bag.bag_start_ns):.3f} | "
                f"{ns_to_s(int(row['end_bag_time'])-bag.bag_start_ns):.3f} | {float(row['duration_s']):.3f} |"
            )

    lines.extend([
        "",
        "## Gate",
        "",
        f"- four main rotations: {len(rotations) == 4}",
        f"- standalone nudges: {len(standalone)}",
        f"- event_window_valid: {valid}",
        "",
        "Step A only. No yaw integration, translation conclusion, scan registration or root-cause declaration was performed.",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Top-level workflow
# ---------------------------------------------------------------------------
def run_audit(bag: LoadedBag, out_dir: Path, args: argparse.Namespace) -> bool:
    out_dir.mkdir(parents=True, exist_ok=True)
    ensure_physical_ground_truth(out_dir)

    expanded = expand_tf(bag.tf)
    audit_rows = [
        audit_stamped_topic("/odom_combined", bag.odom, "nav_msgs/msg/Odometry"),
        audit_stamped_topic("/mobile_base/sensors/imu_data", bag.imu, "sensor_msgs/msg/Imu"),
        audit_stamped_topic("/scan", bag.scan, "sensor_msgs/msg/LaserScan"),
        audit_no_header_topic("/robotvel", bag.robotvel, "dlrobot_robot_msg/msg/Data"),
        audit_no_header_topic("/cmd_vel", bag.cmd_vel, "geometry_msgs/msg/Twist"),
        audit_tf_stream(bag.tf, expanded),
        audit_target_tf(expanded, bag.odom),
        audit_static_tf(bag.tf_static),
    ]

    intervals, command_threshold = detect_cmd_intervals(
        bag.cmd_vel,
        max_sample_gap_s=args.cmd_sample_gap_s,
        main_duration_s=args.main_duration_s,
        main_peak=args.main_peak,
    )
    rotations, standalone, ambiguous = attach_nudges(intervals, attach_gap_s=args.nudge_attach_gap_s)
    windows, valid, _ = build_event_windows(
        bag,
        rotations,
        standalone,
        ambiguous,
        settle_s=args.settle_s,
        static_window_s=args.static_window_s,
    )

    for row in audit_rows:
        row["command_detection_threshold_rad_s"] = command_threshold
    write_csv(out_dir / "raw_data_audit.csv", audit_rows)
    write_csv(out_dir / "e01_cmd_intervals.csv", [
        {
            "index": index,
            "start_bag_time": item.start_bag_ns,
            "end_bag_time": item.end_bag_ns,
            "duration_s": item.duration_s,
            "sign": item.sign,
            "sample_count": item.sample_count,
            "peak_abs": item.peak_abs,
            "mean": item.mean,
            "is_nudge": item.is_nudge,
            "attached_to": item.attached_to or "",
            "attachment_role": item.attachment_role or "",
        }
        for index, item in enumerate(intervals, start=1)
    ])
    write_csv(out_dir / "e01_event_windows.csv", windows)
    write_preview(
        out_dir / "E01_PHASE1_PREVIEW.md",
        bag,
        audit_rows,
        intervals,
        rotations,
        standalone,
        windows,
        valid,
    )

    print(f"STEP_A_OK out_dir={out_dir}")
    print(
        f"event_window_valid={valid} rotations={len(rotations)} standalone_nudges={len(standalone)} "
        f"cmd_threshold={command_threshold:.6f}"
    )
    return valid


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="rosbag2 directory")
    parser.add_argument("--out-dir", default=None, help="analysis output directory")
    parser.add_argument("--step", choices=["audit", "metrics"], default="audit")
    parser.add_argument("--confirmed-windows", default=None, help="human-confirmed e01_event_windows.csv for metrics")
    parser.add_argument("--cmd-sample-gap-s", type=float, default=0.5)
    parser.add_argument("--main-duration-s", type=float, default=3.0)
    parser.add_argument("--main-peak", type=float, default=0.08)
    parser.add_argument("--nudge-attach-gap-s", type=float, default=5.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--static-window-s", type=float, default=10.0)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_dir():
        print(f"ERROR: bag directory not found: {bag_path}", file=sys.stderr)
        return 2

    out_dir = (
        Path(args.out_dir).expanduser().resolve()
        if args.out_dir
        else bag_path.parent / "analysis"
    )

    try:
        bag = load_bag(bag_path)
        if args.step == "audit":
            run_audit(bag, out_dir, args)
            return 0

        if not args.confirmed_windows:
            print("REFUSE: --step metrics requires --confirmed-windows", file=sys.stderr)
            return 3
        windows_path = Path(args.confirmed_windows).expanduser().resolve()
        if not windows_path.is_file():
            print(f"ERROR: confirmed windows CSV not found: {windows_path}", file=sys.stderr)
            return 4
        ensure_physical_ground_truth(out_dir)
        run_metrics(bag, windows_path, out_dir)
        print(f"STEP_B_OK out_dir={out_dir}")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 10


if __name__ == "__main__":
    raise SystemExit(main())
