#!/usr/bin/env python3
"""
ROS 2 Localization Bag Analyzer v6.0 Frame Isolation

Designed for ROS 2 Humble / rosbag2 sqlite3.

Purpose
-------
v5 estimated LaserScan wall orientation only in the MAP frame using
    map->odom + odom->base + base->laser
Any AMCL map->odom yaw correction directly moves that map-frame
orientation, so a "scan shift precedes map->odom" observation is circular
and cannot by itself implicate or clear AMCL.

v6 adds a second, independent orientation estimate in the ODOM frame using
only
    odom->base + base->laser
so the two can be compared with a controlled algorithm. If the odom-frame
wall orientation is stable while only the map-frame orientation shifts, the
shift is attributable to map->odom. If the odom-frame orientation shifts,
the scan moved relative to odom (upstream of AMCL).

Key capabilities
----------------
1.  Preserves v5's map-frame orientation calculation and adds an odom-frame
    calculation using the SAME wall-orientation algorithm.
2.  Retains /tf and /odom_combined odom->base sources SEPARATELY. v5
    discarded /odom_combined whenever any /tf odom->base existed; v6 keeps
    both and lets the user choose the analytical source (default
    /odom_combined, the dense ~20 Hz topic).
3.  Deduplicates samples by header stamp before interval statistics and
    interpolation, so duplicate stamps never distort coverage or TF queries.
4.  Separates TF interpolation from event detection. Per-scan map->odom
    change is reported as an interpolated value for plotting, but the first
    significant map->odom correction is detected from RAW, DEDUPLICATED
    map->odom TF samples at their original header stamps. Interpolation is
    never used to decide when the correction began (it can smear a later
    jump backward in time).
5.  Reports both first single-frame and first SUSTAINED (>=2 consecutive
    usable samples) 3-degree crossings for odom- and map-frame shifts.
    Causal ordering uses the sustained crossings.
6.  Characterizes /tf odom->base, /odom_combined, and /tf map->odom coverage
    independently (raw count, unique/duplicate header-stamp count, median
    and max header-stamp and bag-record-time intervals, scan bracket
    coverage %).

Important analytical distinctions
---------------------------------
- Using /odom_combined as the analytical odom->base source improves ANALYSIS
  coverage. It does NOT prove the runtime /tf chain had adequate publication
  frequency; runtime TF consumers use /tf.
- Publisher multiplicity cannot be determined from TF pair counts alone.
- AMCL is implicated only if a RAW abnormal map->odom correction clearly
  PRECEDES the first sustained odom-frame scan deviation.

Typical usage
-------------
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 analyze_localization_bag_v6_frame_isolation.py /path/to/bag
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit(
        "matplotlib is missing. Install it with:\n"
        "  sudo apt install python3-matplotlib"
    ) from exc

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    raise SystemExit(
        "ROS 2 Python packages are unavailable in this shell.\n"
        "Run:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source ~/ros2_ws/install/setup.bash"
    ) from exc

VERSION = "6.0-frame-isolation"

ODOM_TOPIC = "/odom_combined"
AMCL_TOPIC = "/amcl_pose"
IMU_TOPIC = "/mobile_base/sensors/imu_data"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
SCAN_TOPIC = "/scan"
CMD_TOPICS = ("/cmd_vel_nav", "/cmd_vel")

DEFAULT_MAP_FRAME = "map"
DEFAULT_ODOM_FRAME = "odom_combined"
DEFAULT_BASE_FRAME = "base_footprint"
DEFAULT_LASER_FRAME = "laser"

NAN = float("nan")


def normalize_frame(frame: str) -> str:
    return str(frame).lstrip("/")


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def wrap_period_deg(value: float, period: float) -> float:
    return (float(value) + period / 2.0) % period - period / 2.0


def unwrap_degrees(values: Sequence[float]) -> List[float]:
    if not values:
        return []
    out = [float(values[0])]
    offset = 0.0
    previous = float(values[0])
    for raw in values[1:]:
        current = float(raw)
        delta = current - previous
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        out.append(current + offset)
        previous = current
    return out


def stamp_to_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transport_delay_ms(bag_timestamp_ns: int, header_timestamp_ns: int) -> float:
    if header_timestamp_ns <= 0:
        return NAN
    return (int(bag_timestamp_ns) - int(header_timestamp_ns)) / 1e6


def finite_number(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def bracket_rows_by_header(
    rows: Sequence[dict],
    header_times_ns: Sequence[int],
    target_ns: int,
) -> Tuple[Optional[dict], Optional[dict]]:
    if not rows or target_ns <= 0:
        return None, None
    left = bisect.bisect_left(header_times_ns, target_ns)
    if left < len(rows) and header_times_ns[left] == target_ns:
        return rows[left], rows[left]
    previous = rows[left - 1] if left > 0 else None
    next_row = rows[left] if left < len(rows) else None
    return previous, next_row


def bracket_metrics(
    previous: Optional[dict],
    next_row: Optional[dict],
    target_ns: int,
) -> dict:
    prev_ns = int(previous["header_timestamp_ns"]) if previous else 0
    next_ns = int(next_row["header_timestamp_ns"]) if next_row else 0
    return {
        "previous_header_timestamp_ns": prev_ns if previous else "",
        "next_header_timestamp_ns": next_ns if next_row else "",
        "target_to_previous_ms": (target_ns - prev_ns) / 1e6 if previous else NAN,
        "next_to_target_ms": (next_ns - target_ns) / 1e6 if next_row else NAN,
        "bracket_width_ms": (next_ns - prev_ns) / 1e6 if previous and next_row else NAN,
    }


def interpolate_se2_at_header(
    rows: Sequence[dict],
    header_times_ns: Sequence[int],
    target_ns: int,
    max_bracket_ns: Optional[int],
) -> dict:
    previous, next_row = bracket_rows_by_header(rows, header_times_ns, target_ns)
    metrics = bracket_metrics(previous, next_row, target_ns)
    result = {
        **metrics,
        "valid": False,
        "x": NAN,
        "y": NAN,
        "yaw_deg": NAN,
    }
    if previous is None or next_row is None:
        return result
    prev_ns = int(previous["header_timestamp_ns"])
    next_ns = int(next_row["header_timestamp_ns"])
    width_ns = next_ns - prev_ns
    if max_bracket_ns is not None and width_ns > max_bracket_ns:
        return result
    if width_ns <= 0:
        result.update({
            "valid": True,
            "x": float(previous["x"]),
            "y": float(previous["y"]),
            "yaw_deg": float(previous["yaw_deg"]),
        })
        return result
    alpha = (target_ns - prev_ns) / width_ns
    alpha = max(0.0, min(1.0, alpha))
    yaw_delta = wrap_deg(float(next_row["yaw_deg"]) - float(previous["yaw_deg"]))
    result.update({
        "valid": True,
        "x": float(previous["x"]) + alpha * (float(next_row["x"]) - float(previous["x"])),
        "y": float(previous["y"]) + alpha * (float(next_row["y"]) - float(previous["y"])),
        "yaw_deg": wrap_deg(float(previous["yaw_deg"]) + alpha * yaw_delta),
    })
    return result


def nearest_row_by_header(
    rows: Sequence[dict],
    header_times_ns: Sequence[int],
    target_ns: int,
    max_gap_ns: Optional[int],
) -> Tuple[Optional[dict], float]:
    if not rows or target_ns <= 0:
        return None, NAN
    index = bisect.bisect_left(header_times_ns, target_ns)
    candidates: List[int] = []
    if index < len(rows):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None, NAN
    best = min(candidates, key=lambda i: abs(header_times_ns[i] - target_ns))
    gap_ns = header_times_ns[best] - target_ns
    if max_gap_ns is not None and abs(gap_ns) > max_gap_ns:
        return None, gap_ns / 1e6
    return rows[best], gap_ns / 1e6


def nearest_tf_signed_offset_ms(
    previous: Optional[dict],
    next_row: Optional[dict],
    target_ns: int,
) -> float:
    candidates: List[int] = []
    if previous:
        candidates.append(int(previous["header_timestamp_ns"]) - target_ns)
    if next_row:
        candidates.append(int(next_row["header_timestamp_ns"]) - target_ns)
    if not candidates:
        return NAN
    return min(candidates, key=abs) / 1e6


def nearest_row(
    rows: Sequence[dict], times: Sequence[float], target: float, max_gap: float
) -> Optional[dict]:
    if not rows:
        return None
    index = bisect.bisect_left(times, target)
    candidates: List[int] = []
    if index < len(rows):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(times[i] - target))
    if abs(times[best] - target) > max_gap:
        return None
    return rows[best]


def numeric_summary(rows: Sequence[dict], key: str) -> Optional[Tuple[float, float, float]]:
    values = [float(row[key]) for row in rows if finite_number(row.get(key))]
    if not values:
        return None
    return min(values), statistics.median(values), max(values)


def format_summary(rows: Sequence[dict], key: str, unit: str) -> str:
    summary = numeric_summary(rows, key)
    if summary is None:
        return "no data"
    minimum, median, maximum = summary
    return f"min={minimum:.3f} {unit}, median={median:.3f} {unit}, max={maximum:.3f} {unit}"


def resolve_bag_uri(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_dir():
        return path
    if path.is_file() and path.suffix == ".db3":
        return path.parent
    raise FileNotFoundError(f"Unsupported or missing bag path: {path}")


def open_reader(bag_uri: Path) -> Tuple[object, Dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    return reader, types


def compose_se2(a: dict, b: dict) -> dict:
    angle = math.radians(a["yaw_deg"])
    c, s = math.cos(angle), math.sin(angle)
    return {
        "x": a["x"] + c * b["x"] - s * b["y"],
        "y": a["y"] + s * b["x"] + c * b["y"],
        "yaw_deg": wrap_deg(a["yaw_deg"] + b["yaw_deg"]),
    }


def invert_se2(a: dict) -> dict:
    angle = math.radians(a["yaw_deg"])
    c, s = math.cos(angle), math.sin(angle)
    return {
        "x": -c * a["x"] - s * a["y"],
        "y": s * a["x"] - c * a["y"],
        "yaw_deg": wrap_deg(-a["yaw_deg"]),
    }


def write_csv(path: Path, fields: Sequence[str], rows: Iterable[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def command_values(msg: object) -> Tuple[float, float, float]:
    twist = msg.twist if hasattr(msg, "twist") else msg
    return float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)


def transform_row(
    transform: object,
    t: float,
    bag_timestamp_ns: int,
    source_topic: str,
) -> dict:
    tr = transform.transform.translation
    q = transform.transform.rotation
    header_timestamp_ns = stamp_to_ns(transform.header.stamp)
    return {
        "t": t,
        "bag_timestamp_ns": int(bag_timestamp_ns),
        "header_timestamp_ns": header_timestamp_ns,
        "transport_delay_ms": transport_delay_ms(bag_timestamp_ns, header_timestamp_ns),
        "stamp_sec": int(transform.header.stamp.sec),
        "stamp_nanosec": int(transform.header.stamp.nanosec),
        "parent": normalize_frame(transform.header.frame_id),
        "child": normalize_frame(transform.child_frame_id),
        "x": float(tr.x),
        "y": float(tr.y),
        "yaw_deg": math.degrees(yaw_from_quaternion(q.x, q.y, q.z, q.w)),
        "source_topic": source_topic,
    }


def extract_data(
    bag_uri: Path,
    map_frame: str,
    odom_frame: str,
    base_frame: str,
    laser_frame_override: Optional[str],
    scan_stride: int,
) -> dict:
    reader, topic_types = open_reader(bag_uri)

    selected = {
        ODOM_TOPIC,
        AMCL_TOPIC,
        IMU_TOPIC,
        TF_TOPIC,
        TF_STATIC_TOPIC,
        SCAN_TOPIC,
        *CMD_TOPICS,
    }
    classes = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in selected
    }

    data = {
        "topic_types": topic_types,
        "topic_counts": {topic: 0 for topic in topic_types},
        "bag_start_ns": None,
        "odom": [],
        "amcl": [],
        "imu": [],
        "tf_dynamic": [],
        "tf_static": [],
        "tf_map_odom": [],
        "tf_odom_base": [],
        "tf_odom_base_tf": [],
        "tf_odom_base_odom": [],
        "cmd_vel": [],
        "cmd_vel_nav": [],
        "scan": [],
        "tf_pair_counts": {},
        "laser_frame": normalize_frame(laser_frame_override) if laser_frame_override else None,
    }

    bag_start_ns: Optional[int] = None
    scan_index = 0

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        timestamp_ns = int(timestamp_ns)
        if bag_start_ns is None:
            bag_start_ns = timestamp_ns
            data["bag_start_ns"] = timestamp_ns
        data["topic_counts"][topic] = data["topic_counts"].get(topic, 0) + 1
        if topic not in classes:
            continue

        t = (timestamp_ns - bag_start_ns) / 1e9
        msg = deserialize_message(raw, classes[topic])

        if topic == ODOM_TOPIC:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            header_timestamp_ns = stamp_to_ns(msg.header.stamp)
            row = {
                "t": t,
                "bag_timestamp_ns": timestamp_ns,
                "header_timestamp_ns": header_timestamp_ns,
                "transport_delay_ms": transport_delay_ms(timestamp_ns, header_timestamp_ns),
                "stamp_sec": int(msg.header.stamp.sec),
                "stamp_nanosec": int(msg.header.stamp.nanosec),
                "frame_id": normalize_frame(msg.header.frame_id),
                "child_frame_id": normalize_frame(msg.child_frame_id),
                "x": float(p.x),
                "y": float(p.y),
                "yaw_deg": math.degrees(yaw_from_quaternion(q.x, q.y, q.z, q.w)),
                "linear_x": float(msg.twist.twist.linear.x),
                "linear_y": float(msg.twist.twist.linear.y),
                "angular_z": float(msg.twist.twist.angular.z),
            }
            data["odom"].append(row)
            # /odom_combined is a valid odom->base transform fallback (dense).
            if row["frame_id"] == odom_frame and row["child_frame_id"] == base_frame:
                data["tf_odom_base"].append({
                    "t": t,
                    "bag_timestamp_ns": timestamp_ns,
                    "header_timestamp_ns": header_timestamp_ns,
                    "transport_delay_ms": row["transport_delay_ms"],
                    "stamp_sec": row["stamp_sec"],
                    "stamp_nanosec": row["stamp_nanosec"],
                    "parent": odom_frame,
                    "child": base_frame,
                    "x": row["x"],
                    "y": row["y"],
                    "yaw_deg": row["yaw_deg"],
                    "source_topic": ODOM_TOPIC,
                })

        elif topic == AMCL_TOPIC:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            header_timestamp_ns = stamp_to_ns(msg.header.stamp)
            data["amcl"].append({
                "t": t,
                "bag_timestamp_ns": timestamp_ns,
                "header_timestamp_ns": header_timestamp_ns,
                "transport_delay_ms": transport_delay_ms(timestamp_ns, header_timestamp_ns),
                "stamp_sec": int(msg.header.stamp.sec),
                "stamp_nanosec": int(msg.header.stamp.nanosec),
                "frame_id": normalize_frame(msg.header.frame_id),
                "x": float(p.x),
                "y": float(p.y),
                "yaw_deg": math.degrees(yaw_from_quaternion(q.x, q.y, q.z, q.w)),
                "cov_x": float(msg.pose.covariance[0]),
                "cov_y": float(msg.pose.covariance[7]),
                "cov_yaw": float(msg.pose.covariance[35]),
            })

        elif topic == IMU_TOPIC:
            q = msg.orientation
            header_timestamp_ns = stamp_to_ns(msg.header.stamp)
            data["imu"].append({
                "t": t,
                "bag_timestamp_ns": timestamp_ns,
                "header_timestamp_ns": header_timestamp_ns,
                "transport_delay_ms": transport_delay_ms(timestamp_ns, header_timestamp_ns),
                "stamp_sec": int(msg.header.stamp.sec),
                "stamp_nanosec": int(msg.header.stamp.nanosec),
                "frame_id": normalize_frame(msg.header.frame_id),
                "yaw_deg": math.degrees(yaw_from_quaternion(q.x, q.y, q.z, q.w)),
                "gyro_z": float(msg.angular_velocity.z),
            })

        elif topic in CMD_TOPICS:
            lx, ly, az = command_values(msg)
            key = "cmd_vel_nav" if topic == "/cmd_vel_nav" else "cmd_vel"
            data[key].append({
                "t": t,
                "bag_timestamp_ns": timestamp_ns,
                "topic": topic,
                "linear_x": lx,
                "linear_y": ly,
                "linear_speed": math.hypot(lx, ly),
                "angular_z": az,
            })

        elif topic in (TF_TOPIC, TF_STATIC_TOPIC):
            destination = "tf_static" if topic == TF_STATIC_TOPIC else "tf_dynamic"
            for transform in msg.transforms:
                row = transform_row(transform, t, timestamp_ns, topic)
                data[destination].append(row)
                pair = f"{row['parent']}->{row['child']}"
                data["tf_pair_counts"][pair] = data["tf_pair_counts"].get(pair, 0) + 1
                if row["parent"] == map_frame and row["child"] == odom_frame:
                    data["tf_map_odom"].append(row)
                elif row["parent"] == odom_frame and row["child"] == base_frame:
                    data["tf_odom_base"].append(row)

        elif topic == SCAN_TOPIC:
            if scan_index % max(1, scan_stride) == 0:
                frame = normalize_frame(msg.header.frame_id)
                if data["laser_frame"] is None:
                    data["laser_frame"] = frame

                header_timestamp_ns = stamp_to_ns(msg.header.stamp)
                beam_count = len(msg.ranges)
                scan_time = max(0.0, float(msg.scan_time))
                time_increment = max(0.0, float(msg.time_increment))
                if beam_count > 1 and time_increment > 0.0:
                    effective_scan_duration = (beam_count - 1) * time_increment
                else:
                    effective_scan_duration = scan_time
                duration_ns = int(round(effective_scan_duration * 1e9))
                scan_start_timestamp_ns = header_timestamp_ns
                scan_mid_timestamp_ns = (
                    header_timestamp_ns + duration_ns // 2
                    if header_timestamp_ns > 0 else 0
                )
                scan_end_timestamp_ns = (
                    header_timestamp_ns + duration_ns
                    if header_timestamp_ns > 0 else 0
                )

                data["scan"].append({
                    "scan_index": scan_index,
                    "t": t,
                    "bag_timestamp_ns": timestamp_ns,
                    "header_timestamp_ns": header_timestamp_ns,
                    "transport_delay_ms": transport_delay_ms(timestamp_ns, header_timestamp_ns),
                    "stamp_sec": int(msg.header.stamp.sec),
                    "stamp_nanosec": int(msg.header.stamp.nanosec),
                    "frame_id": frame,
                    "angle_min": float(msg.angle_min),
                    "angle_max": float(msg.angle_max),
                    "angle_increment": float(msg.angle_increment),
                    "scan_time": scan_time,
                    "time_increment": time_increment,
                    "beam_count": beam_count,
                    "effective_scan_duration": effective_scan_duration,
                    "scan_start_timestamp_ns": scan_start_timestamp_ns,
                    "scan_mid_timestamp_ns": scan_mid_timestamp_ns,
                    "scan_end_timestamp_ns": scan_end_timestamp_ns,
                    "range_min": float(msg.range_min),
                    "range_max": float(msg.range_max),
                    "ranges": list(msg.ranges),
                })
            scan_index += 1

    for key in (
        "odom", "amcl", "imu", "tf_dynamic", "tf_static",
        "tf_map_odom", "tf_odom_base", "cmd_vel", "cmd_vel_nav", "scan",
    ):
        data[key].sort(key=lambda row: row["t"])

    # v6 change vs v5: do NOT discard /odom_combined when /tf odom->base exists.
    # Keep both sources separately for independent coverage characterization.
    data["tf_odom_base_tf"] = [r for r in data["tf_odom_base"] if r["source_topic"] == TF_TOPIC]
    data["tf_odom_base_odom"] = [r for r in data["tf_odom_base"] if r["source_topic"] == ODOM_TOPIC]
    # data["tf_odom_base"] remains the merged list (sorted by t).
    return data


def find_static_transform(data: dict, parent: str, child: str) -> Optional[dict]:
    direct = [r for r in data["tf_static"] if r["parent"] == parent and r["child"] == child]
    if direct:
        return direct[-1]
    inverse = [r for r in data["tf_static"] if r["parent"] == child and r["child"] == parent]
    if inverse:
        result = invert_se2(inverse[-1])
        result.update({"parent": parent, "child": child, "source_topic": TF_STATIC_TOPIC})
        return result
    all_rows = data["tf_static"]
    for first in all_rows:
        if first["parent"] != parent:
            continue
        middle = first["child"]
        for second in all_rows:
            if second["parent"] == middle and second["child"] == child:
                result = compose_se2(first, second)
                result.update({"parent": parent, "child": child, "source_topic": TF_STATIC_TOPIC})
                return result
    return None


def dedup_by_header_stamp(rows: Sequence[dict]) -> List[dict]:
    """
    Return one row per unique positive header stamp, keeping the earliest by
    (header_stamp, bag_time). Duplicate header stamps never enter interval
    statistics or interpolation.
    """
    seen: set = set()
    out: List[dict] = []
    for r in sorted(
        rows,
        key=lambda r: (int(r.get("header_timestamp_ns", 0)), float(r.get("t", 0.0))),
    ):
        ns = int(r.get("header_timestamp_ns", 0))
        if ns <= 0:
            continue
        if ns in seen:
            continue
        seen.add(ns)
        out.append(r)
    return out


def dominant_segment_orientation(
    scan: dict,
    frame_laser: dict,
    min_segment_length: float,
    max_segment_length: float,
) -> Optional[dict]:
    """
    Estimate dominant straight-wall orientation in the given frame using a
    4-theta circular mean (invariant to line direction and 90-degree symmetry).
    The SAME algorithm is used for map and odom frames so the comparison is
    controlled.
    """
    points: List[Optional[Tuple[float, float]]] = []
    a = scan["angle_min"]
    for r in scan["ranges"]:
        valid = math.isfinite(r) and scan["range_min"] <= r <= scan["range_max"]
        if valid:
            points.append((r * math.cos(a), r * math.sin(a)))
        else:
            points.append(None)
        a += scan["angle_increment"]

    yaw = math.radians(frame_laser["yaw_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    sum_c = 0.0
    sum_s = 0.0
    count = 0

    previous: Optional[Tuple[float, float]] = None
    for point in points:
        if point is None:
            previous = None
            continue
        x = frame_laser["x"] + c * point[0] - s * point[1]
        y = frame_laser["y"] + s * point[0] + c * point[1]
        current = (x, y)
        if previous is not None:
            dx, dy = current[0] - previous[0], current[1] - previous[1]
            length = math.hypot(dx, dy)
            if min_segment_length <= length <= max_segment_length:
                theta = math.atan2(dy, dx)
                weight = length
                sum_c += weight * math.cos(4.0 * theta)
                sum_s += weight * math.sin(4.0 * theta)
                count += 1
        previous = current

    if count < 10:
        return None
    resultant = math.hypot(sum_c, sum_s)
    if resultant <= 1e-12:
        return None
    angle_deg = math.degrees(math.atan2(sum_s, sum_c) / 4.0)
    confidence = resultant / max(1e-12, count * max_segment_length)
    return {
        "wall_orientation_deg": wrap_period_deg(angle_deg, 90.0),
        "segment_count": count,
        "confidence": confidence,
    }


def build_frame_isolation_rows(
    data: dict,
    base_laser: dict,
    timing_max_bracket_ms: float,
    sensor_match_max_ms: float,
    min_segment_length: float,
    max_segment_length: float,
    odom_base_source: str,
) -> List[dict]:
    """Build one row per sampled scan with map- and odom-frame orientation."""
    max_bracket_ns = int(max(0.0, timing_max_bracket_ms) * 1e6)
    sensor_match_max_ns = int(max(0.0, sensor_match_max_ms) * 1e6)

    if odom_base_source == "tf":
        ob_src = data["tf_odom_base_tf"]
    elif odom_base_source == "merged":
        ob_src = data["tf_odom_base"]
    else:
        ob_src = data["tf_odom_base_odom"]

    # Dedup by header stamp before interpolation so duplicate stamps cannot
    # distort the bracket/interpolation.
    ob_rows = dedup_by_header_stamp(ob_src)
    ob_rows.sort(key=lambda r: int(r["header_timestamp_ns"]))
    ob_times = [int(r["header_timestamp_ns"]) for r in ob_rows]

    mo_rows = dedup_by_header_stamp(data["tf_map_odom"])
    mo_rows.sort(key=lambda r: int(r["header_timestamp_ns"]))
    mo_times = [int(r["header_timestamp_ns"]) for r in mo_rows]

    imu_rows = dedup_by_header_stamp(data["imu"])
    imu_rows.sort(key=lambda r: int(r["header_timestamp_ns"]))
    imu_times = [int(r["header_timestamp_ns"]) for r in imu_rows]

    odom_rows = dedup_by_header_stamp(data["odom"])
    odom_rows.sort(key=lambda r: int(r["header_timestamp_ns"]))
    odom_times = [int(r["header_timestamp_ns"]) for r in odom_rows]

    cmd_times = [float(r["t"]) for r in data["cmd_vel"]]
    cmd_nav_times = [float(r["t"]) for r in data["cmd_vel_nav"]]

    out: List[dict] = []
    for scan in data["scan"]:
        start_ns = int(scan["scan_start_timestamp_ns"])
        mid_ns = int(scan["scan_mid_timestamp_ns"]) or int(scan["header_timestamp_ns"])
        end_ns = int(scan["scan_end_timestamp_ns"])
        duration_s = float(scan["effective_scan_duration"])

        ob_start = interpolate_se2_at_header(ob_rows, ob_times, start_ns, max_bracket_ns)
        ob_mid = interpolate_se2_at_header(ob_rows, ob_times, mid_ns, max_bracket_ns)
        ob_end = interpolate_se2_at_header(ob_rows, ob_times, end_ns, max_bracket_ns)
        ob_prev, ob_next = bracket_rows_by_header(ob_rows, ob_times, start_ns)
        ob_metrics = bracket_metrics(ob_prev, ob_next, start_ns)

        mo_mid = interpolate_se2_at_header(mo_rows, mo_times, mid_ns, None)
        mo_prev, mo_next = bracket_rows_by_header(mo_rows, mo_times, mid_ns)
        mo_metrics = bracket_metrics(mo_prev, mo_next, mid_ns)

        imu, imu_gap_ms = nearest_row_by_header(imu_rows, imu_times, mid_ns, sensor_match_max_ns)
        odom, odom_gap_ms = nearest_row_by_header(odom_rows, odom_times, mid_ns, sensor_match_max_ns)
        cmd = nearest_row(data["cmd_vel"], cmd_times, scan["t"], 0.5)
        cmd_nav = nearest_row(data["cmd_vel_nav"], cmd_nav_times, scan["t"], 0.5)

        gyro_z = float(imu["gyro_z"]) if imu else NAN
        imu_yaw = float(imu["yaw_deg"]) if imu else NAN
        odom_az = float(odom["angular_z"]) if odom else NAN
        predicted_imu_deg = (
            math.degrees(gyro_z * duration_s) if finite_number(gyro_z) else NAN
        )
        tf_yaw_change_deg = (
            wrap_deg(ob_end["yaw_deg"] - ob_start["yaw_deg"])
            if ob_start["valid"] and ob_end["valid"] else NAN
        )

        # Odom-frame laser pose: odom->base + base->laser  (NO map->odom).
        odom_laser = compose_se2(ob_mid, base_laser) if ob_mid["valid"] else None
        est_odom = (
            dominant_segment_orientation(scan, odom_laser, min_segment_length, max_segment_length)
            if odom_laser else None
        )
        # Map-frame laser pose: map->odom + odom->base + base->laser.
        if mo_mid["valid"] and ob_mid["valid"]:
            map_laser = compose_se2(compose_se2(mo_mid, ob_mid), base_laser)
            est_map = dominant_segment_orientation(scan, map_laser, min_segment_length, max_segment_length)
        else:
            est_map = None

        out.append({
            "scan_index": int(scan["scan_index"]),
            "t": float(scan["t"]),
            "scan_bag_timestamp_ns": int(scan["bag_timestamp_ns"]),
            "scan_header_timestamp_ns": int(scan["header_timestamp_ns"]),
            "scan_transport_delay_ms": float(scan["transport_delay_ms"]),
            "effective_scan_duration_s": duration_s,
            "scan_start_timestamp_ns": start_ns,
            "scan_mid_timestamp_ns": mid_ns,
            "scan_end_timestamp_ns": end_ns,
            "odom_base_source": odom_base_source,
            "odom_tf_start_valid": int(bool(ob_start["valid"])),
            "odom_tf_mid_valid": int(bool(ob_mid["valid"])),
            "odom_tf_end_valid": int(bool(ob_end["valid"])),
            "odom_tf_bracket_width_ms": ob_metrics["bracket_width_ms"],
            "nearest_odom_tf_signed_offset_ms": nearest_tf_signed_offset_ms(ob_prev, ob_next, start_ns),
            "map_odom_tf_bracket_width_ms": mo_metrics["bracket_width_ms"],
            "odom_base_yaw_deg": ob_mid["yaw_deg"] if ob_mid["valid"] else NAN,
            "tf_yaw_change_during_scan_deg": tf_yaw_change_deg,
            "map_odom_yaw_mid_deg": mo_mid["yaw_deg"] if mo_mid["valid"] else NAN,
            "map_odom_yaw_change_interpolated_deg": NAN,  # filled by apply_frame_baselines
            "imu_yaw_deg": imu_yaw,
            "imu_gyro_z_rad_s": gyro_z,
            "imu_header_gap_ms": imu_gap_ms,
            "odom_angular_z_rad_s": odom_az,
            "cmd_vel_nav_angular_z_rad_s": float(cmd_nav["angular_z"]) if cmd_nav else NAN,
            "predicted_scan_motion_imu_deg": predicted_imu_deg,
            "wall_orientation_map_deg": est_map["wall_orientation_deg"] if est_map else NAN,
            "wall_orientation_odom_deg": est_odom["wall_orientation_deg"] if est_odom else NAN,
            "orientation_shift_map_deg": NAN,   # filled by apply_frame_baselines
            "orientation_shift_odom_deg": NAN,  # filled by apply_frame_baselines
            "segment_count_map": est_map["segment_count"] if est_map else 0,
            "segment_count_odom": est_odom["segment_count"] if est_odom else 0,
            "confidence_map": est_map["confidence"] if est_map else NAN,
            "confidence_odom": est_odom["confidence"] if est_odom else NAN,
        })
    return out


def apply_frame_baselines(rows: List[dict], baseline_seconds: float) -> None:
    """Compute per-frame baselines and shifts; interpolated map->odom change."""
    if not rows:
        return
    t0 = float(rows[0]["t"])

    def median_baseline(key: str) -> float:
        vals = [
            float(r[key]) for r in rows
            if float(r["t"]) <= t0 + baseline_seconds and finite_number(r.get(key))
        ]
        return statistics.median(vals) if vals else NAN

    base_map = median_baseline("wall_orientation_map_deg")
    base_odom = median_baseline("wall_orientation_odom_deg")
    base_mo = median_baseline("map_odom_yaw_mid_deg")

    for r in rows:
        if finite_number(r.get("wall_orientation_map_deg")) and finite_number(base_map):
            r["orientation_shift_map_deg"] = wrap_period_deg(
                r["wall_orientation_map_deg"] - base_map, 90.0
            )
        if finite_number(r.get("wall_orientation_odom_deg")) and finite_number(base_odom):
            r["orientation_shift_odom_deg"] = wrap_period_deg(
                r["wall_orientation_odom_deg"] - base_odom, 90.0
            )
        # Interpolated cumulative map->odom change. For plotting/CSV only;
        # NOT used for event ordering (can smear a jump backward in time).
        if finite_number(r.get("map_odom_yaw_mid_deg")) and finite_number(base_mo):
            r["map_odom_yaw_change_interpolated_deg"] = wrap_deg(
                r["map_odom_yaw_mid_deg"] - base_mo
            )


def detect_raw_map_odom_correction(
    data: dict,
    baseline_seconds: float,
    threshold_deg: float,
    window_start: float,
    window_end: float,
) -> Optional[dict]:
    """
    Detect the first significant map->odom correction from RAW, DEDUPLICATED
    map->odom TF samples at their original header stamps. No interpolation.
    """
    rows = dedup_by_header_stamp(data["tf_map_odom"])
    rows.sort(key=lambda r: int(r["header_timestamp_ns"]))
    if not rows:
        return None
    t0 = float(rows[0]["t"])
    base_vals = [
        float(r["yaw_deg"]) for r in rows
        if float(r["t"]) <= t0 + baseline_seconds
    ]
    baseline = statistics.median(base_vals) if base_vals else float(rows[0]["yaw_deg"])
    for r in rows:
        if not (window_start <= float(r["t"]) <= window_end):
            continue
        cumulative = wrap_deg(float(r["yaw_deg"]) - baseline)
        if abs(cumulative) >= threshold_deg:
            return {
                "header_timestamp_ns": int(r["header_timestamp_ns"]),
                "t": float(r["t"]),
                "bag_timestamp_ns": int(r["bag_timestamp_ns"]),
                "cumulative_correction_deg": cumulative,
                "baseline_yaw_deg": baseline,
                "yaw_deg": float(r["yaw_deg"]),
                "source": "raw deduplicated /tf map->odom_combined",
            }
    return None


def detect_shift_events(
    rows: Sequence[dict],
    shift_key: str,
    confidence_key: str,
    threshold_deg: float,
    min_confidence: float,
    window_start: float,
    window_end: float,
) -> Tuple[Optional[dict], Optional[dict]]:
    """
    Return (first_single_crossing, first_sustained_crossing) within the window.
    Sustained = >=2 consecutive usable samples (finite shift and confidence
    above min_confidence) with |shift| >= threshold.
    """
    single: Optional[dict] = None
    sustained: Optional[dict] = None
    consec = 0
    for r in rows:
        if not (window_start <= float(r["t"]) <= window_end):
            consec = 0
            continue
        v = r.get(shift_key)
        c = r.get(confidence_key)
        usable = finite_number(v) and finite_number(c) and float(c) > min_confidence
        if usable and abs(float(v)) >= threshold_deg:
            if single is None:
                single = r
            consec += 1
            if sustained is None and consec >= 2:
                sustained = r
        else:
            consec = 0
    return single, sustained


def coverage_stats(
    name: str,
    rows: Sequence[dict],
    max_bracket_ms: float,
    scan_mid_times_ns: Sequence[int],
) -> dict:
    """Per-source coverage on deduplicated header stamps."""
    raw_count = len(rows)
    dedup = dedup_by_header_stamp(rows)
    dedup.sort(key=lambda r: int(r["header_timestamp_ns"]))
    unique_count = len(dedup)
    dup_count = raw_count - unique_count

    header_stamps = [int(r["header_timestamp_ns"]) for r in dedup]
    bag_times = [int(r["bag_timestamp_ns"]) for r in dedup]
    header_intervals = [
        (header_stamps[i] - header_stamps[i - 1]) / 1e9
        for i in range(1, len(header_stamps))
    ]
    bag_intervals = [
        (bag_times[i] - bag_times[i - 1]) / 1e9
        for i in range(1, len(bag_times))
    ]

    max_bracket_ns = int(max(0.0, max_bracket_ms) * 1e6)
    bracketable = sum(
        1 for t_ns in scan_mid_times_ns
        if interpolate_se2_at_header(dedup, header_stamps, t_ns, max_bracket_ns)["valid"]
    )
    total_scans = len(scan_mid_times_ns)
    big_gaps = [iv for iv in header_intervals if iv > max_bracket_ms / 1000.0]

    return {
        "source": name,
        "raw_message_count": raw_count,
        "unique_header_stamp_count": unique_count,
        "duplicate_header_stamp_count": dup_count,
        "median_header_stamp_interval_s": statistics.median(header_intervals) if header_intervals else NAN,
        "max_header_stamp_interval_s": max(header_intervals) if header_intervals else NAN,
        "median_bag_record_interval_s": statistics.median(bag_intervals) if bag_intervals else NAN,
        "max_bag_record_interval_s": max(bag_intervals) if bag_intervals else NAN,
        "gaps_exceeding_bracket": len(big_gaps),
        "largest_header_stamp_gap_s": max(header_intervals) if header_intervals else NAN,
        "scans_bracketable": bracketable,
        "total_scans": total_scans,
        "bracketable_pct": (100.0 * bracketable / total_scans) if total_scans else NAN,
    }


def establish_event_ordering(
    rows: List[dict],
    raw_mo_event: Optional[dict],
    shift_threshold_deg: float,
    min_confidence: float,
    window_start: float,
    window_end: float,
) -> dict:
    odom_single, odom_sustained = detect_shift_events(
        rows, "orientation_shift_odom_deg", "confidence_odom",
        shift_threshold_deg, min_confidence, window_start, window_end,
    )
    map_single, map_sustained = detect_shift_events(
        rows, "orientation_shift_map_deg", "confidence_map",
        shift_threshold_deg, min_confidence, window_start, window_end,
    )

    def context(r: Optional[dict]) -> Optional[dict]:
        if r is None:
            return None
        return {
            "t": float(r["t"]),
            "scan_index": int(r["scan_index"]),
            "imu_gyro_z_rad_s": r.get("imu_gyro_z_rad_s", NAN),
            "cmd_vel_nav_angular_z_rad_s": r.get("cmd_vel_nav_angular_z_rad_s", NAN),
            "predicted_scan_motion_imu_deg": r.get("predicted_scan_motion_imu_deg", NAN),
            "orientation_shift_odom_deg": r.get("orientation_shift_odom_deg", NAN),
            "orientation_shift_map_deg": r.get("orientation_shift_map_deg", NAN),
            "map_odom_yaw_change_interpolated_deg": r.get("map_odom_yaw_change_interpolated_deg", NAN),
        }

    return {
        "first_odom_single": context(odom_single),
        "first_odom_sustained": context(odom_sustained),
        "first_map_single": context(map_single),
        "first_map_sustained": context(map_sustained),
        "raw_map_odom_correction": raw_mo_event,
    }


def plot_or_placeholder(path: Path, title: str, xlabel: str, ylabel: str, plotter) -> None:
    plt.figure(figsize=(13, 7))
    plotted = bool(plotter())
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    if plotted:
        plt.legend()
    else:
        plt.text(0.5, 0.5, "No aligned data", ha="center", va="center", transform=plt.gca().transAxes)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def finite_xy(rows: Sequence[dict], x_key: str, y_key: str) -> Tuple[List[float], List[float]]:
    xs: List[float] = []
    ys: List[float] = []
    for row in rows:
        x = row.get(x_key)
        y = row.get(y_key)
        if finite_number(x) and finite_number(y):
            xs.append(float(x))
            ys.append(float(y))
    return xs, ys


def make_frame_isolation_plot(
    rows: Sequence[dict],
    ordering: dict,
    output: Path,
    window_start: float,
    window_end: float,
) -> None:
    window = [r for r in rows if window_start <= float(r["t"]) <= window_end]

    def plotter():
        plotted = False
        for label, key in (
            ("Odom-frame scan shift", "orientation_shift_odom_deg"),
            ("Map-frame scan shift", "orientation_shift_map_deg"),
            ("map->odom change (interpolated)", "map_odom_yaw_change_interpolated_deg"),
            ("Predicted IMU intra-scan motion", "predicted_scan_motion_imu_deg"),
        ):
            xs, ys = finite_xy(window, "t", key)
            if xs:
                plt.plot(xs, ys, label=label, linewidth=1.2)
                plotted = True

        markers = [
            ("first sustained odom-frame shift", ordering.get("first_odom_sustained")),
            ("first sustained map-frame shift", ordering.get("first_map_sustained")),
            ("raw map->odom correction", ordering.get("raw_map_odom_correction")),
        ]
        for label, ev in markers:
            if not ev or not finite_number(ev.get("t")):
                continue
            t = float(ev["t"])
            plt.axvline(t, color="red", alpha=0.5, linestyle="--")
            gyro = ev.get("imu_gyro_z_rad_s", NAN)
            cmd = ev.get("cmd_vel_nav_angular_z_rad_s", NAN)
            extra = ""
            if finite_number(gyro):
                extra += f" gyro_z={gyro:.3f}"
            if finite_number(cmd):
                extra += f" cmd_nav_az={cmd:.3f}"
            plt.annotate(
                f"{label}\nt={t:.3f}s{extra}",
                xy=(t, 0), xytext=(8, 8), textcoords="offset points", fontsize=8,
                color="red",
            )
            plotted = True
        return plotted

    plot_or_placeholder(
        output / f"first_failure_frame_isolation_{window_start:g}_{window_end:g}s.png",
        f"Frame-isolation first-failure window ({window_start:g}-{window_end:g} s)",
        "Bag-relative time (s)",
        "Angle (deg)",
        plotter,
    )


def _fmt_opt(value, ndigits: int = 3) -> str:
    if value is None:
        return "none (no crossing in window)"
    if isinstance(value, dict):
        t = value.get("t")
        if not finite_number(t):
            return "none (no crossing in window)"
        return f"t={float(t):.3f}s"
    if finite_number(value):
        return f"{float(value):.{ndigits}f}"
    return "n/a"


def build_frame_isolation_report(
    data: dict,
    rows: List[dict],
    coverage: List[dict],
    ordering: dict,
    output: Path,
    bag_uri: Path,
    args,
) -> None:
    duration_candidates = [
        r["t"]
        for key in ("odom", "amcl", "imu", "tf_dynamic", "scan", "cmd_vel", "cmd_vel_nav")
        for r in data[key]
    ]
    duration = max(duration_candidates) if duration_candidates else 0.0

    cov_by_name = {c["source"]: c for c in coverage}

    lines: List[str] = [
        f"# Localization Bag Analysis v{VERSION}", "",
        f"- Bag: `{bag_uri}`",
        f"- Parsed duration: `{duration:.3f} s`",
        f"- Analytical odom->base source: `{args.odom_base_source}`",
        f"- Timing TF maximum interpolation bracket: `{args.timing_max_bracket_ms:.1f} ms`",
        f"- IMU/Odom header match tolerance: `{args.sensor_match_max_ms:.1f} ms`",
        f"- Laser frame: `{data['laser_frame']}`",
        f"- Scan shift threshold: `{args.scan_shift_threshold:.2f} deg`",
        f"- Sustained crossing: `>=2 consecutive usable samples` (confidence > {args.min_confidence:.3f})",
        f"- Raw map->odom correction threshold: `{args.mo_correction_threshold_deg:.2f} deg`",
        f"- Event window: `{args.timing_window_start:g}-{args.timing_window_end:g} s`", "",
        "## Message counts", "",
        f"- /odom_combined: `{len(data['odom'])}`",
        f"- /amcl_pose: `{len(data['amcl'])}`",
        f"- IMU: `{len(data['imu'])}`",
        f"- /tf map->odom_combined: `{len(data['tf_map_odom'])}`",
        f"- /tf odom_combined->base_footprint: `{len(data['tf_odom_base_tf'])}`",
        f"- /odom_combined (odom->base fallback): `{len(data['tf_odom_base_odom'])}`",
        f"- /cmd_vel_nav: `{len(data['cmd_vel_nav'])}`",
        f"- /cmd_vel: `{len(data['cmd_vel'])}`",
        f"- sampled scans: `{len(data['scan'])}`",
        f"- frame-isolation rows: `{len(rows)}`", "",
        "## TF source coverage (deduplicated by header stamp)", "",
        "| source | raw | unique stamps | dup | median header int (s) | max header int (s) | max bag int (s) | gaps>bracket | bracketable scans | bracketable % |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for c in coverage:
        lines.append(
            f"| `{c['source']}` | {c['raw_message_count']} | {c['unique_header_stamp_count']} | "
            f"{c['duplicate_header_stamp_count']} | {c['median_header_stamp_interval_s']:.4f} | "
            f"{c['max_header_stamp_interval_s']:.4f} | {c['max_bag_record_interval_s']:.4f} | "
            f"{c['gaps_exceeding_bracket']} | {c['scans_bracketable']}/{c['total_scans']} | "
            f"{c['bracketable_pct']:.1f}% |"
        )

    lines += [
        "", "### Coverage findings", "",
        "Finding A — Analyzer issue (confirmed against v5): v5 discarded the dense "
        "`/odom_combined` odom->base fallback whenever any `/tf` odom->base samples "
        "existed. v6 retains both sources. The `/odom_combined` fallback is dense "
        f"(see coverage table above) and gives far higher scan bracket coverage than `/tf`.",
        "",
        "Finding B — Possible runtime TF issue (not pre-classified): the recorded "
        "`/tf odom_combined->base_footprint` series is the transform runtime TF "
        "consumers actually use. Its rate and header-stamp gaps are characterized "
        "in the coverage table above. Whether this constitutes a real TF publishing "
        "gap cannot be concluded from the bag alone; it requires live verification "
        "(`ros2 topic info /tf --verbose`, `rqt_tf_tree`, publisher rate checks). "
        "Do not conclude there is no real TF publishing gap until the `/tf` source "
        "is independently characterized live.",
        "",
        "Publisher multiplicity cannot be determined from TF pair counts alone. "
        "TF pair counts summing to the total `/tf` transform count does not prove a "
        "single publisher or rule out mixed publishers. Only classify publisher "
        "multiplicity using publisher metadata or live `ros2 topic info /tf --verbose`.",
        "",
        "Analytical distinction: using `/odom_combined` as the v6 analytical odom->base "
        "source improves ANALYSIS coverage. It does NOT prove the runtime `/tf` chain "
        "had adequate publication frequency.",
        "",
        "## Frame-isolation method", "",
        "Map-frame laser pose = `map->odom + odom->base + base->laser` (preserved from v5).",
        "Odom-frame laser pose = `odom->base + base->laser` (NEW; no map->odom).",
        "Both use the SAME dominant-wall-orientation algorithm (4-theta circular mean, "
        "90-degree periodic), so the comparison is controlled. The two estimates differ "
        "only by the map->odom transform.",
        "- Odom-frame wall orientation stable, map-frame shifts => shift attributable to map->odom.",
        "- Odom-frame wall orientation shifts => scan moved relative to odom (upstream of AMCL).",
        "",
        "## Event ordering (raw map->odom correction vs sustained scan shifts)", "",
    ]

    om = ordering.get("raw_map_odom_correction")
    od_s = ordering.get("first_odom_sustained")
    map_s = ordering.get("first_map_sustained")
    od_single = ordering.get("first_odom_single")
    map_single = ordering.get("first_map_single")

    lines += [
        "| event | time | scan_index | imu_gyro_z (rad/s) | cmd_nav angular.z (rad/s) | predicted IMU motion (deg) |",
        "|---|---|---|---|---|---|",
    ]

    def ctx_row(label: str, ev) -> None:
        if not ev:
            lines.append(f"| {label} | none in window | - | - | - | - |")
            return
        t = ev.get("t")
        lines.append(
            f"| {label} | {float(t):.3f}s | {ev.get('scan_index', '-')} | "
            f"{ev.get('imu_gyro_z_rad_s', NAN):.4f} | {ev.get('cmd_vel_nav_angular_z_rad_s', NAN):.4f} | "
            f"{ev.get('predicted_scan_motion_imu_deg', NAN):.3f} |"
        )

    def mo_row() -> None:
        if not om:
            lines.append("| raw map->odom correction | none in window | - | - | - | - |")
            return
        # raw mo event is a TF sample, not a scan; find nearest scan for context.
        nearest = None
        if rows:
            nearest = min(rows, key=lambda r: abs(float(r["t"]) - float(om["t"])))
        lines.append(
            f"| raw map->odom correction | {float(om['t']):.3f}s | "
            f"{nearest['scan_index'] if nearest else '-'} (nearest scan) | "
            f"{nearest.get('imu_gyro_z_rad_s', NAN) if nearest else NAN:.4f} | "
            f"{nearest.get('cmd_vel_nav_angular_z_rad_s', NAN) if nearest else NAN:.4f} | "
            f"{nearest.get('predicted_scan_motion_imu_deg', NAN) if nearest else NAN:.3f} |"
        )

    ctx_row("first odom-frame single crossing", od_single)
    ctx_row("first odom-frame sustained crossing", od_s)
    ctx_row("first map-frame single crossing", map_single)
    ctx_row("first map-frame sustained crossing", map_s)
    mo_row()

    if om:
        lines.append(
            f"\nRaw map->odom correction detail: cumulative "
            f"`{om['cumulative_correction_deg']:.3f} deg` from baseline "
            f"`{om['baseline_yaw_deg']:.3f} deg` at header stamp "
            f"`{om['header_timestamp_ns']}` (source: {om['source']})."
        )
    else:
        lines.append(
            "\nNo raw map->odom correction exceeding the threshold was found in the window."
        )

    lines += ["", "## Interpretation", ""]

    t_mo = float(om["t"]) if om and finite_number(om.get("t")) else None
    t_od = float(od_s["t"]) if od_s and finite_number(od_s.get("t")) else None

    if t_od is not None and t_mo is not None:
        if t_od < t_mo:
            lines.append(
                "- The first sustained ODOM-FRAME scan deviation (t="
                f"{t_od:.3f}s) PRECEDES the raw map->odom correction (t={t_mo:.3f}s). "
                "Per the stated rule, this points UPSTREAM of AMCL. AMCL is NOT "
                "inferred as the root cause."
            )
        else:
            lines.append(
                "- The raw map->odom correction (t=" f"{t_mo:.3f}s) precedes or coincides "
                f"with the first sustained odom-frame scan deviation (t={t_od:.3f}s). "
                "AMCL becomes a POSSIBLE initial contributor, but this alone does not "
                "establish why AMCL changed map->odom."
            )
    elif t_od is not None and t_mo is None:
        lines.append(
            f"- A sustained odom-frame scan deviation occurs (t={t_od:.3f}s) with NO raw "
            "map->odom correction exceeding threshold in the window. This points "
            "UPSTREAM of AMCL. AMCL is NOT inferred as the root cause."
        )
    elif t_od is None and t_mo is not None:
        lines.append(
            "- The odom-frame scan orientation remained stable (no sustained crossing) "
            f"while a raw map->odom correction occurred (t={t_mo:.3f}s). The observed "
            "map-frame error is attributable to map->odom, but this alone does NOT "
            "establish why AMCL changed it."
        )
    elif t_od is None and t_mo is None:
        lines.append(
            "- Neither a sustained odom-frame scan deviation nor a raw map->odom "
            "correction was detected in the window. No causal ordering is established."
        )

    lines += [
        "",
        "### Root-cause rule (enforced)",
        "",
        "AMCL is NOT claimed as the root cause unless a RAW abnormal map->odom "
        "correction clearly PRECEDES the first sustained odom-frame scan deviation. "
        "Interpretation:",
        "- odom-frame shift precedes raw map->odom correction => evidence points upstream of AMCL;",
        "- raw map->odom correction precedes odom-frame shift => AMCL becomes a possible initial contributor;",
        "- odom-frame stable while only map-frame shifts => map-frame error attributable to map->odom, "
        "but this alone does not establish why AMCL changed it.",
        "",
        "## Generated files", "",
        "- `frame_isolation_report.md`",
        "- `scan_frame_isolation.csv`",
        "- `tf_source_coverage.csv`",
        "- `event_ordering.csv`",
        "- `first_failure_frame_isolation_<start>_<end>s.png`",
    ]
    (output / "frame_isolation_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_outputs(
    rows: List[dict],
    coverage: List[dict],
    ordering: dict,
    output: Path,
) -> None:
    isolation_fields = [
        "scan_index", "t", "scan_bag_timestamp_ns", "scan_header_timestamp_ns",
        "scan_transport_delay_ms", "effective_scan_duration_s",
        "scan_start_timestamp_ns", "scan_mid_timestamp_ns", "scan_end_timestamp_ns",
        "odom_base_source",
        "odom_tf_start_valid", "odom_tf_mid_valid", "odom_tf_end_valid",
        "odom_tf_bracket_width_ms", "nearest_odom_tf_signed_offset_ms",
        "map_odom_tf_bracket_width_ms",
        "odom_base_yaw_deg", "tf_yaw_change_during_scan_deg",
        "map_odom_yaw_mid_deg", "map_odom_yaw_change_interpolated_deg",
        "imu_yaw_deg", "imu_gyro_z_rad_s", "imu_header_gap_ms",
        "odom_angular_z_rad_s", "cmd_vel_nav_angular_z_rad_s",
        "predicted_scan_motion_imu_deg",
        "wall_orientation_map_deg", "wall_orientation_odom_deg",
        "orientation_shift_map_deg", "orientation_shift_odom_deg",
        "segment_count_map", "segment_count_odom",
        "confidence_map", "confidence_odom",
    ]
    write_csv(output / "scan_frame_isolation.csv", isolation_fields, rows)

    coverage_fields = [
        "source", "raw_message_count", "unique_header_stamp_count",
        "duplicate_header_stamp_count", "median_header_stamp_interval_s",
        "max_header_stamp_interval_s", "median_bag_record_interval_s",
        "max_bag_record_interval_s", "gaps_exceeding_bracket",
        "largest_header_stamp_gap_s", "scans_bracketable", "total_scans",
        "bracketable_pct",
    ]
    write_csv(output / "tf_source_coverage.csv", coverage_fields, coverage)

    # Flatten ordering into rows for CSV traceability.
    event_rows: List[dict] = []
    for label, key in (
        ("first_odom_single", "first_odom_single"),
        ("first_odom_sustained", "first_odom_sustained"),
        ("first_map_single", "first_map_single"),
        ("first_map_sustained", "first_map_sustained"),
        ("raw_map_odom_correction", "raw_map_odom_correction"),
    ):
        ev = ordering.get(key)
        if not ev:
            event_rows.append({"event": label, "t": "", "scan_index": "",
                               "imu_gyro_z_rad_s": "", "cmd_vel_nav_angular_z_rad_s": "",
                               "predicted_scan_motion_imu_deg": "",
                               "orientation_shift_odom_deg": "", "orientation_shift_map_deg": "",
                               "map_odom_yaw_change_interpolated_deg": "",
                               "cumulative_correction_deg": "", "detail": "none in window"})
            continue
        event_rows.append({
            "event": label,
            "t": ev.get("t", ""),
            "scan_index": ev.get("scan_index", ""),
            "imu_gyro_z_rad_s": ev.get("imu_gyro_z_rad_s", ""),
            "cmd_vel_nav_angular_z_rad_s": ev.get("cmd_vel_nav_angular_z_rad_s", ""),
            "predicted_scan_motion_imu_deg": ev.get("predicted_scan_motion_imu_deg", ""),
            "orientation_shift_odom_deg": ev.get("orientation_shift_odom_deg", ""),
            "orientation_shift_map_deg": ev.get("orientation_shift_map_deg", ""),
            "map_odom_yaw_change_interpolated_deg": ev.get("map_odom_yaw_change_interpolated_deg", ""),
            "cumulative_correction_deg": ev.get("cumulative_correction_deg", ""),
            "detail": "raw deduplicated /tf map->odom_combined" if key == "raw_map_odom_correction" else "",
        })
    write_csv(
        output / "event_ordering.csv",
        ["event", "t", "scan_index", "imu_gyro_z_rad_s", "cmd_vel_nav_angular_z_rad_s",
         "predicted_scan_motion_imu_deg", "orientation_shift_odom_deg",
         "orientation_shift_map_deg", "map_odom_yaw_change_interpolated_deg",
         "cumulative_correction_deg", "detail"],
        event_rows,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze ROS 2 localization with map/odom frame isolation."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--map-frame", default=DEFAULT_MAP_FRAME)
    parser.add_argument("--odom-frame", default=DEFAULT_ODOM_FRAME)
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--laser-frame", default=None,
                        help="Override LaserScan frame; otherwise use /scan header.frame_id")
    parser.add_argument("--odom-base-source", choices=("odom_combined", "tf", "merged"),
                        default="odom_combined",
                        help="Analytical odom->base source (default: dense /odom_combined)")
    parser.add_argument("--scan-stride", type=int, default=1)
    parser.add_argument("--scan-shift-threshold", type=float, default=3.0)
    parser.add_argument("--scan-baseline-seconds", type=float, default=10.0)
    parser.add_argument("--segment-min-length", type=float, default=0.005)
    parser.add_argument("--segment-max-length", type=float, default=0.12)
    parser.add_argument("--mo-correction-threshold-deg", type=float, default=1.0,
                        help="Raw cumulative map->odom correction magnitude threshold")
    parser.add_argument("--min-confidence", type=float, default=0.0,
                        help="Minimum orientation confidence for a usable sustained-crossing sample")
    parser.add_argument("--timing-max-bracket-ms", type=float, default=250.0)
    parser.add_argument("--sensor-match-max-ms", type=float, default=100.0)
    parser.add_argument("--timing-window-start", type=float, default=15.0)
    parser.add_argument("--timing-window-end", type=float, default=30.0)
    args = parser.parse_args()

    if args.scan_stride < 1:
        parser.error("--scan-stride must be >= 1")
    if args.timing_max_bracket_ms < 0:
        parser.error("--timing-max-bracket-ms must be >= 0")
    if args.sensor_match_max_ms < 0:
        parser.error("--sensor-match-max-ms must be >= 0")
    if args.timing_window_end <= args.timing_window_start:
        parser.error("--timing-window-end must be greater than --timing-window-start")

    try:
        bag_uri = resolve_bag_uri(args.bag)
    except FileNotFoundError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    output = (
        args.output.expanduser().resolve()
        if args.output else bag_uri / "analysis_v6_frame_isolation"
    )
    output.mkdir(parents=True, exist_ok=True)
    map_frame = normalize_frame(args.map_frame)
    odom_frame = normalize_frame(args.odom_frame)
    base_frame = normalize_frame(args.base_frame)

    print(f"Localization Bag Analyzer v{VERSION}")
    print(f"Reading bag: {bag_uri}")
    print(f"Frames: {map_frame} -> {odom_frame} -> {base_frame}")
    print(f"Analytical odom->base source: {args.odom_base_source}")

    data = extract_data(
        bag_uri, map_frame, odom_frame, base_frame,
        args.laser_frame, args.scan_stride,
    )
    laser_frame = data["laser_frame"] or normalize_frame(args.laser_frame or DEFAULT_LASER_FRAME)
    base_laser = find_static_transform(data, base_frame, laser_frame)
    if base_laser is None and base_frame == laser_frame:
        base_laser = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    if base_laser is None:
        print(f"Error: no static {base_frame}->{laser_frame} transform found.", file=sys.stderr)
        return 3

    rows = build_frame_isolation_rows(
        data, base_laser, args.timing_max_bracket_ms, args.sensor_match_max_ms,
        args.segment_min_length, args.segment_max_length, args.odom_base_source,
    )
    apply_frame_baselines(rows, args.scan_baseline_seconds)

    raw_mo_event = detect_raw_map_odom_correction(
        data, args.scan_baseline_seconds, args.mo_correction_threshold_deg,
        args.timing_window_start, args.timing_window_end,
    )
    ordering = establish_event_ordering(
        rows, raw_mo_event, args.scan_shift_threshold, args.min_confidence,
        args.timing_window_start, args.timing_window_end,
    )

    scan_mid_times_ns = [
        int(s["scan_mid_timestamp_ns"]) or int(s["header_timestamp_ns"])
        for s in data["scan"]
    ]
    coverage = [
        coverage_stats("/tf odom_combined->base_footprint", data["tf_odom_base_tf"],
                       args.timing_max_bracket_ms, scan_mid_times_ns),
        coverage_stats("/odom_combined (odom->base fallback)", data["tf_odom_base_odom"],
                       args.timing_max_bracket_ms, scan_mid_times_ns),
        coverage_stats("/tf map->odom_combined", data["tf_map_odom"],
                       args.timing_max_bracket_ms, scan_mid_times_ns),
    ]

    save_outputs(rows, coverage, ordering, output)
    make_frame_isolation_plot(
        rows, ordering, output, args.timing_window_start, args.timing_window_end
    )
    build_frame_isolation_report(data, rows, coverage, ordering, output, bag_uri, args)

    usable = sum(1 for r in rows if finite_number(r.get("orientation_shift_odom_deg")))
    print(f"frame-isolation rows: {len(rows)} (usable odom orientation: {usable})")
    print("Event ordering:")
    for label in ("first_odom_sustained", "first_map_sustained", "raw_map_odom_correction"):
        ev = ordering.get(label)
        t = ev.get("t") if ev else None
        print(f"  {label}: {('t=%.3fs' % float(t)) if finite_number(t) else 'none in window'}")
    print("Analysis completed.")
    print(f"Output directory: {output}")
    print(f"Report: {output / 'frame_isolation_report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
