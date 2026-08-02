#!/usr/bin/env python3
"""
ROS 2 Localization Bag Analyzer v5.0 Timing

Designed for ROS 2 Humble / rosbag2 sqlite3.

Key capabilities
----------------
1. Uses one bag-wide time origin for every topic.
2. Reads both /tf and /tf_static.
3. Supports /cmd_vel and /cmd_vel_nav, with Twist or TwistStamped.
4. Reconstructs map -> odom -> base -> laser in SE(2).
5. Falls back to /odom_combined pose when odom->base TF is unavailable.
6. Aligns /amcl_pose with the reconstructed map->base pose.
7. Estimates LaserScan wall orientation in the map frame and detects sudden
   scan-orientation shifts. This does not require /map; it assumes the observed
   environment contains enough approximately straight wall segments.
8. Writes diagnostics explaining why an output series is empty.
9. Compares rosbag record timestamps with message header timestamps.
10. Brackets LaserScan timestamps with dynamic TF header timestamps.
11. Estimates intra-scan rotational motion from IMU, odometry, and TF.
12. Writes scan_tf_timing.csv plus timing-focused diagnostic plots.

Typical usage
-------------
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 analyze_localization_bag_v5_timing.py /path/to/bag

Important limitation
--------------------
The scan-orientation detector estimates dominant straight-line orientation.
It is a diagnostic signal, not a full scan-to-map matcher. For true occupancy
map registration, record /map as well and use a dedicated scan matcher.
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
        "  source ~/ros2_ws/install/setup.bash\n"
    ) from exc

VERSION = "5.0-timing"

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



NAN = float("nan")


def stamp_to_ns(stamp: object) -> int:
    """Convert a ROS builtin_interfaces/Time object to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transport_delay_ms(bag_timestamp_ns: int, header_timestamp_ns: int) -> float:
    """Return bag-record time minus message-header time in milliseconds."""
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
    """
    Return dynamic samples that bracket target_ns by header timestamp.

    When an exact timestamp exists, the same row is returned as both previous
    and next. Otherwise previous.header <= target < next.header.
    """
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
    """Interpolate x/y/yaw between dynamic TF samples using header stamps."""
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
    """Find the nearest message by header stamp, returning signed gap in ms."""
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
    """Compose parent->middle (a) with middle->child (b)."""
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
        "cmd_vel": [],
        "cmd_vel_nav": [],
        "scan": [],
        "tf_pair_counts": {},
        "laser_frame": normalize_frame(laser_frame_override) if laser_frame_override else None,
    }

    # First pass timestamp is the first message in the entire bag, not the first
    # selected topic. This prevents topic-dependent offsets and empty alignment.
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
            # Odom message itself is a valid odom->base transform fallback.
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
                    # Prefer real TF over odom fallback; dedup happens later.
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
                    "scan_end_to_bag_delay_ms": (
                        (timestamp_ns - scan_end_timestamp_ns) / 1e6
                        if scan_end_timestamp_ns > 0 else NAN
                    ),
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

    # If both /tf and odom fallback exist, use /tf rows when available.
    tf_rows = [r for r in data["tf_odom_base"] if r["source_topic"] == TF_TOPIC]
    if tf_rows:
        data["tf_odom_base"] = tf_rows

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

    # One-hop search is sufficient for common base_footprint->base_link->laser chains.
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


def build_aligned_localization(data: dict, max_tf_gap: float) -> List[dict]:
    mo_rows, ob_rows = data["tf_map_odom"], data["tf_odom_base"]
    mo_times = [r["t"] for r in mo_rows]
    ob_times = [r["t"] for r in ob_rows]
    aligned: List[dict] = []
    for amcl in data["amcl"]:
        mo = nearest_row(mo_rows, mo_times, amcl["t"], max_tf_gap)
        ob = nearest_row(ob_rows, ob_times, amcl["t"], max_tf_gap)
        if mo is None or ob is None:
            continue
        mb = compose_se2(mo, ob)
        aligned.append({
            "t": amcl["t"],
            "amcl_x": amcl["x"], "amcl_y": amcl["y"],
            "amcl_yaw_deg": amcl["yaw_deg"],
            "tf_map_base_x": mb["x"], "tf_map_base_y": mb["y"],
            "tf_map_base_yaw_deg": mb["yaw_deg"],
            "position_error_m": math.hypot(mb["x"] - amcl["x"], mb["y"] - amcl["y"]),
            "yaw_error_deg": wrap_deg(mb["yaw_deg"] - amcl["yaw_deg"]),
            "map_odom_time_gap_s": abs(mo["t"] - amcl["t"]),
            "odom_base_time_gap_s": abs(ob["t"] - amcl["t"]),
        })
    return aligned


def build_imu_odom_alignment(data: dict, max_gap: float = 0.15) -> List[dict]:
    times = [r["t"] for r in data["imu"]]
    out: List[dict] = []
    for odom in data["odom"]:
        imu = nearest_row(data["imu"], times, odom["t"], max_gap)
        if imu is None:
            continue
        out.append({
            "t": odom["t"],
            "imu_yaw_deg": imu["yaw_deg"],
            "odom_yaw_deg": odom["yaw_deg"],
            "yaw_difference_deg": wrap_deg(odom["yaw_deg"] - imu["yaw_deg"]),
            "time_gap_s": abs(odom["t"] - imu["t"]),
        })
    return out


def dominant_segment_orientation(
    scan: dict,
    map_laser: dict,
    min_segment_length: float,
    max_segment_length: float,
) -> Optional[dict]:
    """
    Estimate dominant straight-wall orientation in map coordinates.

    Consecutive valid scan points form short line segments. Their orientations
    are averaged using a 4-theta circular mean, making the result invariant to
    line direction and to 90-degree Manhattan-wall symmetry.
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

    yaw = math.radians(map_laser["yaw_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    sum_c = 0.0
    sum_s = 0.0
    count = 0

    previous: Optional[Tuple[float, float]] = None
    for point in points:
        if point is None:
            previous = None
            continue
        x = map_laser["x"] + c * point[0] - s * point[1]
        y = map_laser["y"] + s * point[0] + c * point[1]
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



def build_scan_orientation(
    data: dict,
    map_frame: str,
    odom_frame: str,
    base_frame: str,
    laser_frame: str,
    max_tf_gap: float,
    min_segment_length: float,
    max_segment_length: float,
) -> Tuple[List[dict], Optional[dict], dict]:
    diagnostics = {"missing_map_odom": 0, "missing_odom_base": 0, "weak_scan": 0}
    base_laser = find_static_transform(data, base_frame, laser_frame)
    if base_laser is None and base_frame == laser_frame:
        base_laser = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    if base_laser is None:
        return [], None, diagnostics

    mo_rows, ob_rows = data["tf_map_odom"], data["tf_odom_base"]
    mo_times = [r["t"] for r in mo_rows]
    ob_times = [r["t"] for r in ob_rows]
    out: List[dict] = []

    for scan in data["scan"]:
        # Keep v4's bag-time nearest-neighbour path for backwards-compatible
        # wall-orientation diagnostics. The new timing analysis below uses
        # header-stamp bracketing and does not depend on these nearest rows.
        mo = nearest_row(mo_rows, mo_times, scan["t"], max_tf_gap)
        if mo is None:
            diagnostics["missing_map_odom"] += 1
            continue
        ob = nearest_row(ob_rows, ob_times, scan["t"], max_tf_gap)
        if ob is None:
            diagnostics["missing_odom_base"] += 1
            continue
        map_laser = compose_se2(compose_se2(mo, ob), base_laser)
        estimate = dominant_segment_orientation(
            scan, map_laser, min_segment_length, max_segment_length
        )
        if estimate is None:
            diagnostics["weak_scan"] += 1
            continue
        out.append({
            "scan_index": scan["scan_index"],
            "t": scan["t"],
            "bag_timestamp_ns": scan["bag_timestamp_ns"],
            "header_timestamp_ns": scan["header_timestamp_ns"],
            "transport_delay_ms": scan["transport_delay_ms"],
            "effective_scan_duration": scan["effective_scan_duration"],
            "wall_orientation_deg": estimate["wall_orientation_deg"],
            "segment_count": estimate["segment_count"],
            "confidence": estimate["confidence"],
            "map_laser_x": map_laser["x"],
            "map_laser_y": map_laser["y"],
            "map_laser_yaw_deg": map_laser["yaw_deg"],
        })
    return out, base_laser, diagnostics

def detect_scan_shift_events(
    rows: Sequence[dict],
    threshold_deg: float,
    baseline_seconds: float,
) -> List[dict]:
    if not rows:
        return []
    baseline_values = [
        r["wall_orientation_deg"]
        for r in rows
        if r["t"] <= rows[0]["t"] + baseline_seconds
    ]
    if not baseline_values:
        baseline_values = [rows[0]["wall_orientation_deg"]]
    baseline = statistics.median(baseline_values)
    events: List[dict] = []
    active = False
    for row in rows:
        residual = wrap_period_deg(row["wall_orientation_deg"] - baseline, 90.0)
        row["baseline_orientation_deg"] = baseline
        row["orientation_shift_deg"] = residual
        bad = abs(residual) >= threshold_deg
        if bad and not active:
            events.append({
                "t": row["t"],
                "type": "scan_orientation_shift",
                "value": residual,
                "threshold": threshold_deg,
                "detail": "Dominant scan wall orientation shifted relative to initial baseline",
            })
        active = bad
    return events


def detect_tf_events(data: dict, rate_threshold: float) -> List[dict]:
    events: List[dict] = []
    rows = data["tf_map_odom"]
    for prev, cur in zip(rows, rows[1:]):
        dt = cur["t"] - prev["t"]
        if dt <= 0:
            continue
        delta = wrap_deg(cur["yaw_deg"] - prev["yaw_deg"])
        rate = abs(delta) / dt
        if rate >= rate_threshold:
            events.append({
                "t": cur["t"], "type": "map_odom_yaw_fast_change",
                "value": rate, "threshold": rate_threshold,
                "detail": f"map->odom yaw changed {delta:.3f} deg in {dt:.3f} s",
            })
    return events



def build_scan_timing_rows(
    data: dict,
    scan_rows: Sequence[dict],
    timing_max_bracket_ms: float,
    sensor_match_max_ms: float,
) -> List[dict]:
    """Build one header-stamp timing record for every sampled LaserScan."""
    max_bracket_ns = int(max(0.0, timing_max_bracket_ms) * 1e6)
    sensor_match_max_ns = int(max(0.0, sensor_match_max_ms) * 1e6)

    mo_rows = sorted(
        [r for r in data["tf_map_odom"] if int(r.get("header_timestamp_ns", 0)) > 0],
        key=lambda r: int(r["header_timestamp_ns"]),
    )
    ob_rows = sorted(
        [r for r in data["tf_odom_base"] if int(r.get("header_timestamp_ns", 0)) > 0],
        key=lambda r: int(r["header_timestamp_ns"]),
    )
    imu_rows = sorted(
        [r for r in data["imu"] if int(r.get("header_timestamp_ns", 0)) > 0],
        key=lambda r: int(r["header_timestamp_ns"]),
    )
    odom_rows = sorted(
        [r for r in data["odom"] if int(r.get("header_timestamp_ns", 0)) > 0],
        key=lambda r: int(r["header_timestamp_ns"]),
    )

    mo_times = [int(r["header_timestamp_ns"]) for r in mo_rows]
    ob_times = [int(r["header_timestamp_ns"]) for r in ob_rows]
    imu_times = [int(r["header_timestamp_ns"]) for r in imu_rows]
    odom_times = [int(r["header_timestamp_ns"]) for r in odom_rows]
    cmd_times = [float(r["t"]) for r in data["cmd_vel"]]
    cmd_nav_times = [float(r["t"]) for r in data["cmd_vel_nav"]]

    orientation_by_index = {int(r["scan_index"]): r for r in scan_rows}
    output: List[dict] = []

    for scan in data["scan"]:
        start_ns = int(scan["scan_start_timestamp_ns"])
        mid_ns = int(scan["scan_mid_timestamp_ns"])
        end_ns = int(scan["scan_end_timestamp_ns"])
        duration_s = float(scan["effective_scan_duration"])

        ob_previous, ob_next = bracket_rows_by_header(ob_rows, ob_times, start_ns)
        ob_start_metrics = bracket_metrics(ob_previous, ob_next, start_ns)
        ob_nearest_offset_ms = nearest_tf_signed_offset_ms(ob_previous, ob_next, start_ns)

        mo_previous, mo_next = bracket_rows_by_header(mo_rows, mo_times, start_ns)
        mo_start_metrics = bracket_metrics(mo_previous, mo_next, start_ns)
        mo_nearest_offset_ms = nearest_tf_signed_offset_ms(mo_previous, mo_next, start_ns)

        ob_start = interpolate_se2_at_header(
            ob_rows, ob_times, start_ns, max_bracket_ns
        )
        ob_mid = interpolate_se2_at_header(
            ob_rows, ob_times, mid_ns, max_bracket_ns
        )
        ob_end = interpolate_se2_at_header(
            ob_rows, ob_times, end_ns, max_bracket_ns
        )
        # map->odom is typically published much more slowly than odom->base.
        # Keep its raw bracket diagnostics but do not reject interpolation using
        # the high-rate odom TF bracket threshold. It is contextual AMCL data,
        # not the primary Scan/TF timing validity signal.
        mo_start = interpolate_se2_at_header(
            mo_rows, mo_times, start_ns, None
        )
        mo_mid = interpolate_se2_at_header(
            mo_rows, mo_times, mid_ns, None
        )

        imu, imu_gap_ms = nearest_row_by_header(
            imu_rows, imu_times, mid_ns, sensor_match_max_ns
        )
        odom, odom_gap_ms = nearest_row_by_header(
            odom_rows, odom_times, mid_ns, sensor_match_max_ns
        )

        # Twist messages generally have no header, so command context remains
        # aligned by bag-relative time. It is context only, not timing evidence.
        cmd = nearest_row(data["cmd_vel"], cmd_times, scan["t"], 0.5)
        cmd_nav = nearest_row(data["cmd_vel_nav"], cmd_nav_times, scan["t"], 0.5)

        gyro_z = float(imu["gyro_z"]) if imu else NAN
        odom_angular_z = float(odom["angular_z"]) if odom else NAN
        predicted_imu_deg = (
            math.degrees(gyro_z * duration_s) if finite_number(gyro_z) else NAN
        )
        predicted_odom_deg = (
            math.degrees(odom_angular_z * duration_s)
            if finite_number(odom_angular_z) else NAN
        )
        tf_yaw_change_deg = (
            wrap_deg(ob_end["yaw_deg"] - ob_start["yaw_deg"])
            if ob_start["valid"] and ob_end["valid"] else NAN
        )
        map_odom_yaw_change_deg = (
            wrap_deg(mo_mid["yaw_deg"] - mo_start["yaw_deg"])
            if mo_start["valid"] and mo_mid["valid"] else NAN
        )
        predicted_nearest_tf_error_imu_deg = (
            math.degrees(gyro_z * (ob_nearest_offset_ms / 1000.0))
            if finite_number(gyro_z) and finite_number(ob_nearest_offset_ms) else NAN
        )
        predicted_nearest_tf_error_odom_deg = (
            math.degrees(odom_angular_z * (ob_nearest_offset_ms / 1000.0))
            if finite_number(odom_angular_z) and finite_number(ob_nearest_offset_ms) else NAN
        )

        orientation = orientation_by_index.get(int(scan["scan_index"]), {})
        output.append({
            "scan_index": int(scan["scan_index"]),
            "t": float(scan["t"]),
            "scan_bag_timestamp_ns": int(scan["bag_timestamp_ns"]),
            "scan_header_timestamp_ns": int(scan["header_timestamp_ns"]),
            "scan_transport_delay_ms": float(scan["transport_delay_ms"]),
            "scan_end_to_bag_delay_ms": float(scan["scan_end_to_bag_delay_ms"]),
            "scan_time_s": float(scan["scan_time"]),
            "time_increment_s": float(scan["time_increment"]),
            "beam_count": int(scan["beam_count"]),
            "effective_scan_duration_s": duration_s,
            "scan_start_timestamp_ns": start_ns,
            "scan_mid_timestamp_ns": mid_ns,
            "scan_end_timestamp_ns": end_ns,

            "previous_odom_tf_header_timestamp_ns": ob_start_metrics["previous_header_timestamp_ns"],
            "next_odom_tf_header_timestamp_ns": ob_start_metrics["next_header_timestamp_ns"],
            "scan_to_prev_odom_tf_ms": ob_start_metrics["target_to_previous_ms"],
            "next_odom_tf_to_scan_ms": ob_start_metrics["next_to_target_ms"],
            "odom_tf_bracket_width_ms": ob_start_metrics["bracket_width_ms"],
            "nearest_odom_tf_signed_offset_ms": ob_nearest_offset_ms,

            "previous_map_odom_tf_header_timestamp_ns": mo_start_metrics["previous_header_timestamp_ns"],
            "next_map_odom_tf_header_timestamp_ns": mo_start_metrics["next_header_timestamp_ns"],
            "scan_to_prev_map_odom_tf_ms": mo_start_metrics["target_to_previous_ms"],
            "next_map_odom_tf_to_scan_ms": mo_start_metrics["next_to_target_ms"],
            "map_odom_tf_bracket_width_ms": mo_start_metrics["bracket_width_ms"],
            "nearest_map_odom_tf_signed_offset_ms": mo_nearest_offset_ms,

            "odom_tf_start_valid": int(bool(ob_start["valid"])),
            "odom_tf_mid_valid": int(bool(ob_mid["valid"])),
            "odom_tf_end_valid": int(bool(ob_end["valid"])),
            "map_odom_tf_start_valid": int(bool(mo_start["valid"])),
            "map_odom_tf_mid_valid": int(bool(mo_mid["valid"])),
            "odom_base_yaw_start_deg": ob_start["yaw_deg"],
            "odom_base_yaw_mid_deg": ob_mid["yaw_deg"],
            "odom_base_yaw_end_deg": ob_end["yaw_deg"],
            "tf_yaw_change_during_scan_deg": tf_yaw_change_deg,
            "map_odom_yaw_start_deg": mo_start["yaw_deg"],
            "map_odom_yaw_mid_deg": mo_mid["yaw_deg"],
            "map_odom_yaw_change_during_half_scan_deg": map_odom_yaw_change_deg,

            "imu_header_gap_ms": imu_gap_ms,
            "imu_gyro_z_rad_s": gyro_z,
            "odom_header_gap_ms": odom_gap_ms,
            "odom_angular_z_rad_s": odom_angular_z,
            "cmd_vel_angular_z_rad_s": float(cmd["angular_z"]) if cmd else NAN,
            "cmd_vel_nav_angular_z_rad_s": float(cmd_nav["angular_z"]) if cmd_nav else NAN,
            "predicted_scan_motion_imu_deg": predicted_imu_deg,
            "predicted_scan_motion_odom_deg": predicted_odom_deg,
            "predicted_nearest_tf_error_imu_deg": predicted_nearest_tf_error_imu_deg,
            "predicted_nearest_tf_error_odom_deg": predicted_nearest_tf_error_odom_deg,

            "wall_orientation_deg": orientation.get("wall_orientation_deg", NAN),
            "baseline_orientation_deg": orientation.get("baseline_orientation_deg", NAN),
            "orientation_shift_deg": orientation.get("orientation_shift_deg", NAN),
            "segment_count": orientation.get("segment_count", 0),
            "confidence": orientation.get("confidence", NAN),
        })

    return output


def print_timing_summary(rows: Sequence[dict]) -> None:
    print("Timing analysis summary:")
    print(f"  usable scan timing rows: {len(rows)}")
    print(f"  scan transport delay: {format_summary(rows, 'scan_transport_delay_ms', 'ms')}")
    print(f"  scan to previous odom TF: {format_summary(rows, 'scan_to_prev_odom_tf_ms', 'ms')}")
    print(f"  next odom TF to scan: {format_summary(rows, 'next_odom_tf_to_scan_ms', 'ms')}")
    print(f"  odom TF bracket width: {format_summary(rows, 'odom_tf_bracket_width_ms', 'ms')}")
    print(f"  scan-duration TF yaw change: {format_summary(rows, 'tf_yaw_change_during_scan_deg', 'deg')}")


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


def make_timing_plots(
    timing_rows: Sequence[dict],
    output: Path,
    window_start: float,
    window_end: float,
) -> None:
    def offsets_plot():
        plotted = False
        for label, key in (
            ("Scan transport delay", "scan_transport_delay_ms"),
            ("Scan to previous odom TF", "scan_to_prev_odom_tf_ms"),
            ("Next odom TF to scan", "next_odom_tf_to_scan_ms"),
            ("Odom TF bracket width", "odom_tf_bracket_width_ms"),
        ):
            xs, ys = finite_xy(timing_rows, "t", key)
            if xs:
                plt.plot(xs, ys, label=label)
                plotted = True
        return plotted

    plot_or_placeholder(
        output / "timing_offsets.png",
        "LaserScan and odom TF timing offsets",
        "Bag-relative time (s)",
        "Time offset (ms)",
        offsets_plot,
    )

    def motion_plot():
        plotted = False
        for label, key in (
            ("Observed scan orientation shift", "orientation_shift_deg"),
            ("TF yaw change during scan", "tf_yaw_change_during_scan_deg"),
            ("Predicted scan motion from IMU", "predicted_scan_motion_imu_deg"),
            ("Predicted scan motion from odom", "predicted_scan_motion_odom_deg"),
            ("Predicted nearest-TF error from IMU", "predicted_nearest_tf_error_imu_deg"),
        ):
            xs, ys = finite_xy(timing_rows, "t", key)
            if xs:
                plt.plot(xs, ys, label=label)
                plotted = True
        return plotted

    plot_or_placeholder(
        output / "scan_motion_vs_shift.png",
        "Observed scan shift versus timing and intra-scan motion estimates",
        "Bag-relative time (s)",
        "Angle (deg)",
        motion_plot,
    )

    window = [r for r in timing_rows if window_start <= float(r["t"]) <= window_end]

    def first_failure_plot():
        plotted = False
        for label, key in (
            ("Observed scan orientation shift", "orientation_shift_deg"),
            ("TF yaw change during scan", "tf_yaw_change_during_scan_deg"),
            ("Predicted IMU scan motion", "predicted_scan_motion_imu_deg"),
            ("Predicted nearest-TF error", "predicted_nearest_tf_error_imu_deg"),
        ):
            xs, ys = finite_xy(window, "t", key)
            if xs:
                plt.plot(xs, ys, label=label)
                plotted = True

        map_rows = [r for r in window if finite_number(r.get("map_odom_yaw_start_deg"))]
        if map_rows:
            baseline = float(map_rows[0]["map_odom_yaw_start_deg"])
            plt.plot(
                [float(r["t"]) for r in map_rows],
                [wrap_deg(float(r["map_odom_yaw_start_deg"]) - baseline) for r in map_rows],
                label="map->odom yaw change",
            )
            plotted = True
        return plotted

    plot_or_placeholder(
        output / f"first_failure_{window_start:g}_{window_end:g}s.png",
        f"First-failure timing window ({window_start:g}-{window_end:g} s)",
        "Bag-relative time (s)",
        "Angle (deg)",
        first_failure_plot,
    )

    def angular_velocity_plot():
        plotted = False
        for label, key in (
            ("IMU gyro z", "imu_gyro_z_rad_s"),
            ("Odom angular z", "odom_angular_z_rad_s"),
            ("cmd_vel angular z", "cmd_vel_angular_z_rad_s"),
            ("cmd_vel_nav angular z", "cmd_vel_nav_angular_z_rad_s"),
        ):
            xs, ys = finite_xy(window, "t", key)
            if xs:
                plt.plot(xs, ys, label=label)
                plotted = True
        return plotted

    plot_or_placeholder(
        output / f"first_failure_angular_velocity_{window_start:g}_{window_end:g}s.png",
        f"Angular velocity in first-failure window ({window_start:g}-{window_end:g} s)",
        "Bag-relative time (s)",
        "Angular velocity (rad/s)",
        angular_velocity_plot,
    )


def command_correlation(data: dict, scan_rows: Sequence[dict], max_gap: float) -> List[dict]:
    scan_times = [r["t"] for r in scan_rows]
    mo_times = [r["t"] for r in data["tf_map_odom"]]
    rows: List[dict] = []
    for key in ("cmd_vel_nav", "cmd_vel"):
        for cmd in data[key]:
            scan = nearest_row(scan_rows, scan_times, cmd["t"], max_gap)
            mo = nearest_row(data["tf_map_odom"], mo_times, cmd["t"], max_gap)
            rows.append({
                **cmd,
                "scan_orientation_shift_deg": scan.get("orientation_shift_deg", float("nan")) if scan else float("nan"),
                "map_odom_yaw_deg": mo["yaw_deg"] if mo else float("nan"),
            })
    rows.sort(key=lambda r: r["t"])
    return rows


def plot_or_placeholder(path: Path, title: str, xlabel: str, ylabel: str, plotter) -> None:
    plt.figure(figsize=(12, 7))
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


def make_plots(data: dict, aligned: Sequence[dict], imu_odom: Sequence[dict], scan_rows: Sequence[dict], output: Path) -> None:
    def yaw_plot():
        plotted = False
        for label, rows in (
            ("IMU yaw", data["imu"]), ("Odom yaw", data["odom"]),
            ("AMCL yaw", data["amcl"]), ("map->odom yaw", data["tf_map_odom"]),
            ("odom->base yaw", data["tf_odom_base"]),
        ):
            if rows:
                plt.plot([r["t"] for r in rows], unwrap_degrees([r["yaw_deg"] for r in rows]), label=label)
                plotted = True
        return plotted
    plot_or_placeholder(output / "yaw_compare.png", "Localization yaw comparison", "Time (s)", "Unwrapped yaw (deg)", yaw_plot)

    def loc_plot():
        if not aligned:
            return False
        plt.plot([r["t"] for r in aligned], [r["position_error_m"] for r in aligned], label="Position error (m)")
        plt.plot([r["t"] for r in aligned], [abs(r["yaw_error_deg"]) for r in aligned], label="Absolute yaw error (deg)")
        return True
    plot_or_placeholder(output / "localization_error.png", "AMCL pose versus TF-composed map->base pose", "Time (s)", "Error", loc_plot)

    def mo_plot():
        rows = data["tf_map_odom"]
        if not rows:
            return False
        plt.plot([r["t"] for r in rows], [r["x"] for r in rows], label="map->odom x (m)")
        plt.plot([r["t"] for r in rows], [r["y"] for r in rows], label="map->odom y (m)")
        plt.plot([r["t"] for r in rows], unwrap_degrees([r["yaw_deg"] for r in rows]), label="map->odom yaw (deg)")
        return True
    plot_or_placeholder(output / "map_odom_correction.png", "AMCL map->odom correction", "Time (s)", "Correction value", mo_plot)

    def io_plot():
        if not imu_odom:
            return False
        plt.plot([r["t"] for r in imu_odom], [r["yaw_difference_deg"] for r in imu_odom], label="Odom yaw - IMU yaw")
        return True
    plot_or_placeholder(output / "imu_odom_alignment.png", "IMU and odometry yaw consistency", "Time (s)", "Yaw difference (deg)", io_plot)

    def cmd_plot():
        plotted = False
        for label, key in (("cmd_vel_nav", "cmd_vel_nav"), ("cmd_vel", "cmd_vel")):
            rows = data[key]
            if rows:
                plt.plot([r["t"] for r in rows], [r["angular_z"] for r in rows], label=f"{label} angular.z")
                plotted = True
        return plotted
    plot_or_placeholder(output / "cmd_vel_timeline.png", "Navigation angular velocity commands", "Time (s)", "angular.z (rad/s)", cmd_plot)

    def scan_plot():
        if not scan_rows:
            return False
        plt.plot([r["t"] for r in scan_rows], [r["orientation_shift_deg"] for r in scan_rows], label="Scan wall-orientation shift")
        return True
    plot_or_placeholder(output / "scan_orientation_shift.png", "LaserScan orientation shift in map frame", "Time (s)", "Shift from initial baseline (deg)", scan_plot)

    def combined_plot():
        plotted = False
        if scan_rows:
            plt.plot([r["t"] for r in scan_rows], [r["orientation_shift_deg"] for r in scan_rows], label="Scan orientation shift (deg)")
            plotted = True
        if data["tf_map_odom"]:
            base = data["tf_map_odom"][0]["yaw_deg"]
            plt.plot([r["t"] for r in data["tf_map_odom"]], [wrap_deg(r["yaw_deg"] - base) for r in data["tf_map_odom"]], label="map->odom yaw change (deg)")
            plotted = True
        for label, key in (("cmd_vel_nav angular.z", "cmd_vel_nav"), ("cmd_vel angular.z", "cmd_vel")):
            if data[key]:
                plt.plot([r["t"] for r in data[key]], [r["angular_z"] for r in data[key]], label=label)
                plotted = True
        return plotted
    plot_or_placeholder(output / "command_vs_scan_shift.png", "Commands, AMCL correction, and scan shift", "Time (s)", "Value", combined_plot)



def build_report(
    data: dict,
    aligned: Sequence[dict],
    imu_odom: Sequence[dict],
    scan_rows: Sequence[dict],
    timing_rows: Sequence[dict],
    events: Sequence[dict],
    base_laser: Optional[dict],
    scan_diag: dict,
    output: Path,
    bag_uri: Path,
    max_tf_gap: float,
    timing_max_bracket_ms: float,
    sensor_match_max_ms: float,
) -> None:
    duration_candidates = [
        r["t"]
        for key in ("odom", "amcl", "imu", "tf_dynamic", "scan", "cmd_vel", "cmd_vel_nav")
        for r in data[key]
    ]
    duration = max(duration_candidates) if duration_candidates else 0.0
    lines = [
        f"# Localization Bag Analysis v{VERSION}", "",
        f"- Bag: `{bag_uri}`", f"- Parsed duration: `{duration:.3f} s`",
        f"- Legacy TF nearest-neighbor tolerance: `{max_tf_gap:.3f} s`",
        f"- Timing TF maximum interpolation bracket: `{timing_max_bracket_ms:.1f} ms`",
        f"- IMU/Odom header match tolerance: `{sensor_match_max_ms:.1f} ms`",
        f"- Laser frame: `{data['laser_frame']}`", "",
        "## Message counts", "",
        f"- Odom: `{len(data['odom'])}`",
        f"- AMCL: `{len(data['amcl'])}`",
        f"- IMU: `{len(data['imu'])}`",
        f"- map->odom TF: `{len(data['tf_map_odom'])}`",
        f"- odom->base samples: `{len(data['tf_odom_base'])}`",
        f"- /cmd_vel_nav: `{len(data['cmd_vel_nav'])}`",
        f"- /cmd_vel: `{len(data['cmd_vel'])}`",
        f"- sampled scans: `{len(data['scan'])}`",
        f"- usable scan-orientation samples: `{len(scan_rows)}`",
        f"- scan timing rows: `{len(timing_rows)}`", "",
        "## Alignment diagnostics", "",
        f"- AMCL/TF aligned samples: `{len(aligned)}`",
        f"- IMU/Odom aligned samples: `{len(imu_odom)}`",
        f"- scan samples missing map->odom: `{scan_diag['missing_map_odom']}`",
        f"- scan samples missing odom->base: `{scan_diag['missing_odom_base']}`",
        f"- scan samples with insufficient line evidence: `{scan_diag['weak_scan']}`",
        f"- base->laser static transform found: `{'yes' if base_laser else 'no'}`", "",
        "## Header-stamp timing summary", "",
        f"- Scan transport delay: `{format_summary(timing_rows, 'scan_transport_delay_ms', 'ms')}`",
        f"- Scan end-to-bag delay: `{format_summary(timing_rows, 'scan_end_to_bag_delay_ms', 'ms')}`",
        f"- Scan to previous odom TF: `{format_summary(timing_rows, 'scan_to_prev_odom_tf_ms', 'ms')}`",
        f"- Next odom TF to scan: `{format_summary(timing_rows, 'next_odom_tf_to_scan_ms', 'ms')}`",
        f"- Odom TF bracket width: `{format_summary(timing_rows, 'odom_tf_bracket_width_ms', 'ms')}`",
        f"- TF yaw change during one scan: `{format_summary(timing_rows, 'tf_yaw_change_during_scan_deg', 'deg')}`",
        f"- IMU-predicted motion during one scan: `{format_summary(timing_rows, 'predicted_scan_motion_imu_deg', 'deg')}`",
        f"- Predicted nearest-TF angular error: `{format_summary(timing_rows, 'predicted_nearest_tf_error_imu_deg', 'deg')}`", "",
        "## Detected events", "",
    ]
    if events:
        for event in events[:100]:
            lines.append(
                f"- t=`{event['t']:.3f}s` **{event['type']}**: "
                f"value `{event['value']:.3f}`, threshold `{event['threshold']:.3f}` "
                f"— {event['detail']}"
            )
    else:
        lines.append("- No configured threshold event detected.")

    lines += ["", "## TF pairs present", ""]
    for pair, count in sorted(data["tf_pair_counts"].items()):
        lines.append(f"- `{pair}`: `{count}`")

    lines += ["", "## Interpretation", ""]
    if not aligned:
        lines.append(
            "- AMCL/TF comparison is empty. Inspect frame names, TF pairs, and "
            "the legacy alignment tolerance; this is not equivalent to zero error."
        )
    if not data["cmd_vel"] and not data["cmd_vel_nav"]:
        lines.append(
            "- No velocity command messages were present. Empty command plots "
            "reflect missing bag data, not zero commands."
        )
    if not scan_rows:
        lines.append(
            "- Scan shift could not be estimated. Common causes are a missing "
            "base->laser static TF, missing dynamic TF, or insufficient straight-wall evidence."
        )
    else:
        max_row = max(scan_rows, key=lambda r: abs(r["orientation_shift_deg"]))
        lines.append(
            f"- Maximum detected scan-orientation shift: "
            f"`{max_row['orientation_shift_deg']:.2f} deg` at t=`{max_row['t']:.3f}s`."
        )
        lines.append(
            "- The wall-orientation signal is diagnostic rather than a full scan-to-map matcher. "
            "Treat it as evidence of a shift, not an exact registration ground truth."
        )
    if timing_rows:
        lines.append(
            "- Compare observed orientation_shift_deg against both "
            "tf_yaw_change_during_scan_deg and predicted_nearest_tf_error_imu_deg. "
            "The first estimates intra-scan motion; the second estimates a timestamp-selection effect."
        )
        lines.append(
            "- A small TF timestamp offset does not rule out motion distortion: a rotating lidar "
            "can accumulate several degrees across one scan even when header/TF timing is correct."
        )

    lines += [
        "", "## Generated files", "",
        "- `report.md`", "- `odom.csv`", "- `amcl.csv`", "- `imu.csv`",
        "- `tf.csv`", "- `aligned_localization.csv`", "- `scan_orientation.csv`",
        "- `scan_tf_timing.csv`", "- `events.csv`", "- `cmd_vel.csv`",
        "- `cmd_vel_nav.csv`", "- `yaw_compare.png`", "- `localization_error.png`",
        "- `map_odom_correction.png`", "- `imu_odom_alignment.png`",
        "- `cmd_vel_timeline.png`", "- `scan_orientation_shift.png`",
        "- `command_vs_scan_shift.png`", "- `timing_offsets.png`",
        "- `scan_motion_vs_shift.png`", "- `first_failure_<start>_<end>s.png`",
        "- `first_failure_angular_velocity_<start>_<end>s.png`",
    ]
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_outputs(
    data: dict,
    aligned: Sequence[dict],
    imu_odom: Sequence[dict],
    scan_rows: Sequence[dict],
    timing_rows: Sequence[dict],
    events: Sequence[dict],
    correlation: Sequence[dict],
    output: Path,
) -> None:
    write_csv(
        output / "odom.csv",
        ["t", "bag_timestamp_ns", "header_timestamp_ns", "transport_delay_ms",
         "stamp_sec", "stamp_nanosec", "frame_id", "child_frame_id", "x", "y",
         "yaw_deg", "linear_x", "linear_y", "angular_z"],
        data["odom"],
    )
    write_csv(
        output / "amcl.csv",
        ["t", "bag_timestamp_ns", "header_timestamp_ns", "transport_delay_ms",
         "stamp_sec", "stamp_nanosec", "frame_id", "x", "y", "yaw_deg",
         "cov_x", "cov_y", "cov_yaw"],
        data["amcl"],
    )
    write_csv(
        output / "imu.csv",
        ["t", "bag_timestamp_ns", "header_timestamp_ns", "transport_delay_ms",
         "stamp_sec", "stamp_nanosec", "frame_id", "yaw_deg", "gyro_z"],
        data["imu"],
    )
    tf_rows = sorted(data["tf_dynamic"] + data["tf_static"], key=lambda r: r["t"])
    write_csv(
        output / "tf.csv",
        ["t", "bag_timestamp_ns", "header_timestamp_ns", "transport_delay_ms",
         "stamp_sec", "stamp_nanosec", "parent", "child", "x", "y", "yaw_deg",
         "source_topic"],
        tf_rows,
    )
    write_csv(
        output / "aligned_localization.csv",
        ["t", "amcl_x", "amcl_y", "amcl_yaw_deg", "tf_map_base_x",
         "tf_map_base_y", "tf_map_base_yaw_deg", "position_error_m",
         "yaw_error_deg", "map_odom_time_gap_s", "odom_base_time_gap_s"],
        aligned,
    )
    write_csv(
        output / "imu_odom_alignment.csv",
        ["t", "imu_yaw_deg", "odom_yaw_deg", "yaw_difference_deg", "time_gap_s"],
        imu_odom,
    )
    write_csv(
        output / "scan_orientation.csv",
        ["scan_index", "t", "bag_timestamp_ns", "header_timestamp_ns",
         "transport_delay_ms", "effective_scan_duration", "wall_orientation_deg",
         "baseline_orientation_deg", "orientation_shift_deg", "segment_count",
         "confidence", "map_laser_x", "map_laser_y", "map_laser_yaw_deg"],
        scan_rows,
    )
    timing_fields = [
        "scan_index", "t", "scan_bag_timestamp_ns", "scan_header_timestamp_ns",
        "scan_transport_delay_ms", "scan_end_to_bag_delay_ms", "scan_time_s",
        "time_increment_s", "beam_count", "effective_scan_duration_s",
        "scan_start_timestamp_ns", "scan_mid_timestamp_ns", "scan_end_timestamp_ns",
        "previous_odom_tf_header_timestamp_ns", "next_odom_tf_header_timestamp_ns",
        "scan_to_prev_odom_tf_ms", "next_odom_tf_to_scan_ms",
        "odom_tf_bracket_width_ms", "nearest_odom_tf_signed_offset_ms",
        "previous_map_odom_tf_header_timestamp_ns", "next_map_odom_tf_header_timestamp_ns",
        "scan_to_prev_map_odom_tf_ms", "next_map_odom_tf_to_scan_ms",
        "map_odom_tf_bracket_width_ms", "nearest_map_odom_tf_signed_offset_ms",
        "odom_tf_start_valid", "odom_tf_mid_valid", "odom_tf_end_valid",
        "map_odom_tf_start_valid", "map_odom_tf_mid_valid",
        "odom_base_yaw_start_deg", "odom_base_yaw_mid_deg", "odom_base_yaw_end_deg",
        "tf_yaw_change_during_scan_deg", "map_odom_yaw_start_deg",
        "map_odom_yaw_mid_deg", "map_odom_yaw_change_during_half_scan_deg",
        "imu_header_gap_ms", "imu_gyro_z_rad_s", "odom_header_gap_ms",
        "odom_angular_z_rad_s", "cmd_vel_angular_z_rad_s",
        "cmd_vel_nav_angular_z_rad_s", "predicted_scan_motion_imu_deg",
        "predicted_scan_motion_odom_deg", "predicted_nearest_tf_error_imu_deg",
        "predicted_nearest_tf_error_odom_deg", "wall_orientation_deg",
        "baseline_orientation_deg", "orientation_shift_deg", "segment_count",
        "confidence",
    ]
    write_csv(output / "scan_tf_timing.csv", timing_fields, timing_rows)
    write_csv(output / "events.csv", ["t", "type", "value", "threshold", "detail"], events)
    write_csv(
        output / "cmd_vel.csv",
        ["t", "bag_timestamp_ns", "topic", "linear_x", "linear_y", "linear_speed", "angular_z"],
        data["cmd_vel"],
    )
    write_csv(
        output / "cmd_vel_nav.csv",
        ["t", "bag_timestamp_ns", "topic", "linear_x", "linear_y", "linear_speed", "angular_z"],
        data["cmd_vel_nav"],
    )
    write_csv(
        output / "command_scan_correlation.csv",
        ["t", "topic", "linear_x", "linear_y", "linear_speed", "angular_z",
         "scan_orientation_shift_deg", "map_odom_yaw_deg"],
        correlation,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze ROS 2 localization, LaserScan timing, and TF timing from rosbag2."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--map-frame", default=DEFAULT_MAP_FRAME)
    parser.add_argument("--odom-frame", default=DEFAULT_ODOM_FRAME)
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument(
        "--laser-frame", default=None,
        help="Override LaserScan frame; otherwise use /scan header.frame_id",
    )
    parser.add_argument(
        "--max-tf-gap", type=float, default=0.35,
        help="Legacy bag-time tolerance used by v4-compatible orientation reconstruction",
    )
    parser.add_argument(
        "--scan-stride", type=int, default=1,
        help="Analyze every Nth scan; timing analysis should normally use 1",
    )
    parser.add_argument("--scan-shift-threshold", type=float, default=3.0)
    parser.add_argument("--scan-baseline-seconds", type=float, default=10.0)
    parser.add_argument("--segment-min-length", type=float, default=0.005)
    parser.add_argument("--segment-max-length", type=float, default=0.12)
    parser.add_argument("--map-odom-rate-threshold", type=float, default=20.0)
    parser.add_argument(
        "--timing-max-bracket-ms", type=float, default=250.0,
        help="Maximum dynamic-TF header-stamp bracket accepted for interpolation",
    )
    parser.add_argument(
        "--sensor-match-max-ms", type=float, default=100.0,
        help="Maximum header-stamp gap when matching IMU/Odom to scan midpoint",
    )
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
        if args.output else bag_uri / "analysis_v5_timing"
    )
    output.mkdir(parents=True, exist_ok=True)
    map_frame = normalize_frame(args.map_frame)
    odom_frame = normalize_frame(args.odom_frame)
    base_frame = normalize_frame(args.base_frame)

    print(f"Localization Bag Analyzer v{VERSION}")
    print(f"Reading bag: {bag_uri}")
    print(f"Frames: {map_frame} -> {odom_frame} -> {base_frame}")

    data = extract_data(
        bag_uri, map_frame, odom_frame, base_frame,
        args.laser_frame, args.scan_stride,
    )
    laser_frame = data["laser_frame"] or normalize_frame(
        args.laser_frame or DEFAULT_LASER_FRAME
    )
    aligned = build_aligned_localization(data, args.max_tf_gap)
    imu_odom = build_imu_odom_alignment(data)
    scan_rows, base_laser, scan_diag = build_scan_orientation(
        data, map_frame, odom_frame, base_frame, laser_frame,
        args.max_tf_gap, args.segment_min_length, args.segment_max_length,
    )
    events = detect_tf_events(data, args.map_odom_rate_threshold)
    events += detect_scan_shift_events(
        scan_rows, args.scan_shift_threshold, args.scan_baseline_seconds
    )
    events.sort(key=lambda r: r["t"])

    timing_rows = build_scan_timing_rows(
        data,
        scan_rows,
        args.timing_max_bracket_ms,
        args.sensor_match_max_ms,
    )
    correlation = command_correlation(data, scan_rows, args.max_tf_gap)

    save_outputs(
        data, aligned, imu_odom, scan_rows, timing_rows, events, correlation, output
    )
    make_plots(data, aligned, imu_odom, scan_rows, output)
    make_timing_plots(
        timing_rows, output, args.timing_window_start, args.timing_window_end
    )
    build_report(
        data, aligned, imu_odom, scan_rows, timing_rows, events,
        base_laser, scan_diag, output, bag_uri, args.max_tf_gap,
        args.timing_max_bracket_ms, args.sensor_match_max_ms,
    )
    print_timing_summary(timing_rows)

    print("Analysis completed.")
    print(f"Output directory: {output}")
    print(f"Report: {output / 'report.md'}")
    print(f"Scan timing CSV: {output / 'scan_tf_timing.csv'}")
    print(f"Timing offsets plot: {output / 'timing_offsets.png'}")
    print(f"Motion comparison plot: {output / 'scan_motion_vs_shift.png'}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
