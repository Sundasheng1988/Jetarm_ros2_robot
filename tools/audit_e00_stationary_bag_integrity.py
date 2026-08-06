#!/usr/bin/env python3
"""E00_static raw-data integrity + yaw source alignment + scan wall-fit auditor.

First version: supports only the E00_static baseline bag (stationary robot,
AMCL/Nav2/SLAM off). It does NOT draw a single root cause; it reports counts,
timing integrity, transport-delay statistics, multi-source yaw alignment, and
odom-frame wall-angle stability, marking outliers against data-derived
median/MAD thresholds (k configurable).

Design rules (per approved plan + corrections):
  - header stamp is the physical-event timeline; bag timestamp is recording
    order + transport delay. /odom_combined and /tf are kept independent.
  - /robotvel has no header -> time_basis=bag_timestamp; alignment_gap reported.
  - /tf is expanded; only the odom_combined->base_footprint edge is matched
    against /odom_combined by TransformStamped.header.stamp.
  - /tf_static is treated as latched static (no rate/transport delay).
  - transport delay = bag_ts - header_ts, output raw; it INCLUDES cross-host
    clock offset (PC vs Jetson). We never subtract a fixed offset; we analyze
    jitter, percentiles, spikes, and drift over time.
  - wall angle uses a 180 deg period (NOT 90 deg / 4-theta). ROI configurable.
    Fit failure -> valid=false + reason; wall fit never blocks raw-data audit.
  - QoS durability enum: 1=TRANSIENT_LOCAL, 2=VOLATILE (recorded, not judged).

Outputs (into --out-dir, default <bag>/analysis/):
  raw_data_audit.csv, yaw_source_alignment.csv, scan_wall_fit.csv,
  experiment_summary.md
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:  # pragma: no cover
    print(f"ERROR: missing dependency: {exc}", file=sys.stderr)
    sys.exit(2)


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #

DURABILITY_LABEL = {0: "SYSTEM_DEFAULT", 1: "TRANSIENT_LOCAL", 2: "VOLATILE"}
RELIABILITY_LABEL = {0: "SYSTEM_DEFAULT", 1: "RELIABLE", 2: "BEST_EFFORT"}


def ns_to_s(ns: int) -> float:
    return ns * 1e-9


def wrap_180_deg(deg: float) -> float:
    """Wrap to (-90, 90] using a 180 deg period (opposite walls equivalent)."""
    return ((deg + 90.0) % 180.0) - 90.0


def wrap_pi_rad(rad: float) -> float:
    """Wrap to (-pi/2, pi/2] using a pi period."""
    return ((rad + math.pi / 2.0) % math.pi) - math.pi / 2.0


def quat_to_yaw_rad(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (rotation about z) from a quaternion, full 2pi range."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def median_mad(vals: List[float]) -> Tuple[float, float, float, float, float]:
    """Return (median, mad, p95, p99, max) for a list of floats. NaN-safe."""
    if not vals:
        return (float("nan"),) * 5
    arr = np.asarray(vals, dtype=float)
    med = float(np.median(arr))
    mad = float(np.median(np.abs(arr - med)))
    p95 = float(np.percentile(arr, 95))
    p99 = float(np.percentile(arr, 99))
    mx = float(np.max(arr))
    return med, mad, p95, p99, mx


def count_outliers(vals: List[float], k: float) -> int:
    if not vals:
        return 0
    arr = np.asarray(vals, dtype=float)
    med = float(np.median(arr))
    mad = float(np.median(np.abs(arr - med)))
    if mad == 0 or math.isnan(mad):
        return 0
    return int(np.sum(np.abs(arr - med) > k * mad))


def parse_qos(topic_meta_qos: str) -> Tuple[Optional[int], Optional[int]]:
    """Best-effort parse of reliability/durability ints from offered_qos_profiles."""
    if not topic_meta_qos:
        return None, None
    rel = None
    dur = None
    for line in topic_meta_qos.splitlines():
        s = line.strip()
        if s.startswith("reliability:"):
            try:
                rel = int(s.split(":", 1)[1].strip())
            except ValueError:
                pass
        elif s.startswith("durability:"):
            try:
                dur = int(s.split(":", 1)[1].strip())
            except ValueError:
                pass
    return rel, dur


def parse_roi(spec: str) -> List[Tuple[float, float]]:
    """Parse ROI like '-10:10,170:190' -> list of (lo, hi) deg ranges."""
    ranges: List[Tuple[float, float]] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" not in part:
            continue
        lo_s, hi_s = part.split(":", 1)
        ranges.append((float(lo_s), float(hi_s)))
    return ranges


# --------------------------------------------------------------------------- #
# Bag loading
# --------------------------------------------------------------------------- #

class BagLoader:
    def __init__(self, bag_uri: Path):
        self.bag_uri = bag_uri
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        self.topic_types = {
            t.name: t.type for t in self.reader.get_all_topics_and_types()
        }
        self.qos_by_topic = {
            t.name: t.offered_qos_profiles for t in self.reader.get_all_topics_and_types()
        }
        self.classes = {name: get_message(tname) for name, tname in self.topic_types.items()}

    def read(self) -> Dict[str, List[Tuple[int, Optional[int], object]]]:
        """Return {topic: [(bag_ts_ns, header_stamp_ns_or_None, msg), ...]}."""
        out: Dict[str, List[Tuple[int, Optional[int], object]]] = {
            t: [] for t in self.topic_types
        }
        while self.reader.has_next():
            topic, raw, bag_ts = self.reader.read_next()
            cls = self.classes.get(topic)
            if cls is None:
                continue
            msg = deserialize_message(raw, cls)
            header_stamp = self._extract_header_stamp(msg)
            out[topic].append((bag_ts, header_stamp, msg))
        return out

    @staticmethod
    def _extract_header_stamp(msg) -> Optional[int]:
        h = getattr(msg, "header", None)
        if h is None:
            return None
        stamp = getattr(h, "stamp", None)
        if stamp is None:
            return None
        ns = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        return ns


# --------------------------------------------------------------------------- #
# Raw-data audit
# --------------------------------------------------------------------------- #

def _content_key_for_msg(msg) -> str:
    """Stable string for duplicate-conflict detection (pose/transform/fields)."""
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        p = msg.pose.pose
        o = p.orientation
        return f"p:{p.position.x:.6f},{p.position.y:.6f},{p.position.z:.6f}|o:{o.x:.6f},{o.y:.6f},{o.z:.6f},{o.w:.6f}"
    if hasattr(msg, "transforms"):
        parts = []
        for tf in msg.transforms:
            parts.append(
                f"{tf.header.frame_id}->{tf.child_frame_id}|"
                f"{tf.transform.translation.x:.6f},{tf.transform.translation.y:.6f},{tf.transform.translation.z:.6f}|"
                f"{tf.transform.rotation.x:.6f},{tf.transform.rotation.y:.6f},{tf.transform.rotation.z:.6f},{tf.transform.rotation.w:.6f}"
            )
        return ";".join(parts)
    if hasattr(msg, "x") and hasattr(msg, "y") and hasattr(msg, "z"):
        return f"rv:{msg.x:.6f},{msg.y:.6f},{msg.z:.6f}"
    return repr(msg)


def audit_topic(
    topic: str,
    records: List[Tuple[int, Optional[int], object]],
    msg_type: str,
    qos_str: str,
    k: float,
) -> Dict[str, object]:
    n = len(records)
    has_header = any(r[1] is not None for r in records)
    bag_ts = [r[0] for r in records]
    header_ts = [r[1] for r in records if r[1] is not None]

    bag_intervals = [ns_to_s(bag_ts[i + 1] - bag_ts[i]) for i in range(len(bag_ts) - 1)]
    b_med, b_mad, b_p95, b_p99, b_max = median_mad(bag_intervals)

    row: Dict[str, object] = {
        "source": topic,
        "msg_type": msg_type,
        "count": n,
        "header_stamp_available": has_header,
        "time_basis": "header_stamp" if has_header else "bag_timestamp",
        "bag_timestamp_monotonic": all(bag_ts[i] <= bag_ts[i + 1] for i in range(len(bag_ts) - 1)),
    }

    # header-stamp stats (only when present)
    if has_header and header_ts:
        h_intervals = [ns_to_s(header_ts[i + 1] - header_ts[i]) for i in range(len(header_ts) - 1)]
        h_med, h_mad, h_p95, h_p99, h_max = median_mad(h_intervals)
        row.update(
            {
                "header_stamp_monotonic": all(
                    header_ts[i] <= header_ts[i + 1] for i in range(len(header_ts) - 1)
                ),
                "duplicate_header_stamps": len(header_ts) - len(set(header_ts)),
                "median_interval_s": h_med,
                "mad_interval_s": h_mad,
                "p95_interval_s": h_p95,
                "p99_interval_s": h_p99,
                "max_gap_s": h_max,
                "outlier_interval_count": count_outliers(h_intervals, k),
            }
        )
        # conflicting duplicates: same header stamp, different content
        by_stamp: Dict[int, set] = {}
        for r in records:
            st = r[1]
            if st is None:
                continue
            by_stamp.setdefault(st, set()).add(_content_key_for_msg(r[2]))
        conflicts = sum(1 for st, keys in by_stamp.items() if len(keys) > 1)
        row["conflicting_duplicate_stamps"] = conflicts
        # transport delay = bag_ts - header_ts (raw; includes cross-host offset)
        delays = [ns_to_s(r[0] - r[1]) for r in records if r[1] is not None]
        d_med, d_mad, d_p95, d_p99, d_max = median_mad(delays)
        d_min = float(np.min(delays)) if delays else float("nan")
        row.update(
            {
                "transport_delay_median_s": d_med,
                "transport_delay_mad_s": d_mad,
                "transport_delay_p95_s": d_p95,
                "transport_delay_p99_s": d_p99,
                "transport_delay_max_s": d_max,
                "transport_delay_min_s": d_min,
                "transport_delay_drift_s": (d_max - d_min) if delays else float("nan"),
                "transport_delay_note": "raw bag_ts-header_ts; includes cross-host clock offset",
            }
        )
    else:
        # /robotvel style: no header -> use bag-timestamp intervals only
        row.update(
            {
                "header_stamp_monotonic": "",
                "duplicate_header_stamps": "",
                "median_interval_s": b_med,
                "mad_interval_s": b_mad,
                "p95_interval_s": b_p95,
                "p99_interval_s": b_p99,
                "max_gap_s": b_max,
                "outlier_interval_count": count_outliers(bag_intervals, k),
                "conflicting_duplicate_stamps": "",
                "transport_delay_median_s": "",
                "transport_delay_mad_s": "",
                "transport_delay_p95_s": "",
                "transport_delay_p99_s": "",
                "transport_delay_max_s": "",
                "transport_delay_min_s": "",
                "transport_delay_drift_s": "",
                "transport_delay_note": "no header; time_basis=bag_timestamp; transport delay N/A",
            }
        )

    rel, dur = parse_qos(qos_str)
    row["qos_reliability"] = rel if rel is not None else ""
    row["qos_reliability_label"] = RELIABILITY_LABEL.get(rel, "") if rel is not None else ""
    row["qos_durability"] = dur if dur is not None else ""
    row["qos_durability_label"] = DURABILITY_LABEL.get(dur, "") if dur is not None else ""
    row["bag_median_interval_s"] = b_med
    row["bag_max_gap_s"] = b_max
    return row


def expand_tf_edges(
    tf_records: List[Tuple[int, Optional[int], object]],
) -> List[Tuple[int, int, str, str, object]]:
    """Expand /tf TFMessages into [(bag_ts, header_ts, parent, child, tf_msg)]."""
    edges: List[Tuple[int, int, str, str, object]] = []
    for bag_ts, hts, msg in tf_records:
        for tf in msg.transforms:
            st = (
                int(tf.header.stamp.sec) * 1_000_000_000 + int(tf.header.stamp.nanosec)
            )
            edges.append((bag_ts, st, tf.header.frame_id, tf.child_frame_id, tf))
    return edges


def audit_tf_edge(
    edge_name: str,
    parent: str,
    child: str,
    edges: List[Tuple[int, int, str, str, object]],
    odom_header_stamps: List[int],
    k: float,
) -> Dict[str, object]:
    sel = [e for e in edges if e[2] == parent and e[3] == child]
    stamps = [e[1] for e in sel]
    bag_ts = [e[0] for e in sel]
    intervals = [ns_to_s(stamps[i + 1] - stamps[i]) for i in range(len(stamps) - 1)]
    med, mad, p95, p99, mx = median_mad(intervals)

    odom_set = set(odom_header_stamps)
    matched = [s for s in stamps if s in odom_set]
    odom_set_tf = set(stamps)
    odom_without_tf = len(odom_set - odom_set_tf)

    # conflicting transforms: same stamp, different translation/rotation
    by_stamp: Dict[int, set] = {}
    for _, st, _, _, tf in sel:
        t = tf.transform.translation
        r = tf.transform.rotation
        key = f"{t.x:.6f},{t.y:.6f},{t.z:.6f}|{r.x:.6f},{r.y:.6f},{r.z:.6f},{r.w:.6f}"
        by_stamp.setdefault(st, set()).add(key)
    conflicts = sum(1 for keys in by_stamp.values() if len(keys) > 1)

    delays = [ns_to_s(bag_ts[i] - stamps[i]) for i in range(len(sel))]
    d_med, d_mad, d_p95, d_p99, d_max = median_mad(delays)
    d_min = float(np.min(delays)) if delays else float("nan")

    return {
        "source": f"tf_edge:{edge_name}",
        "msg_type": "geometry_msgs/TransformStamped",
        "count": len(sel),
        "header_stamp_available": True,
        "time_basis": "header_stamp",
        "bag_timestamp_monotonic": all(bag_ts[i] <= bag_ts[i + 1] for i in range(len(bag_ts) - 1)),
        "header_stamp_monotonic": all(stamps[i] <= stamps[i + 1] for i in range(len(stamps) - 1)),
        "duplicate_header_stamps": len(stamps) - len(set(stamps)),
        "conflicting_duplicate_stamps": conflicts,
        "median_interval_s": med,
        "mad_interval_s": mad,
        "p95_interval_s": p95,
        "p99_interval_s": p99,
        "max_gap_s": mx,
        "outlier_interval_count": count_outliers(intervals, k),
        "transport_delay_median_s": d_med,
        "transport_delay_mad_s": d_mad,
        "transport_delay_p95_s": d_p95,
        "transport_delay_p99_s": d_p99,
        "transport_delay_max_s": d_max,
        "transport_delay_min_s": d_min,
        "transport_delay_drift_s": (d_max - d_min) if delays else float("nan"),
        "transport_delay_note": "raw; includes cross-host clock offset",
        "tf_stamp_match_rate_vs_odom": (len(matched) / len(stamps)) if stamps else float("nan"),
        "odom_stamps_without_this_tf": odom_without_tf,
        "qos_reliability": "",
        "qos_reliability_label": "",
        "qos_durability": "",
        "qos_durability_label": "",
        "bag_median_interval_s": med,
        "bag_max_gap_s": mx,
    }


def audit_tf_static(
    tf_static_records: List[Tuple[int, Optional[int], object]],
) -> Dict[str, object]:
    """Latched static: no rate/transport delay. Check parent/child + conflicts."""
    transforms = []
    for _, _, msg in tf_static_records:
        for tf in msg.transforms:
            transforms.append(tf)
    seen: Dict[str, set] = {}
    for tf in transforms:
        key = f"{tf.header.frame_id}->{tf.child_frame_id}"
        t = tf.transform.translation
        r = tf.transform.rotation
        val = f"{t.x:.6f},{t.y:.6f},{t.z:.6f}|{r.x:.6f},{r.y:.6f},{r.z:.6f},{r.w:.6f}"
        seen.setdefault(key, set()).add(val)
    conflicting = sum(1 for keys in seen.values() if len(keys) > 1)
    edges = ";".join(seen.keys()) if seen else ""
    # pull a representative base_footprint->laser extrinsic if present
    rep = ""
    for tf in transforms:
        if tf.header.frame_id == "base_footprint" and tf.child_frame_id == "laser":
            t = tf.transform.translation
            r = tf.transform.rotation
            rep = f"t=[{t.x:.3f},{t.y:.3f},{t.z:.3f}] r=[{r.x:.3f},{r.y:.3f},{r.z:.3f},{r.w:.3f}]"
    return {
        "source": "tf_static",
        "msg_type": "tf2_msgs/TFMessage",
        "count": len(transforms),
        "header_stamp_available": True,
        "time_basis": "latched_static",
        "edges": edges,
        "conflicting_static_transforms": conflicting,
        "base_footprint_to_laser": rep,
        "note": "latched; no rate/transport-delay computed",
    }


# --------------------------------------------------------------------------- #
# Yaw source alignment
# --------------------------------------------------------------------------- #

def extract_yaw_sources(
    loaded: Dict[str, List[Tuple[int, Optional[int], object]]],
    tf_edges: List[Tuple[int, int, str, str, object]],
) -> Dict[str, List[Tuple[int, float]]]:
    """Return {source: [(stamp_ns, yaw_rad), ...]} on header-stamp timeline
    (robotvel uses bag timestamp and is integrated separately)."""
    src: Dict[str, List[Tuple[int, float]]] = {}

    imu = loaded.get("/mobile_base/sensors/imu_data", [])
    src["imu_orientation"] = []
    for _, hts, msg in imu:
        if hts is None:
            continue
        o = msg.orientation
        src["imu_orientation"].append((hts, quat_to_yaw_rad(o.x, o.y, o.z, o.w)))

    odom = loaded.get("/odom_combined", [])
    src["odom_pose"] = []
    for _, hts, msg in odom:
        if hts is None:
            continue
        o = msg.pose.pose.orientation
        src["odom_pose"].append((hts, quat_to_yaw_rad(o.x, o.y, o.z, o.w)))

    edge = [e for e in tf_edges if e[2] == "odom_combined" and e[3] == "base_footprint"]
    src["tf_odom_to_base"] = []
    for _, st, _, _, tf in edge:
        r = tf.transform.rotation
        src["tf_odom_to_base"].append((st, quat_to_yaw_rad(r.x, r.y, r.z, r.w)))

    # robotvel.z integrated on bag-timestamp timeline
    rv = loaded.get("/robotvel", [])
    src["robotvel_integrated"] = []
    if rv:
        cum = 0.0
        prev_ts = rv[0][0]
        src["robotvel_integrated"].append((prev_ts, 0.0))
        for i in range(1, len(rv)):
            ts = rv[i][0]
            dt = ns_to_s(ts - prev_ts)
            cum += float(rv[i][2].z) * dt
            src["robotvel_integrated"].append((ts, cum))
            prev_ts = ts
    return src


def nearest_value(series: List[Tuple[int, float]], ts_ns: int) -> Tuple[Optional[float], Optional[float]]:
    """Nearest (yaw, gap_s) to ts_ns in series sorted by stamp."""
    if not series:
        return None, None
    stamps = [s for s, _ in series]
    # linear nearest (series small enough)
    best_i = 0
    best_d = abs(stamps[0] - ts_ns)
    for i in range(1, len(stamps)):
        d = abs(stamps[i] - ts_ns)
        if d < best_d:
            best_d = d
            best_i = i
    return series[best_i][1], ns_to_s(best_d)


def align_yaw_sources(
    sources: Dict[str, List[Tuple[int, float]]],
    k: float,
) -> List[Dict[str, object]]:
    ref_name = "imu_orientation"
    ref = sources.get(ref_name, [])
    ref_stamps = [s for s, _ in ref]
    rows: List[Dict[str, object]] = []
    for name, series in sources.items():
        if not series:
            rows.append({"source": name, "n_samples": 0})
            continue
        stamps = [s for s, _ in series]
        yaws = [y for _, y in series]
        t0 = stamps[0]
        t_end = stamps[-1]
        dur_min = ns_to_s(t_end - t0) / 60.0
        yaw0 = yaws[0]
        yaw_end = yaws[-1]
        delta = yaw_end - yaw0
        # residuals vs IMU reference (nearest-stamp pairing)
        residuals_deg: List[float] = []
        gaps_s: List[float] = []
        for s, y in series:
            ry, gap = nearest_value(ref, s)
            if ry is None:
                continue
            residuals_deg.append(math.degrees(wrap_pi_rad(y - ry)))
            if gap is not None:
                gaps_s.append(gap)
        r_med, r_mad, r_p95, r_p99, r_max = median_mad(residuals_deg)
        g_med, _, _, _, _ = median_mad(gaps_s)
        rows.append(
            {
                "source": name,
                "n_samples": len(series),
                "time_basis": "bag_timestamp" if name == "robotvel_integrated" else "header_stamp",
                "t_start_ns": t0,
                "t_end_ns": t_end,
                "duration_min": dur_min,
                "yaw_start_deg": math.degrees(yaw0),
                "yaw_end_deg": math.degrees(yaw_end),
                "yaw_delta_deg": math.degrees(delta),
                "drift_deg_per_min": (math.degrees(delta) / dur_min) if dur_min > 0 else float("nan"),
                "residual_vs_imu_median_deg": r_med,
                "residual_vs_imu_mad_deg": r_mad,
                "residual_vs_imu_p95_deg": r_p95,
                "residual_vs_imu_p99_deg": r_p99,
                "residual_vs_imu_max_deg": r_max,
                "residual_outlier_count": count_outliers(residuals_deg, k),
                "alignment_gap_median_s": g_med,
                "alignment_note": (
                    "time_basis differs from reference; gap = distance to nearest IMU stamp"
                    if name == "robotvel_integrated"
                    else "same header-stamp basis as reference"
                ),
            }
        )
    return rows


# --------------------------------------------------------------------------- #
# Scan wall fit
# --------------------------------------------------------------------------- #

def fit_wall_pca(
    angles_rad: np.ndarray, ranges: np.ndarray, roi: List[Tuple[float, float]]
) -> Tuple[bool, str, Optional[float], Optional[float]]:
    """Fit dominant wall direction by PCA in selected ROI. 180 deg period.

    Returns (valid, reason, wall_angle_laser_rad, residual_rad).
    """
    mask = np.zeros_like(ranges, dtype=bool)
    for lo, hi in roi:
        a = math.radians(lo)
        b = math.radians(hi)
        if lo <= hi:
            mask |= (angles_rad >= a) & (angles_rad <= b) & np.isfinite(ranges) & (ranges > 0.05)
        else:  # wrap-around range
            mask |= (
                ((angles_rad >= a) | (angles_rad <= b)) & np.isfinite(ranges) & (ranges > 0.05)
            )
    pts = np.stack(
        [ranges[mask] * np.cos(angles_rad[mask]), ranges[mask] * np.sin(angles_rad[mask])],
        axis=1,
    )
    if pts.shape[0] < 5:
        return False, f"too_few_roi_points:{int(pts.shape[0])}", None, None
    ctr = pts.mean(axis=0)
    cov = np.cov((pts - ctr).T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    if not np.all(np.isfinite(eigvals)) or eigvals[-1] <= 0:
        return False, "pca_failed", None, None
    axis = eigvecs[:, -1]
    wall_angle = math.atan2(axis[1], axis[0])  # pi-period already (axis sign ambiguous)
    # residual: mean orthogonal distance to fitted line
    normal = np.array([-math.sin(wall_angle), math.cos(wall_angle)])
    resid = float(np.mean(np.abs((pts - ctr) @ normal)))
    return True, "", wrap_pi_rad(wall_angle), resid


def build_odom_yaw_lookup(
    tf_edges: List[Tuple[int, int, str, str, object]],
) -> List[Tuple[int, float]]:
    edge = [e for e in tf_edges if e[2] == "odom_combined" and e[3] == "base_footprint"]
    out: List[Tuple[int, float]] = []
    for _, st, _, _, tf in edge:
        r = tf.transform.rotation
        out.append((st, quat_to_yaw_rad(r.x, r.y, r.z, r.w)))
    out.sort(key=lambda x: x[0])
    return out


def odom_yaw_at(lookup: List[Tuple[int, float]], ts_ns: int, tol_s: float) -> Tuple[Optional[float], bool]:
    if not lookup:
        return None, False
    stamps = [s for s, _ in lookup]
    # binary search nearest
    import bisect
    i = bisect.bisect_left(stamps, ts_ns)
    cand = []
    if i < len(stamps):
        cand.append(i)
    if i > 0:
        cand.append(i - 1)
    best_i = min(cand, key=lambda j: abs(stamps[j] - ts_ns))
    gap = abs(stamps[best_i] - ts_ns)
    if ns_to_s(gap) > tol_s:
        return lookup[best_i][1], False
    return lookup[best_i][1], True


def audit_scans(
    scan_records: List[Tuple[int, Optional[int], object]],
    tf_edges: List[Tuple[int, int, str, str, object]],
    roi: List[Tuple[float, float]],
    tf_tol_s: float,
) -> List[Dict[str, object]]:
    odom_lookup = build_odom_yaw_lookup(tf_edges)
    rows: List[Dict[str, object]] = []
    for idx, (bag_ts, hts, msg) in enumerate(scan_records):
        stamp_ns = hts if hts is not None else bag_ts
        n_beams = len(msg.ranges)
        a_min = float(msg.angle_min)
        a_max = float(msg.angle_max)
        a_inc = float(msg.angle_increment)
        angles = a_min + a_inc * np.arange(n_beams, dtype=float)
        ranges = np.asarray(msg.ranges, dtype=float)
        valid, reason, wall_laser_rad, resid = fit_wall_pca(angles, ranges, roi)

        # midpoint stamp for rigid midpoint TF lookup
        scan_time = float(getattr(msg, "scan_time", 0.0))
        mid_ns = stamp_ns + int((scan_time / 2.0) * 1e9)
        odom_yaw, tf_valid = odom_yaw_at(odom_lookup, mid_ns, tf_tol_s)
        wall_odom_deg = ""
        if valid and odom_yaw is not None:
            wall_odom_deg = math.degrees(wrap_pi_rad(wall_laser_rad + odom_yaw))

        rows.append(
            {
                "scan_idx": idx,
                "header_stamp_ns": stamp_ns,
                "bag_stamp_ns": bag_ts,
                "frame_id": getattr(msg.header, "frame_id", ""),
                "angle_min": a_min,
                "angle_max": a_max,
                "angle_increment": a_inc,
                "scan_time": scan_time,
                "time_increment": float(getattr(msg, "time_increment", 0.0)),
                "n_beams": n_beams,
                "n_valid_roi": int(np.sum(np.isfinite(ranges))),
                "valid": valid,
                "fail_reason": reason,
                "wall_angle_laser_deg": (
                    math.degrees(wall_laser_rad) if wall_laser_rad is not None else ""
                ),
                "wall_fit_residual_rad": resid if resid is not None else "",
                "tf_lookup_valid": tf_valid,
                "wall_angle_odom_deg": wall_odom_deg,
            }
        )
    return rows


# --------------------------------------------------------------------------- #
# Output writers
# --------------------------------------------------------------------------- #

def write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    if not rows:
        path.write_text("")
        return
    # union of keys, preserving first-seen order
    fieldnames: List[str] = []
    seen = set()
    for r in rows:
        for k in r.keys():
            if k not in seen:
                seen.add(k)
                fieldnames.append(k)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fieldnames})


def fmt(v: object, nd: int = 6) -> str:
    if v is None or v == "":
        return ""
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        return f"{v:.{nd}f}"
    return str(v)


def write_summary(
    path: Path,
    bag_uri: Path,
    loader: BagLoader,
    audit_rows: List[Dict[str, object]],
    tf_static_row: Dict[str, object],
    tf_edge_row: Optional[Dict[str, object]],
    yaw_rows: List[Dict[str, object]],
    scan_rows: List[Dict[str, object]],
    args: argparse.Namespace,
    bag_info_counts: Dict[str, int],
) -> None:
    lines: List[str] = []
    lines.append("# E00_static raw-data integrity audit\n")
    lines.append(f"- bag: `{bag_uri}`")
    lines.append(f"- outlier_k: {args.outlier_k}")
    lines.append(f"- roi: {args.roi}")
    lines.append(f"- tf_match_tol_s: {args.tf_match_tol_s}")
    lines.append("")

    lines.append("## 1. Counts vs ros2 bag info\n")
    lines.append("| topic | script_count | bag_info_count | match |")
    lines.append("|---|---|---|---|")
    for r in audit_rows:
        name = str(r["source"])
        bi_topic = r.get("bag_info_topic", name)
        bi = bag_info_counts.get(bi_topic, "")
        sc = r.get("count", "")
        match = "Y" if str(bi) == str(sc) else ("N/A" if bi == "" else "MISMATCH")
        lines.append(f"| {name} | {sc} | {bi} | {match} |")
    lines.append("")

    lines.append("## 2. Raw-data integrity (key fields)\n")
    lines.append(
        "| source | count | basis | header_mono | dup | conflict | median_int_s |"
        " max_gap_s | tport_med_s | tport_p95_s | tport_max_s | outliers |"
    )
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in audit_rows:
        lines.append(
            f"| {r['source']} | {r['count']} | {r['time_basis']} | "
            f"{r.get('header_stamp_monotonic','')} | {r.get('duplicate_header_stamps','')} | "
            f"{r.get('conflicting_duplicate_stamps','')} | {fmt(r.get('median_interval_s',''),6)} | "
            f"{fmt(r.get('max_gap_s',''),6)} | {fmt(r.get('transport_delay_median_s',''),6)} | "
            f"{fmt(r.get('transport_delay_p95_s',''),6)} | {fmt(r.get('transport_delay_max_s',''),6)} | "
            f"{r.get('outlier_interval_count','')} |"
        )
    lines.append("")

    lines.append("## 3. TF edge-level: odom_combined -> base_footprint\n")
    if tf_edge_row:
        lines.append(f"- edge_count: {tf_edge_row['count']}")
        lines.append(
            f"- stamp_match_rate_vs_odom: "
            f"{fmt(tf_edge_row.get('tf_stamp_match_rate_vs_odom'),4)}"
        )
        lines.append(f"- odom_stamps_without_this_tf: {tf_edge_row['odom_stamps_without_this_tf']}")
        lines.append(f"- max_gap_s: {fmt(tf_edge_row.get('max_gap_s'),6)}")
        lines.append(
            f"- conflicting_duplicate_stamps: {tf_edge_row['conflicting_duplicate_stamps']}"
        )
        lines.append(
            "- NOTE: a count below /odom_combined means TF edge messages are fewer; "
            "this is reported as-is and is NOT a declared root cause."
        )
    else:
        lines.append("- no odom_combined->base_footprint edge found in /tf")
    lines.append("")

    lines.append("## 4. Transport delay\n")
    lines.append(
        "- transport delay = bag_timestamp - header_timestamp, output RAW. "
        "It INCLUDES cross-host clock offset (PC vs Jetson); a fixed offset is "
        "NOT subtracted. Focus: jitter (MAD), p95/p99, max, drift (max-min)."
    )
    lines.append("")
    lines.append("| source | med_s | mad_s | p95_s | p99_s | max_s | min_s | drift_s |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for r in audit_rows:
        if r.get("transport_delay_median_s") == "":
            continue
        lines.append(
            f"| {r['source']} | {fmt(r.get('transport_delay_median_s'),6)} | "
            f"{fmt(r.get('transport_delay_mad_s'),6)} | {fmt(r.get('transport_delay_p95_s'),6)} | "
            f"{fmt(r.get('transport_delay_p99_s'),6)} | {fmt(r.get('transport_delay_max_s'),6)} | "
            f"{fmt(r.get('transport_delay_min_s'),6)} | {fmt(r.get('transport_delay_drift_s'),6)} |"
        )
    lines.append("")

    lines.append("## 5. /tf_static (latched)\n")
    lines.append(f"- count: {tf_static_row['count']}")
    lines.append(f"- edges: {tf_static_row['edges']}")
    lines.append(f"- conflicting_static_transforms: {tf_static_row['conflicting_static_transforms']}")
    lines.append(f"- base_footprint->laser: {tf_static_row['base_footprint_to_laser']}")
    lines.append("")

    lines.append("## 6. Yaw source alignment (static; expect ~0 drift)\n")
    lines.append(
        "| source | n | basis | yaw_delta_deg | drift_deg/min | "
        "res_vs_imu_med_deg | res_vs_imu_p95_deg | res_max_deg | outliers | gap_s |"
    )
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for r in yaw_rows:
        if r.get("n_samples", 0) == 0:
            lines.append(f"| {r['source']} | 0 | - | - | - | - | - | - | - | - |")
            continue
        lines.append(
            f"| {r['source']} | {r['n_samples']} | {r['time_basis']} | "
            f"{fmt(r.get('yaw_delta_deg'),4)} | {fmt(r.get('drift_deg_per_min'),4)} | "
            f"{fmt(r.get('residual_vs_imu_median_deg'),4)} | {fmt(r.get('residual_vs_imu_p95_deg'),4)} | "
            f"{fmt(r.get('residual_vs_imu_max_deg'),4)} | {r.get('residual_outlier_count','')} | "
            f"{fmt(r.get('alignment_gap_median_s'),6)} |"
        )
    lines.append("")

    lines.append("## 7. Scan wall fit (180 deg period; secondary, non-blocking)\n")
    valid_scans = [r for r in scan_rows if r["valid"]]
    failed = len(scan_rows) - len(valid_scans)
    lines.append(f"- total_scans: {len(scan_rows)}; valid_fits: {len(valid_scans)}; failed: {failed}")
    if valid_scans:
        odom_angles = [
            float(r["wall_angle_odom_deg"]) for r in valid_scans if r["wall_angle_odom_deg"] != ""
        ]
        laser_angles = [float(r["wall_angle_laser_deg"]) for r in valid_scans]
        om_med, om_mad, om_p95, om_p99, om_max = median_mad(odom_angles)
        la_med, la_mad, la_p95, la_p99, la_max = median_mad(laser_angles)
        lines.append(
            f"- wall_angle_laser_deg: median={fmt(la_med,4)} mad={fmt(la_mad,4)} "
            f"p95={fmt(la_p95,4)} max={fmt(la_max,4)}"
        )
        lines.append(
            f"- wall_angle_odom_deg:   median={fmt(om_med,4)} mad={fmt(om_mad,4)} "
            f"p95={fmt(om_p95,4)} max={fmt(om_max,4)}"
        )
        tf_invalid = sum(1 for r in valid_scans if not r["tf_lookup_valid"])
        lines.append(f"- scans_with_tf_lookup_out_of_tol: {tf_invalid}")
    lines.append("")

    lines.append("## 8. Outliers & data-derived thresholds\n")
    lines.append(
        f"- Outliers flagged where |val - median| > {args.outlier_k} * MAD "
        "(MAD per source). Only flagged, NOT auto-root-cause."
    )
    lines.append("")
    lines.append("## 9. Conclusion\n")
    lines.append(
        "- This report states data integrity and outliers only. "
        "It does NOT identify a single root cause of the yaw failure. "
        "Decide Gate progression from the tables above."
    )
    path.write_text("\n".join(lines) + "\n")


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="E00_static raw-data / yaw / scan auditor")
    ap.add_argument("--bag", required=True, help="bag directory")
    ap.add_argument("--out-dir", default=None, help="output dir (default <bag>/analysis)")
    ap.add_argument("--outlier-k", type=float, default=10.0, help="MAD multiplier for outliers")
    ap.add_argument("--roi", default="-10:10,170:190", help="wall ROI deg ranges, e.g. -10:10,170:190")
    ap.add_argument("--tf-match-tol", dest="tf_match_tol_s", type=float, default=0.1,
                    help="tolerance (s) for TF lookup at scan midpoint")
    ap.add_argument("--no-scan-fit", action="store_true", help="skip scan wall fit")
    args = ap.parse_args(argv)

    bag_uri = Path(args.bag).expanduser().resolve()
    if not bag_uri.is_dir():
        print(f"ERROR: bag dir not found: {bag_uri}", file=sys.stderr)
        return 2
    out_dir = Path(args.out_dir) if args.out_dir else bag_uri / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    loader = BagLoader(bag_uri)
    loaded = loader.read()

    # ros2 bag info counts (from reader metadata) for cross-check
    bag_info_counts = {name: len(recs) for name, recs in loaded.items()}

    # ---- raw data audit (per topic) ----
    audit_rows: List[Dict[str, object]] = []
    for topic in sorted(loaded.keys()):
        msg_type = loader.topic_types.get(topic, "")
        qos_str = loader.qos_by_topic.get(topic, "")
        # /tf_static handled separately
        if topic == "/tf_static":
            continue
        if topic == "/tf":
            # topic-level row for raw /tf TFMessage stream
            row = audit_topic(topic, loaded[topic], msg_type, qos_str, args.outlier_k)
            row["source"] = "/tf (TFMessage stream)"
            row["bag_info_topic"] = "/tf"
            audit_rows.append(row)
            continue
        audit_rows.append(audit_topic(topic, loaded[topic], msg_type, qos_str, args.outlier_k))

    # ---- /tf expansion + edge audit ----
    tf_edges = expand_tf_edges(loaded.get("/tf", []))
    tf_edge_row = audit_tf_edge(
        "odom_combined->base_footprint",
        "odom_combined",
        "base_footprint",
        tf_edges,
        [r[1] for r in loaded.get("/odom_combined", []) if r[1] is not None],
        args.outlier_k,
    )
    audit_rows.append(tf_edge_row)

    # ---- /tf_static ----
    tf_static_row = audit_tf_static(loaded.get("/tf_static", []))

    # ---- yaw alignment ----
    yaw_sources = extract_yaw_sources(loaded, tf_edges)
    yaw_rows = align_yaw_sources(yaw_sources, args.outlier_k)

    # ---- scan wall fit ----
    scan_rows: List[Dict[str, object]] = []
    if not args.no_scan_fit:
        roi = parse_roi(args.roi)
        scan_rows = audit_scans(loaded.get("/scan", []), tf_edges, roi, args.tf_match_tol_s)

    # ---- write outputs ----
    write_csv(out_dir / "raw_data_audit.csv", audit_rows)
    write_csv(out_dir / "yaw_source_alignment.csv", yaw_rows)
    write_csv(out_dir / "scan_wall_fit.csv", scan_rows)
    write_summary(
        out_dir / "experiment_summary.md",
        bag_uri,
        loader,
        audit_rows,
        tf_static_row,
        tf_edge_row,
        yaw_rows,
        scan_rows,
        args,
        bag_info_counts,
    )

    print(f"OK wrote outputs to {out_dir}")
    print(f"   raw_data_audit.csv, yaw_source_alignment.csv, scan_wall_fit.csv, experiment_summary.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
