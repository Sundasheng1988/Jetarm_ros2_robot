#!/usr/bin/env python3
"""
Post-process existing v6 frame-isolation CSV + raw v5 tf.csv to add:
  - map->odom time semantics (transform_tolerance-corrected source-scan time)
  - /tf odom->base age/bracket per scan (raw /tf source, not /odom_combined)
  - runtime /tf odom->base header-stamp gap report for 15-30s
No rosbag rerun. Reads only existing CSVs.
"""
import csv
import statistics
import bisect
from pathlib import Path

TRANSFORM_TOLERANCE_S = 3.0   # verified active amcl.transform_tolerance (nav2_params.yaml)
SCAN_WINDOW = (20.5, 24.5)
GAP_WINDOW = (15.0, 30.0)
GAP_THRESHOLDS_MS = (100.0, 250.0)
MO_CORRECTION_THRESHOLD_DEG = 1.0
BASELINE_S = 10.0

V6_DIR = Path("test_logs/nav2_baseline_v0/b_to_a_yaw_drift_02/analysis_v6_frame_isolation")
V5_TF = Path("test_logs/nav2_baseline_v0/b_to_a_yaw_drift_02/analysis_v5_timing/tf.csv")
V6_SCAN = V6_DIR / "scan_frame_isolation.csv"

# crossings from v6 event_ordering.csv (bag-relative seconds)
CROSSINGS = {
    "first_odom_single": 22.449564929,
    "first_odom_sustained": 22.449612187,
    "first_map_sustained": 23.873388999,
}


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fnum(x):
    try:
        import math
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False


def wrap_deg(v):
    return (float(v) + 180.0) % 360.0 - 180.0


def dedup_by_header(rows):
    seen = set()
    out = []
    for r in sorted(rows, key=lambda r: (int(r["header_timestamp_ns"]), float(r["t"]))):
        ns = int(r["header_timestamp_ns"])
        if ns <= 0 or ns in seen:
            continue
        seen.add(ns)
        out.append(r)
    return out


def main():
    tf_rows = read_csv(V5_TF)
    scan_rows = read_csv(V6_SCAN)

    # bag_start_ns from first scan row: scan_bag_timestamp_ns - round(t*1e9)
    s0 = scan_rows[0]
    bag_start_ns = int(s0["scan_bag_timestamp_ns"]) - round(float(s0["t"]) * 1e9)

    tf_ob = dedup_by_header([r for r in tf_rows
                             if r["parent"] == "odom_combined" and r["child"] == "base_footprint"
                             and r["source_topic"] == "/tf"])
    tf_ob.sort(key=lambda r: int(r["header_timestamp_ns"]))
    ob_hdr = [int(r["header_timestamp_ns"]) for r in tf_ob]

    tf_mo = dedup_by_header([r for r in tf_rows
                             if r["parent"] == "map" and r["child"] == "odom_combined"
                             and r["source_topic"] == "/tf"])
    tf_mo.sort(key=lambda r: int(r["header_timestamp_ns"]))

    # ---- Step 2: map->odom time semantics ----
    mo_baseline_vals = [float(r["yaw_deg"]) for r in tf_mo if float(r["t"]) <= BASELINE_S]
    mo_baseline = statistics.median(mo_baseline_vals) if mo_baseline_vals else float(tf_mo[0]["yaw_deg"])

    mo_semantics = []
    prev_yaw = None
    for r in tf_mo:
        bag_t = float(r["t"])
        hdr_ns = int(r["header_timestamp_ns"])
        transport_ms = float(r["transport_delay_ms"])
        hdr_bag_s = bag_t - transport_ms / 1000.0          # tf header time, bag-relative
        est_src_scan_s = hdr_bag_s - TRANSFORM_TOLERANCE_S  # physical source-scan time, bag-relative
        yaw = float(r["yaw_deg"])
        step = wrap_deg(yaw - prev_yaw) if prev_yaw is not None else 0.0
        cum = wrap_deg(yaw - mo_baseline)
        mo_semantics.append({
            "bag_record_time_s": bag_t,
            "tf_header_time_bag_rel_s": hdr_bag_s,
            "configured_transform_tolerance_s": TRANSFORM_TOLERANCE_S,
            "estimated_source_scan_time_s": est_src_scan_s,
            "bag_time_minus_estimated_source_scan_time_s": bag_t - est_src_scan_s,
            "yaw_deg": yaw,
            "step_yaw_change_deg": step,
            "cumulative_yaw_change_deg": cum,
        })
        prev_yaw = yaw

    # first >= threshold crossing by bag time and by estimated source scan time
    def first_cross(key):
        for r in mo_semantics:
            if abs(r["cumulative_yaw_change_deg"]) >= MO_CORRECTION_THRESHOLD_DEG:
                return r
        return None
    mo_first_bag = first_cross("bag")
    # for source-scan-time ordering, sort by estimated_source_scan_time and find first crossing
    mo_by_src = sorted(mo_semantics, key=lambda r: r["estimated_source_scan_time_s"])
    mo_first_src = next((r for r in mo_by_src if abs(r["cumulative_yaw_change_deg"]) >= MO_CORRECTION_THRESHOLD_DEG), None)

    # ---- Step 3: 20.5-24.5s scan table ----
    def nearest_ob_age_and_bracket(scan_mid_hdr_ns):
        idx = bisect.bisect_left(ob_hdr, scan_mid_hdr_ns)
        prev_ns = ob_hdr[idx - 1] if idx > 0 else None
        next_ns = ob_hdr[idx] if idx < len(ob_hdr) else None
        ages = []
        if prev_ns is not None:
            ages.append(abs(scan_mid_hdr_ns - prev_ns))
        if next_ns is not None:
            ages.append(abs(scan_mid_hdr_ns - next_ns))
        age_ms = min(ages) / 1e6 if ages else float("nan")
        width_ms = (next_ns - prev_ns) / 1e6 if (prev_ns is not None and next_ns is not None) else float("nan")
        return age_ms, width_ms

    # mo aligned by estimated_source_scan_time to scan_mid_hdr (absolute ns)
    mo_src_ns = [int(r["header_timestamp_ns"]) - int(round(TRANSFORM_TOLERANCE_S * 1e9)) for r in tf_mo]
    mo_src_ns_sorted = sorted(range(len(tf_mo)), key=lambda i: mo_src_ns[i])
    mo_src_keys = [mo_src_ns[i] for i in mo_src_ns_sorted]

    def nearest_mo_step_cum(scan_mid_hdr_ns):
        idx = bisect.bisect_left(mo_src_keys, scan_mid_hdr_ns)
        cands = []
        if idx < len(mo_src_keys):
            cands.append(mo_src_ns_sorted[idx])
        if idx > 0:
            cands.append(mo_src_ns_sorted[idx - 1])
        if not cands:
            return float("nan"), float("nan")
        best = min(cands, key=lambda i: abs(mo_src_ns[i] - scan_mid_hdr_ns))
        return mo_semantics[best]["step_yaw_change_deg"], mo_semantics[best]["cumulative_yaw_change_deg"]

    window_rows = []
    for s in scan_rows:
        t = float(s["t"])
        if not (SCAN_WINDOW[0] <= t <= SCAN_WINDOW[1]):
            continue
        scan_mid_hdr_ns = int(s["scan_mid_timestamp_ns"])
        age_ms, width_ms = nearest_ob_age_and_bracket(scan_mid_hdr_ns)
        step, cum = nearest_mo_step_cum(scan_mid_hdr_ns)
        window_rows.append({
            "t": t,
            "orientation_shift_odom_deg": float(s["orientation_shift_odom_deg"]) if fnum(s.get("orientation_shift_odom_deg")) else float("nan"),
            "orientation_shift_map_deg": float(s["orientation_shift_map_deg"]) if fnum(s.get("orientation_shift_map_deg")) else float("nan"),
            "confidence_odom": float(s["confidence_odom"]) if fnum(s.get("confidence_odom")) else float("nan"),
            "map_odom_step_change_deg": step,
            "map_odom_cumulative_change_deg": cum,
            "imu_gyro_z_rad_s": float(s["imu_gyro_z_rad_s"]) if fnum(s.get("imu_gyro_z_rad_s")) else float("nan"),
            "predicted_scan_motion_imu_deg": float(s["predicted_scan_motion_imu_deg"]) if fnum(s.get("predicted_scan_motion_imu_deg")) else float("nan"),
            "tf_yaw_change_during_scan_deg": float(s["tf_yaw_change_during_scan_deg"]) if fnum(s.get("tf_yaw_change_during_scan_deg")) else float("nan"),
            "nearest_tf_odom_base_age_ms": age_ms,
            "tf_odom_base_bracket_width_ms": width_ms,
            "cmd_vel_nav_angular_z_rad_s": float(s["cmd_vel_nav_angular_z_rad_s"]) if fnum(s.get("cmd_vel_nav_angular_z_rad_s")) else float("nan"),
        })

    # ---- Step 4: /tf odom->base header-stamp gaps in 15-30s ----
    gaps = []
    for i in range(1, len(tf_ob)):
        prev = tf_ob[i - 1]
        cur = tf_ob[i]
        gap_ms = (int(cur["header_timestamp_ns"]) - int(prev["header_timestamp_ns"])) / 1e6
        if gap_ms <= GAP_THRESHOLDS_MS[0]:
            continue
        # bag-time span of the gap
        prev_bag = float(prev["t"])
        cur_bag = float(cur["t"])
        # only report if gap intersects [15,30] by bag time
        if cur_bag < GAP_WINDOW[0] or prev_bag > GAP_WINDOW[1]:
            continue
        overlaps = []
        for name, tc in CROSSINGS.items():
            if prev_bag <= tc <= cur_bag:
                overlaps.append(name)
        gaps.append({
            "gap_index": i,
            "prev_bag_time_s": prev_bag,
            "next_bag_time_s": cur_bag,
            "header_stamp_gap_ms": gap_ms,
            "gt_100ms": gap_ms > GAP_THRESHOLDS_MS[0],
            "gt_250ms": gap_ms > GAP_THRESHOLDS_MS[1],
            "overlaps_crossings": ";".join(overlaps) if overlaps else "",
        })

    # ---- write CSVs ----
    def write(path, fields, rows):
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

    mo_fields = ["bag_record_time_s", "tf_header_time_bag_rel_s", "configured_transform_tolerance_s",
                 "estimated_source_scan_time_s", "bag_time_minus_estimated_source_scan_time_s",
                 "yaw_deg", "step_yaw_change_deg", "cumulative_yaw_change_deg"]
    write(V6_DIR / "map_odom_time_semantics.csv", mo_fields, mo_semantics)

    win_fields = ["t", "orientation_shift_odom_deg", "orientation_shift_map_deg", "confidence_odom",
                  "map_odom_step_change_deg", "map_odom_cumulative_change_deg", "imu_gyro_z_rad_s",
                  "predicted_scan_motion_imu_deg", "tf_yaw_change_during_scan_deg",
                  "nearest_tf_odom_base_age_ms", "tf_odom_base_bracket_width_ms",
                  "cmd_vel_nav_angular_z_rad_s"]
    write(V6_DIR / "first_failure_window_20_5_24_5s.csv", win_fields, window_rows)

    gap_fields = ["gap_index", "prev_bag_time_s", "next_bag_time_s", "header_stamp_gap_ms",
                  "gt_100ms", "gt_250ms", "overlaps_crossings"]
    write(V6_DIR / "tf_odom_base_gaps_15_30s.csv", gap_fields, gaps)

    # ---- print ----
    print("=" * 100)
    print("STEP 2: map->odom time semantics (transform_tolerance = 3.0s)")
    print("=" * 100)
    print(f"mo baseline (median yaw, t<=10s) = {mo_baseline:.4f} deg")
    print(f"raw /tf map->odom samples in 15-30s (bag time):")
    print(f"{'bag_t':>7} {'tf_hdr':>7} {'est_src':>7} {'bag-est':>7} {'yaw':>8} {'step':>7} {'cum':>7}")
    for r in mo_semantics:
        if not (15.0 <= r["bag_record_time_s"] <= 30.0):
            continue
        print(f"{r['bag_record_time_s']:7.3f} {r['tf_header_time_bag_rel_s']:7.3f} "
              f"{r['estimated_source_scan_time_s']:7.3f} {r['bag_time_minus_estimated_source_scan_time_s']:7.3f} "
              f"{r['yaw_deg']:8.3f} {r['step_yaw_change_deg']:7.3f} {r['cumulative_yaw_change_deg']:7.3f}")
    print("\nFirst >=1.0deg cumulative correction:")
    if mo_first_bag:
        print(f"  by bag publication time      : bag_t={mo_first_bag['bag_record_time_s']:.3f}s  "
              f"est_src_scan={mo_first_bag['estimated_source_scan_time_s']:.3f}s  cum={mo_first_bag['cumulative_yaw_change_deg']:.3f}")
    if mo_first_src:
        print(f"  by estimated source scan time: bag_t={mo_first_src['bag_record_time_s']:.3f}s  "
              f"est_src_scan={mo_first_src['estimated_source_scan_time_s']:.3f}s  cum={mo_first_src['cumulative_yaw_change_deg']:.3f}")

    print()
    print("=" * 100)
    print("STEP 3: first-failure window 20.5-24.5s (every usable scan)")
    print("=" * 100)
    print(f"{'t':>6} {'sod':>7} {'sma':>7} {'conf_o':>7} {'mo_step':>7} {'mo_cum':>7} {'gyro_z':>7} "
          f"{'pred_imu':>8} {'tf_d_yaw':>8} {'ob_age':>7} {'ob_brkt':>7} {'cmd_nav':>7}")
    for r in window_rows:
        print(f"{r['t']:6.2f} {r['orientation_shift_odom_deg']:7.2f} {r['orientation_shift_map_deg']:7.2f} "
              f"{r['confidence_odom']:7.1f} {r['map_odom_step_change_deg']:7.3f} {r['map_odom_cumulative_change_deg']:7.3f} "
              f"{r['imu_gyro_z_rad_s']:7.3f} {r['predicted_scan_motion_imu_deg']:8.3f} {r['tf_yaw_change_during_scan_deg']:8.3f} "
              f"{r['nearest_tf_odom_base_age_ms']:7.1f} {r['tf_odom_base_bracket_width_ms']:7.1f} {r['cmd_vel_nav_angular_z_rad_s']:7.2f}")

    print()
    print("=" * 100)
    print("STEP 4: /tf odom_combined->base_footprint header-stamp gaps in 15-30s")
    print("=" * 100)
    n100 = sum(1 for g in gaps if g["gt_100ms"])
    n250 = sum(1 for g in gaps if g["gt_250ms"])
    print(f"gaps >100ms: {n100}   gaps >250ms: {n250}")
    print(f"{'idx':>4} {'prev_bag':>8} {'next_bag':>8} {'gap_ms':>9} {'>250':>5}  overlaps")
    for g in gaps:
        print(f"{g['gap_index']:4d} {g['prev_bag_time_s']:8.3f} {g['next_bag_time_s']:8.3f} "
              f"{g['header_stamp_gap_ms']:9.1f} {str(g['gt_250ms']):>5}  {g['overlaps_crossings'] or '-'}")

    print()
    print("Crossing times (bag): " + ", ".join(f"{k}={v:.3f}s" for k, v in CROSSINGS.items()))


if __name__ == "__main__":
    main()
