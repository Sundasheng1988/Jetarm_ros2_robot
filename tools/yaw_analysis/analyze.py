#!/usr/bin/env python3
"""
analyze.py — ROS2 rosbag yaw calibration analysis tool.

Usage
-----
    python3 analyze.py <bag_path> [options]

See ``python3 analyze.py --help`` for full argument list.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

from .parser import BagParser
from .segment import SegmentDetector, compute_segment_metrics
from .report import ReportGenerator
from .plot import PlotGenerator


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    """Build and return the argument parser."""
    parser = argparse.ArgumentParser(
        description="Analyse yaw accuracy from a ROS2 rosbag.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Example:\n"
            "  python3 -m yaw_analysis.analyze ~/ros2_ws/debug_bags/test_bag \\\n"
            "      --target-angle 90 \\\n"
            "      --imu-topic /mobile_base/sensors/imu_data \\\n"
            "      --odom-topic /odom_combined \\\n"
            "      --robotvel-topic /robotvel \\\n"
            "      --cmd-topic /cmd_vel\n"
        ),
    )

    # Positional
    parser.add_argument(
        "bag_path",
        type=str,
        help="Path to the rosbag directory (containing .db3 + metadata.yaml)",
    )

    # Topic arguments
    parser.add_argument(
        "--imu-topic",
        type=str,
        default=None,
        help="IMU topic (default: /mobile_base/sensors/imu_data)",
    )
    parser.add_argument(
        "--odom-topic",
        type=str,
        default=None,
        help="Odometry topic (default: /odom_combined)",
    )
    parser.add_argument(
        "--robotvel-topic",
        type=str,
        default=None,
        help="Robot velocity topic (default: /robotvel)",
    )
    parser.add_argument(
        "--cmd-topic",
        type=str,
        default=None,
        help="Command velocity topic (default: /cmd_vel)",
    )

    # Analysis parameters
    parser.add_argument(
        "--target-angle",
        type=float,
        default=90.0,
        help="Expected rotation angle in degrees (default: 90)",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.02,
        help="Angular velocity threshold (rad/s) for motion detection "
             "(default: 0.02)",
    )
    parser.add_argument(
        "--merge-gap",
        type=float,
        default=0.5,
        help="Max gap (seconds) between same-type commands to merge "
             "(default: 0.5)",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=1.0,
        help="Extra time (seconds) appended per segment for sensor settling "
             "(default: 1.0)",
    )

    # Output
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Output directory for report.md / report.csv / plot.png "
             "(default: same directory as bag_path)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip PNG plot generation",
    )
    parser.add_argument(
        "--show-plot",
        action="store_true",
        help="Also display the plot interactively (requires GUI backend)",
    )

    return parser


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def run(args: Optional[Sequence[str]] = None) -> int:
    """Execute the yaw analysis pipeline.

    Parameters
    ----------
    args : sequence of str, optional
        CLI arguments (defaults to ``sys.argv[1:]``).

    Returns
    -------
    int
        Exit code (0 = success).
    """
    parser = build_parser()
    parsed = parser.parse_args(args)

    bag_path = Path(parsed.bag_path).expanduser().resolve()
    output_dir = Path(parsed.output_dir or bag_path.parent).expanduser().resolve()

    if not bag_path.exists():
        print(f"ERROR: Bag path does not exist: {bag_path}", file=sys.stderr)
        return 1

    if not bag_path.is_dir():
        print(
            f"ERROR: Expected a bag directory, got: {bag_path}",
            file=sys.stderr,
        )
        return 1

    # Default topics
    topic_defaults = {
        "imu": "/mobile_base/sensors/imu_data",
        "odom": "/odom_combined",
        "robotvel": "/robotvel",
        "cmd_vel": "/cmd_vel",
    }

    # Resolve topics: CLI value or default
    imu_topic = parsed.imu_topic or topic_defaults["imu"]
    odom_topic = parsed.odom_topic or topic_defaults["odom"]
    robotvel_topic = parsed.robotvel_topic or topic_defaults["robotvel"]
    cmd_topic = parsed.cmd_topic or topic_defaults["cmd_vel"]

    # ---- Step 1: Parse bag ----
    print(f"\n  Reading bag: {bag_path}")
    print(f"    IMU topic     : {imu_topic}")
    print(f"    Odom topic    : {odom_topic}")
    print(f"    RobotVel topic: {robotvel_topic}")
    print(f"    CmdVel topic  : {cmd_topic}")
    print()

    parser_ = BagParser(
        bag_path,
        imu_topic=imu_topic,
        odom_topic=odom_topic,
        robotvel_topic=robotvel_topic,
        cmd_topic=cmd_topic,
    )

    try:
        bag_data = parser_.parse()
    except Exception as exc:
        print(f"ERROR: Failed to parse bag: {exc}", file=sys.stderr)
        return 1

    if not bag_data.has_data():
        print(
            "ERROR: No data found for any of the requested topics.",
            file=sys.stderr,
        )
        return 1

    print(f"    Messages read: {bag_data.topic_counts}")
    print()

    # ---- Step 2: Detect segments ----
    detector = SegmentDetector(
        threshold=parsed.threshold,
        merge_gap=parsed.merge_gap,
        settle_time=parsed.settle_time,
        target_angle=parsed.target_angle,
    )

    if bag_data.cmd_vel is not None and not bag_data.cmd_vel.empty:
        segments = detector.detect(bag_data.cmd_vel)
        print(f"  Detected {len(segments)} segment(s)")
    else:
        print("  WARNING: No cmd_vel data — cannot auto-detect segments.")
        print("  Creating a single bag-length segment as fallback.")
        from .segment import Segment, SegmentType, TimeWindow
        # Create a single STATIC segment covering the bag duration
        all_t = _collect_timestamps(bag_data)
        if all_t is not None:
            segments = [
                Segment(
                    type=SegmentType.STATIC,
                    window=TimeWindow(float(all_t.min()), float(all_t.max())),
                )
            ]
        else:
            segments = []

    # ---- Step 3: Compute metrics ----
    compute_segment_metrics(segments, bag_data)

    # ---- Step 4: Generate report ----
    report_gen = ReportGenerator(output_dir=str(output_dir))
    report_gen.generate(
        segments,
        bag_data.topic_counts,
        target_angle=parsed.target_angle,
        to_terminal=True,
        to_markdown=True,
        to_csv=True,
    )

    # ---- Step 5: Generate plot ----
    if not parsed.no_plot:
        plot_gen = PlotGenerator(output_dir=str(output_dir))
        png_path = plot_gen.generate(bag_data, segments, show=parsed.show_plot)
        print(f"\n  Plot saved: {png_path}")
    else:
        print("\n  Plot generation skipped (--no-plot).")

    print(f"  Report     : {output_dir / 'report.md'}")
    print(f"  CSV        : {output_dir / 'report.csv'}")
    print()

    return 0


def _collect_timestamps(bag_data) -> Optional["np.ndarray"]:  # noqa: F821
    """Collect all timestamps across all topics as one sorted array."""
    import numpy as np
    ts_list = []
    for key in ("imu", "odom", "robotvel", "cmd_vel"):
        df = getattr(bag_data, key, None)
        if df is not None and not df.empty:
            ts_list.append(df["t"].to_numpy())
    if not ts_list:
        return None
    return np.concatenate(ts_list)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """CLI entry point."""
    sys.exit(run())


if __name__ == "__main__":
    main()
