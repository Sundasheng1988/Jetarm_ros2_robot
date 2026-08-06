"""
Report generation — terminal output, Markdown report, and CSV export.

All formatting is collected into this single module so that changing the
report layout never touches analysis logic.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from .segment import Segment, SegmentType
from .utils import angular_diff_report


# ---------------------------------------------------------------------------
# ReportGenerator
# ---------------------------------------------------------------------------

@dataclass
class ReportGenerator:
    """Generates human-readable and machine-readable yaw analysis reports.

    Parameters
    ----------
    output_dir : str or Path
        Directory where ``report.md`` and ``report.csv`` are written.
    """

    output_dir: str | Path = "."

    # Internal state
    _lines: list[str] = field(default_factory=list, repr=False)
    _cached_segments: list[Segment] = field(default_factory=list, repr=False)

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def generate(
        self,
        segments: list[Segment],
        topic_counts: dict[str, int],
        target_angle: Optional[float] = None,
        *,
        to_terminal: bool = True,
        to_markdown: bool = True,
        to_csv: bool = True,
    ) -> str:
        """Build and optionally output the full report.

        Parameters
        ----------
        segments : list[Segment]
            Analysed segments (must have metrics computed).
        topic_counts : dict[str, int]
            Number of messages per topic.
        target_angle : float, optional
            Expected rotation angle.
        to_terminal : bool
            Print report to stdout.
        to_markdown : bool
            Write ``report.md``.
        to_csv : bool
            Write ``report.csv``.

        Returns
        -------
        str
            Full report text.
        """
        self._lines.clear()
        self._cached_segments = segments

        self._write_header(target_angle)
        self._write_topic_counts(topic_counts)
        self._write_segments_list(segments)
        self._write_conclusion(segments)

        report = "\n".join(self._lines)

        if to_terminal:
            print(report)

        out_path = Path(self.output_dir)
        out_path.mkdir(parents=True, exist_ok=True)

        if to_markdown:
            (out_path / "report.md").write_text(report)

        if to_csv:
            self._write_csv(out_path / "report.csv")

        return report

    # ------------------------------------------------------------------
    # Section builders
    # ------------------------------------------------------------------

    def _write_header(self, target_angle: Optional[float]) -> None:
        self._lines.append("=" * 55)
        self._lines.append("  Yaw Calibration Report")
        self._lines.append("=" * 55)
        self._lines.append("")
        if target_angle is not None:
            self._lines.append(f"  Target Angle : {target_angle:+}°")
            self._lines.append("")

    def _write_topic_counts(self, counts: dict[str, int]) -> None:
        if not counts:
            return
        self._lines.append("-" * 55)
        self._lines.append("  Topic Message Counts")
        self._lines.append("-" * 55)
        for topic, count in counts.items():
            self._lines.append(f"  {topic:<40s} {count}")
        self._lines.append("")

    def _write_segments_list(self, segments: list[Segment]) -> None:
        """Write per-segment detail."""
        rotation_segs = [s for s in segments if s.is_rotation]
        static_segs = [s for s in segments if s.is_static]
        self._lines.append("")
        self._lines.append(
            f"  Detected Segments: {len(rotation_segs)} motion, "
            f"{len(static_segs)} static"
        )
        self._lines.append("")

        for seg in segments:
            if seg.is_rotation:
                self._write_rotation_segment(seg)
            elif seg.is_static:
                self._write_static_segment(seg)

    def _write_rotation_segment(self, seg: Segment) -> None:
        label = f"{seg.name}  ({seg.window.start:.1f}s → {seg.window.end:.1f}s)"
        self._lines.append("")
        self._lines.append(f"  ┌─ {label} {'─' * max(0, 50 - len(label))}")
        self._lines.append("")

        expected = seg.expected_angle
        if expected is not None:
            self._lines.append(f"  │   Expected        : {expected:+8.2f}°")

        self._lines.append(
            angular_diff_report(
                seg.imu_orientation_delta, expected, "IMU Orientation ΔYaw",
            )
        )
        self._lines.append(
            angular_diff_report(
                seg.imu_gyro_integral, expected, "IMU Gyro Integral",
            )
        )
        self._lines.append(
            angular_diff_report(seg.odom_delta, expected, "Odom ΔYaw"),
        )
        self._lines.append(
            angular_diff_report(
                seg.robotvel_integral, expected, "RobotVel Integral",
            )
        )

        # Summary row
        self._lines.append("  │")
        best = self._best_estimate(seg)
        if best is not None and expected is not None:
            self._lines.append(
                f"  │   Best Estimate   : {best:+8.2f}°  "
                f"(err: {best - expected:+7.2f}°)"
            )
        self._lines.append("  └" + "─" * 54)

    def _write_static_segment(self, seg: Segment) -> None:
        label = f"{seg.name}  ({seg.window.start:.1f}s → {seg.window.end:.1f}s)"
        self._lines.append("")
        self._lines.append(f"  ┌─ {label} {'─' * max(0, 50 - len(label))}")
        self._lines.append("")

        if seg.gyro_bias is not None:
            self._lines.append(f"  │   Gyro Bias       : {seg.gyro_bias:+.6f} rad/s")
        if seg.gyro_std is not None:
            self._lines.append(f"  │   Gyro Std        : {seg.gyro_std:.6f} rad/s")
        if seg.orientation_drift is not None:
            self._lines.append(
                f"  │   Orientation Drift: {seg.orientation_drift:+6.3f}°"
            )
        if seg.odom_drift is not None:
            self._lines.append(f"  │   Odom Drift      : {seg.odom_drift:+6.3f}°")
        self._lines.append("  └" + "─" * 54)

    def _write_conclusion(self, segments: list[Segment]) -> None:
        rotation_segs = [s for s in segments if s.is_rotation]
        if not rotation_segs:
            self._lines.append("\n  No rotation segments detected.\n")
            return

        self._lines.append("")
        self._lines.append("-" * 55)
        self._lines.append("  Error Summary  (measured - expected, degrees)")
        self._lines.append("-" * 55)
        self._lines.append(
            f"  {'Segment':<12s} {'IMU Orient':>10s} {'Gyro':>10s} "
            f"{'Odom':>10s} {'RobotVel':>10s}"
        )
        self._lines.append(
            f"  {'-'*12:<12s} {'-'*10:>10s} {'-'*10:>10s} "
            f"{'-'*10:>10s} {'-'*10:>10s}"
        )

        for seg in rotation_segs:
            exp = seg.expected_angle
            vals = {
                "IMU": seg.imu_orientation_delta,
                "Gyro": seg.imu_gyro_integral,
                "Odom": seg.odom_delta,
                "RV": seg.robotvel_integral,
            }
            line = f"  {seg.name:<12s}"
            for v in vals.values():
                if v is not None and exp is not None:
                    line += f" {v - exp:+9.2f}"
                else:
                    line += f" {'N/A':>10s}"
            self._lines.append(line)

        self._lines.append("")
        self._lines.append("=" * 55)
        self._lines.append("  Analysis complete.")
        self._lines.append("=" * 55)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _best_estimate(seg: Segment) -> Optional[float]:
        """Return the most reliable measurement for this segment.

        Heuristic: prefer IMU orientation, fall back to gyro integral,
        then odom, then robotvel.
        """
        if seg.imu_orientation_delta is not None:
            return seg.imu_orientation_delta
        if seg.imu_gyro_integral is not None:
            return seg.imu_gyro_integral
        if seg.odom_delta is not None:
            return seg.odom_delta
        if seg.robotvel_integral is not None:
            return seg.robotvel_integral
        return None

    # ------------------------------------------------------------------
    # CSV export
    # ------------------------------------------------------------------

    def _write_csv(self, path: Path) -> None:
        """Write a machine-readable CSV with one row per segment."""
        fieldnames = [
            "segment",
            "type",
            "start_time",
            "end_time",
            "expected_angle",
            "imu_orientation_delta",
            "imu_orientation_error",
            "imu_gyro_integral",
            "imu_gyro_error",
            "odom_delta",
            "odom_error",
            "robotvel_integral",
            "robotvel_error",
            "gyro_bias",
            "gyro_std",
            "orientation_drift",
            "odom_drift",
        ]

        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for row in _segments_to_csv_rows(self._cached_segments):
                writer.writerow(row)


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------

def _round_opt(val: Optional[float], ndigits: int) -> Optional[float]:
    """Round *val* to *ndigits* or return None."""
    return round(val, ndigits) if val is not None else None


def _sub_opt(val: Optional[float], ref: Optional[float]) -> Optional[float]:
    """Return ``val - ref`` or ``None`` if either is ``None``."""
    if val is not None and ref is not None:
        return round(val - ref, 2)
    return None


def _segments_to_csv_rows(
    segments: list[Segment],
) -> list[dict[str, str | float | None]]:
    """Convert all segments to CSV row dicts."""
    rows: list[dict[str, str | float | None]] = []
    for seg in segments:
        exp = seg.expected_angle
        row: dict[str, str | float | None] = {
            "segment": seg.name,
            "type": str(seg.type),
            "start_time": round(seg.window.start, 3),
            "end_time": round(seg.window.end, 3),
            "expected_angle": exp,
            "imu_orientation_delta": _round_opt(seg.imu_orientation_delta, 2),
            "imu_orientation_error": _sub_opt(seg.imu_orientation_delta, exp),
            "imu_gyro_integral": _round_opt(seg.imu_gyro_integral, 2),
            "imu_gyro_error": _sub_opt(seg.imu_gyro_integral, exp),
            "odom_delta": _round_opt(seg.odom_delta, 2),
            "odom_error": _sub_opt(seg.odom_delta, exp),
            "robotvel_integral": _round_opt(seg.robotvel_integral, 2),
            "robotvel_error": _sub_opt(seg.robotvel_integral, exp),
            "gyro_bias": _round_opt(seg.gyro_bias, 6),
            "gyro_std": _round_opt(seg.gyro_std, 6),
            "orientation_drift": _round_opt(seg.orientation_drift, 3),
            "odom_drift": _round_opt(seg.odom_drift, 3),
        }
        rows.append(row)
    return rows
