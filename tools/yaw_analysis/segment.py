"""
Segment detection — automatically identify CW, CCW, and Static segments
from ``/cmd_vel`` angular velocity commands.

Rules
-----
- ``abs(angular.z) <= threshold`` → STATIC
- ``angular.z > threshold``       → CCW  (counter-clockwise, positive yaw)
- ``angular.z < -threshold``      → CW   (clockwise, negative yaw)

Continuous same-type commands are merged into a single segment.  Short
gaps (``merge_gap`` seconds or less) between same-type segments are
merged across the gap.  After each command transition, a ``settle_time``
period is appended to the segment so sensor readings settle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np
import pandas as pd

from .utils import yaw_series_delta


# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

class SegmentType(Enum):
    """Type of a detected motion segment."""

    STATIC = "STATIC"
    CW = "CW"
    CCW = "CCW"

    def __str__(self) -> str:
        return self.value

    @property
    def short_label(self) -> str:
        """Compact label used in tables."""
        return {"STATIC": "St", "CW": "CW", "CCW": "CCW"}[self.value]


@dataclass
class TimeWindow:
    """Start and end of a time interval in UNIX seconds."""

    start: float
    end: float

    @property
    def duration(self) -> float:
        return self.end - self.start

    def contains(self, t: float) -> bool:
        return self.start <= t <= self.end


@dataclass
class Segment:
    """A single detected motion segment (CW, CCW, or STATIC)."""

    type: SegmentType
    window: TimeWindow
    index: int = 0

    # --- Computed metrics (filled by Analyzer) ---
    imu_orientation_delta: Optional[float] = None
    imu_gyro_integral: Optional[float] = None
    odom_delta: Optional[float] = None
    robotvel_integral: Optional[float] = None

    imu_orientation_initial: Optional[float] = None
    imu_orientation_final: Optional[float] = None
    odom_initial: Optional[float] = None
    odom_final: Optional[float] = None

    # --- Static-segment statistics ---
    gyro_bias: Optional[float] = None
    gyro_std: Optional[float] = None
    orientation_drift: Optional[float] = None
    odom_drift: Optional[float] = None

    # --- Expected angle for rotation segments ---
    expected_angle: Optional[float] = None

    @property
    def name(self) -> str:
        """Human-readable name e.g. ``CW1``, ``STATIC_2``."""
        return f"{self.type}_{self.index}"

    @property
    def is_rotation(self) -> bool:
        return self.type in (SegmentType.CW, SegmentType.CCW)

    @property
    def is_static(self) -> bool:
        return self.type == SegmentType.STATIC


# ---------------------------------------------------------------------------
# SegmentDetector
# ---------------------------------------------------------------------------

@dataclass
class SegmentDetector:
    """Detect motion segments from command-velocity data.

    Parameters
    ----------
    threshold : float
        Absolute angular velocity threshold (rad/s).  Values **at or below**
        this threshold are considered static.  (default 0.02)
    merge_gap : float
        Maximum gap (seconds) between same-type command windows to merge
        them into a single segment.  (default 0.5)
    settle_time : float
        Additional time (seconds) appended after each command transition
        so that sensor readings settle into the new state.  (default 1.0)
    target_angle : float, optional
        Expected rotation angle in degrees (e.g. 180).  When set, CW
        segments are assigned ``expected_angle = -target_angle`` and CCW
        segments ``+target_angle``.
    """

    threshold: float = 0.02
    merge_gap: float = 0.5
    settle_time: float = 1.0
    target_angle: Optional[float] = None

    def detect(self, cmd_vel: pd.DataFrame) -> list[Segment]:
        """Run detection on command velocity data.

        Parameters
        ----------
        cmd_vel : pd.DataFrame
            Must contain columns ``t`` and ``angular_z``.

        Returns
        -------
        list[Segment]
            Detected segments in chronological order.
        """
        if cmd_vel.empty:
            return []

        # 1. Classify each command tick
        types = self._classify(cmd_vel["angular_z"].to_numpy())

        # 2. Find contiguous runs → raw intervals
        raw_intervals = self._find_raw_intervals(cmd_vel["t"].to_numpy(), types)

        # 3. Merge same-type intervals that are close enough
        merged = self._merge_same_type(raw_intervals)

        # 4. Build Segment objects with settle padding
        segments = self._build_segments(merged, cmd_vel)

        # 5. Index segments in display order
        counters: dict[str, int] = {}
        for seg in segments:
            key = seg.type.value
            counters[key] = counters.get(key, 0) + 1
            seg.index = counters[key]

        return segments

    # ------------------------------------------------------------------
    # Internal steps
    # ------------------------------------------------------------------

    def _classify(self, angular_z: np.ndarray) -> np.ndarray:
        """Return an array of :class:`SegmentType` for each tick."""
        abs_z = np.abs(angular_z)
        types = np.empty(len(angular_z), dtype=object)
        types[abs_z <= self.threshold] = SegmentType.STATIC
        types[angular_z > self.threshold] = SegmentType.CCW
        types[angular_z < -self.threshold] = SegmentType.CW
        return types

    @staticmethod
    def _find_raw_intervals(
        timestamps: np.ndarray,
        types: np.ndarray,
    ) -> list[tuple[SegmentType, float, float]]:
        """Find contiguous runs of the same segment type.

        Returns
        -------
        list of (type, start_t, end_t)
        """
        if len(timestamps) == 0:
            return []

        intervals: list[tuple[SegmentType, float, float]] = []
        cur_type = types[0]
        cur_start = timestamps[0]

        for i in range(1, len(timestamps)):
            if types[i] != cur_type:
                intervals.append((cur_type, cur_start, timestamps[i - 1]))
                cur_type = types[i]
                cur_start = timestamps[i]

        # Last interval
        intervals.append((cur_type, cur_start, timestamps[-1]))
        return intervals

    def _merge_same_type(
        self,
        intervals: list[tuple[SegmentType, float, float]],
    ) -> list[tuple[SegmentType, float, float]]:
        """Merge consecutive same-type intervals within ``merge_gap``."""
        if not intervals:
            return []

        merged: list[tuple[SegmentType, float, float]] = [intervals[0]]

        for i in range(1, len(intervals)):
            prev_type, prev_start, prev_end = merged[-1]
            cur_type, cur_start, cur_end = intervals[i]

            gap = cur_start - prev_end

            if cur_type == prev_type and gap <= 0.0:
                # Overlapping or abutting — merge
                merged[-1] = (prev_type, min(prev_start, cur_start),
                              max(prev_end, cur_end))
            elif cur_type == prev_type and 0.0 < gap <= 1.0:
                # Short gap — merge if merge_gap allows
                merged[-1] = (prev_type, min(prev_start, cur_start),
                              max(prev_end, cur_end))
            else:
                merged.append((cur_type, cur_start, cur_end))

        return merged

    def _build_segments(
        self,
        intervals: list[tuple[SegmentType, float, float]],
        cmd_vel: pd.DataFrame,
    ) -> list[Segment]:
        """Convert merged intervals into :class:`Segment` objects.

        The segment window is expanded by ``settle_time`` on each side
        (within the bag's time bounds) to allow sensor settling.
        """
        t_min = cmd_vel["t"].min()
        t_max = cmd_vel["t"].max()
        segments: list[Segment] = []

        for seg_type, start, end in intervals:
            pad = self.settle_time
            win_start = max(t_min, start - pad)
            win_end = min(t_max, end + pad)

            seg = Segment(
                type=seg_type,
                window=TimeWindow(win_start, win_end),
            )

            # Expected angle
            if seg_type == SegmentType.CW and self.target_angle is not None:
                seg.expected_angle = -abs(self.target_angle)
            elif seg_type == SegmentType.CCW and self.target_angle is not None:
                seg.expected_angle = abs(self.target_angle)
            elif seg_type == SegmentType.STATIC:
                seg.expected_angle = 0.0

            segments.append(seg)

        return segments


# ---------------------------------------------------------------------------
# Metrics computer
# ---------------------------------------------------------------------------

def compute_segment_metrics(
    segments: list[Segment],
    bag_data: "BagData",  # noqa: F821 — forward ref; imported at runtime
) -> None:
    """Populate each segment's computed metrics from parsed bag data.

    This function mutates the segment objects in-place.

    Parameters
    ----------
    segments : list[Segment]
        Segments to annotate (from :meth:`SegmentDetector.detect`).
    bag_data : BagData
        Parsed bag data (from :class:`BagParser.parse`).
    """
    for seg in segments:
        _compute_rotation_metrics(seg, bag_data)
        _compute_static_metrics(seg, bag_data)


def _slice_df(
    df: Optional[pd.DataFrame],
    window: TimeWindow,
) -> Optional[pd.DataFrame]:
    """Return rows of *df* that fall within *window*, or ``None``."""
    if df is None or df.empty:
        return None
    mask = (df["t"] >= window.start) & (df["t"] <= window.end)
    sliced = df.loc[mask]
    return sliced if not sliced.empty else None


def _compute_rotation_metrics(seg: Segment, bag_data: "BagData") -> None:  # noqa: F821
    """Compute yaw deltas for IMU, odom, gyro integral, robotvel integral."""
    if not seg.is_rotation:
        return

    # --- IMU orientation (cumulative step-wise delta, robust to ±180° wrap) ---
    imu_df = _slice_df(bag_data.imu, seg.window)
    if imu_df is not None and len(imu_df) > 1:
        yaw_vals = imu_df["yaw"].to_numpy()
        seg.imu_orientation_initial = float(yaw_vals[0])
        seg.imu_orientation_final = float(yaw_vals[-1])
        seg.imu_orientation_delta = yaw_series_delta(yaw_vals)

        # Gyro integral delta inside this window
        # (integral is cumulative from bag start, so subtract the initial value)
        if "gyro_integral" in imu_df.columns:
            seg.imu_gyro_integral = float(
                imu_df["gyro_integral"].iloc[-1]
                - imu_df["gyro_integral"].iloc[0]
            )

    # --- Odom (cumulative step-wise delta, robust to ±180° wrap) ---
    odom_df = _slice_df(bag_data.odom, seg.window)
    if odom_df is not None and len(odom_df) > 1:
        yaw_vals = odom_df["yaw"].to_numpy()
        seg.odom_initial = float(yaw_vals[0])
        seg.odom_final = float(yaw_vals[-1])
        seg.odom_delta = yaw_series_delta(yaw_vals)

    # --- RobotVel integral (delta within window) ---
    vel_df = _slice_df(bag_data.robotvel, seg.window)
    if vel_df is not None and "vel_integral" in vel_df.columns:
        seg.robotvel_integral = float(
            vel_df["vel_integral"].iloc[-1]
            - vel_df["vel_integral"].iloc[0]
        )


def _compute_static_metrics(seg: Segment, bag_data: "BagData") -> None:  # noqa: F821
    """Compute static-segment statistics (bias, drift, etc.)."""
    if not seg.is_static:
        return

    imu_df = _slice_df(bag_data.imu, seg.window)
    if imu_df is not None and len(imu_df) > 1:
        gyro = imu_df["gyro_z"].to_numpy()
        seg.gyro_bias = float(np.mean(gyro))
        seg.gyro_std = float(np.std(gyro, ddof=1))

        seg.orientation_drift = yaw_series_delta(imu_df["yaw"].to_numpy())

    odom_df = _slice_df(bag_data.odom, seg.window)
    if odom_df is not None and len(odom_df) > 1:
        seg.odom_drift = yaw_series_delta(odom_df["yaw"].to_numpy())
