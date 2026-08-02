"""
yaw_analysis — ROS2 rosbag yaw calibration analysis toolkit.

A production-quality tool for analyzing yaw (heading) accuracy from multiple
sensor/data sources in ROS2 Humble rosbags.  Designed for mobile robot chassis
debugging and calibration.

Exports
-------
SegmentType
    Enum: CW, CCW, STATIC
TimeWindow
    Named tuple for start/end timestamps
Segment
    Detected rotation or static segment with computed metrics
BagData
    Raw parsed data container (DataFrames per topic)
AnalysisResult
    Top-level analysis output passed to report and plot modules
"""

from .utils import (
    yaw_from_quaternion,
    yaw_from_quaternion_msg,
    degrees_shortest_distance,
    normalize_degrees,
    unwrap_degrees_series,
    yaw_series_delta,
    accumulate_angular,
    segment_stats,
)
from .parser import BagParser, BagData
from .segment import SegmentType, TimeWindow, Segment, SegmentDetector, compute_segment_metrics
from .report import ReportGenerator
from .plot import PlotGenerator

__all__ = [
    "yaw_from_quaternion",
    "yaw_from_quaternion_msg",
    "degrees_shortest_distance",
    "normalize_degrees",
    "unwrap_degrees_series",
    "accumulate_angular",
    "segment_stats",
    "BagParser",
    "BagData",
    "SegmentType",
    "TimeWindow",
    "Segment",
    "SegmentDetector",
    "compute_segment_metrics",
    "ReportGenerator",
    "PlotGenerator",
]
