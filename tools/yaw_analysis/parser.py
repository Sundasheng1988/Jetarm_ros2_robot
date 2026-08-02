"""
Rosbag parser — reads a ROS2 Humble bag and returns structured DataFrames.

Supported topics (configurable via CLI):
    - IMU topic    → orientation quaternion + angular_velocity.z
    - Odom topic   → pose.pose.orientation
    - RobotVel topic → z (angular velocity)
    - CmdVel topic → angular.z
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np
import pandas as pd

from .utils import yaw_from_quaternion_msg, accumulate_angular


# ---------------------------------------------------------------------------
# BagParser
# ---------------------------------------------------------------------------

@dataclass
class BagData:
    """Container for all parsed rosbag data.

    Each attribute holds a :class:`pandas.DataFrame` with at least a
    ``t`` column (UNIX seconds) and topic-specific value columns, *or*
    ``None`` when the topic was not found in the bag.
    """

    imu: Optional[pd.DataFrame] = None
    """Columns: ``t``, ``yaw``, ``gyro_z`` (rad/s)"""
    odom: Optional[pd.DataFrame] = None
    """Columns: ``t``, ``yaw``"""
    robotvel: Optional[pd.DataFrame] = None
    """Columns: ``t``, ``angular_z`` (rad/s)"""
    cmd_vel: Optional[pd.DataFrame] = None
    """Columns: ``t``, ``angular_z`` (rad/s)"""

    topic_counts: dict[str, int] = field(default_factory=dict)
    """Number of messages read per topic."""

    def has_data(self) -> bool:
        """Return ``True`` if at least one topic has data."""
        return any(df is not None and not df.empty for df in
                   [self.imu, self.odom, self.robotvel, self.cmd_vel])


class BagParser:
    """Read a ROS2 bag and extract structured data for yaw analysis.

    Parameters
    ----------
    bag_path : str or Path
        Path to the rosbag directory.
    imu_topic : str, optional
        IMU topic name.
    odom_topic : str, optional
        Odometry topic name.
    robotvel_topic : str, optional
        Robot velocity topic name.
    cmd_topic : str, optional
        Command velocity topic name.
    """

    def __init__(
        self,
        bag_path: str | Path,
        *,
        imu_topic: Optional[str] = None,
        odom_topic: Optional[str] = None,
        robotvel_topic: Optional[str] = None,
        cmd_topic: Optional[str] = None,
    ) -> None:
        self._bag_path = Path(bag_path)
        self._topics: dict[str, Optional[str]] = {
            "imu": imu_topic,
            "odom": odom_topic,
            "robotvel": robotvel_topic,
            "cmd_vel": cmd_topic,
        }

        # Filter out None-valued topics
        self._active_topics: dict[str, str] = {
            k: v for k, v in self._topics.items() if v is not None
        }

        # Lazy-loaded reader
        self._reader: Any = None          # noqa: ANN401  — rosbag2_py.SequentialReader
        self._type_map: dict[str, str] = {}
        self._msg_cache: dict[str, Any] = {}

        # Validate bag path early
        if not self._bag_path.exists():
            raise FileNotFoundError(f"Bag path does not exist: {self._bag_path}")
        if not self._bag_path.is_dir():
            raise NotADirectoryError(
                f"Expected a bag directory (containing .db3 + metadata.yaml), "
                f"got: {self._bag_path}"
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def parse(self) -> BagData:
        """Read the bag and return structured data.

        Returns
        -------
        BagData
        """
        data_lists: dict[str, list[dict[str, float]]] = {k: [] for k in self._active_topics}
        counts: dict[str, int] = {k: 0 for k in self._active_topics}

        self._open_reader()

        while self._reader.has_next():
            topic, data, t_ns = self._reader.read_next()
            if topic not in self._type_map:
                continue

            key = self._topic_to_key(topic)
            if key is None:
                continue

            ts = t_ns / 1e9
            msg = self._deserialize(topic, data)
            if msg is None:
                continue

            counts[key] += 1

            if key == "imu":
                self._parse_imu(msg, ts, data_lists["imu"])
            elif key == "odom":
                self._parse_odom(msg, ts, data_lists["odom"])
            elif key == "robotvel":
                self._parse_robotvel(msg, ts, data_lists["robotvel"])
            elif key == "cmd_vel":
                self._parse_cmd_vel(msg, ts, data_lists["cmd_vel"])

        self._close_reader()

        # Build BagData
        result = BagData(topic_counts=counts)
        for key in self._active_topics:
            df = self._lists_to_df(data_lists[key], key)
            setattr(result, key, df)

        return result

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _open_reader(self) -> None:
        """Open the rosbag reader and build type/topic maps."""
        import rosbag2_py
        from rosidl_runtime_py.utilities import get_message

        storage = rosbag2_py.StorageOptions(
            uri=str(self._bag_path),
            storage_id="sqlite3",
        )
        converter = rosbag2_py.ConverterOptions("", "")
        reader = rosbag2_py.SequentialReader()
        reader.open(storage, converter)

        self._reader = reader
        self._get_message = get_message  # bind once

        # Build type map from ALL bag topics
        all_topics = reader.get_all_topics_and_types()
        self._type_map = {t.name: t.type for t in all_topics}

        # Validate requested topics exist
        requested = list(self._active_topics.values())
        for topic in requested:
            if topic not in self._type_map:
                warnings.warn(
                    f"Topic '{topic}' not found in bag.  Skipping.",
                    stacklevel=2,
                )

    def _close_reader(self) -> None:
        """Release the reader."""
        if self._reader is not None:
            try:
                self._reader = None
            except Exception:
                pass

    def _topic_to_key(self, topic: str) -> Optional[str]:
        """Map a ROS topic name back to internal key, or ``None``."""
        for key, t in self._active_topics.items():
            if t == topic:
                return key
        return None

    def _deserialize(self, topic: str, data: bytes) -> Optional[Any]:  # noqa: ANN401
        """Deserialize a raw rosbag message."""
        from rclpy.serialization import deserialize_message

        msg_type_str = self._type_map.get(topic)
        if msg_type_str is None:
            return None

        # Cache message class lookups
        if msg_type_str not in self._msg_cache:
            try:
                self._msg_cache[msg_type_str] = self._get_message(msg_type_str)
            except Exception as exc:
                warnings.warn(
                    f"Cannot load message type '{msg_type_str}' for topic "
                    f"'{topic}': {exc}",
                    stacklevel=2,
                )
                return None

        try:
            return deserialize_message(data, self._msg_cache[msg_type_str])
        except Exception as exc:
            warnings.warn(
                f"Failed to deserialize message on '{topic}': {exc}",
                stacklevel=2,
            )
            return None

    # ------------------------------------------------------------------
    # Per-topic parsers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_imu(
        msg: Any,          # noqa: ANN401 — sensor_msgs/Imu
        ts: float,
        records: list[dict[str, float]],
    ) -> None:
        yaw = yaw_from_quaternion_msg(msg.orientation)
        records.append({
            "t": ts,
            "yaw": yaw,
            "gyro_z": msg.angular_velocity.z,  # rad/s
        })

    @staticmethod
    def _parse_odom(
        msg: Any,          # noqa: ANN401 — nav_msgs/Odometry or similar
        ts: float,
        records: list[dict[str, float]],
    ) -> None:
        yaw = yaw_from_quaternion_msg(msg.pose.pose.orientation)
        records.append({"t": ts, "yaw": yaw})

    @staticmethod
    def _parse_robotvel(
        msg: Any,          # noqa: ANN401 — message with .z (angular Z)
        ts: float,
        records: list[dict[str, float]],
    ) -> None:
        try:
            wz = float(msg.z)
        except AttributeError:
            # Some robotvel messages may use .angular.z — try that
            try:
                wz = float(msg.angular.z)
            except AttributeError:
                warnings.warn(
                    "robotvel message has neither .z nor .angular.z; skipping",
                    stacklevel=2,
                )
                return
        records.append({"t": ts, "angular_z": wz})

    @staticmethod
    def _parse_cmd_vel(
        msg: Any,          # noqa: ANN401 — geometry_msgs/Twist
        ts: float,
        records: list[dict[str, float]],
    ) -> None:
        records.append({
            "t": ts,
            "angular_z": msg.angular.z,  # rad/s
        })

    # ------------------------------------------------------------------
    # DataFrame builder
    # ------------------------------------------------------------------

    @staticmethod
    def _lists_to_df(
        records: list[dict[str, float]],
        key: str,
    ) -> Optional[pd.DataFrame]:
        """Convert a list-of-dicts to a sorted DataFrame (or None if empty)."""
        if not records:
            return None
        df = pd.DataFrame(records).sort_values("t").reset_index(drop=True)

        # For IMU, also add a cumulative gyro integral column
        if key == "imu" and len(df) > 1:
            df["gyro_integral"] = accumulate_angular(
                df["t"].to_numpy(),
                df["gyro_z"].to_numpy(),
            )

        # For robotvel, add cumulative integral
        if key == "robotvel" and len(df) > 1:
            df["vel_integral"] = accumulate_angular(
                df["t"].to_numpy(),
                df["angular_z"].to_numpy(),
            )

        return df
