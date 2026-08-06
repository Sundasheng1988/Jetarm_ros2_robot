"""
Utility functions for yaw analysis.

All angle arithmetic operates in **degrees** throughout the project to keep
reports human-readable without conversion.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# Quaternion → yaw
# ---------------------------------------------------------------------------

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (heading) from a quaternion, returned in degrees.

    Uses the standard Z-axis Euler-angle formula:
        yaw = atan2(2(w*z + x*y), 1 - 2(y² + z²))

    Parameters
    ----------
    x, y, z, w : float
        Quaternion components.

    Returns
    -------
    float
        Yaw in degrees, in range (-180, 180].
    """
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def yaw_from_quaternion_msg(
    msg: "Any",  # noqa: ANN401
) -> float:
    """Convenience wrapper — extract yaw from a ROS quaternion message.

    Accepts any message-like object with ``.x``, ``.y``, ``.z``, ``.w``
    attributes (e.g. ``geometry_msgs/Quaternion``).

    Parameters
    ----------
    msg : object
        ROS quaternion message (or any object with the four fields).

    Returns
    -------
    float
        Yaw in degrees.
    """
    return yaw_from_quaternion(msg.x, msg.y, msg.z, msg.w)


# ---------------------------------------------------------------------------
# Angle arithmetic
# ---------------------------------------------------------------------------

def degrees_shortest_distance(a: float, b: float) -> float:
    """Shortest signed distance ``b - a`` on the circle, in degrees.

    The result is in the range (-180, 180].
    """
    d = (b - a) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def normalize_degrees(angle: float) -> float:
    """Normalise an angle to (-180, 180] degrees."""
    return degrees_shortest_distance(0.0, angle)


def unwrap_degrees_series(angles: np.ndarray) -> np.ndarray:
    """Unwrap a 1-D array of degree values so large jumps are removed.

    Wraps :func:`numpy.unwrap` after converting to radians.
    """
    return np.degrees(np.unwrap(np.radians(angles)))


def yaw_series_delta(angles: np.ndarray) -> float:
    """Total signed rotation from a yaw time-series, robust to ±180° wraps.

    Computes the cumulative sum of shortest-arc differences between
    **consecutive samples**.  This recovers the true signed rotation even
    when the series crosses the ±180° boundary, as long as each individual
    step is less than 180° (which is guaranteed at typical IMU rates).

    Parameters
    ----------
    angles : np.ndarray
        1-D array of yaw values in degrees, in [-180, 180).

    Returns
    -------
    float
        Signed cumulative rotation in degrees.
    """
    if len(angles) < 2:
        return 0.0
    diffs = np.empty(len(angles) - 1)
    for i in range(len(angles) - 1):
        diffs[i] = degrees_shortest_distance(angles[i], angles[i + 1])
    return float(np.sum(diffs))


# ---------------------------------------------------------------------------
# Integration helpers
# ---------------------------------------------------------------------------

def accumulate_angular(
    timestamps: np.ndarray,
    angular_z: np.ndarray,
) -> np.ndarray:
    """Cumulative trapezoidal integral of angular velocity → total yaw.

    ``angular_z`` is expected in **rad/s** — the result is in **degrees**.

    Parameters
    ----------
    timestamps : np.ndarray
        1-D array of UNIX timestamps (seconds).
    angular_z : np.ndarray
        1-D array of angular velocity around Z (rad/s).

    Returns
    -------
    np.ndarray
        Cumulative integral in degrees, same length as inputs.  The first
        element is always 0.
    """
    dt = np.diff(timestamps)
    avg_w = (angular_z[:-1] + angular_z[1:]) / 2.0
    integral = np.concatenate(([0.0], np.cumsum(np.degrees(avg_w * dt))))
    return integral


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def segment_stats(
    values: np.ndarray,
) -> dict[str, float]:
    """Basic statistics for a 1-D data array.

    Returns a dict with ``mean``, ``std``, ``min``, ``max``, ``drift``
    (last - first).
    """
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values, ddof=1)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "drift": float(values[-1] - values[0]),
    }


def angular_diff_report(
    measured: Optional[float],
    expected: Optional[float],
    label: str,
    width: int = 20,
) -> str:
    """Format one line of a rotation comparison table.

    Parameters
    ----------
    measured : float or None
        Measured delta-yaw (degrees).
    expected : float or None
        Reference/expected delta-yaw (degrees).
    label : str
        Row label (e.g. "IMU Orientation").
    width : int
        Label column width for alignment.

    Returns
    -------
    str
        Formatted line such as ``"  IMU Orientation ΔYaw : -178.5°  | err: +1.5°"``.
    """
    if measured is None:
        return f"  {label:<{width}s}: N/A"
    line = f"  {label:<{width}s}: {measured:+8.2f}°"
    if expected is not None:
        err = degrees_shortest_distance(expected, measured)
        line += f"  | err: {err:+7.2f}°"
    return line
