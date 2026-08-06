"""
Plotting — generate a multi-panel time-series figure with all yaw data.

Layout (5 subplots sharing the X axis):
    1. Command velocity (angular.z)
    2. IMU orientation yaw (with gyro integral overlay)
    3. Odom yaw
    4. RobotVel integrated yaw
    5. Gyro Z (raw angular velocity)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from .parser import BagData
from .segment import Segment, SegmentType


@dataclass
class PlotGenerator:
    """Generates a multi-panel PNG figure of yaw data.

    Parameters
    ----------
    output_dir : str or Path
        Directory where ``plot.png`` is saved.
    dpi : int
        Figure DPI.
    figsize : tuple[int, int]
        Figure width x height in inches.
    """

    output_dir: str | Path = "."
    dpi: int = 150
    figsize: tuple[float, float] = (14, 10)

    # Colour palette
    _COLORS: dict[str, str] = field(default_factory=lambda: {
        "cmd_vel": "#1f77b4",
        "imu_yaw": "#d62728",
        "gyro_int": "#ff7f0e",
        "odom": "#2ca02c",
        "rv_int": "#9467bd",
        "gyro_z": "#8c564b",
        "segment_bg_cw": "#ffcccc",
        "segment_bg_cw_alpha": 0.15,
        "segment_bg_ccw": "#ccffcc",
        "segment_bg_ccw_alpha": 0.15,
        "segment_bg_static": "#eeeeee",
        "segment_bg_static_alpha": 0.10,
    }, repr=False)

    def generate(
        self,
        bag_data: BagData,
        segments: list[Segment],
        *,
        show: bool = False,
    ) -> str:
        """Create the figure and save to ``plot.png``.

        Parameters
        ----------
        bag_data : BagData
            Parsed bag data (from :class:`BagParser.parse`).
        segments : list[Segment]
            Detected segments (for background shading).
        show : bool
            Also open the plot interactively (``plt.show()``).

        Returns
        -------
        str
            Path to the saved PNG file.
        """
        import matplotlib
        matplotlib.use("Agg")  # non-interactive backend for headless

        import matplotlib.pyplot as plt
        from matplotlib.patches import FancyBboxPatch

        # ---- Build figure ----
        fig, axes = plt.subplots(
            nrows=5, ncols=1, sharex=True,
            figsize=self.figsize,
            gridspec_kw={"hspace": 0.30},
        )
        ax_cmd, ax_imu, ax_odom, ax_rv, ax_gyro = axes

        # ---- Style ----
        fig.patch.set_facecolor("white")
        font_small = 8

        # ---- 1. Command velocity ----
        self._plot_cmd_vel(ax_cmd, bag_data.cmd_vel, segments)
        self._shade_segments(ax_cmd, segments)

        # ---- 2. IMU yaw + gyro integral ----
        self._plot_imu(ax_imu, bag_data.imu, segments)
        self._shade_segments(ax_imu, segments)

        # ---- 3. Odom yaw ----
        self._plot_odom(ax_odom, bag_data.odom, segments)
        self._shade_segments(ax_odom, segments)

        # ---- 4. RobotVel integral ----
        self._plot_robotvel(ax_rv, bag_data.robotvel, segments)
        self._shade_segments(ax_rv, segments)

        # ---- 5. Gyro Z raw ----
        self._plot_gyro_z(ax_gyro, bag_data.imu, segments)
        self._shade_segments(ax_gyro, segments)

        # ---- X label (bottom only) ----
        ax_gyro.set_xlabel("Time (s)", fontsize=font_small + 1)

        # ---- Save ----
        out_path = Path(self.output_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        png_path = out_path / "plot.png"
        fig.savefig(str(png_path), dpi=self.dpi, bbox_inches="tight")
        plt.close(fig)

        if show:
            plt.show()

        return str(png_path)

    # ------------------------------------------------------------------
    # Per-subplot plotters
    # ------------------------------------------------------------------

    def _plot_cmd_vel(
        self,
        ax: "matplotlib.axes.Axes",         # noqa: F821
        cmd_vel: Optional["pd.DataFrame"],   # noqa: F821
        segments: list[Segment],
    ) -> None:
        if cmd_vel is not None and not cmd_vel.empty:
            t = cmd_vel["t"].to_numpy()
            w = np.degrees(cmd_vel["angular_z"].to_numpy())
            ax.step(t, w, where="post",
                    color=self._COLORS["cmd_vel"],
                    linewidth=0.8, label="cmd_vel angular.z")
        self._decorate(ax, "cmd_vel", "Angular velocity (°/s)")

    def _plot_imu(
        self,
        ax: "matplotlib.axes.Axes",         # noqa: F821
        imu: Optional["pd.DataFrame"],      # noqa: F821
        segments: list[Segment],
    ) -> None:
        if imu is not None and not imu.empty:
            t = imu["t"].to_numpy()
            ax.plot(t, imu["yaw"].to_numpy(),
                    color=self._COLORS["imu_yaw"],
                    linewidth=1.0, label="IMU orientation yaw")
            if "gyro_integral" in imu.columns:
                ax.plot(t, imu["gyro_integral"].to_numpy(),
                        color=self._COLORS["gyro_int"],
                        linewidth=0.8, linestyle="--", label="Gyro integral")
        self._decorate(ax, "IMU Yaw", "Yaw (°)")
        ax.legend(fontsize=7, loc="best")

    def _plot_odom(
        self,
        ax: "matplotlib.axes.Axes",         # noqa: F821
        odom: Optional["pd.DataFrame"],     # noqa: F821
        segments: list[Segment],
    ) -> None:
        if odom is not None and not odom.empty:
            ax.plot(odom["t"].to_numpy(), odom["yaw"].to_numpy(),
                    color=self._COLORS["odom"],
                    linewidth=1.0, label="Odom yaw")
        self._decorate(ax, "Odom Yaw", "Yaw (°)")
        ax.legend(fontsize=7, loc="best")

    def _plot_robotvel(
        self,
        ax: "matplotlib.axes.Axes",         # noqa: F821
        robotvel: Optional["pd.DataFrame"], # noqa: F821
        segments: list[Segment],
    ) -> None:
        if robotvel is not None and not robotvel.empty:
            if "vel_integral" in robotvel.columns:
                ax.plot(robotvel["t"].to_numpy(),
                        robotvel["vel_integral"].to_numpy(),
                        color=self._COLORS["rv_int"],
                        linewidth=1.0, label="RobotVel integral")
        self._decorate(ax, "RobotVel Integral", "Yaw (°)")
        ax.legend(fontsize=7, loc="best")

    def _plot_gyro_z(
        self,
        ax: "matplotlib.axes.Axes",         # noqa: F821
        imu: Optional["pd.DataFrame"],      # noqa: F821
        segments: list[Segment],
    ) -> None:
        if imu is not None and not imu.empty:
            t = imu["t"].to_numpy()
            ax.plot(t, np.degrees(imu["gyro_z"].to_numpy()),
                    color=self._COLORS["gyro_z"],
                    linewidth=0.6, label="Gyro Z (raw)")
            ax.axhline(y=0, color="gray", linewidth=0.4, linestyle=":")
        self._decorate(ax, "Gyro Z", "Angular velocity (°/s)")
        ax.legend(fontsize=7, loc="best")

    # ------------------------------------------------------------------
    # Segment background shading
    # ------------------------------------------------------------------

    def _shade_segments(
        self,
        ax: "matplotlib.axes.Axes",         # noqa: F821
        segments: list[Segment],
    ) -> None:
        """Add translucent vertical bands for each detected segment."""
        c = self._COLORS
        for seg in segments:
            if seg.type == SegmentType.CW:
                color = c["segment_bg_cw"]
                alpha = c["segment_bg_cw_alpha"]
            elif seg.type == SegmentType.CCW:
                color = c["segment_bg_ccw"]
                alpha = c["segment_bg_ccw_alpha"]
            else:
                color = c["segment_bg_static"]
                alpha = c["segment_bg_static_alpha"]

            ax.axvspan(
                seg.window.start, seg.window.end,
                facecolor=color, alpha=alpha, zorder=0,
            )

            # Label the segment at the top of the axis
            y_top = ax.get_ylim()[1] if ax.get_ylim() else 1.0
            mid_t = (seg.window.start + seg.window.end) / 2.0
            ax.text(
                mid_t, y_top, seg.name,
                fontsize=6, ha="center", va="top",
                color="gray", alpha=0.6,
                clip_on=True,
            )

    # ------------------------------------------------------------------
    # Axis decoration helper
    # ------------------------------------------------------------------

    @staticmethod
    def _decorate(
        ax: "matplotlib.axes.Axes",         # noqa: F821
        title: str,
        ylabel: str,
    ) -> None:
        """Apply consistent tick and label styling."""
        ax.tick_params(labelsize=8)
        ax.set_title(title, fontsize=9, fontweight="bold")
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, alpha=0.3, linewidth=0.4)
        ax.margins(x=0.01)
