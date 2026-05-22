#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from app.audit_utils import (
    assign_track,
    compute_track_metrics,
    extract_xyz,
    parse_roi_message,
)


# ============================================================
# node
# ============================================================

class RoiDetectionAuditNode(Node):
    def __init__(self):
        super().__init__("roi_detection_audit_node")

        self.declare_parameter("sample_count", 100)
        self.declare_parameter("max_duration_sec", 30.0)
        self.declare_parameter("output_dir", "artifacts/perception_audit")
        self.declare_parameter("topic", "/world_model/roi_objects")
        self.declare_parameter("distance_threshold", 0.05)

        g = self.get_parameter
        self.target_count = int(g("sample_count").value)
        self.max_duration = float(g("max_duration_sec").value)
        self.output_dir = str(g("output_dir").value)
        self.audit_topic = str(g("topic").value)
        self.distance_threshold = float(g("distance_threshold").value)

        self.samples: List[Dict[str, Any]] = []
        self.tracks: Dict[str, Dict[str, Any]] = {}
        self.start_ts: float = time.time()

        self.create_subscription(
            String, self.audit_topic, self._on_msg, 10
        )
        self._check_timer = self.create_timer(0.5, self._check_complete)

        self.get_logger().info(
            f"✅ 审计开始  topic={self.audit_topic}  "
            f"samples={self.target_count}  max_dur={self.max_duration}s  "
            f"out_dir={self.output_dir}"
        )

    def _on_msg(self, msg: String):
        if len(self.samples) >= self.target_count:
            return
        try:
            data = parse_roi_message(msg.data)
        except Exception as e:
            self.get_logger().warn(f"parse failed: {e}")
            return

        frame_entry = {
            "frame_index": len(self.samples) + 1,
            "received_at": time.time(),
            "objects": [],
        }
        for obj in data.get("objects", []):
            xyz = extract_xyz(obj)
            obj_out = {
                "class_name": str(obj.get("class_name", "unknown")),
                "color": str(obj.get("color", "unknown")),
                "confidence": float(obj.get("confidence", 0.5)),
            }
            if xyz:
                obj_out["xyz"] = [xyz[0], xyz[1], xyz[2]]
                tid = assign_track(self.tracks, xyz, self.distance_threshold)
                t = self.tracks[tid]
                t["frames"].append(obj_out.copy())
                sx, sy, sz = t["xyz_sum"]
                t["xyz_sum"] = [sx + xyz[0], sy + xyz[1], sz + xyz[2]]
                t["xyz_count"] += 1
                n = t["xyz_count"]
                t["xyz_mean"] = [
                    t["xyz_sum"][0] / n,
                    t["xyz_sum"][1] / n,
                    t["xyz_sum"][2] / n,
                ]
            else:
                obj_out["xyz"] = None
            frame_entry["objects"].append(obj_out)
        self.samples.append(frame_entry)

    def _check_complete(self):
        elapsed = time.time() - self.start_ts
        if len(self.samples) >= self.target_count:
            self.get_logger().info(f"目标样本数 {self.target_count} 已到达")
            self._finalize()
        elif elapsed >= self.max_duration:
            self.get_logger().info(f"最大时长 {self.max_duration}s 已达 ({elapsed:.1f}s)")
            self._finalize()

    def _finalize(self):
        self._check_timer.cancel()
        try:
            self.destroy_subscription(
                self.create_subscription(String, self.audit_topic, lambda _: None, 10)
            )
        except Exception:
            pass

        od = Path(self.output_dir)
        od.mkdir(parents=True, exist_ok=True)

        self._write_jsonl(od)
        self._write_summary(od)
        self._write_report(od)
        self._console_summary()

        self.get_logger().info("🔚 审计完成，节点关闭")
        rclpy.shutdown()

    # ---- writer helpers ----

    def _write_jsonl(self, od: Path):
        p = od / "raw_samples.jsonl"
        with open(p, "w", encoding="utf-8") as f:
            for s in self.samples:
                f.write(json.dumps(s, ensure_ascii=False) + "\n")
        self.get_logger().info(f"  📄 {p}")

    def _write_summary(self, od: Path):
        track_metrics = []
        for t in self.tracks.values():
            m = compute_track_metrics(t)
            if m:
                track_metrics.append(m)

        total_tracks = len(track_metrics)
        unstable_count = sum(1 for m in track_metrics if m.get("unstable"))
        summary = {
            "metadata": {
                "total_frames": len(self.samples),
                "duration_sec": round(time.time() - self.start_ts, 2),
                "distance_threshold": self.distance_threshold,
                "topic": self.audit_topic,
            },
            "tracks": sorted(track_metrics, key=lambda m: -m.get("frames_seen", 0)),
            "global": {
                "total_tracks": total_tracks,
                "unstable_tracks": unstable_count,
                "avg_class_stability": round(
                    sum(m.get("class_stability_ratio", 1) for m in track_metrics) / max(total_tracks, 1), 4
                ),
                "avg_color_stability": round(
                    sum(m.get("color_stability_ratio", 1) for m in track_metrics) / max(total_tracks, 1), 4
                ),
                "avg_label_stability": round(
                    sum(m.get("label_stability_ratio", 1) for m in track_metrics) / max(total_tracks, 1), 4
                ),
            },
        }
        p = od / "audit_summary.json"
        with open(p, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f"  📊 {p}")

    def _write_report(self, od: Path):
        track_metrics = []
        for t in self.tracks.values():
            m = compute_track_metrics(t)
            if m:
                track_metrics.append(m)

        lines = []
        lines.append("# ROI Detection Audit Report\n")
        lines.append(f"**Frames**: {len(self.samples)}  |  ")
        lines.append(f"**Tracks**: {len(track_metrics)}  |  ")
        lines.append(f"**Duration**: {round(time.time() - self.start_ts, 1)}s  |  ")
        lines.append(f"**Distance Threshold**: {self.distance_threshold}m\n")
        lines.append("---\n")

        lines.append("## Per-Track Metrics\n")
        lines.append("| Track | Frames | Dominant Label | Class Stability | Color Stability | Label Stability | Conf Mean | XYZ Mean | Switches | Stable? |")
        lines.append("|-------|--------|----------------|-----------------|-----------------|-----------------|-----------|----------|----------|---------|")
        for m in track_metrics:
            stable = "✅" if not m["unstable"] else "🔴"
            xyz_s = f"[{m['xyz_mean'][0]:.3f}, {m['xyz_mean'][1]:.3f}, {m['xyz_mean'][2]:.3f}]"
            lines.append(
                f"| {m['track_id']} | {m['frames_seen']} | {m['dominant_label']} | "
                f"{m['class_stability_ratio']:.2f} | {m['color_stability_ratio']:.2f} | "
                f"{m['label_stability_ratio']:.2f} | {m['confidence_mean']:.2f} | "
                f"{xyz_s} | {m['label_switch_count']} | {stable} |"
            )
        lines.append("")

        lines.append("## Global Summary\n")
        total_tracks = len(track_metrics)
        unstable_count = sum(1 for m in track_metrics if m.get("unstable"))
        avg_cls = sum(m.get("class_stability_ratio", 1) for m in track_metrics) / max(total_tracks, 1)
        avg_col = sum(m.get("color_stability_ratio", 1) for m in track_metrics) / max(total_tracks, 1)
        avg_lbl = sum(m.get("label_stability_ratio", 1) for m in track_metrics) / max(total_tracks, 1)
        lines.append(f"- **Total tracks**: {total_tracks}")
        lines.append(f"- **Unstable tracks**: {unstable_count}")
        lines.append(f"- **Avg class stability**: {avg_cls:.2f}")
        lines.append(f"- **Avg color stability**: {avg_col:.2f}")
        lines.append(f"- **Avg label stability**: {avg_lbl:.2f}")
        lines.append("")

        for m in track_metrics:
            if not m["unstable"]:
                continue
            lines.append(f"## Unstable Track: {m['track_id']}\n")
            lines.append(f"- **Dominant label**: {m['dominant_label']}")
            lines.append(f"- **Label switches**: {m['label_switch_count']}")
            lines.append(f"- **Label stability ratio**: {m['label_stability_ratio']:.2f}\n")
            lines.append("### Label Transition Counts\n")
            lines.append("| Label | Count |")
            lines.append("|-------|-------|")
            for lbl, cnt in sorted(m["label_counts"].items(), key=lambda x: -x[1]):
                lines.append(f"| {lbl} | {cnt} |")
            lines.append("")

        lines.append("## Recommendations\n")
        lines.append("Based on observed label stability ratio and switch frequency:\n")
        min_stability = min((m["label_stability_ratio"] for m in track_metrics), default=1.0)
        if min_stability > 0.85:
            lines.append("- **Perception is stable.** Minimal temporal filtering needed.")
            lines.append("- StableObjectTracker: voting window 3-5 frames, EMA alpha 0.5, TTL 1.0s")
        elif min_stability > 0.60:
            lines.append("- **Moderate instability.** Temporal voting recommended.")
            lines.append("- StableObjectTracker: voting window 10 frames, EMA alpha 0.3, TTL 2.0s")
        else:
            lines.append("- **High instability.** Aggressive temporal filtering required.")
            lines.append("- StableObjectTracker: voting window 15-20 frames, EMA alpha 0.2, TTL 3.0s")
            lines.append("- Consider tuning LAB color thresholds and shape classification rules.")
        lines.append("")
        lines.append("## Future YOLO + ROI Fusion Path\n")
        lines.append("YOLO provides stable semantic class names but no color or reliable world coordinates. ")
        lines.append("ROI provides color + world pose but unstable class assignment. ")
        lines.append("After StableObjectTracker is complete, a fusion layer can merge both sources ")
        lines.append("by spatial proximity matching: YOLO class_name takes priority for semantics, ")
        lines.append("ROI color + xyz provides spatial grounding.\n")

        report_path = od / "roi_perception_audit.md"
        with open(report_path, "w", encoding="utf-8") as f:
            f.write("".join(lines))
        self.get_logger().info(f"  📝 {report_path}")

    def _console_summary(self):
        track_metrics = []
        for t in self.tracks.values():
            m = compute_track_metrics(t)
            if m:
                track_metrics.append(m)

        dur = time.time() - self.start_ts
        self.get_logger().info("=" * 60)
        self.get_logger().info("  ROI Detection Audit — 完成")
        self.get_logger().info(f"  时长:   {dur:.1f}s")
        self.get_logger().info(f"  帧数:   {len(self.samples)} / {self.target_count}")
        self.get_logger().info(f"  轨数:   {len(track_metrics)} 发现")
        self.get_logger().info("")
        for m in track_metrics:
            icon = "✅" if not m["unstable"] else "🔴"
            self.get_logger().info(
                f"  {m['track_id']}  {m['dominant_label']:<16s}  "
                f"{m['frames_seen']:>3d} 帧  {icon}  ({m['label_stability_ratio']:.2f})"
            )
        self.get_logger().info("")
        self.get_logger().info(f"  报告: {self.output_dir}")
        self.get_logger().info("=" * 60)


# ============================================================
# entry point
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = RoiDetectionAuditNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
