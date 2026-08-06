#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from app.audit_utils import extract_xyz, parse_roi_message
from app.stable_tracker_utils import (
    build_stable_object,
    ema_update,
    extract_frame_data,
    extract_rpy,
    match_or_create_track,
    prune_expired_tracks,
)

_qos_best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class StableObjectTrackerNode(Node):
    def __init__(self):
        super().__init__("stable_object_tracker_node")

        # ---- parameters ----
        self.declare_parameter("input_topic", "/world_model/perception_objects")
        self.declare_parameter("output_topic", "/world_model/stable_objects")
        self.declare_parameter("distance_threshold", 0.05)
        self.declare_parameter("voting_window", 20)
        self.declare_parameter("min_frames", 5)
        self.declare_parameter("ttl_sec", 3.0)
        self.declare_parameter("ema_alpha", 0.2)
        self.declare_parameter("min_frames_roi_only", 8)

        g = self.get_parameter
        self.input_topic = str(g("input_topic").value)
        self.output_topic = str(g("output_topic").value)
        self.distance_threshold = float(g("distance_threshold").value)
        self.voting_window = int(g("voting_window").value)
        self.min_frames = int(g("min_frames").value)
        self.ttl_sec = float(g("ttl_sec").value)
        self.ema_alpha = float(g("ema_alpha").value)
        self.min_frames_roi_only = int(g("min_frames_roi_only").value)

        # ---- state ----
        self.tracks = {}

        # ---- pub / sub ----
        self._pub = self.create_publisher(String, self.output_topic, 10)

        subscription_qos = _qos_best_effort if "/world_model/roi_objects" in self.input_topic else 10
        self.create_subscription(
            String, self.input_topic, self._on_raw, subscription_qos
        )

        self.get_logger().info(
            f"✅ track 已启动  in={self.input_topic}  out={self.output_topic}"
            f"  dist={self.distance_threshold}  win={self.voting_window}"
            f"  min_frames={self.min_frames}  ttl={self.ttl_sec}s  ema_alpha={self.ema_alpha}"
        )

    def _on_raw(self, msg: String):
        now = time.time()
        try:
            data = parse_roi_message(msg.data)
        except Exception as e:
            self.get_logger().warn(f"parse failed: {e}")
            return

        for obj in data.get("objects", []):
            xyz = extract_xyz(obj)
            if xyz is None:
                continue

            frame_data = extract_frame_data(obj)
            rpy = extract_rpy(obj)

            tid, track = match_or_create_track(
                self.tracks, xyz, self.distance_threshold, self.voting_window
            )
            track["frames"].append(frame_data)
            track["xyz_latest"] = list(xyz)
            if rpy is not None:
                track["rpy_latest"] = list(rpy)
            track["confidence_smooth"] = ema_update(
                track["confidence_smooth"], frame_data["confidence"], self.ema_alpha
            )
            track["last_seen"] = now
            track["total_frames"] += 1

        expired = prune_expired_tracks(self.tracks, now, self.ttl_sec)
        if expired:
            self.get_logger().debug(f"pruned {len(expired)} expired tracks: {expired}")

        objects_out = []
        for t in self.tracks.values():
            sobj = build_stable_object(t, self.min_frames, self.min_frames_roi_only)
            if sobj is not None:
                objects_out.append(sobj)

        msg_out = String()
        msg_out.data = json.dumps({"objects": objects_out}, ensure_ascii=False)
        self._pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = StableObjectTrackerNode()
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
