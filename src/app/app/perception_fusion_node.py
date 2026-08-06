#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from app.audit_utils import parse_roi_message
from app.perception_fusion_utils import (
    build_cache_entry,
    build_fused_object,
    build_roi_only_object,
    build_yolo_only_object,
    deduplicate_entries,
    match_objects,
    prune_cache,
)

_qos_best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class PerceptionFusionNode(Node):
    def __init__(self):
        super().__init__("perception_fusion_node")

        self.declare_parameter("yolo_topic", "/world_model/objects")
        self.declare_parameter("roi_topic", "/world_model/roi_objects")
        self.declare_parameter("output_topic", "/world_model/perception_objects")
        self.declare_parameter("distance_threshold", 0.06)
        self.declare_parameter("cache_ttl_sec", 2.0)
        self.declare_parameter("publish_rate_hz", 3.0)

        g = self.get_parameter
        self.yolo_topic = str(g("yolo_topic").value)
        self.roi_topic = str(g("roi_topic").value)
        self.output_topic = str(g("output_topic").value)
        self.distance_threshold = float(g("distance_threshold").value)
        self.cache_ttl_sec = float(g("cache_ttl_sec").value)
        self.publish_rate_hz = float(g("publish_rate_hz").value)

        self._yolo_cache = []
        self._roi_cache = []
        self._yolo_seq = 0
        self._roi_seq = 0

        self.create_subscription(String, self.yolo_topic, self._on_yolo, 10)
        self.create_subscription(
            String, self.roi_topic, self._on_roi, _qos_best_effort
        )
        self._pub = self.create_publisher(String, self.output_topic, 10)
        self._timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._publish_fused
        )

        self.get_logger().info(
            f"✅ 融合节点启动  yolo={self.yolo_topic}  roi={self.roi_topic}"
            f"  out={self.output_topic}  dist={self.distance_threshold}"
            f"  ttl={self.cache_ttl_sec}s  rate={self.publish_rate_hz}Hz"
        )

    def _on_yolo(self, msg: String):
        try:
            data = parse_roi_message(msg.data)
        except Exception as e:
            self.get_logger().warn(f"yolo parse failed: {e}")
            return

        now = time.time()
        added = 0
        for obj in data.get("objects", []):
            entry = build_cache_entry(obj, "yolo", self._yolo_seq, now)
            if entry is not None:
                self._yolo_seq += 1
                self._yolo_cache.append(entry)
                added += 1
        prune_cache(self._yolo_cache, now, self.cache_ttl_sec)

    def _on_roi(self, msg: String):
        try:
            data = parse_roi_message(msg.data)
        except Exception as e:
            self.get_logger().warn(f"roi parse failed: {e}")
            return

        now = time.time()
        added = 0
        for obj in data.get("objects", []):
            entry = build_cache_entry(obj, "roi", self._roi_seq, now)
            if entry is not None:
                self._roi_seq += 1
                self._roi_cache.append(entry)
                added += 1
        prune_cache(self._roi_cache, now, self.cache_ttl_sec)

    def _publish_fused(self):
        now = time.time()
        prune_cache(self._yolo_cache, now, self.cache_ttl_sec)
        prune_cache(self._roi_cache, now, self.cache_ttl_sec)

        roi_unique = deduplicate_entries(self._roi_cache, self.distance_threshold)
        yolo_unique = deduplicate_entries(self._yolo_cache, self.distance_threshold)

        fused, roi_only, yolo_only = match_objects(
            roi_unique, yolo_unique, self.distance_threshold
        )

        objects_out = []
        seq = 0

        for yolo_entry, roi_entry, match_dist in fused:
            seq += 1
            obj = build_fused_object(yolo_entry, roi_entry, match_dist, now)
            obj["id"] = seq
            objects_out.append(obj)

        for roi_entry in roi_only:
            seq += 1
            obj = build_roi_only_object(roi_entry, now)
            obj["id"] = seq
            objects_out.append(obj)

        for yolo_entry in yolo_only:
            seq += 1
            obj = build_yolo_only_object(yolo_entry, now)
            obj["id"] = seq
            objects_out.append(obj)

        msg = String()
        msg.data = json.dumps({"objects": objects_out}, ensure_ascii=False)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusionNode()
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
