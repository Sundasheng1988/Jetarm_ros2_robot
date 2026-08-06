#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 live toy-block color detector.

Subscribes:
  /depth_cam/rgb/image_raw  (sensor_msgs/Image)

Publishes:
  /toy_block/detection     (std_msgs/String; JSON)
  /toy_block/debug_image   (sensor_msgs/Image; bgr8)
  /toy_block/mask          (sensor_msgs/Image; mono8)

This first live version intentionally detects one colored block in the
validated fixed-camera work area. It does not command the robot.
"""

from __future__ import annotations

from collections import deque
import json
import math
import time
from typing import Any

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .classifier_core import (
    DetectorConfig,
    detect_block,
    draw_error_overlay,
    draw_overlay,
)


class ToyBlockColorDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("toy_block_color_detector")

        self.declare_parameter("image_topic", "/depth_cam/rgb/image_raw")
        self.declare_parameter("detection_topic", "/toy_block/detection")
        self.declare_parameter("debug_image_topic", "/toy_block/debug_image")
        self.declare_parameter("mask_topic", "/toy_block/mask")
        self.declare_parameter("process_hz", 5.0)
        self.declare_parameter("expected_width", 640)
        self.declare_parameter("expected_height", 480)
        self.declare_parameter("gate_margin_px", 8)
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("max_center_jump_px", 8.0)
        self.declare_parameter("max_area_change_ratio", 0.20)
        self.declare_parameter("publish_failed_frames", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        detection_topic = str(self.get_parameter("detection_topic").value)
        debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        mask_topic = str(self.get_parameter("mask_topic").value)

        process_hz = float(self.get_parameter("process_hz").value)
        if process_hz <= 0.0:
            raise ValueError("process_hz must be > 0")
        self.process_period_sec = 1.0 / process_hz

        stable_frames = int(self.get_parameter("stable_frames").value)
        if stable_frames < 1:
            raise ValueError("stable_frames must be >= 1")
        self.stable_frames = stable_frames
        self.max_center_jump_px = float(
            self.get_parameter("max_center_jump_px").value
        )
        self.max_area_change_ratio = float(
            self.get_parameter("max_area_change_ratio").value
        )
        self.publish_failed_frames = bool(
            self.get_parameter("publish_failed_frames").value
        )

        self.config = DetectorConfig(
            expected_width=int(self.get_parameter("expected_width").value),
            expected_height=int(self.get_parameter("expected_height").value),
            gate_margin_px=int(self.get_parameter("gate_margin_px").value),
        )

        self.bridge = CvBridge()
        self.last_process_monotonic = 0.0
        self.history: deque[dict[str, Any]] = deque(maxlen=self.stable_frames)
        self.frame_count = 0
        self.success_count = 0
        self.failure_count = 0

        self.detection_pub = self.create_publisher(String, detection_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 1)
        self.mask_pub = self.create_publisher(Image, mask_topic, 1)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Toy-block detector started: "
            f"image={self.image_topic}, process_hz={process_hz:.2f}, "
            f"expected={self.config.expected_width}x{self.config.expected_height}, "
            f"gate_margin={self.config.gate_margin_px}"
        )
        self.get_logger().info(
            "Current scope: one colored block, fixed camera/work area, "
            "yellow/green/cyan/blue/purple."
        )

    def _temporal_state(self, result: dict[str, Any]) -> tuple[bool, int]:
        classification = result["classification"]
        metrics = result["metrics"]

        current = {
            "color": str(classification["predicted_color"]),
            "classified": classification["status"] == "CLASSIFIED",
            "review_status": str(result["review_status"]),
            "centroid": tuple(map(float, metrics["centroid"])),
            "area": float(metrics["pixel_count"]),
        }
        self.history.append(current)

        same_color_count = 0
        for item in reversed(self.history):
            if (
                item["classified"]
                and item["review_status"] == "AUTO_OK"
                and item["color"] == current["color"]
                and current["classified"]
                and current["review_status"] == "AUTO_OK"
            ):
                same_color_count += 1
            else:
                break

        if len(self.history) < self.stable_frames:
            return False, same_color_count

        latest = self.history[-1]
        if not latest["classified"] or latest["review_status"] != "AUTO_OK":
            return False, same_color_count

        colors = {item["color"] for item in self.history}
        if len(colors) != 1:
            return False, same_color_count

        latest_cx, latest_cy = latest["centroid"]
        latest_area = max(1.0, latest["area"])

        for item in self.history:
            cx, cy = item["centroid"]
            if math.hypot(cx - latest_cx, cy - latest_cy) > self.max_center_jump_px:
                return False, same_color_count

            area_ratio = abs(item["area"] - latest_area) / latest_area
            if area_ratio > self.max_area_change_ratio:
                return False, same_color_count

            if not item["classified"] or item["review_status"] != "AUTO_OK":
                return False, same_color_count

        return True, same_color_count

    @staticmethod
    def _stamp_dict(msg: Image) -> dict[str, int]:
        return {
            "sec": int(msg.header.stamp.sec),
            "nanosec": int(msg.header.stamp.nanosec),
        }

    def _publish_detection(
        self,
        msg: Image,
        payload: dict[str, Any],
    ) -> None:
        output = String()
        output.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.detection_pub.publish(output)

    def image_callback(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_process_monotonic < self.process_period_sec:
            return
        self.last_process_monotonic = now
        self.frame_count += 1

        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
            image = np.ascontiguousarray(image)

            result, mask, _ = detect_block(image, self.config)
            stable, stable_count = self._temporal_state(result)

            # Re-render after temporal state is known.
            overlay = draw_overlay(
                image,
                mask,
                tuple(result["object_gate_xyxy_exclusive"]),
                result,
                stable=stable,
                stable_count=stable_count,
            )

            classification = result["classification"]
            metrics = result["metrics"]

            payload = {
                "stamp": self._stamp_dict(msg),
                "frame_id": msg.header.frame_id,
                "status": result["review_status"],
                "review_reasons": result["review_reasons"],
                "classification_status": classification["status"],
                "color": classification["predicted_color"],
                "color_cn": classification["predicted_color_cn"],
                "confidence": classification["confidence"],
                "stable": stable,
                "stable_count": stable_count,
                "bbox_xywh": metrics["bbox_xywh"],
                "centroid_px": metrics["centroid"],
                "angle_deg": metrics["angle_deg"],
                "mask_area_px": metrics["pixel_count"],
                "solidity": metrics["solidity"],
                "extent": metrics["extent"],
                "hue_center_opencv": classification["features"][
                    "hue_circular_center_opencv"
                ],
                "lab_a_median_opencv": classification["features"][
                    "a_median_opencv"
                ],
                "lab_b_median_opencv": classification["features"][
                    "b_median_opencv"
                ],
                "detected_bbox_xywh": result["detected_bbox_xywh"],
                "object_gate_xyxy_exclusive": result[
                    "object_gate_xyxy_exclusive"
                ],
            }
            self._publish_detection(msg, payload)

            debug_msg = self.bridge.cv2_to_imgmsg(
                overlay,
                encoding="bgr8",
            )
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(
                mask,
                encoding="mono8",
            )
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            self.success_count += 1
            if self.success_count == 1 or self.success_count % 25 == 0:
                self.get_logger().info(
                    "Detection: "
                    f"color={classification['predicted_color']} "
                    f"conf={classification['confidence']:.2f} "
                    f"status={result['review_status']} "
                    f"stable={stable} "
                    f"center={metrics['centroid']} "
                    f"angle={metrics['angle_deg']:.1f}"
                )

        except Exception as exc:  # keep the camera subscription alive
            self.failure_count += 1
            self.history.clear()
            error_text = f"{type(exc).__name__}: {exc}"

            if self.publish_failed_frames:
                payload = {
                    "stamp": self._stamp_dict(msg),
                    "frame_id": msg.header.frame_id,
                    "status": "FAILED",
                    "classification_status": "UNKNOWN",
                    "color": "unknown",
                    "color_cn": "未知",
                    "confidence": 0.0,
                    "stable": False,
                    "stable_count": 0,
                    "error": error_text,
                }
                self._publish_detection(msg, payload)

                try:
                    image = self.bridge.imgmsg_to_cv2(
                        msg,
                        desired_encoding="bgr8",
                    )
                    overlay = draw_error_overlay(
                        np.ascontiguousarray(image),
                        error_text,
                    )
                    debug_msg = self.bridge.cv2_to_imgmsg(
                        overlay,
                        encoding="bgr8",
                    )
                    debug_msg.header = msg.header
                    self.debug_pub.publish(debug_msg)
                except Exception:
                    pass

            if self.failure_count == 1 or self.failure_count % 25 == 0:
                self.get_logger().warning(
                    f"Detection failed ({self.failure_count}): {error_text}"
                )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ToyBlockColorDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
