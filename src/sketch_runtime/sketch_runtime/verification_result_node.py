import rclpy
import json
import time
import math
import threading

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String, Bool

from sketch_runtime.verification_result import VerificationResult, extract_source_xyz


class VerificationResultNode(Node):

    def __init__(self):
        super().__init__("verification_result_node")

        # Declare parameters with defaults
        self.declare_parameter("dry_run", True)
        self.declare_parameter("enable_precheck", True)
        self.declare_parameter("enable_postcheck", True)
        self.declare_parameter("distance_threshold", 0.1)
        self.declare_parameter("postcheck_delay_sec", 1.0)

        self.dry_run: bool = self.get_parameter("dry_run").value
        self.enable_precheck: bool = self.get_parameter("enable_precheck").value
        self.enable_postcheck: bool = self.get_parameter("enable_postcheck").value
        self.distance_threshold: float = self.get_parameter("distance_threshold").value
        self.postcheck_delay_sec: float = self.get_parameter("postcheck_delay_sec").value

        # State
        self._pending_precheck = None
        self._pending_place_check = None
        self._last_stable_objects = None
        self._lock = threading.Lock()

        # Subscription for /grounded_task_context (precheck trigger)
        qos_precheck = rclpy.qos.QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription_task_context = self.create_subscription(
            String,
            "/grounded_task_context",
            self.task_context_callback,
            qos_profile=qos_precheck,
        )

        # Subscription for /world_model/stable_objects (observation feed)
        self.subscription_stable_objects = self.create_subscription(
            String,
            "/world_model/stable_objects",
            self.stable_objects_callback,
            qos_profile=qos_profile_sensor_data,
        )

        # Subscription for /executor/done (postcheck trigger)
        qos_executor = rclpy.qos.QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription_executor_done = self.create_subscription(
            Bool,
            "/executor/done",
            self.executor_done_callback,
            qos_profile=qos_executor,
        )

        # Publication for /runtime/verification_result
        self.publisher = self.create_publisher(
            String,
            "/runtime/verification_result",
            10,
        )

        # Postcheck timer
        self._postcheck_timer = None

    def task_context_callback(self, msg: String):
        if not self.enable_precheck:
            return

        try:
            context = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Failed to parse grounded_task_context JSON")
            return

        status = context.get("status", "")
        intent = context.get("intent", "")

        if status != "ok" or intent not in ("pick", "grasp", "place"):
            return

        target_object = context.get("target_object", {})
        target_class = target_object.get("class_name", "")
        target_color = target_object.get("color", "")

        # ── place intent: store pending, no precheck publish ──
        if intent == "place":
            target_pose = context.get("target_pose", {})
            tgt_xyz_raw = target_pose.get("xyz", [0.0, 0.0, 0.0])
            try:
                target_xyz = [float(tgt_xyz_raw[0]), float(tgt_xyz_raw[1]), float(tgt_xyz_raw[2])]
            except (TypeError, IndexError, ValueError):
                target_xyz = [0.0, 0.0, 0.0]

            with self._lock:
                self._pending_precheck = None
                self._pending_place_check = {
                    "context": context,
                    "target_class": target_class,
                    "target_color": target_color,
                    "target_xyz": target_xyz,
                }
            self.get_logger().info(
                f"Place context stored: class={target_class}, target_xyz={target_xyz}"
            )
            return

        xyz = extract_source_xyz(target_object)

        with self._lock:
            self._pending_place_check = None

            if self._last_stable_objects is None:
                result = VerificationResult(
                    stage="precheck",
                    success=False,
                    confidence=0.0,
                    reason="no_stable_objects_received",
                    evidence={
                        "target_class": target_class,
                        "target_color": target_color,
                        "source_xyz": xyz,
                        "found_objects": [],
                        "best_match_found": False,
                        "min_distance": float("inf"),
                        "total_stable_objects": 0,
                    },
                )
                self._pending_precheck = None
                self._publish(result)
                return

            stable_objects_list = self._parse_stable_objects(self._last_stable_objects)
            candidates = self._find_matching_objects(
                stable_objects_list, target_class, target_color, xyz, self.distance_threshold
            )

            evidence = self._build_precheck_evidence(
                candidates, target_class, target_color, xyz, stable_objects_list
            )

            success = len(candidates) > 0
            confidence = 1.0 if success else 0.0

            reason = "object_found" if success else "no_matching_object_near_source"

            precheck_result = VerificationResult(
                stage="precheck",
                success=success,
                confidence=confidence,
                reason=reason,
                evidence=evidence,
            )

            if success:
                self._pending_precheck = {
                    "context": context,
                    "precheck": precheck_result,
                    "timestamp": time.time(),
                }
            else:
                self._pending_precheck = None

            self._publish(precheck_result)
            self.get_logger().info(
                f"Precheck: success={success}, reason={reason}, found={len(candidates)}"
            )

    def stable_objects_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            objects_list = data if isinstance(data, list) else data.get("objects", [])
        except (json.JSONDecodeError, TypeError, AttributeError):
            objects_list = []
        with self._lock:
            self._last_stable_objects = json.dumps(objects_list)

    def executor_done_callback(self, msg: Bool):
        if not self.enable_postcheck:
            return

        if not msg.data:
            return

        # ── place post_place check ──
        if self._pending_place_check is not None:
            if self._postcheck_timer is not None:
                self._postcheck_timer.cancel()
                self.destroy_timer(self._postcheck_timer)
                self._postcheck_timer = None
            self._postcheck_timer = self.create_timer(
                self.postcheck_delay_sec, self._run_post_place_check
            )
            return

        if self._pending_precheck is None:
            return

        if self._postcheck_timer is not None:
            self._postcheck_timer.cancel()
            self.destroy_timer(self._postcheck_timer)
            self._postcheck_timer = None

        self._postcheck_timer = self.create_timer(
            self.postcheck_delay_sec, self._run_postcheck
        )

    def _run_postcheck(self):
        postcheck_result = None
        object_still_at_source = None
        success = None
        reason = None

        with self._lock:
            pending = self._pending_precheck
            self._pending_precheck = None

            if pending is not None:
                precheck_result = pending.get("precheck")

                if (precheck_result is not None
                        and self._last_stable_objects is not None):
                    target_class = precheck_result.evidence.get("target_class", "")
                    target_color = precheck_result.evidence.get("target_color", "")
                    source_xyz = precheck_result.evidence.get("source_xyz", [0.0, 0.0, 0.0])

                    stable_objects_list = self._parse_stable_objects(self._last_stable_objects)
                    candidates = self._find_matching_objects(
                        stable_objects_list, target_class, target_color, source_xyz, self.distance_threshold
                    )

                    evidence = self._build_postcheck_evidence(
                        candidates, target_class, target_color, source_xyz, stable_objects_list
                    )

                    object_still_at_source = evidence.get("object_still_at_source", True)
                    success = not object_still_at_source

                    reason = (
                        "object_no_longer_at_source"
                        if success
                        else "object_still_at_source"
                    )

                    postcheck_result = VerificationResult(
                        stage="postcheck",
                        success=success,
                        confidence=1.0 if success else 0.0,
                        reason=reason,
                        evidence=evidence,
                    )

        self._clear_postcheck_timer()

        if postcheck_result is not None:
            self._publish(postcheck_result)
            self.get_logger().info(
                f"Postcheck: success={success}, reason={reason}, still_at_source={object_still_at_source}"
            )

    # ── post_place (place intent) ──

    def _run_post_place_check(self):
        postcheck_result = None
        success = None
        reason = None

        with self._lock:
            pending = self._pending_place_check
            self._pending_place_check = None

            if pending is not None and self._last_stable_objects is not None:
                target_class = pending.get("target_class", "")
                target_color = pending.get("target_color", "")
                target_xyz = pending.get("target_xyz", [0.0, 0.0, 0.0])

                stable_objects_list = self._parse_stable_objects(self._last_stable_objects)
                candidates = self._find_matching_objects(
                    stable_objects_list, target_class, target_color,
                    target_xyz, self.distance_threshold
                )

                evidence = self._build_post_place_evidence(
                    candidates, target_class, target_color, target_xyz,
                    stable_objects_list
                )

                object_found = len(candidates) > 0
                success = object_found
                reason = (
                    "object_found_at_target"
                    if success
                    else "object_not_found_at_target"
                )

                postcheck_result = VerificationResult(
                    stage="post_place",
                    success=success,
                    confidence=1.0 if success else 0.0,
                    reason=reason,
                    evidence=evidence,
                )

        self._clear_postcheck_timer()

        if postcheck_result is not None:
            self._publish(postcheck_result)
            self.get_logger().info(
                f"Post_place: success={success}, reason={reason}"
            )

    def _build_post_place_evidence(
        self,
        candidates_near: list,
        target_class: str,
        target_color: str,
        target_xyz: list,
        stable_objects_list: list,
    ) -> dict:
        return {
            "target_class": target_class,
            "target_color": target_color,
            "target_xyz": target_xyz,
            "object_found_at_target": len(candidates_near) > 0,
            "objects_near_target": candidates_near,
            "total_stable_objects": len(stable_objects_list),
        }

    def _clear_postcheck_timer(self):
        """Destroy and reset the postcheck timer."""
        if self._postcheck_timer is not None:
            try:
                self.destroy_timer(self._postcheck_timer)
            except Exception:
                pass
            self._postcheck_timer = None

    def _publish(self, result: VerificationResult):
        msg = String()
        msg.data = json.dumps(result.to_dict())
        self.publisher.publish(msg)

    def _euclidean_distance(self, a: list, b: list) -> float:
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

    def _parse_stable_objects(self, data_str: str) -> list:
        try:
            data = json.loads(data_str)
            if isinstance(data, list):
                return data
            if isinstance(data, dict) and "objects" in data:
                return data["objects"]
        except (json.JSONDecodeError, TypeError, AttributeError):
            pass
        return []

    def _find_matching_objects(
        self,
        stable_objects_list: list,
        class_name: str,
        color: str,
        source_xyz: list,
        distance_threshold: float,
    ) -> list:
        candidates = []

        for obj in stable_objects_list:
            obj_class = obj.get("class_name", "")
            obj_color = obj.get("color", "")
            obj_xyz = obj.get("pose", {}).get("xyz", [0.0, 0.0, 0.0])

            # Priority 1: match by object_id (stable_objects doesn't have this field, skip)

            # Priority 2: match by class_name (case-insensitive)
            if class_name and obj_class.lower() != class_name.lower():
                continue

            # Color is optional — ignored if "unknown" or ""
            if color and color.lower() not in ("unknown", ""):
                if obj_color.lower() != color.lower():
                    continue

            # Compute distance
            distance = self._euclidean_distance(obj_xyz, source_xyz)

            if distance <= distance_threshold:
                candidates.append({
                    "class_name": obj_class,
                    "color": obj_color,
                    "distance": round(distance, 4),
                    "track_id": obj.get("track_id", None),
                    "xyz": obj_xyz,
                })

        return candidates

    def _build_precheck_evidence(
        self,
        candidates: list,
        target_class: str,
        target_color: str,
        source_xyz: list,
        stable_objects_list: list,
    ) -> dict:
        min_dist = min((c["distance"] for c in candidates), default=float("inf"))
        return {
            "target_class": target_class,
            "target_color": target_color,
            "source_xyz": source_xyz,
            "found_objects": candidates,
            "best_match_found": len(candidates) > 0,
            "min_distance": min_dist,
            "total_stable_objects": len(stable_objects_list),
        }

    def _build_postcheck_evidence(
        self,
        candidates_near: list,
        target_class: str,
        target_color: str,
        source_xyz: list,
        stable_objects_list: list,
    ) -> dict:
        return {
            "target_class": target_class,
            "target_color": target_color,
            "source_xyz": source_xyz,
            "object_still_at_source": len(candidates_near) > 0,
            "objects_near_source": candidates_near,
            "total_stable_objects": len(stable_objects_list),
        }


def main(args=None):
    rclpy.init(args=args)
    node = VerificationResultNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
