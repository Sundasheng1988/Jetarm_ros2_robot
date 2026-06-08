#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robotops.event_store import EventStore


class RobotOpsRecorderNode(Node):
    def __init__(self):
        super().__init__("robotops_recorder_node")

        default_db = str(Path.home() / ".ros" / "robotops.db")
        self.declare_parameter("db_path", default_db)

        db_path = str(self.get_parameter("db_path").value)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f" RobotOpsRecorderNode — db_path={db_path}")
        self.get_logger().info(" Subscribing to /runtime/state, /runtime/log, "
                               "/runtime/execution_result, /runtime/verification_result")
        self.get_logger().info("=" * 60)

        try:
            self.store = EventStore(db_path)
        except Exception as e:
            self.get_logger().error(f"Failed to open database: {e}")
            raise

        self.create_subscription(
            String, "/runtime/state", self._on_state, 10
        )
        self.create_subscription(
            String, "/runtime/log", self._on_log, 10
        )
        self.create_subscription(
            String, "/runtime/execution_result", self._on_execution_result, 10
        )
        self.create_subscription(
            String, "/runtime/verification_result", self._on_verification_result, 10
        )

    def _on_state(self, msg: String):
        self._persist("/runtime/state", msg.data)

    def _on_log(self, msg: String):
        event_id = None
        try:
            data = json.loads(msg.data)
            event_id = data.get("event_id")
        except Exception:
            pass
        self._persist("/runtime/log", msg.data, event_id=event_id)

    def _on_execution_result(self, msg: String):
        self._persist("/runtime/execution_result", msg.data)

    def _on_verification_result(self, msg: String):
        self._persist("/runtime/verification_result", msg.data)

    def _persist(self, source_topic: str, payload: str,
                 event_id: str = None):
        try:
            data = json.loads(payload)
            task_id = data.get("task_id") or "unknown_task"
        except Exception:
            task_id = "unknown_task"

        try:
            self.store.save_event(source_topic, task_id, payload, event_id=event_id)
        except Exception as e:
            self.get_logger().warn(
                f"[robotops] failed to persist {source_topic}: {e}"
            )

    def destroy_node(self):
        self.store.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotOpsRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
