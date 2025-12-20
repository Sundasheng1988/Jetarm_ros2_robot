# world_model_node
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
from typing import List

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import EnvObject, EnvObjectArray


def dist(p1, p2):
    return math.sqrt(
        (p1.x - p2.x) ** 2 +
        (p1.y - p2.y) ** 2 +
        (p1.z - p2.z) ** 2
    )


class WorldObject:
    """内部世界模型对象（非 ROS msg）"""
    def __init__(self, obj: EnvObject):
        self.label = obj.label
        self.pose = obj.pose
        self.confidence = obj.confidence
        self.last_seen = time.time()
        self.hit_count = 1

    def update(self, obj: EnvObject):
        self.pose = obj.pose
        self.confidence = max(self.confidence, obj.confidence)
        self.last_seen = time.time()
        self.hit_count += 1


class WorldModelNode(Node):
    def __init__(self):
        super().__init__('world_model_node')

        # ===== 参数 =====
        self.merge_distance = self.declare_parameter(
            'merge_distance', 0.06   # 米，同一物体的空间阈值
        ).value

        self.min_hits = self.declare_parameter(
            'min_hits', 3            # 连续命中多少次才认为“存在”
        ).value

        self.ttl = self.declare_parameter(
            'ttl', 1.5               # 秒，多久没看到就移除
        ).value

        # ===== 世界状态 =====
        self.world_objects: List[WorldObject] = []

        # ===== ROS IO =====
        self.sub_env = self.create_subscription(
            EnvObjectArray,
            '/env_objects',
            self._on_env_objects,
            10
        )

        self.pub_world = self.create_publisher(
            EnvObjectArray,
            '/world_objects',
            10
        )

        # 定时发布稳定世界
        self.timer = self.create_timer(0.2, self._publish_world)

        self.get_logger().info(
            '🌍 world_model_node 已启动 '
            f'(merge_distance={self.merge_distance}m, '
            f'min_hits={self.min_hits}, ttl={self.ttl}s)'
        )

    def _on_env_objects(self, msg: EnvObjectArray):
        now = time.time()

        for obj in msg.objects:
            matched = False

            for wo in self.world_objects:
                if wo.label != obj.label:
                    continue

                d = dist(wo.pose.position, obj.pose.position)
                if d < self.merge_distance:
                    wo.update(obj)
                    matched = True
                    break

            if not matched:
                self.world_objects.append(WorldObject(obj))

        # 清理超时对象
        self.world_objects = [
            wo for wo in self.world_objects
            if (now - wo.last_seen) <= self.ttl
        ]

    def _publish_world(self):
        msg = EnvObjectArray()

        for wo in self.world_objects:
            if wo.hit_count < self.min_hits:
                continue

            obj = EnvObject()
            obj.label = wo.label
            obj.confidence = wo.confidence
            obj.pose = wo.pose
            msg.objects.append(obj)

        
        self.pub_world.publish(msg)
        self.get_logger().debug(
            f'发布 world_objects: {len(msg.objects)} 个'
        )


def main():
    rclpy.init()
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
