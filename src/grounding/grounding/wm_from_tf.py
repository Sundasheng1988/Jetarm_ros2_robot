#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, math, zlib, re
from typing import Dict, Set, List
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import tf2_ros
from rclpy.time import Time

def quat_to_rpy(qx,qy,qz,qw):
    # ZYX
    sinr_cosp = 2*(qw*qx + qy*qz); cosr_cosp = 1 - 2*(qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2*(qw*qy - qz*qx)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2*(qw*qz + qx*qy); cosy_cosp = 1 - 2*(qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]

def guess_class_and_color(frame_id: str) -> (str, str):
    s = frame_id.lower()
    color = "yellow" if "yellow" in s or "huang" in s else \
            "blue"   if "blue"   in s or "lan"   in s else \
            "red"    if "red"    in s or "hong"  in s else \
            "green"  if "green"  in s or "lv"    in s else \
            "black"  if "black"  in s else \
            "white"  if "white"  in s else ""
    cls = "cup"  if "cup"  in s or "bei"  in s else \
          "ball" if "ball" in s or "qiu"  in s else \
          "bottle" if "bottle" in s else \
          "box" if "box" in s else ""
    return cls, color

def stable_id(name: str) -> int:
    return int(zlib.crc32(name.encode("utf-8")) & 0xffffffff) % 100000

class WMFromTF(Node):
    def __init__(self):
        super().__init__("wm_from_tf")
        self.pub = self.create_publisher(String, "/world_model/objects", 10)

        # 参数
        self.declare_parameter("world_frame", "table")
        self.declare_parameter("frame_prefixes", ["cup", "ball", "obj"])
        self.declare_parameter("explicit_frames", [])
        self.declare_parameter("rate_hz", 5.0)

        self.world = self.get_parameter("world_frame").get_parameter_value().string_value
        self.prefixes: List[str] = [p.lower() for p in self.get_parameter("frame_prefixes").value]
        self.explicit: List[str] = self.get_parameter("explicit_frames").value

        # 采集 TF 中的 child_frame_id
        self.frames: Set[str] = set(self.explicit)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, 10)

        self.tfbuf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.listener = tf2_ros.TransformListener(self.tfbuf, self)

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self.tick)
        self.get_logger().info(f"[wm_from_tf] world_frame={self.world} prefixes={self.prefixes} explicit={self.explicit}")

    def on_tf(self, msg: TFMessage):
        for t in msg.transforms:
            self.frames.add(t.child_frame_id)

    def _is_object_frame(self, name: str) -> bool:
        s = name.lower()
        return any(s.startswith(p) or f"_{p}_" in s or s.endswith(p) for p in self.prefixes)

    def tick(self):
        objs = []
        now = Time()
        for f in list(self.frames):
            if not self._is_object_frame(f):
                continue
            try:
                tr = self.tfbuf.lookup_transform(self.world, f, now, timeout=rclpy.duration.Duration(seconds=0.05))
                t = tr.transform.translation
                q = tr.transform.rotation
                rpy = quat_to_rpy(q.x,q.y,q.z,q.w)
                cls, color = guess_class_and_color(f)
                # 若识别不到类别，跳过（Grounding 需要 class_name）
                if not cls:
                    continue
                objs.append({
                    "id": stable_id(f),
                    "class_name": cls,
                    "color": color,
                    "pose": {
                        "frame": self.world,
                        "xyz": [round(t.x,4), round(t.y,4), round(t.z,4)],
                        "rpy": [round(rpy[0],3), round(rpy[1],3), round(rpy[2],3)]
                    },
                    "confidence": 0.9,
                    "updated_at": self.get_clock().now().nanoseconds/1e9
                })
            except Exception:
                # 查不到就略过
                pass

        if objs:
            self.pub.publish(String(data=json.dumps({"objects": objs}, ensure_ascii=False)))

def main(args=None):
    rclpy.init(args=args)
    node = WMFromTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
