# static_env_report_node
#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import EnvObjectArray, EnvObject
from servo_controller_msgs.msg import ServosPosition
import time
import json

def merge_objects(objects, z_thresh=0.08):
    """
    将 world_model 的原始 objects 合并为逻辑物体
    - 同 label
    - z 距离小于阈值
    """
    merged = []

    # 按 z 从近到远排序，优先保留近的
    objects = sorted(objects, key=lambda o: o["z"])

    for obj in objects:
        found = False
        for m in merged:
            if (
                obj["label"] == m["label"]
                and abs(obj["z"] - m["z"]) < z_thresh
            ):
                # 合并：取最近 + 最大置信度
                m["z"] = min(m["z"], obj["z"])
                m["confidence"] = max(m["confidence"], obj["confidence"])
                found = True
                break

        if not found:
            merged.append(obj.copy())

    return merged



class StaticEnvReportNode(Node):
    def __init__(self):
        super().__init__("static_env_report_node")

        self.get_logger().info("📌 静态环境识别节点已启动（不移动机械臂，只识别面前场景）")

        # ==== 指令输入 ====
        self.cmd_sub = self.create_subscription(
            String, "/voice_input/input", self.on_cmd, 10
        )

        # ==== YOLO/world_model 输入 ====
        self.wm_sub = self.create_subscription(
            String, "/world_model/objects", self.on_world_model, 10
        )

        # ==== 发布环境对象 ====
        self.env_pub = self.create_publisher(EnvObjectArray, "/env_objects", 10)

        # ==== 控制 face_follow ====
        self.face_ctrl_pub = self.create_publisher(String, "/face_follow/control", 10)

        # ==== 状态 ====
        self.collecting = False       # 是否处于采集模式
        self.cache = []               # 缓存 world_model 的对象
        self.start_ts = 0             # 时间戳 window

    # ============================================================
    # 收到 static_env_report 语音命令
    # ============================================================
    def on_cmd(self, msg: String):
        text = msg.data.strip()

        if text != "static_env_report":
            return

        self.get_logger().info("🔍 收到 static_env_report → 开始静态环境识别")
        self.start_collect()

    # ============================================================
    # 暂停 face_follow（不移动机械臂）
    # ============================================================
    def pause_face_follow(self):
        msg = String()
        msg.data = "pause_for_action"
        self.face_ctrl_pub.publish(msg)
        self.get_logger().info("⏸ 已暂停 face_follow（头部保持不动）")

    # ============================================================
    # 恢复 face_follow
    # ============================================================
    def resume_face_follow(self):
        msg = String()
        msg.data = "resume"
        self.face_ctrl_pub.publish(msg)
        self.get_logger().info("▶ 已恢复 face_follow")

    # ============================================================
    # 启动静态采集
    # ============================================================
    def start_collect(self):
        self.pause_face_follow()
        
         # ✅ 新增：如果上一次 timer 还存在，先清理
        if hasattr(self, "finish_timer") and self.finish_timer is not None:
            self.finish_timer.cancel()
            self.finish_timer = None

        # 清空缓存
        self.cache = []
        self.collecting = True
        self.start_ts = time.time()

        self.get_logger().info("📸 正在采集 world_model 物体信息…")

        # 0.7 秒采集窗口，确保 YOLO/world_model 有足够时间输出
        # self.create_timer(0.7, self.finish_collect)
        self.finish_timer = self.create_timer(0.7, self.finish_collect)

    # ============================================================
    # world_model 输入
    # ============================================================
    def on_world_model(self, msg: String):
        if not self.collecting:
            return

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"JSON 解析失败: {e}")
            return

        objects = data.get("objects", [])
        if not objects:
            return

        for obj in objects:
            try:
                conf = float(obj.get("confidence", 0.0))
                if conf <= 0.0:
                    continue

                x, y, z = obj["pose"]["xyz"]
                self.cache.append({
                    "label": obj["class_name"],
                    "x": float(x),
                    "y": float(y),
                    "z": float(z),
                    "confidence": conf,
                })
            except Exception as e:
                self.get_logger().warn(f"解析对象失败: {e}")


    # ============================================================
    # 结束采集
    # ============================================================
    def finish_collect(self):
        if not self.collecting:
            return

        self.collecting = False

        # 去重逻辑（与 env_scan_node 一致）
        
        #// 25/12/14  删除这一整段
        #uniq = {}
        #for o in self.cache:
        #    key = (o["label"], round(o["x"], 1), round(o["y"], 1))
        #    
        #    if key not in uniq:
        #        uniq[key] = o
        #    else:
        #        uniq[key]["confidence"] = max(
        #            uniq[key]["confidence"],
        #            o.get("confidence", 0.0)
        #        )
              
        #for _, o in uniq.items():
        #    obj = EnvObject()
        #    obj.label = o["label"]
        #    obj.pose.position.x = o["x"]
        #    obj.pose.position.y = o["y"]
        #    obj.pose.position.z = o["z"]
        #    obj.confidence = o.get("confidence", 0.0)   # ✅ 关键一行
        #    out.objects.append(obj)
        #//
        
        # ===== 新的逻辑物体合并 =====
        merged = merge_objects(self.cache)
        
        out = EnvObjectArray()
        
        for o in merged:
            obj = EnvObject()
            obj.label = o["label"]
            obj.pose.position.x = o["x"]
            obj.pose.position.y = o["y"]
            obj.pose.position.z = o["z"]
            obj.confidence = o.get("confidence", 0.0)
            out.objects.append(obj)

        # 发布最终环境结果
        self.env_pub.publish(out)

        self.get_logger().info(
            f"📢 静态识别完成，共发现 {len(out.objects)} 个物体: {[o.label for o in out.objects]}"
        )

        # 恢复 face_follow
        self.resume_face_follow()
        
        if self.finish_timer is not None:
            self.finish_timer.cancel()
            self.finish_timer = None


def main():
    rclpy.init()
    node = StaticEnvReportNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

