#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
from vision_interfaces.msg import DetectionResult
from robot_interfaces.msg import EnvObjectArray, EnvObject
import time
import json


class EnvScanNode(Node):
    def __init__(self):
        super().__init__("env_scan_node")

        # 接收 env_scan 指令
        self.cmd_sub = self.create_subscription(
            String, "/voice_input/input", self.cmd_callback, 10
        )

        # 订阅 YOLO 多目标检测 (你按自己实际 topic 改)
        self.det_sub = self.create_subscription(
            DetectionResult, "/vision_target", self.on_detection, 10
        )
        
        # 订阅 world_model 世界坐标（真正可用的物体 xyz）
        self.wm_sub = self.create_subscription(
            String, "/world_model/objects", self.on_world_model, 10
        )

        # 发布最终环境结果
        self.env_pub = self.create_publisher(EnvObjectArray, "/env_objects", 10)

        # 发布暂停/恢复 face_follow
        self.face_ctrl_pub = self.create_publisher(String, "/face_follow/control", 10)

        # 订阅 face_follow 当前所在 yaw（必须动态更新）
        self.face_status_sub = self.create_subscription(
            String, "/face_follow/status", self.face_status_callback, 10
        )

        # servo 控制
        self.joints_pub = self.create_publisher(ServosPosition, "/servo_controller", 10)

        # 记录当前头部方向（来自 face_follow）
        self.current_yaw = None

        # 扫描状态
        self.scanning = False
        self.cache = []

        self.get_logger().info("🌍 env_scan_node 已启动（支持从 face_follow 读取 yaw）")

    # ========== face_follow 的 yaw/pitch 同步 ==========
    def face_status_callback(self, msg: String):
        # 格式：yaw:656,pitch:354
        parts = msg.data.split(",")
        for p in parts:
            if p.startswith("yaw:"):
                self.current_yaw = int(p.split(":")[1])
        # self.get_logger().info(f"同步 face_follow yaw={self.current_yaw}")

    # ========== 接收扫描命令 ==========
    def cmd_callback(self, msg: String):
        if msg.data.strip() != "env_scan":
            return

        self.get_logger().info("🔍 收到 env_scan 指令 → 准备进入扫描流程")
        self.start_scan()

    # ========== 暂停 face_follow ==========
    def pause_face_follow(self):
        msg = String()
        msg.data = "pause_for_action"
        self.face_ctrl_pub.publish(msg)
        self.get_logger().info("⏸ 已暂停面部跟随")
        time.sleep(0.2)

    # ========== 恢复 face_follow ==========
    def resume_face_follow(self):
        msg = String()
        msg.data = "resume"
        self.face_ctrl_pub.publish(msg)
        self.get_logger().info("▶ 已恢复面部跟随")

    # ========== 扫描入口 ==========
    def start_scan(self):
        self.pause_face_follow()
        
        if self.current_yaw is None:
            self.get_logger().warn("⚠ 未收到 face_follow yaw，使用默认中心 500")
            self.current_yaw = 500

        self.scanning = True
        self.cache = []
        self.scan_start_ts = time.time() #增加了时间戳

        # 基于当前 yaw 生成扫描路线
        start = self.current_yaw
        left  = 300
        right = 700

        # 按“从当前 → 最左 → 最右”排序
        if start < 500:
            self.scan_points = [start, left, right]
        else:
            self.scan_points = [start, right, left]

        self.get_logger().info(f"📡 当前 yaw={start}, 扫描路线={self.scan_points}")

        self.step = 0
        self.move_next_point()

    # ========== 舵机平滑运动 ==========
    def ease_move(self, servo_id, start, end, duration):
        steps = 20
        for i in range(steps):
            t = i / (steps - 1)
            ease = 3*t*t - 2*t*t*t
            pos = int(start + (end - start) * ease)

            set_servo_position(
                self.joints_pub,
                duration/steps,
                ((servo_id, pos),)
            )
            time.sleep(duration/steps)

    # ========== 转动到扫描点 ==========
    def move_next_point(self):
        if self.step >= len(self.scan_points):
            self.finish_scan()
            return

        target = self.scan_points[self.step]
        self.step += 1

        self.get_logger().info(f"📍 扫描移动至 yaw={target}")

        self.ease_move(1, self.current_yaw, target, duration=1.2)
        self.current_yaw = target

        # 等待稳定 & YOLO 输出
        time.sleep(0.5)

        # 继续下一个点
        self.move_next_point()
    
    # ========== 接收 world_model 的世界坐标 ==========
    def on_world_model(self, msg: String):
        if not self.scanning:
            return

        try:
            data = json.loads(msg.data)
            objects = data.get("objects", [])
        except Exception as e:
            self.get_logger().warn(f"JSON 解析失败: {e}")
            return

        for obj in objects:
            #ts = obj.get("updated_at", 0)
            
            #if ts < self.scan_start_ts:
            #    continue  # 过滤扫描前的数据

            try:
                self.cache.append({
                    "label": obj["class_name"],
                    "x": float(obj["pose"]["xyz"][0]),
                    "y": float(obj["pose"]["xyz"][1]),
                    "z": float(obj["pose"]["xyz"][2]),
                    "conf": float(obj.get("confidence", 1.0))
                })
            except Exception as e:
                self.get_logger().warn(f"world_model 对象解析失败: {e}")

    # ========== YOLO 检测回调 ==========
    def on_detection(self, msg: DetectionResult):
        pass

    # ========== 扫描结束 ==========
    def finish_scan(self):
        self.scanning = False

        # 去重
        uniq = {}
        for o in self.cache:
            key = (o["label"], round(o["x"], 1), round(o["y"], 1))
            uniq[key] = o

        out = EnvObjectArray()
        for k, o in uniq.items():
            obj = EnvObject()
            obj.label = o["label"]
            obj.pose.position.x = float(o["x"])
            obj.pose.position.y = float(o["y"])
            obj.pose.position.z = float(o["z"])
            out.objects.append(obj)

        self.env_pub.publish(out)

        self.get_logger().info(f"扫描完成，发现 {len(out.objects)} 个目标")

        self.resume_face_follow()


def main():
    rclpy.init()
    node = EnvScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

