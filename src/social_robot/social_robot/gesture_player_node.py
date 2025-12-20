#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
import time


class GesturePlayerNode(Node):
    def __init__(self):
        super().__init__("gesture_player")

        # 接收动作指令
        self.cmd_sub = self.create_subscription(
            String,
            "/gesture/cmd",
            self.cmd_callback,
            10
        )

        # 控制 face_follow (pause/resume)
        self.face_ctrl_pub = self.create_publisher(String, "/face_follow/control", 10)

        # 舵机控制
        self.joints_pub = self.create_publisher(
            ServosPosition,
            "/servo_controller",
            10
        )
        
        self.face_status_sub = self.create_subscription(
            String,
            "/face_follow/status",
            self.status_callback,
            10
        )

        self.get_logger().info("🤖 gesture_player_node 已启动（安全模式：点头 nod）")
        
    def status_callback(self, msg: String):
        # 例：msg.data = "yaw:656,pitch:354"
        parts = msg.data.split(",")
        for p in parts:
            if p.startswith("yaw:"):
                self.current_yaw = int(p.split(":")[1])
            if p.startswith("pitch:"):
                self.current_pitch = int(p.split(":")[1])

        self.get_logger().info(
            f"📡 已同步 face_follow 当前 yaw={self.current_yaw} pitch={self.current_pitch}"
        )


    # ----------------------------------------------------------
    # 接收动作命令
    # ----------------------------------------------------------
    def cmd_callback(self, msg: String):
        cmd = msg.data.strip()

        if cmd == "nod":
            self.do_nod()
            
        elif cmd == "shake":
            self.do_shake()
        
        else:
            self.get_logger().warn(f"未知动作：{cmd}")

    # ----------------------------------------------------------
    # 点头动作（ID4 上下移动）
    # ----------------------------------------------------------
    def do_nod(self):
        self.get_logger().info("👉 执行动作：点头 nod")

        # 1) 暂停 face_follow
        self.pause_face_follow()

        # 2) 安全执行动作
        self.nod_action()

        # 3) 恢复 face_follow
        self.resume_face_follow()
    
    def do_shake(self):
        self.get_logger().info("👉 执行动作：摇头 shake")

        self.pause_face_follow()
        self.shake_action()
        self.resume_face_follow()

    # ----------------------------------------------------------
    def pause_face_follow(self):
        msg = String()
        msg.data = "pause_for_action"
        self.face_ctrl_pub.publish(msg)
        self.get_logger().info("⏸ 已暂停面部跟随")
        time.sleep(0.2)

    def resume_face_follow(self):
        msg = String()
        msg.data = "resume"
        self.face_ctrl_pub.publish(msg)
        self.get_logger().info("▶ 已恢复面部跟随")
        time.sleep(0.3)

    # ----------------------------------------------------------
    # 安全点头动作（自动限幅 + 小步移动）
    # ----------------------------------------------------------
    def safe_move(self, servo_id, target, duration=0.5):
        """
        安全舵机移动（限制范围 + 固定速度）
        """

        # 限幅保护
        if servo_id == 4:             # pitch
            target = max(100, min(740, target))
        elif servo_id == 1:           # yaw
            target = max(0, min(1000, target))

        # 下发动作
        set_servo_position(
            self.joints_pub,
            duration,
            ((servo_id, int(target)),)
        )
        time.sleep(duration)
        
        self.get_logger().info(f"[MOVE] servo {servo_id}: target={target}, duration={duration}")
        
    def ease_move(self, servo_id, start, end, duration):
        """
                 让舵机使用 EaseInOut（慢-快-慢）曲线平滑运动
        """
        steps = 20
        for i in range(steps):
            t = i / (steps - 1)
            # cubic ease-in-out
            ease = 3*t*t - 2*t*t*t
            pos = int(start + (end - start) * ease)

            set_servo_position(
                self.joints_pub,
                duration/steps,
                ((servo_id, pos),)
            )
            time.sleep(duration/steps)

    # ----------------------------------------------------------
    # 点头动作：中 → 低 → 高 → 中
    # ----------------------------------------------------------
    def nod_action(self):
        """
        超自然点头：使用 ease_move 进行平滑插值
        轨迹：中 -> 下 -> 上 -> 中
        """
        pitch_id = 4
        center = 350

        # 幅度（可微调）
        down = center - 80     # 350 → 310（自然点头）
        up   = center + 80     # 350 → 380（小幅抬头）

        # 每段动作的时间
        seg_time = 0.3   # 三段共 0.66s，人类正常点头是 0.6–0.9 秒

        self.get_logger().info("🤖 [NOD] 开始自然点头动作")

        # ---- 1）中 -> 下 ----
        self.ease_move(pitch_id, center, down, seg_time)

        # ---- 2）下 -> 上 ----
        self.ease_move(pitch_id, down, up, seg_time)

        # ---- 3）上 -> 中 ----
        self.ease_move(pitch_id, up, center, seg_time)
       

        self.get_logger().info("🤖 点头完成（自然模式）")
    
    def shake_action(self):
        yaw_id = 1
        # === 关键修复：以当前 yaw 为摇头中心 ===
        center = self.current_yaw if hasattr(self, "current_yaw") else 500

        # 人类自然摇头幅度：左右各 100（不会太夸张）
        left  = center - 80    # 500 → 380
        right = center + 80    # 500 → 620

        seg = 0.35  # 更快一点显得更自然（0.8 秒左右一整个摇头）

        self.get_logger().info("🤖 [SHAKE] 开始摇头动作")

        # ---- 中 → 左 ----
        self.ease_move(yaw_id, center, left, seg)

        # ---- 左 → 右 ----
        self.ease_move(yaw_id, left, right, seg*1.3)

        # ---- 右 → 左 ----
        self.ease_move(yaw_id, right, left, seg*1.3)

        # ---- 左 → 中 ----
        self.ease_move(yaw_id, left, center, seg)

        self.get_logger().info("🤖 摇头完成（自然模式）")


def main():
    rclpy.init()
    node = GesturePlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

