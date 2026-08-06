# FaceFollowNode v1.0-stable
# 2025-12-14
# 独立运行，不依赖语音节点

#!/usr/bin/env python3
# encoding: utf-8
# 只做人脸跟踪的安全版节点（基于原厂 FaceTracker 逻辑）

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np

# ★ 原厂依赖：使用 servo_controller 的总线舵机接口
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position

# ★ 原厂人脸跟踪算法：直接复用 app.face_tracker.FaceTracker
import social_robot.face_tracker as face_tracker


class FaceFollowNode(Node):
    def __init__(self):
        super().__init__('face_follow')

        # 参数：摄像头 topic，可在 launch 里重映射
        self.declare_parameter('camera_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('display', False)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.display = bool(self.get_parameter('display').value)

        # 图像转换
        self.bridge = CvBridge()

        # ★ 舵机控制：和原 object_tracking 一样，走 /servo_controller
        self.joints_pub = self.create_publisher(
            ServosPosition,
            '/servo_controller',
            1
        )

        # ★ 原厂人脸跟踪器（里面已经包含 PID、人脸中心计算等完整逻辑）
        self.tracker = face_tracker.FaceTracker()
        
        # === 新增：记录当前舵机位置（无反馈系统必须自己记） ===
        self.last_pitch = 350   # 启动时默认与 goback 对齐
        self.last_yaw = 500

        # ★ 启动时回到原厂 goback 安全姿态
        self.goback()

        # 订阅摄像头图像
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            1
        )
        
        # Pause/Resume 控制
        self.paused = True
        
        # ========= 新增：等待语音节点启动 =========
        # ========= 25/12/14：# 预留接口：未来如需重新绑定语音系统=========
        self.voice_ready = True
        self.get_logger().info("🤖 FaceFollowNode 已启动 ")

        self.control_sub = self.create_subscription(
            String,
            '/face_follow/control',
            self.control_callback,
            10
        )

        self.get_logger().info('🚀 原厂 Face Follow Node 已启动（只做人脸跟踪 + goback 姿态）')
        
        self.status_pub = self.create_publisher(String, "/face_follow/status", 10)
        
        # 25/12/14新增
        self.last_send_ts = 0.0
        self.send_interval = 0.08   # 80ms ≈ 12.5Hz（非常合适）

        self.freeze_reason = ""   # "" | "pause" | "action"

    # ------------------------------------------------
    # 原厂 goback 姿态（完全照抄 object_tracking.goback）
    # ------------------------------------------------
    def goback(self):
        self.get_logger().info('📌 回到 goback 安全姿态...')
        # 注意：这里会把整只机械臂复位到原厂安全姿态
        set_servo_position(
            self.joints_pub,
            1.5,
            (
                (10, 200),
                (5, 500),
                (4, 115),   # ← 4 号：头部俯仰安全水平位
                (3, 130),
                (2, 560),
                (1, 500),   # ← 1 号：左右居中
            )
        )
        
        # 记录 head 相关舵机（只记录 1 和 4）
        self.last_pitch = 115
        self.last_yaw = 500
        
    def head_default_pose(self):
        """
                    语音节点启动完毕后进入的默认准备姿态：抬头看向用户
        """
        self.get_logger().info("📌 进入默认抬头姿态...")
        set_servo_position(
            self.joints_pub,
            1.5,
            (
                (10, 200),
                (5, 500),
                (4, 350),   # ← 4 号：头部俯仰安全水平位
                (3, 85),
                (2, 730),
                (1, 500),   # ← 1 号：左右居中
            )
        )
        self.last_pitch = 350
        self.last_yaw = 500
    
    # ------------------------------------------------
    # 添加控制回调
    # ------------------------------------------------    
    def control_callback(self, msg: String):
        cmd = (msg.data or "").strip()
        if not cmd:
            return

        # ========== A) 语音节点启动完毕（voice_ready） ==========
        #if cmd == "voice_ready":
        #    self.get_logger().info("🎤 收到语音节点已启动 → 进入准备状态（抬头默认姿态）")
        #    self.voice_ready = True

            # 语音节点准备好后，先进入“抬头待命”状态，但仍然不跟随
        #self.paused = True
            # self.head_default_pose()
         #   return

        # ========== A) 休眠：goback + 冻结 ==========
        if cmd == "pause":
            self.freeze_reason = "pause"
            self.get_logger().info("⏸ 面部跟随暂停")
            self.paused = True
            
            # ---- 回到你的默认姿态（1-5：500,560,130,115,500）----
            self.goback()

            # pause 时清 PID，但不同步 pitch/yaw
            self.tracker.pid_pitch.clear()
            self.tracker.pid_yaw.clear()

            # 向外发布当前舵机位置（保留你原来的能力）
            #status = String()
            #status.data = f"yaw:{self.last_yaw},pitch:{self.last_pitch}"
            #self.status_pub.publish(status)
            return

        # ========== B) 唤醒：抬头 → 跟随 ==========
        if cmd == "resume":
            self.freeze_reason = ""
            # 12/14取消逻辑
            # 如果语音节点还没准备好，就拒绝开始跟随
            #if not self.voice_ready:
            #    self.get_logger().warn("⚠️ 收到 resume 但语音节点尚未准备好（未收到 voice_ready）→ 忽略")
            #    return
            self.get_logger().info("▶ 面部跟随启动（不依赖 voice_ready）")

            # 1) 先暂停跟随
            self.paused = True

            # 2) 先抬头
            self.get_logger().info("📌 resume → 先进入默认抬头姿态")
            self.head_default_pose()

            # ===（可选，但推荐）延时 0.2s，确保舵机稳定===
            import time
            time.sleep(0.2)
            
            # 3)恢复时强制同步 tracker 内部状态
            self.tracker.pitch = self.last_pitch
            self.tracker.yaw = self.last_yaw
            self.tracker.pid_pitch.clear()
            self.tracker.pid_yaw.clear()

            # 4) 再进入跟随模式
            self.paused = False


            self.get_logger().info(
                f"🔄 resume 完成：tracker 同步到 last_pitch={self.last_pitch}, last_yaw={self.last_yaw}"
            )
            return
            
        # ========== C：原地冻结 ==========
        if cmd == "pause_for_action":
            self.freeze_reason = "action"
            self.get_logger().info("⏸ face_follow: 收到 pause_for_action（仅暂停，不回到 goback）")
            self.paused = True

            # 清 PID，停止跟随，但不改变关节位置
            self.tracker.pid_pitch.clear()
            self.tracker.pid_yaw.clear()

            return
        
        # ========== B2) 从原地冻结恢复：不回默认姿态，直接继续跟随 ==========
        if cmd == "resume_from_action":
            self.freeze_reason = ""
            self.get_logger().info("▶ face_follow: resume_from_action（原地恢复跟随，不回 head_default_pose）")

            # 1) 先保持冻结，做状态同步
            self.paused = True

            # 2) 同步 tracker 状态，清 PID，避免跳变
            self.tracker.pitch = self.last_pitch
            self.tracker.yaw   = self.last_yaw
            self.tracker.pid_pitch.clear()
            self.tracker.pid_yaw.clear()

            # 3) 开始跟随
            self.paused = False

            self.get_logger().info(
                f"🔄 resume_from_action 完成：tracker 同步到 last_pitch={self.last_pitch}, last_yaw={self.last_yaw}"
            )
            return


    # ------------------------------------------------
    # 图像回调：只做人脸跟踪 → 输出 1 号 & 4 号舵机脉冲
    # ------------------------------------------------
    def image_callback(self, msg: Image):
        # 语音节点未启动完成 → 直接不处理
        # 25/12/14 取消 语音节点消息收集

        # 当前处于暂停状态 → 不进行人脸跟随，只清 PID
        if self.paused:
            #self.tracker.pid_yaw.clear()
            #self.tracker.pid_pitch.clear()
            return

        # 和原厂一样：用 "rgb8" 读图
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        result_image = np.copy(rgb_image)

        # ★ 调用原厂 FaceTracker 逻辑
        #   返回值 p_y: (pitch, yaw) = (4 号脉冲, 1 号脉冲)
        result_image, p_y = self.tracker.proc(rgb_image, result_image)
       
        if self.paused:
            self.tracker.pid_yaw.clear()
            self.tracker.pid_pitch.clear()
            return

        if p_y is not None:
            pitch_raw, yaw_raw = p_y  # tracker 原始输出

            # ---- 限幅 ----
            pitch = int(max(100, min(740, pitch_raw)))
            yaw   = int(max(0,   min(1000, yaw_raw)))
            
            # ★★★ Smooth 低通滤波（选项 B）★★★
            #ALPHA = 0.2  # 越大越灵敏，越小越平滑（0.2 ~ 0.3 推荐）
            #pitch = int(self.last_pitch * (1 - ALPHA) + pitch * ALPHA)
            #yaw   = int(self.last_yaw   * (1 - ALPHA) + yaw   * ALPHA)

            #print("\n=== FaceFollowNode DEBUG ===")
            #print(f"raw from tracker:  pitch={pitch_raw}, yaw={yaw_raw}")
            #print(f"after limit:       pitch={pitch}, yaw={yaw}")
            
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_send_ts >= self.send_interval:
                set_servo_position(
                    self.joints_pub,
                    0.02,
                    (
                        (1, yaw),
                        (4, pitch),
                    )
                )
                self.last_send_ts = now
            
            # === 新增：记录最新的舵机位置 ===
            self.last_pitch = pitch
            self.last_yaw = yaw

        # 可选：调试显示图像（默认关闭）
        if self.display:
            import cv2
            # result_image 本身是 RGB，这里转回 BGR 再显示
            bgr = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("face_follow", bgr)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FaceFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

