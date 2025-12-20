#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class ExecutorDoneSayer(Node):
    """
    订阅执行器完成信号（Bool），自动在 /speech_reply 播报“任务完成”。
    如你的执行器是其它话题/类型，请按需改 done_topic 和回调逻辑。
    """
    def __init__(self):
        super().__init__("executor_done_sayer")
        self.done_topic = self.declare_parameter("done_topic", "/executor/done").get_parameter_value().string_value
        self.reply_topic = self.declare_parameter("reply_topic", "/speech_reply").get_parameter_value().string_value
        self.success_text = self.declare_parameter("success_text", "任务已完成。需要继续吗？").get_parameter_value().string_value

        self.pub_reply = self.create_publisher(String, self.reply_topic, 10)
        self.sub_done  = self.create_subscription(Bool, self.done_topic, self._on_done, 10)
        self.get_logger().info(f"✅ executor_done_sayer 启动，监听 {self.done_topic} -> 播报到 {self.reply_topic}")

    def _on_done(self, msg: Bool):
        if msg.data:
            self.pub_reply.publish(String(data=self.success_text))

def main(args=None):
    rclpy.init(args=args)
    node = ExecutorDoneSayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
