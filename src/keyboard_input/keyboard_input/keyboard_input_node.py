#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('keyboard_input_node')
        self.pub = self.create_publisher(String, '/text_input', 10)
        self.get_logger().info("⌨️ 键盘输入模式，输入内容按回车发送，Ctrl+C 退出")

    def run(self):
        while rclpy.ok():
            try:
                text = input("输入指令: ").strip()
                if text:
                    self.pub.publish(String(data=text))
                    self.get_logger().info(f"📤 已发送: {text}")
            except (EOFError, KeyboardInterrupt):
                break

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

