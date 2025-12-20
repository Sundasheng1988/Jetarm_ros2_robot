# llm_command_parser_node.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re, json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import List

COLOR_MAP = {
    "红":"red","红色":"red",
    "蓝":"blue","蓝色":"blue",
    "绿":"green","绿色":"green",
    "黄":"yellow","黄色":"yellow",
    "白":"white","白色":"white",
    "黑":"black","黑色":"black",
    "紫":"purple","紫色":"purple",
    "网球":"tennis","网球球":"tennis",
}

CLASS_MAP = {
    "圆柱":"cylinder","原柱":"cylinder","圆柱体":"cylinder","cylinder":"cylinder",
    "方块":"cube","立方体":"cube","cube":"cube",
    "球":"ball","小球":"ball","ball":"ball",
    "杯":"cup","杯子":"cup","大圆":"cup","cup":"cup",
}

SIDE_MAP = {
    "右":"right_side","右边":"right_side","右侧":"right_side","right":"right_side",
    "左":"left_side","左边":"left_side","左侧":"left_side","left":"left_side",
    "中":"center","中间":"center","中心":"center","middle":"center","center":"center",
    "右手边":"right_side","左手边":"left_side",
}

def parse(text: str):
    t = text.strip()
    act = "pick"
    to_side = None
    color = None
    klass = None

    # 颜色
    for k,v in COLOR_MAP.items():
        if k in t:
            color = v
            break
    # 类别
    for k,v in CLASS_MAP.items():
        if k in t.lower():  # 兼容英文
            klass = v
            break
    # 目标区域
    for k,v in SIDE_MAP.items():
        if k in t.lower():
            to_side = v
            break

    # 组合 from token（无则 unknown_object）
    if color and klass:
        from_tok = f"{color}_{klass}"
    elif klass:
        from_tok = klass
    elif color:
        from_tok = f"{color}_unknown"
    else:
        from_tok = "unknown_object"

    cmd = {
        "action": act,
        "from": from_tok,
        "to": to_side or "right_side",
        "steps": ["move_to_source", "gripper_open", "gripper_close", "move_to_target", "gripper_open"],
        "raw": text
    }
    return cmd

class LLMCommandParserNode(Node):
    def __init__(self):
        super().__init__("llm_command_parser_node")
        
        # ✅ 保留旧的键盘输入订阅
        self.sub = self.create_subscription(String, "/keyboard_input/input", self.on_text, 10)
         # ✅ 新增：语音输入订阅（最小改动）
        self.sub_voice = self.create_subscription(String, "/voice_input/input", self.on_text, 10)
        
        self.pub = self.create_publisher(String, "/parsed_command", 10)
        self.get_logger().info("🧩 LLM Command Parser started (支持 keyboard_input 与 voice_input).")

    def on_text(self, msg: String):
        # ✅ 新增：去掉中文-中文之间的空格（Vosk 常见分词空格）
        text = (msg.data or "").strip()
        text_norm = re.sub(r'(?<=[\u4e00-\u9fff])\s+(?=[\u4e00-\u9fff])', '', text)
        
        cmd = parse(text_norm)
        self.get_logger().info(f"✅ JSON 指令: {cmd}")
        self.pub.publish(String(data=json.dumps(cmd, ensure_ascii=False)))

def main(args=None):
    rclpy.init(args=args)
    node = LLMCommandParserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

