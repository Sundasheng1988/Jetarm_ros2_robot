#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def pose(frame, xyz, rpy): return {"frame":frame,"xyz":xyz,"rpy":rpy}

class WMDummy(Node):
    def __init__(self):
        super().__init__("wm_dummy_pub")
        self.pub = self.create_publisher(String, "/world_model/objects", 10)
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        now = time.time()
        objs = [
            {"id":1,"class_name":"cup","color":"yellow","pose":pose("table",[-0.30,0.00,0.02],[0,0,1.57]),"confidence":0.92,"updated_at":now},
            {"id":2,"class_name":"ball","color":"blue","pose":pose("table",[-0.28,0.18,0.02],[0,0,0]),"confidence":0.95,"updated_at":now},
            {"id":3,"class_name":"cup","color":"red","pose":pose("table",[-0.27,-0.15,0.02],[0,0,1.57]),"confidence":0.90,"updated_at":now},
        ]
        self.pub.publish(String(data=json.dumps({"objects":objs}, ensure_ascii=False)))

def main():
    rclpy.init(); node = WMDummy(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == "__main__":
    main()

