#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, time
from typing import Dict, Any, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data


COLOR_SET = {"red","blue","yellow","green","black","white"}

# —— 左右/中心同义词映射（大小写/中英文皆可）
SIDE_ALIASES = {
    # 左
    "left": "left_side", "leftside": "left_side", "left_side": "left_side",
    "l": "left_side", "ls": "left_side",
    "左": "left_side", "左边": "left_side", "左側": "left_side", "左侧": "left_side",
    "左手边": "left_side", "左邊": "left_side", "左邊兒": "left_side", "左邊儿": "left_side",
    # 右
    "right": "right_side", "rightside": "right_side", "right_side": "right_side",
    "r": "right_side", "rs": "right_side",
    "右": "right_side", "右边": "right_side", "右側": "right_side", "右侧": "right_side",
    "右手边": "right_side", "右邊": "right_side", "右邊兒": "right_side", "右邊儿": "right_side",
    # 中
    "center": "center", "middle": "center",
    "中": "center", "中间": "center", "中心": "center"
}

def now_ts() -> float:
    return time.time()

def split_from_token(token: str) -> Tuple[Optional[str], Optional[str]]:
    t = (token or "").strip().lower()
    if not t or t == "unknown_object":
        return None, None
    parts = t.split("_")
    if parts[0] in COLOR_SET and len(parts) >= 2:
        return "_".join(parts[1:]), parts[0]
    return t, None

def pose_dict(frame: str, xyz, rpy) -> Dict[str, Any]:
    return {"frame": frame, "xyz": list(map(float, xyz)), "rpy": list(map(float, rpy))}

def dist2_origin(pose: Dict[str, Any]) -> float:
    try:
        x, y, z = pose["xyz"]
        return x*x + y*y + z*z
    except Exception:
        return float("inf")

# —— 从原始文本里推断左右/中心
def _infer_side_from_text(raw: str) -> Optional[str]:
    if not raw:
        return None
    t = raw.strip().lower()
    # 先判“右”，避免“左右”并列时误判
    if ("右" in t) or (" right" in t) or ("右边" in t) or ("右側" in t) or ("右侧" in t) or ("右手边" in t):
        return "right_side"
    if ("左" in t) or (" left" in t) or ("左边" in t) or ("左側" in t) or ("左侧" in t) or ("左手边" in t):
        return "left_side"
    if ("中" in t) or ("中间" in t) or ("中心" in t) or (" middle" in t) or (" center" in t):
        return "center"
    return None

def _normalize_to_slot(llm_to: Optional[str], raw_text: Optional[str],
                       allow_infer: bool) -> Optional[str]:
    """优先使用 LLM 的 to；若为空/none，则在允许时从原句推断。返回标准槽位或 None。"""
    tok = (llm_to or "").strip().lower()
    if tok and tok not in ("none", "null"):
        return SIDE_ALIASES.get(tok, tok)

    if allow_infer:
        guess = _infer_side_from_text(raw_text or "")
        if guess:
            return guess
    return None


class GroundingNode(Node):
    def __init__(self):
        super().__init__("grounding_node")
        self.sub_cmd = self.create_subscription(String, "/parsed_command", self.on_cmd, 10)
        self.sub_wm  = self.create_subscription(String, "/world_model/roi_objects", self.on_world_model, qos_profile_sensor_data)
        self.pub_goal = self.create_publisher(String, "/grounded_goal", 10)
        self.create_service(Trigger, "/grounding/clear_memory", self.on_reset)

        # 参数
        self.declare_parameter("reuse_last_object", True)
        self.declare_parameter("allow_side_inference", True)        # 允许从原句推断左右
        self.declare_parameter("default_side_when_missing", "")      # 无法确定时的默认槽位（空=不默认）
        self.declare_parameter("raw_text_topic", "/keyboard_input/input")  # ⬅️ 新增：原句话题
        self.declare_parameter("publish_runtime", False)              # 是否额外发布 runtime TaskContext

        self.reuse_last = bool(self.get_parameter("reuse_last_object").value)
        self.allow_side_infer = bool(self.get_parameter("allow_side_inference").value)
        self.default_side_when_missing = str(self.get_parameter("default_side_when_missing").value).strip()
        self.raw_text_topic = str(self.get_parameter("raw_text_topic").value).strip()
        self.publish_runtime = bool(self.get_parameter("publish_runtime").value)

        self.pub_runtime = None
        if self.publish_runtime:
            self.pub_runtime = self.create_publisher(String, "/grounded_task_context", 10)

        # 订阅原始输入文本（可选）
        self.last_raw_text: str = ""
        try:
            self.sub_raw = self.create_subscription(String, self.raw_text_topic, self._on_raw_text, 10)
            self.get_logger().info(f"📝 raw_text 订阅: {self.raw_text_topic}")
        except Exception as e:
            self.sub_raw = None
            self.get_logger().warn(f"⚠️ 无法订阅原句话题 {self.raw_text_topic}: {e}")

        # 1) 内部默认落点表（与你确认的一致）
        self.place_map = self._default_place_map()
        # 2) 允许用扁平参数 place_map.<slot>.(frame|xyz|rpy) 覆盖
        self._load_place_map_overrides()

        # 运行态内存
        self.objects: Dict[int, Dict[str, Any]] = {}
        self.last_object_id: Optional[int] = None

        self.get_logger().info("🧭 Grounding Node started.")

    def _on_raw_text(self, msg: String):
        self.last_raw_text = msg.data or ""

    def _default_place_map(self) -> Dict[str, Any]:
        return {
            # 你确认过的 base 系左右落点
            "right_side": pose_dict("base", [ 0.0, -0.200, 0.020], [0,0,1.57]),
            "left_side":  pose_dict("base", [ 0.0,  0.200, 0.020], [0,0,1.57]),

            # 其余按你给的
            "front_side": pose_dict("table", [0.19, 0.00, 0.02], [0,0,1.57]),
            "bin_a":      pose_dict("bin_a_center", [0.00, -0.25, 0.02], [0,0,0]),
            "bin_b":      pose_dict("bin_b_center", [0.01, -0.25, 0.02], [0,0,0]),
            "tray_a":     pose_dict("tray_a",       [0.00,  0.15, 0.02], [0,0,0]),
            "tray_b":     pose_dict("tray_b",       [0.01,  0.15, 0.02], [0,0,0]),
        }

    def on_world_model(self, msg: String):
        try:
            data = json.loads(msg.data)
            for o in data.get("objects", []):
                oid = int(o.get("id"))
                self.objects[oid] = {
                    "id": oid,
                    "class_name": (o.get("class_name") or "").lower(),
                    "color": (o.get("color") or "").lower(),
                    "pose": o.get("pose") or {},
                    "confidence": float(o.get("confidence") or 0.0),
                    "updated_at": float(o.get("updated_at") or now_ts()),
                }
        except Exception as e:
            self.get_logger().error(f"解析 /world_model/roi_objects 失败: {e}\n原文: {msg.data}")

    def on_cmd(self, msg: String):
        # 输入如：{"action":"pick","from":"red_cup","to":"right_side", ...}
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"解析 /parsed_command JSON 失败: {e}\n原文: {msg.data}")
            return

        # 1) 基本字段
        intent   = (data.get("action") or "").lower()
        from_tok = (data.get("from") or "").lower()

        # 2) 原始语句：优先 parsed JSON 的 raw/text/utterance；否则回退到最近一次键盘输入
        raw_text = (data.get("raw")
                    or data.get("text")
                    or data.get("utterance")
                    or self.last_raw_text
                    or "")

        # 3) to 的归一化：优先 LLM；如缺失则在允许时从原句推断；仍无则看是否有默认槽位
        to_tok = _normalize_to_slot(data.get("to"), raw_text, self.allow_side_infer)
        if not to_tok and self.default_side_when_missing:
            to_tok = SIDE_ALIASES.get(self.default_side_when_missing.lower(),
                                      self.default_side_when_missing)

        cls, color = split_from_token(from_tok)

        obj: Optional[Dict[str, Any]] = None
        used_memory = False
        if intent in ("pick","hold","place","pour","grasp","release"):
            if from_tok == "unknown_object" and self.reuse_last and self.last_object_id in self.objects:
                obj = self.objects[self.last_object_id]
                used_memory = True
            else:
                obj = self._select_object(cls, color)

        target_pose = self.place_map.get(to_tok) if to_tok else None

        out = {
            "intent": intent,
            "object_hints": {"class": cls, "color": color} if cls else {},
            "object_id": obj["id"] if obj else -1,
            "source_pose": obj.get("pose") if obj else None,
            "target_pose": target_pose,
            "used_last_object_memory": used_memory,
            "status": "ok",
            "detail": ""
        }

        # 规则校验
        if intent == "move":
            if not target_pose:
                out.update(status="no_target", detail="MOVE 缺少目标位姿")
        elif intent in ("pick","place","pour"):
            if not obj:
                out.update(status="no_match", detail=f"未找到对象（class={cls}, color={color})")
            elif intent == "pick" and not target_pose:
                out.update(status="no_target", detail="PICK 缺少放置目标位姿")
        elif intent in ("hold","grasp","release") and not obj:
            out.update(status="no_match", detail=f"未找到对象（class={cls}, color={color})")

        if obj:
            self.last_object_id = obj["id"]

        self.pub_goal.publish(String(data=json.dumps(out, ensure_ascii=False)))
        self.get_logger().info(f"🎯 grounded_goal: {out}")

        if self.pub_runtime is not None:
            rto = None
            if obj:
                rto = {
                    "class_name": (obj.get("class_name") or "").lower(),
                    "color": (obj.get("color") or "").lower(),
                    "source": "grounding",
                    "world_frame": (obj.get("pose") or {}).get("frame", "base"),
                    "world_x": float((obj.get("pose") or {}).get("xyz", [0,0,0])[0]),
                    "world_y": float((obj.get("pose") or {}).get("xyz", [0,0,0])[1]),
                    "world_z": float((obj.get("pose") or {}).get("xyz", [0,0,0])[2]),
                    "object_id": str(obj.get("id", "")),
                }
            rt = {
                "intent": intent,
                "parsed_command": data,
                "target_object": rto,
                "target_pose": target_pose,
                "status": out.get("status", ""),
                "detail": out.get("detail", ""),
            }
            self.pub_runtime.publish(String(data=json.dumps(rt, ensure_ascii=False)))

    def _select_object(self, cls: Optional[str], color: Optional[str]) -> Optional[Dict[str, Any]]:
        cand: List[Dict[str,Any]] = []
        for o in self.objects.values():
            ok_cls = True if not cls else (o["class_name"] == cls or cls in o["class_name"] or o["class_name"] in cls)
            ok_color = True if not color else (o["color"] == color)
            if ok_cls and ok_color:
                cand.append(o)
        if not cand:
            return None
        cand.sort(key=lambda x: (-(x.get("confidence") or 0.0),
                                 -(x.get("updated_at") or 0.0),
                                 dist2_origin(x.get("pose") or {})))
        return cand[0]

    def on_reset(self, req, res):
        self.last_object_id = None
        res.success = True
        res.message = "cleared"
        self.get_logger().info("🧹 cleared last_object_id")
        return res

    def _load_place_map_overrides(self):
        """从参数前缀 place_map.* 读取覆盖，支持：
           place_map.left_side.frame: "table"
           place_map.left_side.xyz:   [-0.25, 0.20, 0.02]
           place_map.left_side.rpy:   [0.0, 0.0, 1.57]
        """
        params = self.get_parameters_by_prefix('place_map')
        if not params:
            return
        pm = dict(self.place_map)
        for rel_name, p in params.items():
            parts = rel_name.split('.')  # 例如 ['left_side','xyz']
            if len(parts) != 2:
                continue
            slot, field = parts
            if slot not in pm:
                pm[slot] = {
                    "frame": self.get_parameter('world_frame').get_parameter_value().string_value
                              if self.has_parameter('world_frame') else "table",
                    "xyz": [0.0, 0.0, 0.0],
                    "rpy": [0.0, 0.0, 0.0],
                }
            if field in ('frame', 'xyz', 'rpy'):
                pm[slot][field] = p.value
        self.place_map = pm
        self.get_logger().info(f"[grounding] place_map overrides loaded: {list(pm.keys())}")


def main(args=None):
    rclpy.init(args=args)
    node = GroundingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

