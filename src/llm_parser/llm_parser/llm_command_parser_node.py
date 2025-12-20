#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LLM Command Parser Node (强鲁棒修订版)
要点：
- 先本地规则解析（deterministic），LLM 仅作为增强；LLM 失败也能得到可执行指令。
- 校验从“硬性必须四字段”改为“宽进严出”：允许 action/from/to/steps 缺省，后续统一补齐。
- 目的地（to）严格按“放到/放在/放入/至/搬到/移动到”后的方位词优先；否则取全文最后位置词兜底。
- 支持英文位置别名（left/right/front/back/center/middle）。
- steps 永远按动作模板生成（杜绝模型漂移）。
"""

import os
import json
import re
from typing import Optional, Tuple, Any, Dict, List

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# ============== System Prompt（尽量约束，但不强依赖其正确性） ==============
SYSTEM_PROMPT = """
你是机器人动作规划器。尽量只输出一个 JSON 对象（不写自然语言），包含尽可能多的字段：
- action: {"pick","place","move","pour","grasp","release","hold"}
- from: 下划线英文（如 yellow_cup / red_ball / cup / ball）
- to: {"left_side","right_side","front_side","back_side","center","bin_a","bin_b","tray_a","tray_b","station_1","station_2","none"}
- steps: 字符串数组（若不确定可留空数组）

如果不确定，也输出尽可能合理的草稿 JSON（允许缺少字段），不要写解释文字。
"""

# ===================== 词表与别名 =====================
SYN_FROM = {
    # 直连映射
    "黄色杯子": "yellow_cup", "黄色的杯子": "yellow_cup",
    "红色的杯子": "red_cup",  "蓝色的杯子": "blue_cup",
    "红色的球": "red_ball",   "蓝色的球": "blue_ball",

    # 球
    "球": "ball", "红球": "red_ball", "红色球": "red_ball",
    "蓝球": "blue_ball", "蓝色球": "blue_ball",
    "绿球": "green_ball", "绿色球": "green_ball",

    # 杯子
    "杯子": "cup", "玻璃杯": "glass_cup", "茶杯": "tea_cup", "马克杯": "mug",
    "红色杯子": "red_cup", "蓝色杯子": "blue_cup", "绿色杯子": "green_cup",
    "被子": "cup",  # ASR 错词
}

# 中文 + 英文别名
SYN_TO = {
    # 中文
    "左": "left_side", "左边": "left_side", "左侧": "left_side", "左方": "left_side",
    "右": "right_side", "右边": "right_side", "右侧": "right_side", "右方": "right_side",
    "前": "front_side", "前面": "front_side", "前方": "front_side", "眼前": "front_side", "面前": "front_side",
    "正前": "center", "中间": "center", "中央": "center", "正中": "center",
    "后": "back_side", "后面": "back_side", "后方": "back_side",
    "a框": "bin_a", "框a": "bin_a", "箱a": "bin_a", "料箱a": "bin_a", "料框a": "bin_a",
    "b框": "bin_b", "框b": "bin_b", "箱b": "bin_b", "料箱b": "bin_b", "料框b": "bin_b",
    "托盘a": "tray_a", "托盘b": "tray_b",
    "工位1": "station_1", "工位2": "station_2",
    # 英文（常见 LLM 输出）
    "left": "left_side", "right": "right_side",
    "front": "front_side", "back": "back_side",
    "center": "center", "middle": "center",
    
    "bina": "bin_a", "bin a": "bin_a", "bin_a": "bin_a",
    "binb": "bin_b", "bin b": "bin_b", "bin_b": "bin_b",
}

ALLOWED_ACTIONS = {"pick", "place", "move", "pour", "grasp", "release", "hold"}
ALLOWED_PLACES = {v.lower() for v in SYN_TO.values()} | {"none"}

COLOR_CN2EN = {
    "红": "red", "红色": "red",
    "蓝": "blue", "蓝色": "blue",
    "黄": "yellow", "黄色": "yellow",
    "绿": "green", "绿色": "green",
    "黑": "black", "黑色": "black",
    "白": "white", "白色": "white",
}
OBJECT_CN2EN = {
    "杯": "cup", "杯子": "cup", "玻璃杯": "glass_cup", "茶杯": "tea_cup", "马克杯": "mug",
    "球": "ball", "瓶": "bottle", "瓶子": "bottle", "盒": "box", "盒子": "box",
}

ALLOWED_STEPS = {
    "move_to_source", "gripper_open", "gripper_close",
    "move_to_target", "tilt_to_pour"
}

# 指代词（用于“这个/那个/它”复用上一对象）
PRONOUNS = ("这个","那个","它","它的","这一个","那一个","it","its","this","that")

STEP_FIXES = {
    "move__to_source": "move_to_source",
    "move_ to source": "move_to_source",
    "move_ to_ source": "move_to_source",
    "move_to_souce": "move_to_source",
    "move_to_taret": "move_to_target",
    "move_ to_ target": "move_to_target",
    "gripper open": "gripper_open",
    "gripper close": "gripper_close",
    "tilt_to_por": "tilt_to_pour",
}

_SNAKE_RE = re.compile(r"^[a-z]+(_[a-z0-9]+)*$")

# —— from 中的“伪位置词”判定（如 center_center / leftside 等）——
_PLACE_KEYS = {
    "left","right","front","back","center","middle",
    "bina","binb","bin","tray","station","side"
}
def _contains_place_like(tok: str) -> bool:
    key = (tok or "").lower().replace("_","").replace(" ","")
    return any(p in key for p in _PLACE_KEYS)

# ===================== 工具函数 =====================
def _slugify_cn(name: str) -> str:
    s = (name or "").strip()
    s = s.replace("的", "")
    s = re.sub(r"[^\w\s\-]+", "", s)
    s = re.sub(r"\s+", "_", s)
    return s.lower()

def extract_first_json_object(text: str) -> Optional[str]:
    cleaned = (text or "").replace("```json", "```").replace("```JSON", "```")
    if "```" in cleaned:
        parts = cleaned.split("```")
        candidates = [p.strip() for p in parts if "{" in p and "}" in p]
        cleaned = candidates[0] if candidates else cleaned.replace("```", "")
    s = cleaned
    start, depth, in_str, esc = -1, 0, False, False
    for i, ch in enumerate(s):
        if ch == '"' and not esc:
            in_str = not in_str
        esc = (ch == '\\' and not esc) if in_str else False
        if in_str:
            continue
        if ch == '{':
            if depth == 0:
                start = i
            depth += 1
        elif ch == '}':
            if depth > 0:
                depth -= 1
                if depth == 0 and start != -1:
                    return s[start:i+1]
    return None

def has_object_token(s: str) -> bool:
    """是否包含明确的物体词（中英都支持）"""
    s = (s or "").lower()
    zh = any(k in s for k in ["杯","杯子","玻璃杯","茶杯","马克杯","球","瓶子","瓶","盒子","盒"])
    en = any(w in s for w in ["cup","mug","glass","bottle","box","ball","can","bowl"])
    return zh or en
    
def has_place_token(s: str) -> bool:
    # 能解析出位置就算有“方位词”
    return bool(extract_dest_from_text(s or ""))

# 宽进：允许缺字段；后续 normalize 统一补齐
def soft_validate(data: Any) -> Dict[str, Any]:
    if not isinstance(data, dict):
        return {}
    out: Dict[str, Any] = {}
    for k in ("action", "from", "to", "steps"):
        if k in data:
            out[k] = data[k]
    # 统一：非 list 的 steps 一律置空
    if not isinstance(out.get("steps"), list):
        out["steps"] = []
    return out

def coerce_and_sanitize_steps(steps_in: List[Any], action: str) -> List[str]:
    flat: List[str] = []

    def push(token: str):
        if not token:
            return
        t = token.strip().lower().replace(" ", "_")
        t = STEP_FIXES.get(t, t)
        if t in ALLOWED_STEPS:
            flat.append(t)

    for it in steps_in or []:
        if isinstance(it, str):
            push(it)
        elif isinstance(it, dict):
            for k, v in it.items():
                if isinstance(k, str): push(k)
                if isinstance(v, str): push(v)
        elif isinstance(it, list):
            for sub in it:
                if isinstance(sub, str): push(sub)

    # 去重
    seen = set()
    flat = [x for x in flat if not (x in seen or seen.add(x))]

    # 动作模板兜底
    if action == "hold":
        return ["move_to_source", "gripper_open", "gripper_close"]
    if action == "pick":
        return ["move_to_source", "gripper_open", "gripper_close", "move_to_target", "gripper_open"]
    if action == "place":
        return ["move_to_target", "gripper_open"]
    if action == "pour":
        return ["move_to_source", "gripper_close", "move_to_target", "tilt_to_pour", "gripper_open"]
    if action == "grasp":
        return ["gripper_close"]
    if action == "release":
        return ["gripper_open"]
    return flat or ["move_to_target"]

# -------- 目的地抽取（放字优先，含英文别名） --------
DEST_PATTERNS = [
    r"(放到|放在|放入|放至)\s*(左边|右边|左手边|右手边|中间|前面|后面|左侧|右侧|前方|后方|A框|B框|托盘A|托盘B|工位1|工位2|left|right|front|back|center|middle|bin[_ ]?a|bin[_ ]?b)",
    r"(移到|移动到|搬到)\s*(左边|右边|左手边|右手边|中间|前面|后面|左侧|右侧|前方|后方|A框|B框|托盘A|托盘B|工位1|工位2|left|right|front|back|center|middle|bin[_ ]?a|bin[_ ]?b)",
    r"(?i)\b(put|place|move)\b.*?\b(to|into)\b\s*(left|right|front|back|center|middle|bin[_ ]?a|bin[_ ]?b)\b",
]

def _norm_place_token(tok: str) -> str:
    tok = (tok or "").strip().lower().replace("的", "")
    key = tok.replace(" ", "").replace("_", "")
    # 先试归一化 key
    if key in SYN_TO: 
        return SYN_TO[key]
    # 再回退原样查
    return SYN_TO.get(tok, tok)

def extract_dest_from_text(user_text: str) -> str:
    s = (user_text or "").strip()
    for pat in DEST_PATTERNS:
        m = re.search(pat, s)
        if m:
            # 英文第三个分组是位置，中文第二个分组是位置
            place_raw = m.group(3) if "(?i)" in pat or "put" in pat else m.group(2)
            place_norm = _norm_place_token(place_raw).lower()
            if place_norm in ALLOWED_PLACES:
                return place_norm
    # 兜底：全文扫描最后一个方位词（支持英文）
    last_hit = None
    for k in SYN_TO.keys():
        # 含中文字符 → 用子串匹配；纯英文 → 用单词边界
        if re.search(r'[\u4e00-\u9fff]', k):
            if k in s:
                last_hit = SYN_TO[k]
        else:
            if re.search(rf"\b{k}\b", s, flags=re.I):
                last_hit = SYN_TO[k]
    return last_hit if last_hit in ALLOWED_PLACES else ""

# -------- 本地规则解析（无 LLM 也可用） --------
def local_rule_parse(user_text: str) -> Dict[str, Any]:
    t = (user_text or "").strip().lower()

    # 中英动词识别
    moving  = any(w in t for w in ["移到","移动到","移动","move to","move"])
    placing = any(w in t for w in ["放到","放在","放入","放至","搬到","put","place"])
    holding = any(w in t for w in ["拿取","拿起","拿","取","抓","握住","pick","grab","take","hold"])

    if (("move" in t or "move to" in t) and not has_object_token(t)) or \
       (any(w in t for w in ["移到","移动到","移动"]) and not has_object_token(t)):
        action = "move"
    elif placing or (moving and has_object_token(t)):
        action = "pick"
    elif holding:
        action = "hold"
    else:
        action = "pick"

    # 颜色（英文也支持）
    color = ""
    for cn, en in COLOR_CN2EN.items():
        if cn in t:
            color = en; break
    if not color:
        for en in ["red","blue","yellow","green","black","white"]:
            if en in t: color = en; break

    # 类别（英文也支持）
    obj_base = ""
    if any(k in t for k in ["杯","杯子","玻璃杯","茶杯","马克杯","cup","mug","glass"]):
        obj_base = "cup"
    elif ("球" in t) or ("ball" in t):
        obj_base = "ball"
    else:
        for cn, en in OBJECT_CN2EN.items():
            if cn in t:
                obj_base = en; break
        if not obj_base:
            for k,en2 in [("bottle","bottle"),("box","box")]:
                if k in t: obj_base = en2; break

    frm = f"{color + '_' if color else ''}{obj_base}" if obj_base else "unknown_object"
    frm = _slugify_cn(SYN_FROM.get(frm, frm))

    to_norm = extract_dest_from_text(t)
    if action == "hold":
        to_norm = "none"
    if not to_norm:
        to_norm = "none"

    steps = coerce_and_sanitize_steps([], action)
    return {"action": action, "from": frm, "to": to_norm, "steps": steps}


def normalize_fields(d: Dict[str, Any], user_text: str) -> Dict[str, Any]:
    out = dict(d)

    # --- action（更稳的判定） ---
    t = (user_text or "").lower()

    # 动词/语义信号
    moving  = any(w in t for w in ["移到", "移动到", "移动", "move to", "move"])
    placing = any(w in t for w in ["放到", "放在", "放入", "放至", "搬到", "put", "place"]) or ("放" in t and has_place_token(t))
    holding = any(w in t for w in ["拿取", "拿起", "拿", "取", "抓", "握住", "pick", "grab", "take", "hold"])

    has_obj   = has_object_token(t)
    has_place = has_place_token(t)

    a = (out.get("action") or "").lower()

    # 1) 统一 place -> pick（下游只处理 pick/hold/move）
    if a == "place":
        a = "pick"

    # 2) 非法值清空，稍后用句子语义强覆盖
    if a not in ALLOWED_ACTIONS:
        a = ""

    # 3) 基于句子语义的“强覆盖”（优先级最高）
    if moving and not has_obj:
        a = "move"                                          # 只有移动词、没有对象 => 纯移动
    elif placing or (moving and has_obj) or (has_obj and has_place):
        a = "pick"                                          # 放/搬/移动物体，或“有对象+有目标”
    elif holding or (has_obj and not (moving or placing)):
        a = "hold"                                          # 拿/取/抓，或仅提到物体无目标
    elif not a:
        a = "move" if has_place else "pick"                 # 兜底：只有位置词 => move，否则 pick

    # 4) 二次纠偏（更稳）
    if a == "move" and has_obj and has_place:
        a = "pick"                                          # 防止“移到 + 物体 + 目标”被误判为 move
    if a == "pick" and not (placing or moving or has_place):
        a = "hold"                                          # 没有“放/移/目标”的 pick 更像 hold

    out["action"] = a

    # --- 先检查 LLM 给的 from 是否像“位置” ---
    frm_in = (out.get("from") or "").strip()
    if (not frm_in) or _contains_place_like(frm_in) or (_norm_place_token(frm_in) in ALLOWED_PLACES):
        frm_in = "unknown_object"

    # --- 用原句重建颜色+类别（中英都支持） ---
    color = ""
    for cn, en in COLOR_CN2EN.items():
        if cn in t:
            color = en; break
    if not color:
        for en in ["red","blue","yellow","green","black","white"]:
            if en in t: color = en; break

    if any(k in t for k in ["杯","杯子","玻璃杯","茶杯","马克杯","cup","mug","glass"]):
        obj_base = "cup"
    elif ("球" in t) or ("ball" in t):
        obj_base = "ball"
    else:
        obj_base = ""
        for cn, en in OBJECT_CN2EN.items():
            if cn in t: obj_base = en; break
        if not obj_base:
            for k, en2 in [("bottle","bottle"),("box","box")]:
                if k in t: obj_base = en2; break

    frm = f"{color + '_' if color else ''}{obj_base}" if obj_base else frm_in
    frm = _slugify_cn(SYN_FROM.get(frm, frm))

    # —— 最终防抖：凡是像位置词、或不合规命名，都视作未知对象 ——
    if _contains_place_like(frm) or (_norm_place_token(frm) in ALLOWED_PLACES) or (not _SNAKE_RE.match(frm)) or (not frm):
        frm = "unknown_object"
    out["from"] = frm

    # --- to（位置 + 放字优先纠偏 + 英文别名） ---
    to_raw = (out.get("to") or "").strip()
    to_norm = _norm_place_token(to_raw).lower()
    dest_hint = extract_dest_from_text(user_text)  # ← 从中文/英文句子里抽“左/右/前/后/…”
    
    if a == "hold":
        to_norm = "none"
    else:
        if dest_hint and dest_hint in ALLOWED_PLACES and dest_hint != "none":
            # 原句有明确目的地 → 强覆盖
            to_norm = dest_hint
        else:
            # 原句没有明确目的地 → 接受 LLM 的 to；若不合法再置 none
            if to_norm not in ALLOWED_PLACES:
                to_norm = "none"
    out["to"] = to_norm

    # --- steps（模板兜底） ---
    out["steps"] = coerce_and_sanitize_steps(out.get("steps") or [], a)

    # ---- 若句子里既有对象词、又有目的地，而当前仍是 move，则升级为 pick ----
    if out.get("action") == "move" and has_object_token(user_text) and has_place_token(user_text):
        out["action"] = "pick"
        out["steps"] = coerce_and_sanitize_steps([], "pick")

    return out


# ===================== ROS2 节点 =====================
class LLMCommandParserNode(Node):
    def __init__(self):
        super().__init__('llm_command_parser_node')
        self.sub = self.create_subscription(String, '/text_input', self.on_text, 10)
        self.pub = self.create_publisher(String, '/parsed_command', 10)
        
        self.create_service(Trigger, '/llm_parser/clear_memory', self.on_reset_memory)
        
        # 👉 指代记忆：上一条明确对象
        self.last_object: str = ""

        self.model = os.getenv('LLM_MODEL', 'qwen:1.8b')
        base = os.getenv('OLLAMA_BASE', 'http://127.0.0.1:11434')
        self.api_chat = f"{base.rstrip('/')}/api/chat"
        self.timeout_s = float(os.getenv('OLLAMA_TIMEOUT', '60'))

        self.session = requests.Session()
        self.get_logger().info(f"🤖 LLM Command Parser Node 启动 model={self.model} url={self.api_chat}")

    def build_payload(self, user_text: str, repair: Optional[str] = None) -> Dict[str, Any]:
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        if repair:
            messages.append({"role": "assistant", "content": repair})
        messages.append({"role": "user", "content": user_text})
        return {
            "model": self.model,
            "format": "json",
            "messages": messages,
            "stream": False,
            "options": {"temperature": 0.0, "num_ctx": 2048}
        }
    
    def _clean_llm_draft(self, draft: dict) -> dict:
        d = dict(draft or {})
        for k in ("action", "from", "to"):
            v = d.get(k)
            if not isinstance(v, str):
                d[k] = ""   # 交给 normalize 用原句重建
        if not isinstance(d.get("steps"), list):
            d["steps"] = []
        return d

    def on_text(self, msg: String):
        # 先做输入清洗（去句首标点/空白）
        user_input = re.sub(r'^[\s:：，。,.!?！？]+', '', msg.data.strip())
        self.get_logger().info(f"🗣️ 收到输入: {user_input}")

        # 1) 本地规则先行（保证可用）
        local = local_rule_parse(user_input)
        merged = normalize_fields(local, user_input)

        # 2) 调 LLM（尽力而为）；若失败或输出垃圾，就完全忽略它，保留本地结果
        try:
            ok, draft = self.try_llm(user_input)
            if ok:
                # draft → 宽进 → normalize（会用原句强纠偏）
                merged = normalize_fields(self._clean_llm_draft(draft), user_input)
        except Exception as e:
            self.get_logger().warning(f"LLM 异常：{e}，使用本地解析结果")

        # 3) 指代兜底：本句无显式物体词且包含指代词时，沿用上一对象
        if (not has_object_token(user_input)) and any(p in user_input for p in PRONOUNS):
            if getattr(self, "last_object", "") and merged.get("from") == "unknown_object":
                merged["from"] = self.last_object

        # 4) 发布
        self.publish_ok(merged)

        # 5) 发布后更新记忆（只有在本次 from 明确时才记录）
        if merged.get("from") and merged["from"] != "unknown_object":
            self.last_object = merged["from"]
    
    def on_reset_memory(self, req, res):
        try:
            # 如果你有多个记忆变量，在这里一并清掉
            self.last_object = ""     # 代词指代的上一对象
            # self.last_place = ""    # （如有位置记忆，也清）
            res.success = True
            res.message = "llm_parser memory cleared"
            self.get_logger().info("🧹 llm_parser: cleared pronoun memory")
        except Exception as e:
            res.success = False
            res.message = f"error: {e}"
        return res

    def try_llm(self, user_input: str) -> Tuple[bool, Dict[str, Any]]:
        # 第一次请求
        resp = self.session.post(self.api_chat, json=self.build_payload(user_input), timeout=self.timeout_s)
        resp.raise_for_status()
        raw_text = (resp.json().get("message") or {}).get("content", "") or ""

        obj = None
        try:
            json_text = extract_first_json_object(raw_text) or raw_text
            obj = json.loads(json_text)
        except Exception:
            # 第二次加 repair
            resp2 = self.session.post(self.api_chat, json=self.build_payload(user_input, repair="请只给 JSON 对象，字段可缺省，勿写文字。"), timeout=self.timeout_s)
            resp2.raise_for_status()
            raw_text2 = (resp2.json().get("message") or {}).get("content", "") or ""
            try:
                json_text = extract_first_json_object(raw_text2) or raw_text2
                obj = json.loads(json_text)
            except Exception:
                return False, {}

        draft = soft_validate(obj)
        if not draft:
            return False, {}
        return True, draft

    def publish_ok(self, parsed: Dict[str, Any]):
        out = json.dumps(parsed, ensure_ascii=False)
        self.get_logger().info(f"✅ JSON 指令: {out}")
        self.pub.publish(String(data=out))


def main(args=None):
    rclpy.init(args=args)
    node = LLMCommandParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

