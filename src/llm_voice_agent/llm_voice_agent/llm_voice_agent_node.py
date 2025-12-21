# llm_voice_agent_node
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, time, re, os, unicodedata
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from collections import deque
from pathlib import Path
from robot_interfaces.msg import EnvObjectArray  # 若你定义的 msg 名字不同请替换
from llm_voice_agent.utils.class_name_translator import translate_class_name


PUNC_PATTERN = re.compile(r"[^\w\u4e00-\u9fff]+", re.UNICODE)

def _norm_text_for_wake(t: str) -> str:
    t = unicodedata.normalize("NFKC", t or "")
    t = t.lower()
    # 只保留中英文/数字，去空白和标点
    return re.sub(r"[^\w\u4e00-\u9fff]+", "", t)

def norm_text(t: str) -> str:
    # 全角半角统一、大小写统一
    t = unicodedata.normalize("NFKC", t or "")
    t = t.lower()
    # 去掉空白 & 绝大多数标点（保留中英字母/数字/中文）
    t = PUNC_PATTERN.sub("", t)
    return t

# 12/11新增 口语小数朗读函数
_CN_DIGITS = {
    "0": "零", "1": "一", "2": "二", "3": "三", "4": "四",
    "5": "五", "6": "六", "7": "七", "8": "八", "9": "九"
}

def speak_decimal(x: float, digits: int = 2) -> str:
    """
    0.17 -> 零点一七
    0.64 -> 零点六四
    1.25 -> 一点二五
    """
    s = f"{x:.{digits}f}"
    integer, decimal = s.split(".")
    int_cn = "".join(_CN_DIGITS[c] for c in integer)
    dec_cn = "".join(_CN_DIGITS[c] for c in decimal)
    return f"{int_cn}点{dec_cn}"

# 11/1 增加 用于去除MD中的特殊符号
_MD_LINK = re.compile(r'\[([^\]]+)\]\([^)]+\)')
_MD_CODE_SPAN = re.compile(r'`{1,3}[^`]*`{1,3}')
_MD_CODE_BLOCK = re.compile(r'```[\s\S]*?```', re.MULTILINE)
_MD_HEADING = re.compile(r'^\s{0,3}#{1,6}\s*', re.MULTILINE)
_MD_LIST = re.compile(r'^\s*(?:[-*+]|[0-9]+\.)\s+', re.MULTILINE)
_MD_BOLD = re.compile(r'\*\*(.*?)\*\*')
_MD_ITALIC = re.compile(r'\*(.*?)\*')
_MD_TABLE_LINE = re.compile(r'^\s*\|.*\|\s*$', re.MULTILINE)

def strip_markdown_to_speech(text: str) -> str:
    if not text:
        return ''
    # 1) 去代码块/行内代码
    text = _MD_CODE_BLOCK.sub('', text)
    text = _MD_CODE_SPAN.sub('', text)
    # 2) 链接 [txt](url) -> txt
    text = _MD_LINK.sub(r'\1', text)
    # 3) 标题/列表前缀/表格行
    text = _MD_HEADING.sub('', text)
    text = _MD_LIST.sub('', text)
    text = _MD_TABLE_LINE.sub('', text)
    # 4) 粗斜体
    text = _MD_BOLD.sub(r'\1', text)
    text = _MD_ITALIC.sub(r'\1', text)
    # 5) 兜底去掉残余 # * _
    text = text.replace('#', '').replace('*', '').replace('_', '')
    # 6) 折行 → 句号；压缩空白
    text = re.sub(r'\s*\n+\s*', '。', text)
    text = re.sub(r'[ \t]+', ' ', text).strip()
    # 7) 防止空串，句尾补句号
    if text and text[-1] not in '。！？.!?':
        text += '。'
    return text

# 10/26新增一个清洗函数（删掉思考块）
_THINK_BLOCK_RE = re.compile(r"<think>.*?</think>", re.DOTALL | re.IGNORECASE)

def strip_think(text: str) -> str:
    if not text:
        return text
    out = _THINK_BLOCK_RE.sub("", text)
    out = out.replace("<think>", "").replace("</think>", "")
    return out.strip()

# -- 10/26 增加，用于去除思考过程输出
_META_PREFIX_RE = re.compile(
    r'^(好的[，,。]?\s*)?(用户[^。！？]{0,80}?问[^。！？]*?。)+',
    re.IGNORECASE | re.DOTALL
)
_PLANNING_SENT_RE = re.compile(
    r'(?:(?:我[们]?(需要|应该|将|要|打算)[^。！？]{0,80})|(?:首先|接下来|另外|最后)[：:\s，,]?)[^。！？]*[。！？]?',
    re.IGNORECASE
)

def strip_meta(text: str) -> str:
    if not text:
        return text
    t = text.strip()
    t = _META_PREFIX_RE.sub('', t)
    t = _PLANNING_SENT_RE.sub('', t)
    t = re.sub(r'\s+', ' ', t).strip()
    return t or text.strip()

_SENT_SPLIT_RE = re.compile(r'[。！？!?]\s*')
_META_KEYWORDS_RE = re.compile(
    r'(用户|我(需要|应该|将|要)|首先|接下来|然后|最后|综上|我的(计划|思路)|步骤|我会|我要)',
    re.IGNORECASE
)

def keep_user_facing(text: str, max_sents: int = 2) -> str:
    if not text:
        return text
    t = text.strip()
    sents = [s.strip() for s in _SENT_SPLIT_RE.split(t) if s.strip()]
    clean = [s for s in sents if not _META_KEYWORDS_RE.search(s)]
    out_sents = clean if clean else sents
    out = '。'.join(out_sents[:max_sents])
    return out[:120].strip()

# 可选 LLM 依赖（默认不用）
try:
    import requests
except Exception:
    requests = None

def clamp(s: str, n=1200) -> str:
    return s if len(s) <= n else s[:n]

# --- 词典 ---
COLOR_MAP = {
    "红":"red","红色":"red",
    "蓝":"blue","蓝色":"blue",
    "绿":"green","绿色":"green",
    "黄":"yellow","黄色":"yellow",
    "白":"white","白色":"white",
    "黑":"black","黑色":"black",
    "紫":"purple","紫色":"purple",
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

INTENT_VERBS = ['拿','取','抓','抓取','抓起','放','放到','移动','搬','移']
CONFIRM_WORDS = ['确认','确定','执行','好的','好','行','ok','可以','是的','没问题']
CANCEL_WORDS  = ['取消','别','不','先不要','算了','不用了','停止']

SMALLTALK_PATTERNS = [
    r'^你?好(呀|啊)?$', r'^(hi|hello|哈喽)$', r'^(在吗|在不|在)$',
    r'^(收到|ok|好的|嗯|嗯嗯|是的|好)$',
    r'^(早上?好|下午好|晚上好)$', r'^谢谢$'
]
SMALLTALK_RE = re.compile('|'.join(SMALLTALK_PATTERNS), re.IGNORECASE)


# === 25/12/14  L1：语义动作暂停（动作层）===
PAUSE_KEYWORDS = [
    "别跟着我", "不用跟了", "先别动", "回到原位",
]
RESUME_KEYWORDS = [
    "开始跟着", "看着我",
    "继续跟着", "你继续", "继续看着我"
]

# ===== 25/12/21 L2 系统级唤醒 / 休眠（必须带 jack）=====
SYSTEM_WAKE_KEYWORDS = [
    "jack启动系统",
    "jack唤醒系统",
    "jack恢复系统",
]

SYSTEM_SLEEP_KEYWORDS = [
    "jack请先休息吧",
    "jack系统休眠",
]

# ===== 25/12/21 L3：对话静音（不休眠系统）=====
L3_MUTE_KEYWORDS = [
    "别说话",
    "安静",
    "静音",
    "我在拍视频",
    "不要说话",
]

L3_UNMUTE_KEYWORDS = [
    "可以说话了",
    "继续说话",
    "恢复对话",
    "你可以说话了",
]

def is_smalltalk(text: str) -> bool:
    t = text.strip()
    t = re.sub(r'(?<=[\u4e00-\u9fff])\s+(?=[\u4e00-\u9fff])', '', t)
    t = re.sub(r'[，。！!？?\s]+', '', t)
    return bool(SMALLTALK_RE.search(t))

def extract_slots(text: str):
    """从文本中提取 color / klass / side"""
    t = text
    t = re.sub(r'(?<=[\u4e00-\u9fff])\s+(?=[\u4e00-\u9fff])', '', t)
    t = re.sub(r'[，。！!？?\s]+', '', t)
    color = None; klass = None; side = None
    for k,v in COLOR_MAP.items():
        if k in t:
            color = v; break
    tl = t.lower()
    for k,v in CLASS_MAP.items():
        if k in tl or k in t:
            klass = v; break
    for k,v in SIDE_MAP.items():
        if k in tl or k in t:
            side = v; break
    return color, klass, side

def build_cmd_text(color, klass, side):
    parts = []
    if 'red'==color: parts.append('红色')
    elif 'blue'==color: parts.append('蓝色')
    elif 'green'==color: parts.append('绿色')
    elif 'yellow'==color: parts.append('黄色')
    elif 'white'==color: parts.append('白色')
    elif 'black'==color: parts.append('黑色')
    elif 'purple'==color: parts.append('紫色')

    cn_klass = {'cylinder':'圆柱','cube':'方块','ball':'球','cup':'杯'}.get(klass, '物体')
    parts.append(cn_klass)
    cmd = f"拿起{''.join(parts)}"
    if side:
        cn_side = {'right_side':'右侧','left_side':'左侧','center':'中间'}.get(side, '右侧')
        cmd += f'，放到{cn_side}'
    return cmd

class LlmVoiceAgent(Node):
    def __init__(self):
        super().__init__('llm_voice_agent')

        # ===== 参数 =====
        self.query_topic = self.declare_parameter('query_topic', '/speech_query').get_parameter_value().string_value
        self.reply_topic = self.declare_parameter('reply_topic', '/speech_reply').get_parameter_value().string_value
        default_publish_topics = ['/voice_input/input','/keyboard_input/input']
        self.publish_topics = list(
            self.declare_parameter('publish_topics', default_publish_topics)
                .get_parameter_value().string_array_value or default_publish_topics
        )

        # LLM（可选）
        self.use_llm     = bool(self.declare_parameter('use_llm', False).get_parameter_value().bool_value)
        self.ollama_base = self.declare_parameter('ollama_base', 'http://127.0.0.1:11434').get_parameter_value().string_value
        self.model       = self.declare_parameter('model', 'qwen3:8b').get_parameter_value().string_value
        llm_base_param = self.declare_parameter('llm_base', '').get_parameter_value().string_value
        if llm_base_param:
            self.ollama_base = llm_base_param
    
        self.temperature = float(self.declare_parameter('temperature', 0.3).get_parameter_value().double_value)
        self.max_tokens  = int(self.declare_parameter('max_tokens', 160).get_parameter_value().integer_value)
        self.llm_timeout_s = float(self.declare_parameter('llm_timeout_s', 60.0).get_parameter_value().double_value)
        self.num_ctx     = int(self.declare_parameter('num_ctx', 2048).get_parameter_value().integer_value)

        # 唤醒词控制（新增）
        self.use_wakeword    = bool(self.declare_parameter('use_wakeword', True).get_parameter_value().bool_value)
        self.wake_window_s   = float(self.declare_parameter('wake_window_s', 20.0).get_parameter_value().double_value)
        self.wake_cooldown_s = float(self.declare_parameter('wake_cooldown_s', 2.0).get_parameter_value().double_value)
        self._wake_until     = 0.0
        self._last_wake_ts   = 0.0
        # —— 兜底：如果没等到 /tts/done，则定时强制开窗 —
        # self.wake_fallback_ms = int(self.declare_parameter('wake_fallback_ms', 500).get_parameter_value().integer_value)
        #self._wake_fallback_timer = None
        #self.after_interrupt_open_window_s = float(
        #    self.declare_parameter('after_interrupt_open_window_s', 30.0).get_parameter_value().double_value
        #)   

        # 模式控制关键词
        self.start_keywords = list(
            self.declare_parameter(
                'start_keywords',
                ['开始任务','进入任务','启动任务','启动机械臂','进入机械臂模式','开始机械臂','进入执行模式']
            ).get_parameter_value().string_array_value
        )
        self.stop_keywords = list(
            self.declare_parameter(
                'stop_keywords',
                ['退出任务','结束任务','停止任务','回到聊天','退出机械臂','结束机械臂','返回聊天模式']
            ).get_parameter_value().string_array_value
        )
        self.chat_system_prompt = self.declare_parameter(
            'chat_system_prompt',
            '你是中文语音助手，只输出给用户的最终答案：最多两句、合计不超过40字。禁止出现“用户/我需要/首先/接下来/然后/最后/我的计划/思路”等任何元叙述，不得出现<think>标签。'
        ).get_parameter_value().string_value

        # ===== 记忆参数（新增）=====
        self.history_turns = int(self.declare_parameter('history_turns', 8).get_parameter_value().integer_value)
        self.persist_session = bool(self.declare_parameter('persist_session', True).get_parameter_value().bool_value)
        self.session_dir = os.path.expanduser(self.declare_parameter('session_dir', '~/.config/llm_voice_agent/sessions').get_parameter_value().string_value)
        self.clear_on_wakeup = bool(self.declare_parameter('clear_on_wakeup', False).get_parameter_value().bool_value)
        self.max_ctx_chars = int(self.declare_parameter('max_ctx_chars', 8000).get_parameter_value().integer_value)
        
        self.detail_mode = bool(self.declare_parameter('detail_mode', False).get_parameter_value().bool_value)
        self.detail_keywords = list(self.declare_parameter(
            'detail_keywords',
            ['详细','举个例子','举例','行程','攻略','步骤','怎么做','说明','特点','Day1','清单','模板','范例','对比','原理']
        ).get_parameter_value().string_array_value)
        self.detail_max_tokens = int(self.declare_parameter('detail_max_tokens', 600).get_parameter_value().integer_value)
        self.detail_max_ctx_chars = int(self.declare_parameter('detail_max_ctx_chars', 16000).get_parameter_value().integer_value)

        # 手动切换口令
        self._detail_on_cmds  = ['进入详细模式','开启详细模式','详细模式']
        self._detail_off_cmds = ['进入简洁模式','关闭详细模式','简洁模式']

        # ===== ROS IO =====
        self.sub_query = self.create_subscription(String, self.query_topic, self._on_query, 10)
        
        # ===== 12/14 更新 监听环境扫描结果（env_scan_node 输出）=====
        self.sub_env_objects = self.create_subscription(
            EnvObjectArray, 
            "/env_objects", 
            self._on_env_objects, 
            10
        )
        # ===== 环境播报控制 (一次性闸门)=====
        self.env_report_pending = False   # 是否在等待一次环境播报
        self.env_report_done = False      # 本次请求是否已经播报过
                
        self.pub_reply = self.create_publisher(String, self.reply_topic, 10)
        self.pub_texts = [self.create_publisher(String, t, 10) for t in self.publish_topics]
        self.state_pub = self.create_publisher(String, '/voice_agent/state', 10)

        # ===== 对话状态 =====
        self.mode = 'chat'  # chat | task
        self.sleeping = False   # 💤 25/12/14 新增：语音节点休眠态
        self._last_reply = ''
        self._last_reply_ts = 0.0
        self.l3_muted = False #  25/12/21 新增： L3-State Layer： muted 

        # —— 稳定参数：输入/输出去重+时间节流 ——
        self._reply_dedup_window = float(self.declare_parameter('reply_dedup_window_s', 2.0).get_parameter_value().double_value)
        self._min_reply_interval = float(self.declare_parameter('min_reply_interval_s', 1.2).get_parameter_value().double_value)
        self._last_text = ''
        self._last_ts = 0.0
        self._dedup_window = float(self.declare_parameter('dedup_window_s', 3.0).get_parameter_value().double_value)

        # —— 任务槽位 ——
        self.waiting_confirm = False
        self.pending_cmd_text = ''
        self.slots = {'color':None, 'klass':None, 'side':None}

        # —— 严格意图与 LLM 引导开关 ——
        self.strict_intent = bool(self.declare_parameter('strict_intent', True).get_parameter_value().bool_value)
        self.llm_hint_enabled = bool(self.declare_parameter('llm_hint_enabled', True).get_parameter_value().bool_value)

        # ====== 短期记忆：最近 N 轮（user+assistant 为一轮）======
        self._dialog = deque(maxlen=max(2, self.history_turns*2))
        self._session_path = None
        if self.persist_session:
            Path(self.session_dir).mkdir(parents=True, exist_ok=True)
            ts = time.strftime('%Y%m%d-%H%M%S')
            self._session_path = os.path.join(self.session_dir, f'session-{ts}.jsonl')

        # ====== “清空/重置”口令 ======
        self._reset_cmd_keywords = ['清空上下文','重置对话','忘了之前的','重新开始','清空记忆','重置']

        self.get_logger().info(
            f'🧠 llm_voice_agent 已启动；mode={self.mode}；publish_topics={self.publish_topics}；use_llm={self.use_llm}'
        )
        self._set_state('mode:chat')
        
         # === 语音节点启动完毕 → 通知 face_follow 进入“准备状态” ===
        # msg = String()
        # msg.data = "voice_ready"
        # self.face_ctrl_pub.publish(msg)
        # self.get_logger().info("📢 已通知 face_follow：voice_ready")
        
        #11/2 新增 ====== 命中唤醒词 ======
        self.tts_interrupt_pub = self.create_publisher(Bool, '/tts/interrupt', 1)
        
        #11/8 —— 监听 TTS 播放状态（用于“播报结束再开计时”）——
        #self.sub_tts_speaking = self.create_subscription(
        #    Bool, '/tts_speaking', self._on_tts_speaking, 10
        #)
        #self.tts_is_speaking = False
        
        # 11/9 新增：监听 TTS 结束/打断等价完成信号，立刻开窗
        #self.sub_tts_done = self.create_subscription(
        #    Bool, '/tts/done', self._on_tts_done, 10
        #)
        # self.pending_wake_activation = False   # 命中唤醒后，等待播报结束再真正开启窗口
        # self._ignore_tts_false_until = 0.0     # 避免“打断导致的短促 False”误触发
        
        # === 唤醒词：由参数/YAML加载 ===
        self.wake_words = list(
            self.declare_parameter('wake_words', ['jack'])
                .get_parameter_value().string_array_value
        )
        self.wake_regex = list(
            self.declare_parameter('wake_regex', ['j\\s*a\\s*c\\s*k'])
                .get_parameter_value().string_array_value
        )
        self.strip_patterns = list(
            self.declare_parameter('strip_patterns', ['j\\s*a\\s*c\\s*k'])
                .get_parameter_value().string_array_value
        )

        # 预编译
        self._wake_words_norm = { _norm_text_for_wake(w) for w in self.wake_words if w }
        self._wake_regex_compiled  = [re.compile(p, re.IGNORECASE) for p in self.wake_regex]
        self._strip_regex_compiled = [re.compile(p, re.IGNORECASE) for p in self.strip_patterns]
        
        # === 11/24新增：gesture + face_follow 控制 ===
        self.gesture_pub = self.create_publisher(String, "/gesture/cmd", 10)
        self.face_ctrl_pub = self.create_publisher(String, "/face_follow/control", 10)
        
        # === 语音节点启动 → face_follow 进入“准备状态” ===
        #msg = String()
        #msg.data = "voice_ready"
        #self.face_ctrl_pub.publish(msg)
        #self.get_logger().info("📢 已通知 face_follow：voice_ready")

        # === 11/24新增 舵机控制（用于唤醒时回到初始姿态）
        from servo_controller_msgs.msg import ServosPosition
        self.joints_pub = self.create_publisher(ServosPosition, "/servo_controller", 10)
        
        self.gesture_delay_s = float(
            self.declare_parameter('gesture_delay_s', 0.6)
                .get_parameter_value().double_value
        )        

    # ===== 主入口 =====
    def _on_query(self, msg: String):
        raw_text = (msg.data or '').strip()
        if not raw_text:
            return
        
        # 统一归一化（供 L1 / L2 使用）
        norm_raw = re.sub(r'[，。！!？?\s]+', '', raw_text)
        norm = norm_text(raw_text)   # ← 提前定义
        
        now = time.time()
        
        # =====================================================
        # L2：系统级休眠 / 唤醒（最高优先级）
        # =====================================================
        if self.sleeping:
            # ===== L2：仅允许【系统级唤醒指令】=====
            if any(k in norm_raw for k in SYSTEM_WAKE_KEYWORDS):
                self.sleeping = False

                # 👉 系统唤醒时，让机器人抬头看向用户（一次性）
                msg_ctrl = String()
                msg_ctrl.data = "resume"
                self.face_ctrl_pub.publish(msg_ctrl)

                self.get_logger().info("🌅 L2 系统唤醒成功")

                self._say("系统已启动。")
                self._set_state("system_active")

                return

            # ❗L2 sleeping 状态下，其它所有语音一律忽略
            self.get_logger().debug("💤 L2 sleeping，忽略非系统唤醒语音")
            return
            
        # ---- 系统级休眠 ----
        if any(k in norm_raw for k in SYSTEM_SLEEP_KEYWORDS):
            self.sleeping = True
            msg_ctrl = String()
            msg_ctrl.data = "pause"
            self.face_ctrl_pub.publish(msg_ctrl)
            self._say("好的，我先休息了。")
            self._set_state("system_sleeping")
            return
        
        # =====================================================
        # “清空上下文 / 重置对话”
        if self._maybe_handle_memory_commands(norm):
            return

        # =====================================================
        # L3-State：对话静音 / 恢复（不影响系统态）
        # =====================================================

        # ---- L3 unmute：允许唤醒词或明确恢复指令 ----
        if self.l3_muted:
            # 允许两种方式解除静音：
            # 1️⃣ 明确的 unmute 语义
            # 2️⃣ 唤醒词（jack）
            if (
                any(k in norm_raw for k in L3_UNMUTE_KEYWORDS) or
                (self.use_wakeword and self._is_wake_hit(raw_text))
            ):
                self.l3_muted = False
                self.get_logger().info("🔊 L3 unmute：恢复对话")

                # 👉 恢复时，让机器人看向用户（一次性）
                msg_ctrl = String()
                msg_ctrl.data = "resume"
                self.face_ctrl_pub.publish(msg_ctrl)

                self._say("好的，我可以说话了。")
                self._set_state("chat_active")

                # 👉 同时打开 L3 注意力窗口
                self._last_wake_ts = now
                self._wake_until = now + self.wake_window_s

                return

            # ❗ 静音状态下，其它输入一律忽略
            self.get_logger().debug("🔇 L3 muted，忽略语音输入")
            return

        # ---- L3 mute：进入静音（系统不休眠）----
        if any(k in norm_raw for k in L3_MUTE_KEYWORDS):
            self.l3_muted = True

            # 👉 静音时：暂停 face_follow（但系统仍在线）
            msg_ctrl = String()
            msg_ctrl.data = "pause"
            self.face_ctrl_pub.publish(msg_ctrl)

            self._say("好的，我先不说话。")
            self._set_state("chat_muted")

            self.get_logger().info("🔇 进入 L3 muted 状态")
            return

        # =====================================================
        # L1：语义动作控制（不影响系统态）
        # =====================================================
        
        # ---- 语义触发 pause ----
        if any(k in norm_raw for k in PAUSE_KEYWORDS):
            msg_ctrl = String()
            msg_ctrl.data = "pause"
            self.face_ctrl_pub.publish(msg_ctrl)
            self._say("好的，我先不动。")
            self.get_logger().info("🎯 L1：语义 pause")
            return

        # ---- 语义触发 resume ----
        if any(k in norm_raw for k in RESUME_KEYWORDS):
            msg_ctrl = String()
            msg_ctrl.data = "resume"
            self.face_ctrl_pub.publish(msg_ctrl)
            self._say("好的，我看着你。")
            self.get_logger().info("🎯 L1：语义 resume")
            return
        
        # =====================================================
        # L3：唤醒词注意力窗口（只控制“是否处理输入”）
        # - 不改变 sleeping
        # - 不控制 face_follow
        # - 不依赖 TTS done
        # =====================================================
        if self.use_wakeword:
            
            hit = self._is_wake_hit(raw_text)

            if hit and (now - self._last_wake_ts) >= self.wake_cooldown_s:
                
                first_wake = (now > self._wake_until)

                self.get_logger().info(
                    f"🟢 L3_WAKE detected | "
                    f"source=wakeword | "
                    f"first_wake={first_wake} | "
                    f"now={now:.2f}"
                )
                
                # 打开注意力窗口
                self._last_wake_ts = now
                self._wake_until = now + self.wake_window_s
                self.get_logger().info(
                    f"🕓 L3_WAKE window opened | "
                    f"until={self._wake_until:.2f} "
                    f"(duration={self.wake_window_s:.1f}s)"
                )
                
                # ✅ 只在“第一次唤醒”时，叫醒机械臂
                if first_wake:
                    msg = String()
                    msg.data = "resume"
                    self.face_ctrl_pub.publish(msg)
                    self.get_logger().info(
                        "🤖 L3_WAKE first_wake=True → "
                        "face_follow resume issued"
                    )
                else:
                    self.get_logger().debug(
                        "🤖 L3_WAKE within window → "
                        "no face_follow action"
                    )

                # 可选：打断当前 TTS
                self.tts_interrupt_pub.publish(Bool(data=True))
                self.get_logger().debug("🔕 TTS interrupt issued")

                # 去除唤醒词本体
                stripped = self._strip_wakewords(raw_text)
                if not stripped:
                    self._say("在的。")
                    self.get_logger().info("🗣️ L3_WAKE empty payload → short ack")
                    return

                raw_text = stripped
                self.get_logger().debug(
                    f"🧹 L3_WAKE stripped_text='{raw_text}'"
                )

            elif hit:
                self.get_logger().debug(
                    "⏱️ L3_WAKE ignored (cooldown active)"
                )
                return

            elif now > self._wake_until:
                self.get_logger().debug("⏱️ L3 注意力窗口已关闭，忽略输入")
                return

        # 分流
        norm = norm_text(raw_text)
        
        if self.mode == 'chat':
            self._handle_chat(norm_text=norm, raw_text=raw_text)
        else:
            self._handle_task(norm); return
       
    
    def _maybe_nod(self, reply_text: str):
        """
                  检测 LLM 回复是否为肯定句，是 → 自动发布点头命令
        """
        if not reply_text:
            return

        positive_words = ["是的", "对", "没错", "正确", "当然", "嗯", "对的"]

        if any(w in reply_text for w in positive_words):
            self.get_logger().info(f"🤖 检测到肯定回答 → 自动点头 nod")
    
            msg = String()
            msg.data = "nod"
            self.gesture_pub.publish(msg)
    
    def _maybe_shake(self, reply_text: str):
        """
                  检测 LLM 回复是否是否定句，是 → 自动摇头 shake
        """
        if not reply_text:
            return

        negative_words = ["不对", "不是", "不太对", "不可以", "不能", "不行", "错误", "否"]

        if any(w in reply_text for w in negative_words):
            self.get_logger().info(f"🤖 检测到否定回答 → 自动摇头 shake")

            msg = String()
            msg.data = "shake"
            self.gesture_pub.publish(msg)
    
    # ===== 11/8新增：监听tts_speaking话题，用于判断是否可以进入休息 =====        
    #def _on_tts_speaking(self, msg: Bool):
    #    was = self.tts_is_speaking
    #    self.tts_is_speaking = bool(msg.data)

        # 关心从 True -> False 的“播报结束”沿
    #    if was and not self.tts_is_speaking:
    #        now = time.time()
    #        # 打断后的瞬间 False 不计入（防抖）
    #        if now < self._ignore_tts_false_until:
    #            return
    
            # 若处于等待激活状态，则在“播报真正结束”时开始计时
    #        if self.pending_wake_activation:
    #            self._last_wake_ts = now
    #            self._wake_until = now + self.wake_window_s
    #            self.pending_wake_activation = False
    #            self.get_logger().info(
    #                f"🕓 播报结束 -> 开始计时 {self.wake_window_s:.1f}s 唤醒窗口"
    #            )
            
            # 成功开窗后，取消兜底定时器
    #        try:
    #            if self._wake_fallback_timer is not None:
    #                self._wake_fallback_timer.cancel()
    #        except Exception:
    #            pass
    #        self._wake_fallback_timer = None
                
    #def _on_tts_done(self, msg: Bool):
    #    if not msg or not msg.data:
    #        return
    #    now = time.time()
    
    #    if self.pending_wake_activation:
            # 唤醒词路径：开长窗
    #        self._last_wake_ts = now
    #        self._wake_until = now + self.wake_window_s
    #        self.pending_wake_activation = False
     #       self.get_logger().info(
     #           f"🕓 收到 /tts/done → 开始计时 {self.wake_window_s:.1f}s 唤醒窗口（唤醒词路径）"
    #        )
     #   else:
            # ✅ 新增：非唤醒词路径（如“停止/停这”打断，或自然结束）→ 开短窗
    #        short_win = max(0.0, float(self.after_interrupt_open_window_s))
    #        if short_win > 0:
    #            self._last_wake_ts = now
    #            self._wake_until = now + short_win
    #            self.get_logger().info(
    #                f"🕓 收到 /tts/done → 开始计时 {short_win:.1f}s 短窗口（打断/自然结束路径）"
    #            )

        # 取消兜底定时器
     #   try:
    #        if self._wake_fallback_timer is not None:
    #            self._wake_fallback_timer.cancel()
    #    except Exception:
    #        pass
    #    self._wake_fallback_timer = None
    
    # =====12/6 新增 环境扫描结果回调 =====
    def _on_env_objects(self, msg):
        
        # 🚨 休眠状态：禁止任何环境播报
        if self.sleeping:
            return
        """
                 只在“用户主动请求环境播报”时，播报一次 world_objects
        """
        # ❗ 没有请求，不处理
        if not self.env_report_pending:
            return

        # ❗ 已经播报过，不再处理
        if self.env_report_done:
            return

        try:
            summary = self._summarize_env_objects(msg)
            self._say(summary)
            self.get_logger().info(f"🗺️ 环境扫描总结: {summary}")

            # ✅ 播报完成，关闭闸门
            self.env_report_done = True
            self.env_report_pending = False
            self._set_state("env_scan_done")

        except Exception as e:
            self.get_logger().error(f"解析环境扫描结果错误: {e}")
            self._say("扫描完成，但解析失败。")
            self.env_report_done = True
            self.env_report_pending = False
    
    # ===== 12/11 优化 /env_objects 转成自然语言 =====
    def _summarize_env_objects(self, msg):
        # === 情况 1：完全没看到 ===
        if not msg or not msg.objects:
            return "我什么都没看到。"

        objects = msg.objects
        count = len(objects)

        names = []
        distances = []

        for obj in objects:
            label_en = getattr(obj, "label", "")
            label_cn = translate_class_name(label_en) or "物体"
            z = round(obj.pose.position.z, 3)

            names.append(label_cn)

            if z < 1.0:
                dist_text = speak_decimal(z)
            else:
                dist_text = f"{round(z, 2)}"

            distances.append(f"{label_cn}距离约{dist_text}米")

        # === 第一句：数量描述 ===
        if count == 1:
            summary1 = f"前面有一个{names[0]}。"
        else:
            name_list = "和".join([f"一个{name}" for name in names])
            summary1 = f"前面有{count}个物体，{name_list}。"

        # === 第二句：距离 ===
        summary2 = "，".join(distances) + "。"

        return summary1 + summary2
            
    # def _wake_fallback_open(self):
        # 仅当仍在等待激活时才生效
    #    if self.pending_wake_activation:
    #        now = time.time()
    #        self._last_wake_ts = now
    #        self._wake_until = now + self.wake_window_s
    #        self.pending_wake_activation = False
    #        self.get_logger().warning(
    #            f"🕓 Fallback 定时开启 {self.wake_window_s:.1f}s 唤醒窗口（未在 {self.wake_fallback_ms}ms 内收到 /tts/done）"
    #        )
        # 无论是否开启成功，都清理定时器
    #    try:
    #        if self._wake_fallback_timer is not None:
    #            self._wake_fallback_timer.cancel()
    #    except Exception:
    #        pass
    #    self._wake_fallback_timer = None


    # ===== Chat 模式 =====
    def _handle_chat(self, norm_text: str, raw_text: str = None):
        if is_smalltalk(norm_text):
            self._say('在的～🙂 您想聊点什么？')
            self._set_state('chat_idle'); return
        
        # === 新增：识别查看面前物体 ===
        SCAN_FRONT_KEYWORDS = [
            "看面前", "看看面前", "前面有什么", "面前有什么",
            "看前面", "看看前面", "检测前面", "检测面前",
            "看看物体", "检测物体", "看看下面", "看看下面前"
        ]
        
        # === 新增：弱语义“再扫描一次”触发 ===
        RESCAN_KEYWORDS = [
            "再看", "再看看", "你再看", "再看一下",
            "再看下", "再看一眼", "重新看",
            "你看一下", "你看下", "帮我看下"
        ]

        if any(k in norm_text for k in SCAN_FRONT_KEYWORDS):
            self._say("好的，我来看看面前有什么。")
            self.env_report_pending = True
            self.env_report_done = False
            self._publish_command("static_env_report") # ② 执行静态环境扫描
            self.get_logger().info("📢 已触发 static_env_report_node")
            return
        
        if any(k in norm_text for k in RESCAN_KEYWORDS):
            self._say("好的，我再看一下。")
            self.env_report_pending = True
            self.env_report_done = False
            self._publish_command("static_env_report")
            self.get_logger().info("📢 语义触发：重新环境扫描")
            return
        
        if "检测环境" in norm_text or "扫描环境" in norm_text or "看看周围" in norm_text:
            self._say("好的，开始扫描环境。")
            self._publish_command("env_scan")
            return

        user_for_memory = raw_text or norm_text
        use_long = self._should_long_form_this_turn(user_for_memory)

        if self.use_llm and requests is not None:
            reply = self._llm_chat(
                user_for_memory,
                long_form=use_long
            ) or ('我在呢～' if not use_long else '已为你整理。')
            reply = strip_think(reply)
            reply = strip_meta(reply)
            if not use_long:
                reply = keep_user_facing(reply)
        else:
            reply = '我在呢～（当前未启用LLM）。'

        self.get_logger().info(f"[debug] chat_after_clean={reply}")
        self._say(clamp(reply, 2000 if use_long else 600), concise=not use_long)
        
        # ======= 互斥判断：否定优先、再判断肯定 =======
        reply_clean = reply.strip().lower()

        NEG_WORDS = ["不对", "不是", "不太对", "不可以", "不能", "不行", "错误", "否"]
        POS_WORDS = ["是的", "对", "没错", "正确", "当然", "嗯", "对的"]
        
        gesture = None  # ★★★ 防止 UnboundLocalError ★★★

        # ---- 1) 否定优先 ----
        if any(w in reply_clean for w in NEG_WORDS):
            gesture = "shake"
            self.get_logger().info("🤖 自动判断：检测到否定 → 延迟摇头 shake")

        # ---- 2) 再判断肯定 ----
        elif any(w in reply_clean for w in POS_WORDS):
            gesture = "nod"
            self.get_logger().info("🤖 自动判断：检测到肯定 → 延迟点头 nod")

        # ---- 若需要动作，执行延迟触发 ----
        if gesture is not None:
            delay = self.gesture_delay_s  # 参数中默认 0.4 秒
            
            timer = None  # 关键：闭包用
            
            def _do_once():
                nonlocal timer
                msg = String()
                msg.data = gesture
                self.gesture_pub.publish(msg)
                self.get_logger().info(f"🤖 手势已触发 (delay={delay}s): {gesture}")
                
                # 重要：停止 timer，避免无限循环
                if timer is not None:
                    timer.cancel()
                    timer = None

            # 只跑一次的 timer
            timer = self.create_timer(delay, _do_once)
        
        self._remember_turn(user_for_memory, reply)

    # ===== Task 模式 =====
    def _handle_task(self, norm_text: str):
        # 先处理确认/取消
        if self.waiting_confirm:
            if any(w in norm_text for w in CONFIRM_WORDS):
                self._say('好的，开始执行。')
                self._publish_command(self.pending_cmd_text or '执行命令')
                self._reset_dialog(keep_mode=True)
                self._set_state('task_done')
                return
            if any(w in norm_text for w in CANCEL_WORDS):
                self._say('已取消，请继续描述你的任务。')
                self._reset_dialog(keep_mode=True)
                self._set_state('task_cancel')
                return

        # 抽槽
        color, klass, side = extract_slots(norm_text)
        self.slots['color'] = self.slots['color'] or color
        self.slots['klass'] = self.slots['klass'] or klass
        self.slots['side']  = self.slots['side']  or side

        # 意图判定（严格）
        if self.strict_intent:
            looks_intent = any(v in norm_text for v in INTENT_VERBS) or \
                           ((self.slots['klass'] is not None) and (self.slots['color'] is not None))
        else:
            looks_intent = any(v in norm_text for v in INTENT_VERBS) or (self.slots['klass'] or self.slots['color'])

        if not looks_intent:
            if self.llm_hint_enabled and self.use_llm and requests is not None:
                reply = self._llm_hint(norm_text) or '请描述要“拿/放/移动”的对象（颜色+类别），例如“拿红色方块放到右侧”。'
            else:
                reply = '请描述要“拿/放/移动”的对象（颜色+类别），例如“拿红色方块放到右侧”。'
            reply = strip_think(reply)
            reply = strip_meta(reply)
            reply = keep_user_facing(reply)
            self._say(clamp(reply, 200))
            self._set_state('task_waiting'); return

        # 槽位不全 → 逐个补问
        if not self.slots['klass']:
            self._say('请问要操作的目标是方块、圆柱、球还是杯子？')
            self._set_state('task_need_class'); return
        if not self.slots['color']:
            self._say('请问目标是什么颜色？例如红色、蓝色、绿色。')
            self._set_state('task_need_color'); return
        if not self.slots['side']:
            self._say('放到哪里？左侧、右侧还是中间？')
            self._set_state('task_need_side'); return

        # 槽位齐全 → 确认
        cmd_text = build_cmd_text(self.slots['color'], self.slots['klass'], self.slots['side'])
        self.pending_cmd_text = cmd_text
        self.waiting_confirm = True
        self._say(f'将执行：{cmd_text}，是否确认？（说“确认”或“取消”）')
        self._set_state('task_wait_confirm')
        
    def _should_long_form_this_turn(self, raw_text: str) -> bool:
        if self.detail_mode:
            return True
        t = (raw_text or '').strip()
        return any(kw in t for kw in self.detail_keywords)
        
    def _is_wake_hit(self, asr_text: str) -> bool:
        if not asr_text:
            return False
        t_norm = _norm_text_for_wake(asr_text)
        # 1) 词表包含式（归一化后包含即可）
        if any(w and (w in t_norm) for w in self._wake_words_norm):
            return True
        # 2) 正则匹配（原始文本更宽松）
        for rgx in self._wake_regex_compiled:
            if rgx.search(asr_text):
                return True
        return False

    def _strip_wakewords(self, text: str) -> str:
        if not text:
            return text
        out = text
        for rgx in self._strip_regex_compiled:
            out = rgx.sub("", out)
        # 清掉唤醒词前后多余的分隔符/空白
        out = re.sub(r'^[\s,，。;；:：]+', '', out)
        out = re.sub(r'[\s,，。;；:：]+$', '', out)
        return out.strip()


    # ===== LLM（聊天，带历史）=====
    def _llm_chat(self, user_text: str, long_form: bool = False) -> str:
        try:
            url = f'{self.ollama_base}/api/chat'
            msgs = self._build_messages(user_text, long_form=long_form)
            payload = {
                "model": self.model,
                "messages": msgs,
                "options": {
                    "temperature": self.temperature,
                    "num_predict": int(self.detail_max_tokens if long_form else self.max_tokens),
                    "num_ctx": int(self.num_ctx),
                },
                "stream": False
            }
            r = requests.post(url, json=payload, timeout=self.llm_timeout_s)
            self.get_logger().info(f"[debug] http_status={r.status_code}")
            self.get_logger().info(f"[debug] raw_body={r.text[:400]}")
            r.raise_for_status()
            data = r.json()
            if isinstance(data, dict):
                if "message" in data and isinstance(data["message"], dict):
                    return (data["message"].get("content") or "").strip()
                if "choices" in data and data["choices"]:
                    return (data["choices"][0]["message"]["content"] or "").strip()
                if "response" in data:
                    return (data["response"] or "").strip()
            return ''
        except Exception as e:
            self.get_logger().warning(f'LLM 聊天失败：{e}')
            return ''

    # ===== LLM（任务引导）=====
    def _llm_hint(self, text: str) -> str:
        try:
            url = f'{self.ollama_base}/api/generate'
            prompt = (
                "你是一个中文对话助手。用户刚说："
                f"「{text}」。请用中文一行话，友好地引导他给出要操作的对象（颜色+类别），"
                "不要输出 JSON，不要客套，直接提一个关键问题。"
            )
            payload = {
                "model": self.model,
                "prompt": prompt,
                "options": {
                    "temperature": self.temperature,
                    "num_predict": int(self.max_tokens),
                    "num_ctx": int(self.num_ctx),
                },
                "stream": False
            }
            r = requests.post(url, json=payload, timeout=self.llm_timeout_s)
            self.get_logger().info(f"[debug] http_status={r.status_code}")
            self.get_logger().info(f"[debug] raw_body={r.text[:400]}")
            r.raise_for_status()
            data = r.json()
            return (data.get('response') or '').strip()
        except Exception as e:
            self.get_logger().warning(f'LLM 引导失败：{e}')
            return ''

    # ======== 会话记忆：构造 messages ========
    def _build_messages(self, user_text: str, long_form: bool = False):
        msgs = []
        if long_form:
            sys_prompt = (
                '你是中文语音助手，回答**尽量完整清晰**，使用简短段落或要点列举，'
                '必要时分天/分步骤/分指标展开，避免客套与元叙述，不要输出<think>标签。'
            )
            max_chars = self.detail_max_ctx_chars
        else:
            sys_prompt = self.chat_system_prompt or '你是中文语音助手，回答简洁自然，不超过两句。'
            max_chars = self.max_ctx_chars

        msgs.append({'role':'system','content':sys_prompt})
        for m in list(self._dialog):
            msgs.append(m)
        msgs.append({'role':'user','content': user_text})

        def _sum_len(_msgs): return sum(len(m.get('content','')) for m in _msgs)
        while _sum_len(msgs) > max_chars and len(msgs) > 3:
            msgs.pop(1)
        return msgs

    # ======== 会话记忆：写入一轮 ========
    def _remember_turn(self, user_text: str, assistant_text: str):
        self._dialog.append({'role':'user','content': user_text})
        self._dialog.append({'role':'assistant','content': assistant_text})
        if self.persist_session and self._session_path:
            try:
                with open(self._session_path, 'a', encoding='utf-8') as f:
                    f.write(json.dumps({'t':time.time(), 'user':user_text, 'assistant':assistant_text}, ensure_ascii=False)+'\n')
            except Exception as e:
                self.get_logger().warning(f'会话落盘失败：{e}')

    # ======== 口令：清空/重置记忆 ========
    def _maybe_handle_memory_commands(self, norm_text: str) -> bool:
        if any(k in norm_text for k in self._reset_cmd_keywords):
            self._dialog.clear()
            self.get_logger().info('🧹 已清空会话记忆')
            self._say('好的，已清空对话记忆。')
            self._set_state('chat_memory_cleared')
            return True
        return False

    # ===== 发布层 =====
    def _say(self, text: str, concise: bool = True):
        if not text:
            return
        text = strip_think(text)
        text = strip_meta(text)
        if concise:
            text = keep_user_facing(text)
        text = strip_markdown_to_speech(text)

        now = time.time()
        #if (now - self._last_reply_ts) < self._min_reply_interval:
        #    self.get_logger().info(f"⏱️ 忽略过密播报({self._min_reply_interval}s内): {text}")
        #    return
        if text == self._last_reply and (now - self._last_reply_ts) < self._reply_dedup_window:
            self.get_logger().info(f"🗨️ 忽略重复播报({self._reply_dedup_window}s内): {text}")
            return

        self._last_reply = text
        self._last_reply_ts = now
        self.pub_reply.publish(String(data=text))
        self.get_logger().info(f'🗨️ /speech_reply <= {text}')
        self.get_logger().info(f"[debug] chat_clean={text}")

    def _publish_command(self, text: str):
        if not text: return
        msg = String(); msg.data = text
        for pub in self.pub_texts:
            pub.publish(msg)
        self.get_logger().info(f'✅ 下发命令到 {self.publish_topics}: {text}')

    def _reset_dialog(self, keep_mode=False):
        self.waiting_confirm = False
        self.pending_cmd_text = ''
        self.slots = {'color':None, 'klass':None, 'side':None}
        if not keep_mode:
            self.mode = 'chat'
        self._set_state('idle' if self.mode=='task' else 'chat_idle')

    # ===== 状态 =====
    def _set_state(self, s: str):
        self.state_pub.publish(String(data=s))

def main():
    rclpy.init()
    node = LlmVoiceAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

