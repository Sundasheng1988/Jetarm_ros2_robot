# ASR
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import io
import time
import queue
import tempfile
import threading
from typing import Optional, Tuple

import numpy as np
import sounddevice as sd
import webrtcvad

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from funasr import AutoModel
import difflib
import re
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType


def _expand(path: str) -> str:
    if not isinstance(path, str):
        return path
    return os.path.expanduser(os.path.expandvars(path))


def _must_dir(p: str, desc: str):
    if not os.path.isdir(p):
        raise FileNotFoundError(f"[FunASR-Offline] 本地{desc}不存在：{p}")
    return p


class FunASREngine:
    """
    FunASR 封装（强制离线）：
    - 用“本地目录”加载 ASR/VAD/PUNC（三者都必须是本地绝对路径）
    - 失败时回退临时 WAV
    """
    def __init__(
        self,
        model_path: str,
        vad_path: Optional[str],
        punc_path: Optional[str],
        device: str = "cpu",
    ):
        self.model_path = _must_dir(_expand(model_path), "主模型")
        self.vad_path = _must_dir(_expand(vad_path), "VAD") if vad_path else None
        self.punc_path = _must_dir(_expand(punc_path), "标点") if punc_path else None

        self.model = AutoModel(
            model=self.model_path,
            vad_model=self.vad_path,
            punc_model=self.punc_path,
            device=device,          # "cpu" or "cuda"
            disable_update=True,    # 🔒 关闭版本检查（彻底离线）
        )

    def infer_pcm(self, pcm_bytes: bytes, sample_rate: int) -> Tuple[str, float]:
        # 方案1：直接字节流
        try:
            res = self.model.generate(
                input=pcm_bytes,
                fs=sample_rate,
                language="zh",
                use_itn=True,
            )
            return self._parse_result(res)
        except Exception:
            # 方案2：临时 WAV
            try:
                import soundfile as sf
            except Exception as e:
                raise RuntimeError(f"[FunASR] 无法导入 soundfile 以写 WAV：{e}")

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as fp:
                data = np.frombuffer(pcm_bytes, dtype=np.int16)
                sf.write(fp.name, data, samplerate=sample_rate, subtype="PCM_16")
                fp.flush()
                res = self.model.generate(
                    input=fp.name,
                    language="zh",
                    use_itn=True,
                )
                return self._parse_result(res)

    @staticmethod
    def _parse_result(res) -> Tuple[str, float]:
        if isinstance(res, list) and len(res) > 0 and isinstance(res[0], dict):
            d = res[0]
            return (d.get("text", "").strip(), float(d.get("confidence", 1.0)))
        if isinstance(res, dict):
            return (str(res.get("text", "")).strip(), float(res.get("confidence", 1.0)))
        if isinstance(res, str):
            return (res.strip(), 1.0)
        return ("", 0.0)

def _read_str_array_param(node: Node, name: str, default: list[str] | None = None) -> list[str]:
    """
    读取字符串数组参数：
    - YAML: wake_words: ["rebecca", "瑞贝卡"]  ✅
    - 字符串: wake_words: "rebecca,瑞贝卡"     ✅
    - 未提供：使用 default
    """
    if default is None:
        default = []

    # 先按“数组”声明，若 YAML 给的是数组，类型保持为 STRING_ARRAY
    node.declare_parameter(name, default)
    p = node.get_parameter(name).get_parameter_value()

    if p.type == ParameterType.PARAMETER_STRING_ARRAY:
        arr = list(p.string_array_value)
    elif p.type == ParameterType.PARAMETER_STRING:
        s = p.string_value or ""
        arr = [w.strip() for w in s.split(",") if w.strip()]
    else:
        arr = list(default)

    # 归一化：去重、去空
    out, seen = [], set()
    for w in arr:
        t = w.strip()
        if not t or t in seen:
            continue
        out.append(t)
        seen.add(t)
    return out


# ============================================================
# Voice Mode 状态机（Voice Stabilization Patch 1）
#
# VOICE_MODE_NORMAL    正常工作：全部语音照旧进入 /speech_query
# VOICE_MODE_MUTED     静音：普通语音全部丢弃，仅放行 控制命令
# VOICE_MODE_SLEEPING  休眠：普通语音全部丢弃，仅放行 唤醒 + Robot Stop
#
# 检测顺序（所有模式一致，Robot Stop 最高优先级）：
#   1. robot stop
#   2. wake / sleep / mute 模式控制
#   3. confirmation（保留原有导航确认逻辑）
#   4. normal speech
#
# 注意：不关麦克风、不停 FunASR——休眠下仍需要语音唤醒。
# ============================================================
VOICE_MODE_NORMAL = 'voice_mode_normal'
VOICE_MODE_MUTED = 'voice_mode_muted'
VOICE_MODE_SLEEPING = 'voice_mode_sleeping'

# Robot Stop 短语：与 llm_voice_agent_node.NAV_STOP_PHRASES 对齐（只读对齐，
# 不修改 Agent）。整句精确匹配，覆盖用户指定的：
#   “停止移动 / 停止导航 / 取消导航 / 别走了”
VOICE_STOP_PHRASES = frozenset([
    '停止移动', '停止导航', '取消导航', '取消移动',
    '别走了', '别走啦', '别走', '不要走了', '不要走',
    '停下移动', '停下导航',
])

VOICE_STOP_PREFIX_SAFE = frozenset({
    '停止移动',
    '停止导航',
    '取消导航',
    '取消移动',
    '停下移动',
    '停下导航',
})

# ============================================================
# Voice Control Canonical Commands
#
# L2:
#   瑞贝卡，系统休眠
#   瑞贝卡，启动系统
#
# L3:
#   瑞贝卡，静音
#   瑞贝卡，取消静音
#
# 下面保留少量自然语言 alias，但 ASR / Agent 必须保持一致。
# ============================================================

VOICE_MUTE_ENTER_KEYWORDS = (
    '静音',
    '别说话',
    '不要说话',
    '安静',
    '我在拍视频',
)

VOICE_UNMUTE_KEYWORDS = (
    '取消静音',
    '瑞贝卡取消静音',
    '可以说话了',
    '瑞贝卡可以说话了',
    '继续说话',
    '恢复对话',
    '你可以说话了',
)

VOICE_SLEEP_ENTER_KEYWORDS = (
    '瑞贝卡系统休眠',
    'rebecca系统休眠',
    '瑞贝卡请先休息吧',
    'rebecca请先休息吧',
    '瑞贝卡你先休息吧',
    'rebecca你先休息吧',
)

VOICE_WAKE_KEYWORDS = (
    '瑞贝卡启动系统',
    'rebecca启动系统',

    '瑞贝卡系统启动',
    'rebecca系统启动',

    '瑞贝卡唤醒系统',
    'rebecca唤醒系统',

    '瑞贝卡系统唤醒',
    'rebecca系统唤醒',

    '瑞贝卡恢复系统',
    'rebecca恢复系统',

    '瑞贝卡系统恢复',
    'rebecca系统恢复',
)

# ============================================================
# Patch 4A：Self-Echo Control Hardening
#
# TTS 播放期间（interrupt-listen-only）的全部控制匹配都改为
# “归一化 + 剥离唤醒前缀 + 整句精确匹配”，禁止 startswith /
# endswith / substring，防止 Rebecca 自身 TTS 回声碎片
# （“确认开始导航”“或取消放弃”“正在停止移动”）误触
# 确认 / 取消 / 打断。
# ============================================================

# TTS-time 导航确认：仅强确认词。
# FunASR 无真实 confidence 时默认 1.0，conf 门只能作辅助条件，
# 安全边界由窄集合承担。弱确认（好/好的/行/ok/是的/没问题）
# 只保留在 Agent 正常 listening 的 _NAV_CONFIRM_REPLIES，
# 不开放给 TTS-time barge-in。
VOICE_TTS_NAV_CONFIRM_REPLIES = frozenset({
    '确认',
    '我确认',
    '确定',
    '我确定',
    '可以',
    '执行',
    '是的',
    '对的',
    '没错',
})

VOICE_TTS_NAV_CONFIRM_SUFFIX_REPLIES = frozenset({
    '确认',
    '确定',
    '可以',
    '执行',
    '是的',
    '对的',
    '没错',
})

VOICE_TTS_QUESTION_TAILS = (
    '吗',
    '么',
    '呢',
    '吧',
)

# TTS-time 导航取消：整句精确匹配，命中后只把规范文本“取消”
# 发给 Agent，不转发带噪声的原始 ASR 句子。
VOICE_TTS_NAV_CANCEL_REPLIES = frozenset({
    '取消', '不去', '不要', '别去', '先不要', '算了',
    '不用', '不用了', '不去了', '别去了', '先不去', '不确认',
})

# 普通 TTS 打断（仅停播报，不进 Agent）：整句精确匹配，
# 与 Robot STOP（VOICE_STOP_PHRASES）完全分离。
# 必须排除 '停' / '停止' / '好了'，否则 Rebecca 播报
# “正在停止移动”会自己打断自己。
VOICE_TTS_STOP_PHRASES = frozenset({
    '打断', '打住', '停一下', '别说了', '别播了', '别念了',
    '别继续了', '够了', '先这样', '先这样吧', '暂停', '暂停一下',
    '停止播放', '停止播报', '等一下', '等会', '等会儿',
    '闭嘴', 'stop', 'pause',
})

VOICE_TTS_STOP_FORBIDDEN = frozenset({'停', '停止', '好了'})

# Patch 4A.3：
# 真实 barge-in 时，用户控制词可能与 Rebecca 尾音被 ASR 合并。
# 仅对足够明确的 TTS-stop 短语开放“句首匹配”。
#
# 不包含：
#   停 / 停止 / 好了
# 也暂不把“等一下 / 等会 / 先这样”开放为 prefix，
# 因为这些更容易出现在正常聊天内容中。
VOICE_TTS_STOP_PREFIX_SAFE = frozenset({
    '打断',
    '打住',
    '停一下',
    '别说了',
    '别播了',
    '别念了',
    '别继续了',
    '够了',
    '暂停一下',
    '停止播放',
    '停止播报',
    '闭嘴',
    'stop',
    'pause',
})


def classify_tts_control_reply(norm_reply: str, agent_state: str):
    """TTS-time navigation control classifier.

    安全原则：
    1. 只有 nav_wait_confirm 才允许确认/取消；
    2. 干净回复继续 exact match；
    3. 对“Rebecca 问句尾音 + 用户确认”只开放非常窄的 suffix 规则；
    4. 不做任意 substring matching。
    """
    if agent_state != 'nav_wait_confirm':
        return None

    if not norm_reply:
        return None

    # clean exact confirmation
    if norm_reply in VOICE_TTS_NAV_CONFIRM_REPLIES:
        return 'confirm'

    # clean exact cancellation
    if norm_reply in VOICE_TTS_NAV_CANCEL_REPLIES:
        return 'cancel'

    # --------------------------------------------------------
    # Acoustic boundary case:
    #
    # Rebecca: "要让 Eric 去餐厅吗？"
    # User:    "是的"
    #
    # ASR may produce:
    #   "要让eric去餐厅吗是的"
    #
    # Only accept when:
    #   - final suffix is an explicit confirmation reply
    #   - preceding text itself ends like a question
    #
    # Thus this is NOT arbitrary endswith confirmation.
    # --------------------------------------------------------
    for reply in sorted(
        VOICE_TTS_NAV_CONFIRM_SUFFIX_REPLIES,
        key=len,
        reverse=True,
    ):
        if not norm_reply.endswith(reply):
            continue

        prefix = norm_reply[:-len(reply)]

        if (
            prefix
            and prefix.endswith(VOICE_TTS_QUESTION_TAILS)
        ):
            return 'confirm'

    return None


def is_tts_stop_phrase(norm_reply: str, stop_phrases) -> bool:
    """TTS 播放期间的安全打断判断。

    1. 所有配置短语继续支持整句 exact match；
    2. 仅对明确、安全的强打断词支持句首匹配，
       解决用户 barge-in 与 Rebecca 尾音被 ASR 合并的问题；
    3. 不做任意 substring 匹配，避免重新引入 self-echo 误触发。
    """
    if not norm_reply:
        return False

    if norm_reply in stop_phrases:
        return True

    return any(
        phrase in stop_phrases
        and (
            norm_reply.startswith(phrase)
            or norm_reply.endswith(phrase)
        )
        for phrase in VOICE_TTS_STOP_PREFIX_SAFE
    )


def merged_utt_restricted(current_restricted: bool, captured_restricted: bool) -> bool:
    """Patch 4A.1：utterance 级 restricted 锁存。

    utterance 内任意一帧在“仅打断监听”期间采集，整个 utterance 即按
    restricted 处理；之后全局 _interrupt_listen_only 变回 False（TTS
    播完）也不能把它切回正常阈值 / NORMAL 发布分支。
    """
    return bool(current_restricted or captured_restricted)


class SpeechDialogFunASR(Node):
    """
    麦克风 -> WebRTC VAD -> 端点检测 -> FunASR 识别 -> 发布至 topic_out
    特性：
      - ✅ TTS 播放期间**仅监听打断关键词**（其它语音一律丢弃，防回听）
      - 动态静音（按播报文本估时，带上下限）
      - 多轮续听（followup_window_s）、说话续杯（active_extend_s）
      - 自语抑制、短句过滤、口头语清理
      - 进入休眠时友好提醒
      - “唤醒直通”：命中唤醒词时**也发布原句**给 agent（双保险）
      - ✅ Voice Mode 状态机：NORMAL / MUTED / SLEEPING
        （静音/休眠下普通语音在源头丢弃，仅放行控制命令；
         Robot Stop 在任何模式下都放行并固定下发“停止移动”）
    """

    def __init__(self):
        super().__init__("speech_dialog_funasr_node")

        # ===== 设备与采样 =====
        self.device = self.declare_parameter("device", "cpu").get_parameter_value().string_value  # "cpu"/"cuda"
        self.device_index = self.declare_parameter("device_index", -1).get_parameter_value().integer_value
        self.sample_rate = self.declare_parameter("sample_rate", 16000).get_parameter_value().integer_value

        # ===== 本地模型路径（必须是“目录”而非仓库名）=====
        base_dir = _expand(self.declare_parameter(
            "base_dir", "~/ros2_ws/models/funasr").get_parameter_value().string_value)

        model_path = self.declare_parameter(
            "model_path",
            "iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch"
        ).get_parameter_value().string_value
        vad_path = self.declare_parameter(
            "vad_path",
            "iic/speech_fsmn_vad_zh-cn-16k-common-pytorch"
        ).get_parameter_value().string_value
        punc_path = self.declare_parameter(
            "punc_path",
            "iic/punc_ct-transformer_cn-en-common-vocab471067-large"
        ).get_parameter_value().string_value

        self.model_path = _expand(model_path) if model_path.startswith("/") else os.path.join(base_dir, model_path)
        self.vad_path = _expand(vad_path) if vad_path.startswith("/") else os.path.join(base_dir, vad_path)
        self.punc_path = _expand(punc_path) if punc_path.startswith("/") else os.path.join(base_dir, punc_path)

        self.get_logger().info(
            f"✅ 使用本地模型：\n  ASR : {self.model_path}\n  VAD : {self.vad_path}\n  PUNC: {self.punc_path}"
        )

        # ===== VAD 与端点 =====
        self.frame_ms = int(self.declare_parameter("frame_ms", 20).get_parameter_value().integer_value)  # 10/20/30
        self.vad_aggressiveness = int(self.declare_parameter("vad_aggressiveness", 3).get_parameter_value().integer_value)  # 0..3
        self.min_utt_ms = int(self.declare_parameter("min_utt_ms", 600).get_parameter_value().integer_value)
        self.max_sil_ms = int(self.declare_parameter("max_sil_ms", 600).get_parameter_value().integer_value)
        self.max_utt_ms = int(self.declare_parameter("max_utt_ms", 6000).get_parameter_value().integer_value)
        # ✅ 11/8 新增：打断模式的“最短语音时长”阈值（默认 120ms）
        self.interrupt_min_utt_ms = int(self.declare_parameter("interrupt_min_utt_ms", 120).get_parameter_value().integer_value)
        # 打断监听不能再硬截 900ms，否则“瑞贝卡，停止说话”可能只识别到前半句。
        # 静音 endpoint 仍可用较短阈值提前完成识别，无需等满 1800ms。
        self.interrupt_max_utt_ms = int(
            self.declare_parameter("interrupt_max_utt_ms", 1800)
                .get_parameter_value().integer_value
        )
        self.interrupt_max_sil_ms = int(
            self.declare_parameter("interrupt_max_sil_ms", 100)
                .get_parameter_value().integer_value
        )

        # ===== 唤醒 & 跟随 =====
        # 兼容两种命名：enable_wakeup / use_wakeword（launch 里传的是 use_wakeword）
        _enable_default = True
        # self.enable_wakeup = bool(self.declare_parameter("enable_wakeup", _enable_default).get_parameter_value().bool_value)
        # use_wakeword_alias = bool(self.declare_parameter("use_wakeword", self.enable_wakeup).get_parameter_value().bool_value)
        # self.enable_wakeup = use_wakeword_alias

        # 兼容 wakeup_window_s / wake_window_s
        _default_wake = 30.0
        wakeup_window_s_primary = float(self.declare_parameter("wakeup_window_s", _default_wake).get_parameter_value().double_value)
        wakeup_window_s_alias  = float(self.declare_parameter("wake_window_s", wakeup_window_s_primary).get_parameter_value().double_value)
        self.wakeup_window_s = float(wakeup_window_s_alias if wakeup_window_s_alias > 0 else wakeup_window_s_primary)

        self.followup_window_s = float(self.declare_parameter("followup_window_s", 20.0).get_parameter_value().double_value)
        self.active_extend_s   = float(self.declare_parameter("active_extend_s", 10.0).get_parameter_value().double_value)

        # 唤醒词（含常见识别变体）
        self.wake_words = _read_str_array_param(self, "wake_words", default=["rebecca", "瑞贝卡"])
        
        # ===== 过滤阈值 =====
        self.min_avg_conf = float(self.declare_parameter("min_avg_conf", 0.70).get_parameter_value().double_value)
        self.min_text_len = int(self.declare_parameter("min_text_len", 4).get_parameter_value().integer_value)

        # ===== 话题 =====
        self.boot_prompt = self.declare_parameter("boot_prompt", "").get_parameter_value().string_value
        self.topic_out   = self.declare_parameter("topic_out", "/speech_query").get_parameter_value().string_value
        self.reply_topic = self.declare_parameter("reply_topic", "/speech_reply").get_parameter_value().string_value

        # === 这里是关键：创建发布/订阅器（修复 AttributeError） ===
        self.pub_query = self.create_publisher(String, self.topic_out, 10)
        self.pub_reply = self.create_publisher(String, self.reply_topic, 10)
        self.create_subscription(String, self.reply_topic, self._on_tts_reply, 10)
        self.create_subscription(Bool, '/tts_speaking', self._on_tts_speaking, 10)

        # ===== Agent 状态：用于安全放行导航确认词 =====
        self._voice_agent_state = ''
        self.create_subscription(
            String,
            '/voice_agent/state',
            self._on_voice_agent_state,
            10
        )

        # ===== Voice Mode 状态机（Patch 1）：本地权威门控 =====
        # NORMAL：全部语音照旧进入 /speech_query
        # MUTED / SLEEPING：普通语音在源头丢弃，仅放行控制命令
        # 注意：不关麦克风、不停 FunASR，休眠下仍可语音唤醒
        self._voice_mode = VOICE_MODE_NORMAL

        # ===== 自语抑制 & 动态静音参数 =====
        self.self_speech_window_s = float(self.declare_parameter("self_speech_window_s", 4.0).get_parameter_value().double_value)
        self.tts_chars_per_sec    = float(self.declare_parameter("tts_chars_per_sec", 6.0).get_parameter_value().double_value)
        self.tts_mute_margin_s    = float(self.declare_parameter("tts_mute_margin_s", 1.5).get_parameter_value().double_value)
        self.mute_until = 0.0
        self.last_tts_text = ""
        self.last_tts_time = 0.0

        # 更稳的估算 + 上下限
        self.tts_sentence_silence_ms = int(self.declare_parameter("tts_sentence_silence_ms", 120).get_parameter_value().integer_value)
        self.min_dynamic_mute_s = float(self.declare_parameter("min_dynamic_mute_s", 4.0).get_parameter_value().double_value)
        self.max_dynamic_mute_s = float(self.declare_parameter("max_dynamic_mute_s", 15.0).get_parameter_value().double_value)

        # ===== TTS 门控 =====
        self.respect_tts_gate    = bool(self.declare_parameter("respect_tts_gate", True).get_parameter_value().bool_value)
        self.tts_gate_release_ms = int(self.declare_parameter("tts_gate_release_ms", 80).get_parameter_value().integer_value)
        self._tts_speaking = False
        self._tts_last_off_ts = 0.0

        # ===== 语音打断（新增）=====
        self.enable_voice_interrupt = bool(self.declare_parameter("enable_voice_interrupt", True).get_parameter_value().bool_value)
        # Patch 4A：默认集合与 VOICE_TTS_STOP_PHRASES 对齐，
        # 不再包含 '停' / '停止' / '好了' 等会误伤 Rebecca 自身
        # 播报（“正在停止移动”）的单字/短词。
        interrupt_words_str = self.declare_parameter(
            "interrupt_words",
            "打断,打住,停一下,别说了,别播了,别念了,别继续了,"
            "够了,先这样,先这样吧,暂停,暂停一下,停止播放,停止播报,"
            "等一下,等会,等会儿,闭嘴,stop,pause"
        ).get_parameter_value().string_value
        self.interrupt_words = [w.strip() for w in interrupt_words_str.split(",") if w.strip()]
        self.interrupt_topic = self.declare_parameter("interrupt_topic", "/tts/interrupt").get_parameter_value().string_value
        self.pub_interrupt = self.create_publisher(Bool, self.interrupt_topic, 10)

        # Patch 4A：普通 TTS 打断改为“整句精确匹配”集合。
        # 1) “瑞贝卡，别说了”通过先 strip wake prefix 再 exact
        #    match 实现，不再使用宽泛的 wakeword+0~4字 combo regex。
        # 2) 禁止 '停' / '停止' / '好了' 进入有效集合：即使参数
        #    重新带入，也在启动时过滤并打印 warning，不静默接受。
        self._wake_words_lc = [w.lower() for w in self.wake_words]

        self._tts_stop_phrases = set()
        for w in self.interrupt_words:
            n = self._norm_control_text(w)
            if not n:
                continue
            if n in VOICE_TTS_STOP_FORBIDDEN:
                self.get_logger().warning(
                    f"⚠️ interrupt_words 含禁止短语 '{w}'"
                    f"（会误伤自播报回声），已过滤"
                )
                continue
            self._tts_stop_phrases.add(n)

        # ===== 休眠提醒（补回）=====
        self.sleep_notice = bool(self.declare_parameter("sleep_notice", True).get_parameter_value().bool_value)
        self.sleep_prompt = self.declare_parameter(
            "sleep_prompt", "好的，我先休息啦，需要我时说一声“你好”。"
        ).get_parameter_value().string_value

        # ===== VAD 初始化 =====
        if self.frame_ms not in (10, 20, 30):
            self.get_logger().warn(f"frame_ms={self.frame_ms} 非 10/20/30，自动改为 20ms")
            self.frame_ms = 20
        self.frames_per_chunk = int(self.sample_rate * (self.frame_ms / 1000.0))
        self.bytes_per_chunk = self.frames_per_chunk * 2
        self.vad = webrtcvad.Vad(self.vad_aggressiveness)

        # ===== 音频流 =====
        self.q = queue.Queue(maxsize=50)
        self.stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=self.frames_per_chunk,
            dtype="int16",
            channels=1,
            callback=self._on_audio,
            device=self.device_index if self.device_index >= 0 else None,
        )

        # ===== ASR 引擎（离线路径）=====
        try:
            self.asr = FunASREngine(
                model_path=self.model_path,
                vad_path=self.vad_path,
                punc_path=self.punc_path,
                device="cuda" if self.device.lower().startswith("cuda") else "cpu",
            )
        except Exception as e:
            self.get_logger().error(f"FunASR 本地模型加载失败：{e}")
            raise

        # ===== 状态 =====
        # self.active = not self.enable_wakeup
        # self.active_until = 0.0
        # self.last_wakeup = 0.0
        
        # === 11/8 等待 LLM 回复的守护 ===
        self.llm_reply_timeout_s = float(self.declare_parameter("llm_reply_timeout_s", 25.0).get_parameter_value().double_value)
        self.awaiting_reply = False

        # ===== 端点跟踪 =====
        self.speeching = False
        self.cur_pcm = io.BytesIO()
        self.ms_in_cur_utt = 0
        self.ms_sil = 0

        # 仅打断监听模式开关（在主循环里动态决定）
        self._interrupt_listen_only = False
        # Patch 4A.1：utterance 级 restricted 锁存（随语音段累积，随段复位）
        self._utt_restricted = False

        # ===== 线程 =====
        self._stop = threading.Event()
        self.worker = threading.Thread(target=self._main_loop, daemon=True)

        # ===== 启动 =====
        self.stream.start()
        self.worker.start()
        if self.boot_prompt:
            self._publish(self.boot_prompt)

        self.get_logger().info(
            f"✅ speech_dialog_funasr_node 启动: fs={self.sample_rate}Hz, frame={self.frame_ms}ms, device={self.device}"
        )

    # ---------------- 回调与工具 ----------------
    def _on_audio(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"audio status: {status}")

        # 以往：TTS 期间直接 return（→ 无法打断）
        # 现在：若开启 voice interrupt，则仍入队，但主循环只做“打断识别”
        if self.respect_tts_gate and self._tts_speaking and not self.enable_voice_interrupt:
            return

        try:
            b = indata.tobytes() if hasattr(indata, "tobytes") else bytes(indata)
            if len(b) != self.bytes_per_chunk:
                self.get_logger().debug(f"chunk bytes={len(b)} != {self.bytes_per_chunk}")
            # Patch 4A.1：帧入队时记录采集时刻是否处于“仅打断监听”，
            # 主循环据此做 utterance 级锁存，防止 TTS→NORMAL 边界
            # 跨界混音在 TTS 结束后被当成正常语音处理。
            captured_restricted = bool(self._interrupt_listen_only)
            self.q.put_nowait((b, captured_restricted))
        except queue.Full:
            try:
                _ = self.q.get_nowait()
            except queue.Empty:
                pass
            try:
                self.q.put_nowait((b, captured_restricted))
            except Exception:
                pass

    def _publish(self, text: str):
        if not text:
            return
        msg = String()
        msg.data = text
        self.pub_query.publish(msg)
        self.get_logger().info(f"🗣️ 发布: {text}")
        
        # ✅ 11/8关键：进入等待 LLM 回复状态，并拉长本地激活窗口
        self.awaiting_reply = True
        now = time.time()
        # if self.enable_wakeup:
        #     self.active = True
        #     self.active_until = max(self.active_until, now + self.llm_reply_timeout_s)

    def _on_tts_reply(self, msg: String):
        
        """收到 Agent 文本回复：设置兜底动态静音，并在播完后进入续听窗口"""
        self.awaiting_reply = False #11/8 新增
        t = (msg.data or "").strip()
        if not t:
            return

        now = time.time()
        self.last_tts_text = t
        self.last_tts_time = now

        # === 动态静音估算（字数/语速 + 分句停顿 + 余量），并做上下限约束 ===
        chars = len(re.sub(r'\s+', '', t))
        sents = max(1, len(re.findall(r'[。！？.!?]', t)))

        est = chars / max(self.tts_chars_per_sec, 1e-3)
        est += (sents - 1) * (self.tts_sentence_silence_ms / 1000.0)
        est += self.tts_mute_margin_s
        est = max(self.min_dynamic_mute_s, min(est, self.max_dynamic_mute_s))

        self.mute_until = max(self.mute_until, now + est)
        self.get_logger().info(f"🔇 动态静音(估算) {est:.2f}s；硬门控靠 /tts_speaking（至 {self.mute_until:.2f}）")

        # ✅ 播报完后，自动开启续谈窗口
        # if self.enable_wakeup:
        #     self.active = True
        #     self.active_until = max(self.active_until, self.mute_until + self.followup_window_s)

    def _apply_vad_mode_for_context(self):
        """根据当前语音上下文选择 VAD 灵敏度。

        - TTS 播放期间：mode 0，保证 barge-in / STOP 能被抓到
        - nav_wait_confirm：mode 0，保证“是的 / 可以 / 取消”等短回复
        在 TTS 结束后仍能被可靠检测
        - 其它正常场景：恢复用户配置的 vad_aggressiveness
        """
        fast_listen = (
            self._tts_speaking
            or self._voice_agent_state == 'nav_wait_confirm'
        )

        mode = 0 if fast_listen else self.vad_aggressiveness

        try:
            self.vad.set_mode(mode)
        except Exception:
            pass

    def _on_voice_agent_state(self, msg: String):
        """只读保存 Agent 当前状态，用于控制类短语的安全门控。"""
        old_state = self._voice_agent_state
        self._voice_agent_state = (msg.data or '').strip()

        # Patch 4B.1:
        # nav_wait_confirm 本身进入短控制语音监听模式。
        if self._voice_agent_state != old_state:
            self._apply_vad_mode_for_context()

            self.get_logger().debug(
                f"🎚️ Agent state: {old_state!r} -> "
                f"{self._voice_agent_state!r}; "
                f"VAD fast={self._voice_agent_state == 'nav_wait_confirm'}"
            )

    def _on_tts_speaking(self, msg: Bool):
        """TTS 正在说话门控：True 时立即清空端点缓冲；False 延时释放。"""
        if not self.respect_tts_gate:
            return

        speaking = bool(msg.data)
        now = time.time()
        self._tts_speaking = speaking

        if speaking:
            # 进入播报：切到“仅打断监听”模式
            self._interrupt_listen_only = self.enable_voice_interrupt
            self._reset_vad_buffers_if_any()

            # TTS / nav confirmation 共用上下文 VAD 策略
            self._apply_vad_mode_for_context()

            self.mute_until = max(
                self.mute_until,
                now + self.tts_gate_release_ms / 1000.0
            )
            self.get_logger().debug(
                "🔒 TTS 正在说话：ASR 暂停（仅保留打断监听）"
            )
        else:
            self._interrupt_listen_only = False
            self._tts_last_off_ts = now

            # 关键：
            # 如果仍处于 nav_wait_confirm，不恢复 aggressive VAD，
            # 继续保持短控制词敏感监听。
            self._apply_vad_mode_for_context()

            self.get_logger().debug(
                "🔓 TTS 结束说话：ASR 恢复；"
                f"nav_wait_confirm="
                f"{self._voice_agent_state == 'nav_wait_confirm'}"
            )

    def _reset_vad_buffers_if_any(self):
        """安全清空当前语音段与端点累计；不打印噪声性日志。"""
        self.cur_pcm = io.BytesIO()
        self.ms_in_cur_utt = 0
        self.ms_sil = 0
        self.speeching = False
        # Patch 4A.1：语音段清空时同步复位 restricted 锁存
        self._utt_restricted = False

    # ---------------- Voice Mode 状态机（Patch 1） ----------------
    @staticmethod
    def _norm_control_text(text: str) -> str:
        """控制命令归一化：去中英文标点/空白 + 小写（与 Agent 侧风格一致）。"""
        return re.sub(r'[，。！!？?、；;：:,.\s]+', '', text or '').lower()

    def _strip_wake_prefix(self, n: str) -> str:
        """容忍“唤醒词 + 前缀”的控制命令，如“瑞贝卡，停止移动”。"""
        for w in self._wake_words_lc:
            if w and n.startswith(w):
                return n[len(w):]
        return n

    def _is_recent_tts_echo(self, text: str) -> bool:
        """判断 restricted Robot STOP prefix 是否很可能来自 Rebecca TTS 回声。

        这里只会在：
            allow_prefix=True
            -> utt_restricted=True
        的 Robot STOP prefix 分支中调用。

        因此 utterance 级 restricted 锁存本身已经证明：
        当前语音与 TTS 播放发生了声学重叠，不再额外依赖
        self_speech_window_s 的墙钟时间窗口。

        exact Robot STOP 不经过本判断，始终保持最高优先级。
        """
        if not self.last_tts_text:
            return False

        a = self._norm_control_text(text)
        b = self._norm_control_text(self.last_tts_text)

        if not a or not b:
            return False

        sim = difflib.SequenceMatcher(
            None,
            a,
            b,
        ).ratio()

        return (
            a in b
            or b in a
            or sim >= 0.55
        )

    def _handle_robot_stop(
        self,
        text: str,
        *,
        allow_prefix: bool = False,
    ) -> bool:
        """检测顺序第 1 位：Robot Stop，任何 Voice Mode 下都必须放行。

        NORMAL listening:
            只接受规范 Robot STOP 整句，避免：
            “停止移动是什么意思”
            “停止导航怎么用”
            等普通讨论句误触机器人停止。

        TTS / boundary restricted utterance:
            允许有限 prefix-safe 匹配，以容忍：
            “用户 STOP + Rebecca TTS 尾音”
            被 ASR 合并的真实声学情况。

        Safety ordering:
            1. exact STOP 永远最高优先级；
            2. prefix STOP 若疑似 Rebecca 最近 TTS 回声则禁止；
            3. 非回声 prefix 才允许作为 barge-in Robot STOP。
        """
        n = self._strip_wake_prefix(
            self._norm_control_text(text)
        )
        n = re.sub(r'[吧呢啊呀]+$', '', n)

        # -----------------------------------------------------
        # 1) Exact Robot STOP
        #
        # 任意 Voice Mode、任意 TTS 状态下始终最高优先级。
        # 即使当前 TTS 内容恰巧包含相同文本，也宁可安全停止。
        # -----------------------------------------------------
        exact_stop = n in VOICE_STOP_PHRASES

        if exact_stop:
            self.get_logger().info(
                f"🛑 [VoiceMode:{self._voice_mode}] "
                f"Robot STOP exact 命中：'{text}'"
            )

            self._publish('停止移动')
            self.pub_interrupt.publish(Bool(data=True))

            self.mute_until = time.time() + 0.5
            return True

        # -----------------------------------------------------
        # 2) Prefix Robot STOP
        #
        # 只在 TTS / boundary restricted utterance 中开放。
        # -----------------------------------------------------
        prefix_candidate = (
            allow_prefix
            and any(
                n.startswith(phrase)
                for phrase in VOICE_STOP_PREFIX_SAFE
            )
        )

        if not prefix_candidate:
            return False

        # -----------------------------------------------------
        # 3) Prefix self-echo guard
        #
        # Rebecca 自己正在说：
        #   “停止移动就是保持当前位置……”
        #
        # ASR 可能得到：
        #   “停止移动，就是保持当。”
        #
        # 这种情况虽然满足 prefix，却不能触发 Robot STOP。
        #
        # 注意 exact STOP 已在上面提前放行，因此不会削弱：
        #   用户：“停止移动”
        # 的最高优先级安全能力。
        # -----------------------------------------------------
        if self._is_recent_tts_echo(text):
            self.get_logger().info(
                f"🛡️ Robot STOP prefix self-echo suppressed："
                f"'{text}'"
            )
            return False

        # -----------------------------------------------------
        # 4) 非 self-echo 的 restricted prefix
        #
        # 用于真实 barge-in：
        #   用户：“停止移动”
        #   + Rebecca 尾音
        #   -> “停止移动操作和交流”
        # -----------------------------------------------------
        self.get_logger().info(
            f"🛑 [VoiceMode:{self._voice_mode}] "
            f"Robot STOP prefix 命中：'{text}'"
        )

        self._publish('停止移动')
        self.pub_interrupt.publish(Bool(data=True))

        self.mute_until = time.time() + 0.5
        return True


    def _handle_mode_control(self, text: str, conf: float) -> bool:
        """检测顺序第 2 位：MUTED / SLEEPING 下唯一放行的控制命令。

        NORMAL 模式直接返回 False（进入命令由 _handle_mode_enter 处理）。
        返回 True 表示本句已处理，调用方不再进入后续流程。
        """
        n = self._norm_control_text(text)

        # L3 unmute hardening:
        # “不要取消静音 / 别恢复对话”等否定恢复表达
        # 不能因为包含 unmute keyword 而解除静音。
        l3_unmute_hit = any(
            k in n for k in VOICE_UNMUTE_KEYWORDS
        )

        l3_unmute_negated = bool(re.search(
            r'(?:不要|别|不用|无需|不需要).{0,3}'
            r'(?:取消静音|可以说话|继续说话|恢复对话)',
            n,
        ))

        if self._voice_mode == VOICE_MODE_MUTED:
            # “不要取消静音 / 别恢复对话”等否定表达：
            # 保持 MUTED，并消费本句。
            if l3_unmute_negated:
                self.get_logger().info(
                    f"🔇 [VoiceMode] unmute negation guard：'{text}'"
                )
                return True

            # 恢复说话（MUTED -> NORMAL）
            if l3_unmute_hit:
                if conf < self.min_avg_conf:
                    self.get_logger().info(
                        f"🎛️ [VoiceMode] 解除静音低置信度({conf:.2f})，丢弃：{text}"
                    )
                    return True

                self._voice_mode = VOICE_MODE_NORMAL
                self.get_logger().info(
                    f"🔊 [VoiceMode] MUTED -> NORMAL（恢复说话）：'{text}'"
                )

                # 放行原句：Agent 侧同步解除 L3 muted 并播报确认
                self._publish(text)
                return True

            # 系统休眠（MUTED -> SLEEPING）
            if any(k in n for k in VOICE_SLEEP_ENTER_KEYWORDS):
                if conf < self.min_avg_conf:
                    self.get_logger().info(
                        f"🎛️ [VoiceMode] 系统休眠低置信度({conf:.2f})，丢弃：{text}"
                    )
                    return True
                self._voice_mode = VOICE_MODE_SLEEPING
                self.get_logger().info(
                    f"🌙 [VoiceMode] MUTED -> SLEEPING（系统休眠）：'{text}'"
                )
                self._publish(text)
                return True

            return False

        if self._voice_mode == VOICE_MODE_SLEEPING:
            # 唤醒（SLEEPING -> NORMAL）
            if any(k in n for k in VOICE_WAKE_KEYWORDS):
                if conf < self.min_avg_conf:
                    self.get_logger().info(
                        f"🎛️ [VoiceMode] 唤醒低置信度({conf:.2f})，丢弃：{text}"
                    )
                    return True
                self._voice_mode = VOICE_MODE_NORMAL
                self.get_logger().info(
                    f"🌅 [VoiceMode] SLEEPING -> NORMAL（唤醒）：'{text}'"
                )
                self._publish(text)
                return True

            return False

        return False

    def _handle_mode_enter(self, text: str, conf: float) -> bool:
        """检测顺序第 2 位（NORMAL 模式）：静音/休眠进入命令。

        放行原句（Agent 侧同步进入 L3 muted / system sleep），
        同时本地切换 Voice Mode，从源头拦住后续普通语音。
        必须放在自语抑制之后，防止 Rebecca 自己的播报内容触发静音。
        """
        if conf < self.min_avg_conf:
            return False

        n = self._norm_control_text(text)

        # L3 mute hardening:
        #
        # “取消静音”包含裸关键词“静音”，但绝不能进入 MUTED。
        # “不要静音 / 不要请静音”同样不得进入 MUTED。
        #
        # 注意：
        # “不要说话”本身仍是合法 MUTE 指令，因此这里只针对“静音”。

        l3_unmute_hit = any(
            k in n for k in VOICE_UNMUTE_KEYWORDS
        )

        l3_mute_negated = bool(re.search(
            r'(?:不要|别|不用|无需|不需要).{0,3}静音',
            n,
        ))

        # NORMAL 状态下重复“取消静音”是幂等 no-op。
        # 不允许继续落入裸“静音”的 substring matcher。
        if l3_unmute_hit:
            self.get_logger().debug(
                f"🔊 [VoiceMode] already NORMAL，忽略重复解除静音：'{text}'"
            )
            return True

        # “不要静音 / 不要请静音”等否定表达：
        # 保持 NORMAL，不进入 MUTED。
        if l3_mute_negated:
            self.get_logger().info(
                f"🔊 [VoiceMode] mute negation guard：'{text}'"
            )
            return True

        # -----------------------------------------------------
        # 正常进入静音：
        # 这一段必须保留。前面的两个 guard 只负责排除
        # “取消静音 / 不要静音”等假阳性，不能替代真正的 MUTE。
        # -----------------------------------------------------
        if any(k in n for k in VOICE_MUTE_ENTER_KEYWORDS):
            self._voice_mode = VOICE_MODE_MUTED
            self.get_logger().info(
                f"🔇 [VoiceMode] NORMAL -> MUTED（静音）：'{text}'"
            )

            # 放行原句，让 Agent 同步设置 self.l3_muted=True
            self._publish(text)
            return True

        # 系统休眠
        if any(k in n for k in VOICE_SLEEP_ENTER_KEYWORDS):
            self._voice_mode = VOICE_MODE_SLEEPING
            self.get_logger().info(
                f"🌙 [VoiceMode] NORMAL -> SLEEPING（系统休眠）：'{text}'"
            )
            self._publish(text)
            return True

        return False

    # ---------------- 主循环 ----------------
    def _main_loop(self):
        frame_ms = self.frame_ms
        while not self._stop.is_set():
            try:
                chunk, captured_restricted = self.q.get(timeout=0.1)
            except queue.Empty:
                # 空闲期也要检查是否需要从激活态降到休眠并播报提示
                now = time.time()
                # if self.enable_wakeup and self.active and now > self.active_until:
                #     if self.awaiting_reply:
                #         # ✅ 仍在等待 LLM 回复：不给休眠，防抖续命 3s
                #         self.active_until = now + 3.0
                #     else:
                #         self.active = False
                #         self.get_logger().debug("🔕 激活窗口结束，回到静默等待唤醒")
                #         if self.sleep_notice:
                #             self.get_logger().info("💤 进入休眠提示")
                #             self.pub_reply.publish(String(data=self.sleep_prompt))
                continue

            now = time.time()

            # Patch 4A.1：
            # frame 的处理身份必须由“采集时状态”决定，而不是由当前
            # _interrupt_listen_only 决定。
            #
            # 如果当前已经处于一个 restricted utterance 中，那么即使
            # TTS 此刻已经结束，后续 frame 仍属于同一个 restricted utterance，
            # 不能重新经过 normal mute gate，否则会截断跨边界的 STOP / confirm。
            frame_restricted = bool(
                captured_restricted
                or (self.speeching and self._utt_restricted)
            )

            nav_control_fast = (
                self._voice_agent_state == 'nav_wait_confirm'
            )

            if not frame_restricted:
                # TTS 真正在播放时，仍保持硬门控。
                if self.respect_tts_gate and self._tts_speaking:
                    continue

                # Patch 4B.1:
                # nav_wait_confirm 时不能继续受“估算动态静音窗”影响。
                #
                # 否则会出现：
                #   TTS 已经实际播完
                #   但 mute_until 还剩 0.x~1.x 秒
                #   用户自然回答“是的”被直接丢帧。
                if now < self.mute_until and not nav_control_fast:
                    continue

            # 唤醒/激活窗口管理（仅在正常模式下影响发布；打断模式不影响）
            # if self.enable_wakeup and not self._interrupt_listen_only:
            #     if self.active and now > self.active_until:
            #         if self.awaiting_reply:
            #             # ✅ 等待 LLM：延长一点点，避免误休眠
            #             self.active_until = now + 3.0
            #         else:
            #             self.active = False
            #             self.get_logger().debug("🔕 激活窗口结束，回到静默等待唤醒")
            #             if self.sleep_notice:
            #                 self.get_logger().info("💤 进入休眠提示")
            #                 self.pub_reply.publish(String(data=self.sleep_prompt))
            # elif not self.enable_wakeup:
            #     self.active = True

            # VAD
            try:
                is_speech = self.vad.is_speech(chunk, self.sample_rate)
            except Exception as e:
                self.get_logger().warn(f"vad error: {e}")
                is_speech = False

            if is_speech:
                if not self.speeching:
                    self.speeching = True
                    self.ms_in_cur_utt = 0
                    self.ms_sil = 0
                    self.cur_pcm = io.BytesIO()
                    # Patch 4A.1：起始帧决定 restricted 初值
                    self._utt_restricted = captured_restricted
                else:
                    # Patch 4A.1：任一 restricted 帧即把整个 utterance 锁存
                    self._utt_restricted = merged_utt_restricted(
                        self._utt_restricted, captured_restricted
                    )
                self.cur_pcm.write(chunk)
                self.ms_in_cur_utt += frame_ms

                # 说话中续杯（仅正常模式）
                # if not self._interrupt_listen_only and self.enable_wakeup and self.active:
                #     self.active_until = now + self.active_extend_s

                # 打断模式给完整口令留足时间；检测到静音仍会提前 endpoint。
                # Patch 4A.1：端点参数跟随 utterance 锁存值，不跟随实时
                # _interrupt_listen_only（TTS→NORMAL 边界后不得切回正常阈值）。
                nav_control_fast = (
                    self._voice_agent_state == 'nav_wait_confirm'
                )

                max_utt = (
                    self.interrupt_max_utt_ms
                    if (self._utt_restricted or nav_control_fast)
                    else self.max_utt_ms
                )
                if self.ms_in_cur_utt >= max_utt:
                    self._finalize_utterance(reason="max_utt")
            else:
                if self.speeching:
                    self.ms_sil += frame_ms
                    # 端点阈值（打断模式更敏感，可在最大时长前完成）
                    # Patch 4A.1：同样跟随 utterance 锁存值
                    nav_control_fast = (
                    self._voice_agent_state == 'nav_wait_confirm'
                    )

                    needed = (
                    self.interrupt_max_sil_ms
                    if (self._utt_restricted or nav_control_fast)
                    else self.max_sil_ms
                    )
                    if self.ms_sil >= needed:
                        self._finalize_utterance(reason="endpoint")

    def _finalize_utterance(self, reason: str):
        pcm = self.cur_pcm.getvalue()
        utt_ms = self.ms_in_cur_utt
        self.cur_pcm = io.BytesIO()
        self.ms_in_cur_utt = 0
        self.ms_sil = 0
        self.speeching = False
        # Patch 4A.1：先保存 utterance 级 restricted 锁存再复位；
        # 下面的 min_utt 与分支选择都用保存值，不用实时 _interrupt_listen_only
        # （即使 TTS 已播完、全局开关回到 False，跨界 utterance 仍按 restricted 处理）。
        utt_restricted = self._utt_restricted
        self._utt_restricted = False

        # Patch 4B.1:
        # nav_wait_confirm 是短控制回复窗口。
        # “是的 / 可以 / 确认 / 取消”等自然回复通常很短，
        # 不应继续受到 NORMAL 模式 min_utt_ms 的限制。
        nav_control_fast = (
            self._voice_agent_state == 'nav_wait_confirm'
        )

        min_utt = (
            self.interrupt_min_utt_ms
            if (utt_restricted or nav_control_fast)
            else self.min_utt_ms
        )

        if utt_ms < min_utt:
            self.get_logger().debug(
                f"🪶 丢弃短语音 {utt_ms}ms ({reason}) | "
                f"min={min_utt}ms | "
                f"restricted={utt_restricted} | "
                f"nav_fast={nav_control_fast} | "
                f"agent_state={self._voice_agent_state}"
            )
            return

        # 识别
        try:
            text, conf = self.asr.infer_pcm(pcm, self.sample_rate)
            text = (text or "").strip()
        except Exception as e:
            self.get_logger().error(f"ASR 失败：{e}")
            return

        if not text:
            self.get_logger().debug("空识别结果，丢弃")
            return

        # ============================================================
        # Voice Mode 状态机（Patch 1）——统一入口，先于一切分支
        #
        # 检测顺序（所有模式一致）：
        #   1. robot stop（最高优先级，任何模式都必须传递给机器人）
        #   2. wake / sleep / mute 模式控制
        #   3. 导航确认（原有逻辑，仅 NORMAL）
        #   4. 普通语音（原有逻辑，仅 NORMAL）
        # ============================================================
        if self._handle_robot_stop(
            text,
            allow_prefix=utt_restricted,
        ):
            return

        # —— 打断监听模式：只识别控制短语，命中即处理，**不**转发普通语音 ——
        # Patch 4A.1：分支选择用 utterance 锁存值（utt_restricted），
        # 跨 TTS→NORMAL 边界的混音 utterance 不得落入 NORMAL 发布路径。
        if utt_restricted:
            # 静音/休眠模式下，TTS 播放期间同样只放行模式控制命令
            if self._voice_mode != VOICE_MODE_NORMAL:
                if self._handle_mode_control(text, conf):
                    return
                self.get_logger().info(
                    f"🎧 [VoiceMode:{self._voice_mode}] (TTS中) 丢弃语音：'{text}'"
                )
                return

            # Patch 4A：统一“归一化 + 剥离唤醒前缀 + 整句精确匹配”。
            # 禁止 startswith / endswith / substring，防止 Rebecca 的
            # TTS 回声碎片（“确认开始导航”“或取消放弃”“正在停止移动”）
            # 误触确认 / 取消 / 打断。
            reply = self._strip_wake_prefix(self._norm_control_text(text))

            # ---------------------------------------------------------
            # 1) TTS-time 导航确认 / 取消直通
            #
            # 仅当 Agent 处于 nav_wait_confirm，且整句等于强确认 /
            # 取消集合中的词。conf 只作辅助条件（FunASR 无真实置信度
            # 时默认 1.0，安全边界由窄集合承担）。
            # 命中后发布规范文本（确认/取消），不转发原始噪声句。
            # ---------------------------------------------------------
            tts_ctrl = (
                classify_tts_control_reply(reply, self._voice_agent_state)
                if conf >= self.min_avg_conf
                else None
            )
            if tts_ctrl is not None:
                canonical = '确认' if tts_ctrl == 'confirm' else '取消'
                self.get_logger().info(
                    f"✅ TTS-time 导航{tts_ctrl}直通 | "
                    f"state={self._voice_agent_state} | "
                    f"text='{text}' -> '{canonical}'"
                )

                # 先停止 Rebecca 当前播报
                self.pub_interrupt.publish(Bool(data=True))

                # 再把规范控制文本送给 Agent（不带 ASR 噪声）
                self._publish(canonical)

                # 避免刚打断的 TTS 尾音再次形成端点
                self.mute_until = time.time() + 0.3
                return

            # ---------------------------------------------------------
            # 2) 普通 TTS 打断：整句精确匹配独立集合，仅停播报。
            #    “瑞贝卡，别说了”已在 reply 归一化时剥离唤醒前缀。
            # ---------------------------------------------------------
            if is_tts_stop_phrase(reply, self._tts_stop_phrases):
                self.pub_interrupt.publish(Bool(data=True))
                self.mute_until = time.time() + 2.0
                self.get_logger().info(
                    f"🛑 TTS 打断（exact）'{text}' -> 已发送 /tts/interrupt"
                )
                return

            # 其余 TTS 期间语音一律丢弃（回声保护）
            self.get_logger().info(
                f"🎧(interrupt-only) 丢弃非控制语音：'{text}' "
                f"(conf~{conf:.2f}) state={self._voice_agent_state}"
            )
            return

        # —— 正常模式（以下逻辑与之前相同 + 唤醒直通）——
        self.get_logger().info(f"🎧 识别[{reason}] -> '{text}' (conf~{conf:.2f})")

        low_conf = conf < self.min_avg_conf

        # 唤醒逻辑：命中则本地激活 + 直通 agent
        norm = (
            text.replace("，", ",").replace("。", ".")
                .replace("！", "!").replace("？", "?")
        ).lower()
        hit_wake = any(w in norm for w in self._wake_words_lc)

        # if self.enable_wakeup:
        #     if hit_wake:
        #         now = time.time()
        #         self.active = True
        #         self.last_wakeup = now
        #         self.active_until = now + self.wakeup_window_s
        #         self.get_logger().info(f"🔔 唤醒成功（本地激活 {self.wakeup_window_s}s），并直通给 Agent")
        #         # 继续往下：直通
        #     elif not self.active:
        #         return  # 未命中唤醒且静默状态：不发布
        if hit_wake:
            self.get_logger().info("🔔 检测到唤醒词（仅标注，不控制）")

        # 自语抑制（仅在最近播报窗口才检查）
        recent = (time.time() - self.last_tts_time) <= self.self_speech_window_s
        if recent and self.last_tts_text:
            a = text.replace(' ', '')
            b = self.last_tts_text.replace(' ', '')
            sim = difflib.SequenceMatcher(None, a, b).ratio()
            if (a in b) or (b in a) or (sim >= 0.55):
                self.get_logger().info(f"🛡️ 自语过滤：{text} (sim={sim:.2f})")
                return

        # ============================================================
        # Voice Mode 状态机（Patch 1）
        #
        # 静音/休眠：普通语音在源头丢弃，不发布 /speech_query，
        # 不进入 llm_voice_agent（只放行 _handle_mode_control 的控制命令）
        # ============================================================
        if self._voice_mode != VOICE_MODE_NORMAL:
            if self._handle_mode_control(text, conf):
                return
            mode_tag = '🔇' if self._voice_mode == VOICE_MODE_MUTED else '💤'
            self.get_logger().info(
                f"{mode_tag} [VoiceMode:{self._voice_mode}] 丢弃语音：'{text}'"
            )
            return

        # NORMAL 模式：静音/休眠进入命令
        # （检测顺序第 2 位，放在自语抑制之后、确认白名单之前）
        if self._handle_mode_enter(text, conf):
            return

        # ============================================================
        # 控制回复白名单：必须优先于“口头语清理 / 短句过滤”
        #
        # 原因：
        #   “确认。”、“取消。”等控制词虽然很短，但属于机器人状态机
        #   的有效控制输入，不能被 min_text_len=4 过滤。
        #
        # 注意：
        #  这里只处理正常 ASR 模式。
        #   TTS 播放期间仍保持 interrupt-only，不允许“确认”穿透，
        #   避免机器人自己的 TTS 回声误触发导航。
        # ============================================================

        control_reply = re.sub(
            r'[，。！!？?、；;：:\s]+',
            '',
            text
        ).lower()

        CONTROL_REPLIES = {
            # 明确确认
            '确认',
            '确定',
            '执行',
            '好的',
            '好',
            '行',
            'ok',
            '可以',
            '是的',
            '对的',
            '没错',
            '没问题',

            # 明确取消 / 否定
            '取消',
            '不去',
            '不要',
            '别去',
            '先不要',
            '算了',
            '不用',
            '停止',
        }

        if control_reply in CONTROL_REPLIES:
            if low_conf:
                self.get_logger().info(
                    f"🎛️ 控制短语低置信度({conf:.2f})，丢弃：{text}"
                )
                return

            self.get_logger().info(
                f"🎛️ 控制短语直通：{text} -> {control_reply}"
            )
            self._publish(text)
            return


        # ===== 普通文本：口头语清理 + 短句过滤 =====
        cleaned = self._post_clean_cn(text)

        # ============================================================
        # 导航短命令直通
        #
        # “去餐厅 / 去客厅 / 到房间”等虽然字数短，
        # 但具有明确的导航句式，不能被 min_text_len 当口头语过滤。
        #
        # 这里只负责放行，真正是否构成有效导航仍由 Agent 判断。
        # ============================================================
        nav_short = re.sub(
            r'[，。！!？?、；;：:\s]+',
            '',
            cleaned
        )

        NAV_SHORT_RE = re.compile(
            r'^(?:导航到|移动到|前往|到达|去|到).{2,}$'
        )

        if NAV_SHORT_RE.match(nav_short):
            self.get_logger().info(
                f"🧭 导航短句直通：{text} -> {nav_short}"
            )
            self._publish(cleaned)
            return

        if not cleaned or len(cleaned) < self.min_text_len:
            if hit_wake:
                # 唤醒句太短也直通原句
                self._publish(text)
                return

            self.get_logger().info(
                f"🪶 过滤短句/口头语：{cleaned}"
            )
            return

        if low_conf and not hit_wake:
            self.get_logger().info(
                f"低置信度({conf:.2f})，丢弃文本：{cleaned}"
            )
            return

        self._publish(cleaned)

    @staticmethod
    def _post_clean_cn(text: str) -> str:
        if not text:
            return ""
        no_meaning_exact = {
            "我说一下", "你听我说", "你听得到吗", "你听见了吗",
            "嗯", "啊", "哦", "好吧", "行吧", "好的", "可以吗",
            "好好", "那个"
        }
        if text.strip() in no_meaning_exact:
            return ""
        bad_tail = ("吧", "嘛", "呀", "呢", "啊", "哦")
        for t in bad_tail:
            if text.endswith(t) and len(text) <= 10:
                text = text[:-1]
                break
        return text.strip()

    # ---------------- 生命周期 ----------------
    def destroy_node(self):
        try:
            self._stop.set()
            try:
                if self.stream:
                    self.stream.stop()
                    self.stream.close()
            except Exception:
                pass
            if hasattr(self, "worker") and self.worker.is_alive():
                self.worker.join(timeout=1.0)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechDialogFunASR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

