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
        interrupt_words_str = self.declare_parameter(
            "interrupt_words",
            "打断,停,停止,别说了,好了,下一条,跳过,够了,先这样,暂停"
        ).get_parameter_value().string_value
        self.interrupt_words = [w.strip() for w in interrupt_words_str.split(",") if w.strip()]
        self.interrupt_topic = self.declare_parameter("interrupt_topic", "/tts/interrupt").get_parameter_value().string_value
        self.pub_interrupt = self.create_publisher(Bool, self.interrupt_topic, 10)
        
        # ---11/8 增加，打断关键词拓展
        extra_interrupt_syns = [
            "打住","停一下","打断一下","别播了","别念了","别继续了",
            "够了","先这样吧","暂停一下","停停停","停止播放","先这样",
            # ⬇️ 新增
            "等一下","等会儿","等会","先别说","住手","别说话",
            "停","停停","闭嘴","stop","pause","shut up"
        ]
        for w in extra_interrupt_syns:
            if w not in self.interrupt_words:
                self.interrupt_words.append(w)

        # —— 允许“唤醒词 + 打断词”组合命中（中间容忍0~4字）——
              # 先构建小写词表，便于后续统一以小写匹配
        self._interrupt_words_lc = [w.lower() for w in self.interrupt_words]
        self._wake_words_lc = [w.lower() for w in self.wake_words]

        wake_re = "|".join(map(re.escape, self.wake_words))
        intr_re = "|".join(map(re.escape, self.interrupt_words))
        # ✅ 忽略大小写匹配组合（唤醒词 ... 0~4 任意字符 ... 打断词）
        self._interrupt_combo_re = re.compile(
            rf"(?:{wake_re}).{{0,4}}(?:{intr_re})",
            re.IGNORECASE
        )
        # ---11/8 增强--- end

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
            self.q.put_nowait(b)
        except queue.Full:
            try:
                _ = self.q.get_nowait()
            except queue.Empty:
                pass
            try:
                self.q.put_nowait(b)
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

    def _on_voice_agent_state(self, msg: String):
        """只读保存 Agent 当前状态，用于控制类短语的安全门控。"""
        self._voice_agent_state = (msg.data or '').strip()

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
            # ✅ 进入播报：VAD 调宽，单字也能抓到
            try:
                self.vad.set_mode(0)  # 0最宽松，3最激进
            except Exception:
                pass
            # 叠加一个很短的静音窗，给播放器尾音一点余量
            self.mute_until = max(self.mute_until, now + self.tts_gate_release_ms / 1000.0)
            self.get_logger().debug("🔒 TTS 正在说话：ASR 暂停（仅保留打断监听）")
        else:
            # 退出播报：恢复正常
            self._interrupt_listen_only = False
            self._tts_last_off_ts = now
            # ✅ 退出播报：还原成用户配置
            try:
                self.vad.set_mode(self.vad_aggressiveness)
            except Exception:
                pass
            self.get_logger().debug("🔓 TTS 结束说话：ASR 即将恢复")

    def _reset_vad_buffers_if_any(self):
        """安全清空当前语音段与端点累计；不打印噪声性日志。"""
        self.cur_pcm = io.BytesIO()
        self.ms_in_cur_utt = 0
        self.ms_sil = 0
        self.speeching = False

    # ---------------- 主循环 ----------------
    def _main_loop(self):
        frame_ms = self.frame_ms
        while not self._stop.is_set():
            try:
                chunk = self.q.get(timeout=0.1)
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

            # 如果不是“仅打断监听”模式，才应用静音窗口丢帧
            if not self._interrupt_listen_only:
                # 🔒 门控：TTS 说话期间直接丢帧（除非启用仅打断监听）
                if self.respect_tts_gate and self._tts_speaking:
                    continue
                # 🔇 动态静音：在估计的播报期内也丢帧（双保险）
                if now < self.mute_until:
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
                self.cur_pcm.write(chunk)
                self.ms_in_cur_utt += frame_ms

                # 说话中续杯（仅正常模式）
                # if not self._interrupt_listen_only and self.enable_wakeup and self.active:
                #     self.active_until = now + self.active_extend_s

                # 打断模式给完整口令留足时间；检测到静音仍会提前 endpoint。
                max_utt = (
                    self.interrupt_max_utt_ms
                    if self._interrupt_listen_only
                    else self.max_utt_ms
                )
                if self.ms_in_cur_utt >= max_utt:
                    self._finalize_utterance(reason="max_utt")
            else:
                if self.speeching:
                    self.ms_sil += frame_ms
                    # 端点阈值（打断模式更敏感，可在最大时长前完成）
                    needed = (
                        self.interrupt_max_sil_ms
                        if self._interrupt_listen_only
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

        # 最短时长（打断模式也需要一点点）
        # 11/8 更改 min_utt = 240 if self._interrupt_listen_only else self.min_utt_ms
        min_utt = self.interrupt_min_utt_ms if self._interrupt_listen_only else self.min_utt_ms #11/8 新增
        if utt_ms < min_utt:
            self.get_logger().debug(f"丢弃短语音 {utt_ms}ms ({reason})")
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

        # —— 打断监听模式：只识别“打断词”，命中即发出中断信号，并**不**转发给 Agent ——
        if self._interrupt_listen_only:
            # 统一去除标点 / 空白，用于严格控制词匹配
            control_reply = re.sub(
                r'[，。！!？?、；;：:\s]+',
                '',
                text
            ).lower()

            # ---------------------------------------------------------
            # 导航确认安全直通
            #
            # 必须同时满足：
            #   1. Agent 正处于 nav_wait_confirm
            #   2. ASR 结果整句严格等于允许词
            #   3. 置信度达到控制阈值
            #
            # 不允许包含式匹配，避免：
            #   “是否确认” / “请说确认” / “确认开始导航”
            # 被当成用户确认。
            # ---------------------------------------------------------
            NAV_CONFIRM_TOKENS = (
                '确认',
                '我确认',
                '确定',
                '我确定',
                '可以',
            )

            NAV_CONFIRM_BLOCKERS = (
                '取消',
                '不去',
                '不要',
                '别去',
                '算了',
                '不用',
                '停止',
                '不确认',
                '不确定',
            )

            # TTS 与真人讲话可能被 ASR 拼成一句，例如：
            #   “要让Eric去餐厅吗确认”
            # 所以 nav_wait_confirm 下允许确认词位于整句开头或结尾。
            #
            # 安全前提：
            #   1. 必须处于 nav_wait_confirm
            #   2. Rebecca 的导航询问本身不得包含“确认/确定”
            #   3. 有任何明确否定词则禁止确认
            has_confirm_token = (
                any(control_reply.startswith(w) for w in NAV_CONFIRM_TOKENS)
                or any(control_reply.endswith(w) for w in NAV_CONFIRM_TOKENS)
            )

            has_confirm_blocker = any(
                w in control_reply
                for w in NAV_CONFIRM_BLOCKERS
            )

            nav_confirm_hit = (
                self._voice_agent_state == 'nav_wait_confirm'
                and has_confirm_token
                and not has_confirm_blocker
                and conf >= self.min_avg_conf
            )

            if nav_confirm_hit:
                self.get_logger().info(
                    f"✅ 导航确认直通 | "
                    f"state={self._voice_agent_state} | "
                    f"text='{text}'"
                )

                # 先停止 Rebecca 当前播报
                self.pub_interrupt.publish(Bool(data=True))

                # 再把真人确认送给 Agent
                self._publish(text)

                # 避免刚打断的 TTS 尾音再次形成端点
                self.mute_until = time.time() + 0.3
                return

            # ---------------------------------------------------------
            # 原有普通语音打断逻辑
            # ---------------------------------------------------------
            norm = (
                text.replace("，", ",").replace("。", ".")
                    .replace("！", "!").replace("？", "?")
                    .replace(" ", "")
            ).lower()

            hit_interrupt = (
                any(w in norm for w in self._interrupt_words_lc)
                or bool(self._interrupt_combo_re.search(text))
            )

            self.get_logger().info(
                f"🎧(interrupt) -> '{text}' "
                f"(conf~{conf:.2f}) "
                f"state={self._voice_agent_state} "
                f"interrupt={hit_interrupt} "
                f"nav_confirm={nav_confirm_hit}"
            )

            if hit_interrupt:
                self.pub_interrupt.publish(Bool(data=True))
                self.mute_until = time.time() + 2.0
                self.get_logger().info(
                    "🛑 已发送语音打断信号到 /tts/interrupt"
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

