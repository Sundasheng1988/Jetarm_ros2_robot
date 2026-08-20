# tts_speaker_node
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import shlex
import shutil
import time
import threading
import queue
import subprocess
import tempfile
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from llm_voice_agent.voice_backends import (
    VoiceBackendCancelled,
    VoiceBackendError,
    cosyvoice_to_wav,
)
from llm_voice_agent.runtime_control import (
    GenerationQueue,
    ProcessStopError,
    stop_owned_process_group,
)

# ========= Markdown → 语音清洗 =========

_MD_LINK = re.compile(r'\[([^\]]+)\]\([^)]+\)')
_MD_CODE_SPAN = re.compile(r'`{1,3}[^`]*`{1,3}')
_MD_CODE_BLOCK = re.compile(r'```[\s\S]*?```', re.MULTILINE)
_MD_HEADING = re.compile(r'^\s{0,3}#{1,6}\s*', re.MULTILINE)
_MD_LIST_ANY = re.compile(r'^\s*(?:[-*+]|[0-9]+\.)\s+', re.MULTILINE)
_MD_BOLD = re.compile(r'\*\*(.*?)\*\*')
_MD_ITALIC = re.compile(r'\*(.*?)\*')
_MD_TABLE_LINE = re.compile(r'^\s*\|.*\|\s*$', re.MULTILINE)

def _num_to_cn_ordinal(n: int) -> str:
    mapping = {1:'第一',2:'第二',3:'第三',4:'第四',5:'第五',6:'第六',7:'第七',8:'第八',9:'第九',10:'第十'}
    if n in mapping:
        return mapping[n]
    return f'第{n}'

def strip_markdown_to_speech(text: str) -> str:
    if not text:
        return ''

    # 1️⃣ 去代码
    text = _MD_CODE_BLOCK.sub('', text)
    text = _MD_CODE_SPAN.sub('', text)

    # 2️⃣ 链接
    text = _MD_LINK.sub(r'\1', text)

    # 3️⃣ 标题 / 表格
    text = _MD_HEADING.sub('', text)
    text = _MD_TABLE_LINE.sub('', text)

    # 4️⃣ 列表符号（只处理 markdown，不碰正文）
    text = _MD_LIST_ANY.sub('', text)

    # 5️⃣ 粗斜体
    text = _MD_BOLD.sub(r'\1', text)
    text = _MD_ITALIC.sub(r'\1', text)

    # 6️⃣ 换行 → 句号（核心）
    text = re.sub(r'\s*\n+\s*', '。', text)

    # 7️⃣ 清理多余空白
    text = re.sub(r'[ \t]+', ' ', text).strip()

    # 8️⃣ 压缩重复中文标点（只限中文）
    text = re.sub(r'([，。])\1+', r'\1', text)

    # 8.5️⃣ Patch 2：去掉句末标点前残留的逗顿号
    # （「换行→句号」转换的常见残留，如“好的，。”→“好的。”；
    #   短文本整体播报后这类残留会直接进入合成，必须清掉）
    text = re.sub(r'[，、；;：:]+(?=[。！？.!?])', '', text)

    # 9️⃣ 句尾兜底
    if text and text[-1] not in '。！？.!?':
        text += '。'

    return text


# ========= 分句 =========

def split_sentences(text: str, max_len: int) -> List[str]:
    """
    TTS 句子切分（最终版）：
    1) 只按硬句末标点切句：。！？ 
    2) 不在逗号/分号/顿号处切（句内韵律交给 Piper）
    3) 仅在“极端超长句”时做长度兜底切分
    """
    if not text:
        return []

    text = text.strip()
    if not text:
        return []

    # ---------- 第一步：按硬句末标点切 ----------
    hard = '。！？'
    sentences: List[str] = []
    buf: List[str] = []

    for ch in text:
        buf.append(ch)
        if ch in hard:
            s = ''.join(buf).strip()
            if s:
                sentences.append(s)
            buf = []

    # 处理末尾没有句号的情况
    if buf:
        s = ''.join(buf).strip()
        if s:
            if s[-1] not in hard:
                s += '。'
            sentences.append(s)

    # ---------- 第二步：极端超长句兜底 ----------
    final: List[str] = []
    HARD_LIMIT = max_len

    for s in sentences:
        if len(s) <= HARD_LIMIT:
            final.append(s)
        else:
            # ⚠️ 兜底策略：纯长度切，不引入新标点语义
            start = 0
            while start < len(s):
                chunk = s[start:start + HARD_LIMIT]
                chunk = chunk.strip()
                if chunk and chunk[-1] not in hard:
                    chunk += '。'
                final.append(chunk)
                start += HARD_LIMIT

    return final


# ========= 分段策略（Voice Stabilization Patch 2） =========

def plan_segments(text: str, whole_speak_max_len: int, max_sentence_len: int) -> List[str]:
    """
    Patch 2 分段策略：
    1) 清洗后长度 <= whole_speak_max_len：整体合成、整体播放，不拆句。
       短确认回复（“好的，我让Eric去餐厅。”“好的。”“收到。”）只发
       一次 CosyVoice 请求、起一次播放进程，降低响应延迟。
    2) 长文本：仍按自然句切分（split_sentences），保留分段播放能力。
    """
    if not text:
        return []
    whole_max = min(whole_speak_max_len, max_sentence_len)
    if len(text) <= whole_max:
        return [text]
    return split_sentences(text, max_sentence_len)


# ========= 播放器参数归一 =========

def normalize_player_for_file(player_cmd: str) -> list:
    argv = shlex.split(player_cmd)
    norm, skip_next = [], False
    for tok in argv:
        if tok == '-':
            continue
        if tok in ('-t', '--type'):
            skip_next = True
            continue
        if skip_next:
            skip_next = False
            continue
        norm.append(tok)
    if not norm:
        norm = ['aplay', '-q']
    return norm


# ========= 主节点 =========

class TTSSpeakerNode(Node):
    def __init__(self):
        super().__init__('tts_speaker_node')

        # 参数
        self.reply_topic = self.declare_parameter('reply_topic', '/speech_reply').get_parameter_value().string_value
        self.piper_bin = os.path.expanduser(self.declare_parameter('piper_bin', '~/.local/bin/piper').get_parameter_value().string_value)
        self.model_path = os.path.expanduser(self.declare_parameter('model_path', '~/tools/piper/models/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx').get_parameter_value().string_value)
        self.length_scale = float(self.declare_parameter('length_scale', 1.04).get_parameter_value().double_value)
        self.noise_scale = float(self.declare_parameter('noise_scale', 0.65).get_parameter_value().double_value)
        self.noise_w = float(self.declare_parameter('noise_w', 0.88).get_parameter_value().double_value)
        # Piper CLI expects seconds. Keep the millisecond parameter as a
        # migration alias and convert it exactly once.
        self.sentence_silence_ms = int(self.declare_parameter('sentence_silence_ms', 140).get_parameter_value().integer_value)
        self.sentence_silence_s = float(
            self.declare_parameter(
                'sentence_silence_s', self.sentence_silence_ms / 1000.0
            ).get_parameter_value().double_value
        )
        self.player = self.declare_parameter('player', 'ffplay -autoexit -nodisp -loglevel quiet').get_parameter_value().string_value
        self.emit_mode = self.declare_parameter('emit_mode', 'wav').get_parameter_value().string_value
        self.max_sentence_len = int(self.declare_parameter('max_sentence_len', 200).get_parameter_value().integer_value)
        # Patch 2：短文本整体播报阈值。清洗后长度 <= 该值时不拆句，
        # 一次合成一次播放（默认 50 字符；与 max_sentence_len 取小兜底）
        self.whole_speak_max_len = max(
            1,
            int(self.declare_parameter('whole_speak_max_len', 50).get_parameter_value().integer_value),
        )
        self.dedup_window_s = float(self.declare_parameter('dedup_window_s', 1.5).get_parameter_value().double_value)

        # CosyVoice is a separately-running persistent service. Piper remains
        # the local CPU fallback and is never started as a server here.
        self.tts_backend = self.declare_parameter(
            'tts_backend', 'cosyvoice'
        ).get_parameter_value().string_value.strip().lower()
        self.tts_fallback_backend = self.declare_parameter(
            'tts_fallback_backend', 'piper'
        ).get_parameter_value().string_value.strip().lower()
        self.cosyvoice_base_url = self.declare_parameter(
            'cosyvoice_base_url', 'http://127.0.0.1:50000'
        ).get_parameter_value().string_value
        self.cosyvoice_mode = self.declare_parameter(
            'cosyvoice_mode', 'zero_shot'
        ).get_parameter_value().string_value.strip().lower()
        # Fun-CosyVoice3-0.5B-2512 emits 24 kHz PCM.
        self.cosyvoice_sample_rate = int(
            self.declare_parameter('cosyvoice_sample_rate', 24000)
                .get_parameter_value().integer_value
        )
        self.cosyvoice_speaker_id = self.declare_parameter(
            'cosyvoice_speaker_id', ''
        ).get_parameter_value().string_value
        self.cosyvoice_prompt_text = self.declare_parameter(
            'cosyvoice_prompt_text', ''
        ).get_parameter_value().string_value
        self.cosyvoice_prompt_wav = os.path.expanduser(
            self.declare_parameter('cosyvoice_prompt_wav', '')
                .get_parameter_value().string_value
        )
        self.cosyvoice_instruct_text = self.declare_parameter(
            'cosyvoice_instruct_text',
            'You are a helpful assistant. 请用自然、清晰、亲切的普通话播报。<|endofprompt|>',
        ).get_parameter_value().string_value
        self.cosyvoice_connect_timeout_s = float(
            self.declare_parameter('cosyvoice_connect_timeout_s', 5.0)
                .get_parameter_value().double_value
        )
        self.cosyvoice_timeout_s = float(
            self.declare_parameter('cosyvoice_timeout_s', 90.0)
                .get_parameter_value().double_value
        )

        # Silent development mode synthesizes files without opening a player.
        self.play_audio = bool(
            self.declare_parameter('play_audio', True).get_parameter_value().bool_value
        )
        output_dir = self.declare_parameter(
            'wav_output_dir', ''
        ).get_parameter_value().string_value
        self.wav_output_dir = os.path.expanduser(output_dir) if output_dir else ''
        if self.wav_output_dir:
            os.makedirs(self.wav_output_dir, exist_ok=True)

        self.queue_max = max(
            1,
            int(self.declare_parameter('queue_max', 16).get_parameter_value().integer_value),
        )
        # Retained for launch/config compatibility.  Old-generation items are
        # always invalidated and removed on interrupt for safety.
        self.declare_parameter('drop_queue_on_interrupt', True)
        self.process_stop_timeout_s = float(
            self.declare_parameter('process_stop_timeout_s', 0.5)
                .get_parameter_value().double_value
        )

        # 日志
        self.get_logger().info(
            f'🔊 TTS 初始化完成 | backend={self.tts_backend} | '
            f'cosyvoice_mode={self.cosyvoice_mode} | '
            f'cosyvoice_rate={self.cosyvoice_sample_rate} | '
            f'play_audio={self.play_audio} | '
            f'piper_silence={self.sentence_silence_s:.3f}s'
        )

        self._q = GenerationQueue[str](maxsize=self.queue_max)
        self._speaking = False
        self._last_enqueued_text = ''
        self._last_enqueued_ts = 0.0
        self._worker_alive = True
        self._worker = None
        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()

        self.tts_speaking_pub = self.create_publisher(Bool, '/tts_speaking', 1)
        self.tts_done_pub = self.create_publisher(Bool, '/tts/done', 1)  # ✅ 新增：播报结束/打断等价完成事件

        
        # 11/2  —— 播放/合成子进程句柄 —
        self._player_proc = None     # aplay/ffplay 等播放器
        self._piper_proc  = None     # piper 合成进程（PIPE 模式用）
        self._player_generation = None
        self._piper_generation = None
        self._stop_streaming = False # 未来若你做流式合成可用

        # 用户打断会同时停止 TTS 与取消 Agent LLM；Agent 自己的唤醒消息只停止
        # 旧播放。两者在 TTS 内走完全相同的安全停止处理。
        self.sub_interrupt = self.create_subscription(
            Bool, '/tts/interrupt', self._on_interrupt, 10
        )
        self.sub_playback_interrupt = self.create_subscription(
            Bool, '/tts/interrupt_playback_only', self._on_interrupt, 10
        )
        self.create_subscription(String, self.reply_topic, self._on_reply, 10)

        # Start only after every state variable and publisher exists.
        self._worker = threading.Thread(target=self._speech_worker, daemon=True)
        self._worker.start()
    
    def _stop_one_process(self, proc, name: str):
        if proc is None:
            return
        try:
            result = stop_owned_process_group(
                proc,
                term_timeout=self.process_stop_timeout_s,
                kill_timeout=self.process_stop_timeout_s,
            )
        except ProcessStopError as exc:
            self.get_logger().warning(f'{name} 停止失败：{exc}')
            return
        for warning in result.warnings:
            self.get_logger().warning(f'{name} 停止降级：{warning}')
        if result.forced:
            self.get_logger().warning(f'{name} 未及时退出，已强制结束')

    def _detach_playback_locked(self):
        """Detach registered processes while ``_state_lock`` is held."""
        piper = self._piper_proc
        player = self._player_proc
        self._piper_proc = None
        self._player_proc = None
        self._piper_generation = None
        self._player_generation = None
        return piper, player

    def _stop_detached_playback(self, piper, player):
        self._stop_one_process(piper, 'Piper')
        self._stop_one_process(player, '播放器')

    # 统一停止当前登记的进程。成员句柄先原子摘除，新进程不会被旧清理误杀。
    def _stop_playback(self):
        self._stop_streaming = True
        with self._state_lock:
            piper, player = self._detach_playback_locked()
        self._stop_detached_playback(piper, player)
    
    # 11/2  V1.1 新增：打断回调
    def _on_interrupt(self, msg: Bool):
        if not msg or not msg.data:
            return
        self.get_logger().info('⛔ 收到 TTS 打断，停止当前播报')
        # Block a new reply until old processes are confirmed stopped.  This
        # prevents an older interrupt from overwriting the new reply's
        # /tts_speaking=True state or stopping its newly registered player.
        with self._command_lock:
            with self._state_lock:
                generation, dropped = self._q.interrupt()
                piper, player = self._detach_playback_locked()
                self._stop_streaming = True
                was_speaking = self._speaking
                self._speaking = False
            self._stop_detached_playback(piper, player)
            self.get_logger().debug(
                f'打断 generation={generation}，清除旧队列 {dropped} 条'
            )
            if was_speaking:
                self.tts_speaking_pub.publish(Bool(data=False))
            self.tts_done_pub.publish(Bool(data=True))
            self.get_logger().info('✅ /tts/done 已发布（打断触发）')

    # ---- 接收文本并分句
    def _on_reply(self, msg: String):
        raw = (msg.data or '').strip()
        if not raw:
            return

        with self._command_lock:
            self._enqueue_reply(raw)

    def _enqueue_reply(self, raw: str):
        clean = strip_markdown_to_speech(raw)

        now = time.time()
        if (
            clean == self._last_enqueued_text
            and (now - self._last_enqueued_ts) < self.dedup_window_s
        ):
            self.get_logger().info(
                f'⏭️ 跳过重复播报（{self.dedup_window_s:.1f}s窗口）: {clean}'
            )
            return
        self._last_enqueued_text = clean
        self._last_enqueued_ts = now

        if ('先休息啦' in clean) or ('进入休眠提示' in clean):
            last = getattr(self, '_last_sleep_hint_ts', 0.0)
            if (time.time() - last) < 120.0 or (not self._q.empty()):
                self.get_logger().info('⏭️ 跳过休眠提示（防刷屏/防静音叠加）。')
                return
            self._last_sleep_hint_ts = time.time()

        reply_generation = self._q.generation
        enqueued = False

        # Patch 2 分段策略：短文本整体播报（不拆句），长文本按自然句切分
        whole_max = min(self.whole_speak_max_len, self.max_sentence_len)
        if 0 < len(clean) <= whole_max:
            self.get_logger().info(
                f'⚡ 短文本整体播报（{len(clean)}≤{whole_max}字，一次合成播放）'
            )
        for seg in plan_segments(clean, self.whole_speak_max_len, self.max_sentence_len):
            try:
                if not self._q.put_nowait_if_current(reply_generation, seg):
                    self.get_logger().info('⏭️ 回复在排队期间被打断，停止入队')
                    break
                enqueued = True
                self.get_logger().info(f'📥 播放排队: {seg}')
            except queue.Full:
                self.get_logger().warning('⚠️ 队列已满，丢弃一条播报。')

        if enqueued and not self._speaking:
            self._speaking = True
            self.tts_speaking_pub.publish(Bool(data=True))

    # ---- 播放工作线程
    def _speech_worker(self):
        while self._worker_alive:
            try:
                generation, seg = self._q.get(timeout=0.2)
                if not self._generation_is_current(generation):
                    self.get_logger().info('⏭️ 已丢弃旧 generation 句子')
                    self._q.task_done()
                    continue
            except queue.Empty:
                if self._speaking:
                    time.sleep(0.3)
                    with self._command_lock:
                        if self._speaking and self._q.empty():
                            self._speaking = False
                            self.tts_speaking_pub.publish(Bool(data=False))
                            # ✅ 新增：自然播报结束 → 发 done
                            self.tts_done_pub.publish(Bool(data=True))
                            self.get_logger().info('✅ /tts/done 已发布（自然结束）')
                continue

            try:
                self._speak_segment(seg, generation)
            except Exception as e:
                self.get_logger().error(f'❌ TTS 播报失败: {e}')
            finally:
                self._q.task_done()

    def _generation_is_current(self, generation: int) -> bool:
        return self._q.is_current(generation)

    def _tts_backend_order(self):
        primary = self.tts_backend if self.tts_backend in {'cosyvoice', 'piper'} else 'cosyvoice'
        order = [primary]
        fallback = self.tts_fallback_backend
        if fallback in {'cosyvoice', 'piper'} and fallback not in order:
            order.append(fallback)
        return order

    def _speak_segment(self, text: str, generation: int):
        # Preserve the original low-latency Piper pipe path when requested.
        if (
            self.tts_backend == 'piper'
            and self.emit_mode == 'pipe'
            and self.play_audio
        ):
            if self._generation_is_current(generation):
                self._play_by_pipe(text, generation)
            return

        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf_wav:
            wav_path = tf_wav.name

        selected_backend = ''
        started = time.monotonic()
        try:
            for backend in self._tts_backend_order():
                if not self._generation_is_current(generation):
                    return
                try:
                    if backend == 'cosyvoice':
                        cosyvoice_to_wav(
                            base_url=self.cosyvoice_base_url,
                            mode=self.cosyvoice_mode,
                            text=text,
                            wav_path=wav_path,
                            sample_rate=self.cosyvoice_sample_rate,
                            speaker_id=self.cosyvoice_speaker_id,
                            prompt_text=self.cosyvoice_prompt_text,
                            prompt_wav=self.cosyvoice_prompt_wav,
                            instruct_text=self.cosyvoice_instruct_text,
                            timeout=(
                                self.cosyvoice_connect_timeout_s,
                                self.cosyvoice_timeout_s,
                            ),
                            cancel_check=lambda: not self._generation_is_current(
                                generation
                            ),
                        )
                    else:
                        self._synthesize_piper_wav(text, wav_path)
                    selected_backend = backend
                    break
                except VoiceBackendCancelled:
                    self.get_logger().info('⏭️ CosyVoice 合成已被打断')
                    return
                except VoiceBackendError as exc:
                    self.get_logger().warning(f'TTS backend={backend} 不可用：{exc}')
                except Exception as exc:
                    self.get_logger().warning(
                        f'TTS backend={backend} 异常：{type(exc).__name__}'
                    )

            if not selected_backend:
                raise RuntimeError('all configured TTS backends failed')
            if not self._generation_is_current(generation):
                return

            elapsed = time.monotonic() - started
            self.get_logger().info(
                f'TTS 合成完成 | backend={selected_backend} | '
                f'elapsed={elapsed:.2f}s | chars={len(text)}'
            )
            self._persist_wav_if_requested(wav_path)
            if self.play_audio and self._generation_is_current(generation):
                self._play_wav_file(wav_path, generation)
        finally:
            try:
                if os.path.isfile(wav_path):
                    os.remove(wav_path)
            except OSError:
                pass

    def _synthesize_piper_wav(self, text: str, wav_path: str):
        with tempfile.NamedTemporaryFile(suffix='.txt', delete=False) as tf_txt:
            txt_path = tf_txt.name
            tf_txt.write((text.strip() + '\n').encode('utf-8'))

        def _run(use_sentence_silence: bool):
            piper_cmd = [
                self.piper_bin, '-m', self.model_path,
                '-f', txt_path, '-w', wav_path,
                '--length_scale', str(self.length_scale),
                '--noise-scale', str(self.noise_scale),
                '--noise-w', str(self.noise_w),
            ]
            if use_sentence_silence:
                piper_cmd += ['--sentence-silence', str(self.sentence_silence_s)]
            env = {**os.environ, 'LANG': 'C.UTF-8', 'LC_ALL': 'C.UTF-8'}
            return subprocess.run(
                piper_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                check=False,
                env=env,
            )

        try:
            proc = _run(True)
            if proc.returncode != 0:
                detail = proc.stderr.decode('utf-8', errors='ignore')[-300:]
                raise VoiceBackendError(f'Piper failed: {detail}')
            if (not os.path.isfile(wav_path)) or os.path.getsize(wav_path) <= 44:
                self.get_logger().warning('Piper WAV 为空，取消句间静音参数后重试。')
                proc = _run(False)
                if (
                    proc.returncode != 0
                    or not os.path.isfile(wav_path)
                    or os.path.getsize(wav_path) <= 44
                ):
                    raise VoiceBackendError('Piper returned empty audio')
        finally:
            try:
                os.remove(txt_path)
            except OSError:
                pass

    def _persist_wav_if_requested(self, wav_path: str):
        if not self.wav_output_dir:
            return
        stamp = time.strftime('%Y%m%d-%H%M%S')
        name = f'tts-{stamp}-{time.time_ns() % 1_000_000_000:09d}.wav'
        target = os.path.join(self.wav_output_dir, name)
        shutil.copy2(wav_path, target)
        self.get_logger().info(f'💾 WAV 已保存: {target}')

    def _play_wav_file(self, wav_path: str, generation: int):
        player_argv = normalize_player_for_file(self.player)
        with self._state_lock:
            if not self._generation_is_current(generation):
                return
            player = subprocess.Popen(
                player_argv + [wav_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                close_fds=True,
            )
            self._player_proc = player
            self._player_generation = generation
        try:
            player.wait(timeout=60.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning('播放器超时，强制结束')
            self._stop_one_process(player, '播放器')
        finally:
            with self._state_lock:
                if self._player_proc is player:
                    self._player_proc = None
                    self._player_generation = None

    # ---- PIPE 模式
    def _play_by_pipe(self, text: str, generation: int):
        player_argv = shlex.split(self.player)
        if player_argv and os.path.basename(player_argv[0]) == 'ffplay':
            if '-i' not in player_argv:
                player_argv += ['-i', '-']

        piper = None
        player = None
        spawn_error = None
        with self._state_lock:
            if not self._generation_is_current(generation):
                return
            piper = subprocess.Popen(
                [self.piper_bin, '-m', self.model_path,
                 '--length_scale', str(self.length_scale),
                 '--noise-scale', str(self.noise_scale),
                 '--noise-w', str(self.noise_w),
                '--sentence-silence', str(self.sentence_silence_s)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                close_fds=True,
            )
            self._piper_proc = piper
            self._piper_generation = generation
            try:
                player = subprocess.Popen(
                    player_argv,
                    stdin=piper.stdout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                    close_fds=True,
                )
            except Exception as exc:
                spawn_error = exc
                self._piper_proc = None
                self._piper_generation = None
            else:
                self._player_proc = player
                self._player_generation = generation

        if spawn_error is not None:
            self._stop_one_process(piper, 'Piper')
            raise spawn_error

        # Parent must close its duplicate of piper stdout so the player sees
        # EOF when Piper exits.  Feed stdin explicitly instead of communicate(),
        # which would try to read the already-forwarded stdout pipe.
        if piper.stdout is not None:
            piper.stdout.close()

        try:
            try:
                if piper.stdin is not None:
                    piper.stdin.write((text.strip() + '\n').encode('utf-8'))
                    piper.stdin.close()
                piper.wait(timeout=30.0)
            except BrokenPipeError:
                if self._generation_is_current(generation):
                    self.get_logger().warning('Piper 管道提前关闭')
                return
            except subprocess.TimeoutExpired:
                self.get_logger().warning('piper 合成超时，强制打断')
                self._stop_one_process(piper, 'Piper')
                self._stop_one_process(player, '播放器')
                return

            if not self._generation_is_current(generation):
                return

            if player is not None:
                try:
                    player.wait(timeout=30.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning('播放器超时，强制打断')
                    self._stop_one_process(player, '播放器')

        finally:
            with self._state_lock:
                if self._piper_proc is piper:
                    self._piper_proc = None
                    self._piper_generation = None
                if self._player_proc is player:
                    self._player_proc = None
                    self._player_generation = None


    def destroy_node(self):
        self._worker_alive = False
        self._q.interrupt()
        self._stop_playback()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        return super().destroy_node()


def main():
    rclpy.init()
    node = TTSSpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
