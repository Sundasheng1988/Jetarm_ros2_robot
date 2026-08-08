# tts_speaker_node
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, signal
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
        drop_default = bool(
            self.declare_parameter('drop_queue_on_interrupt', True)
                .get_parameter_value().bool_value
        )
        self._interrupt_clears_queue = bool(
            self.declare_parameter('interrupt_clear_queue', drop_default)
                .get_parameter_value().bool_value
        )

        # 日志
        self.get_logger().info(
            f'🔊 TTS 初始化完成 | backend={self.tts_backend} | '
            f'cosyvoice_mode={self.cosyvoice_mode} | '
            f'cosyvoice_rate={self.cosyvoice_sample_rate} | '
            f'play_audio={self.play_audio} | '
            f'piper_silence={self.sentence_silence_s:.3f}s'
        )

        self._q = queue.Queue(maxsize=self.queue_max)
        self._speaking = False
        self._last_enqueued_text = ''
        self._last_enqueued_ts = 0.0
        self._worker_alive = True
        self._worker = None
        self._interrupt_generation = 0
        self._state_lock = threading.Lock()

        self.tts_speaking_pub = self.create_publisher(Bool, '/tts_speaking', 1)
        self.tts_done_pub = self.create_publisher(Bool, '/tts/done', 1)  # ✅ 新增：播报结束/打断等价完成事件

        
        # 11/2  —— 播放/合成子进程句柄 —
        self._player_proc = None     # aplay/ffplay 等播放器
        self._piper_proc  = None     # piper 合成进程（PIPE 模式用）
        self._stop_streaming = False # 未来若你做流式合成可用

        # 11/2新增：打断订阅, 11/8修改
        self._interrupt_flag = False           # 收到打断信号后置 True
        self.sub_interrupt = self.create_subscription(
            Bool, '/tts/interrupt', self._on_interrupt, 10
        )
        self.create_subscription(String, self.reply_topic, self._on_reply, 10)

        # Start only after every state variable and publisher exists.
        self._worker = threading.Thread(target=self._speech_worker, daemon=True)
        self._worker.start()
    
    # 11/2 V1.1 新增：统一的“停止当前播放”
    def _stop_playback(self):
        # 标记：当前有中断
        self._interrupt_flag = True
        # 停止未来的流式（如果你实现的话）
        self._stop_streaming = True

        # 结束 piper（PIPE 模式合成进程）
        try:
            if self._piper_proc and self._piper_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self._piper_proc.pid), signal.SIGINT)
                except Exception:
                    try:
                        self._piper_proc.terminate()
                    except Exception:
                        pass
                try:
                    self._piper_proc.kill()
                except Exception:
                    pass
        except Exception:
            pass
        finally:
            self._piper_proc = None

        # 结束播放器进程（aplay/ffplay）
        try:
            if self._player_proc and self._player_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self._player_proc.pid), signal.SIGINT)
                except Exception:
                    try:
                        self._player_proc.terminate()
                    except Exception:
                        pass
                try:
                    self._player_proc.kill()
                except Exception:
                    pass
        except Exception:
            pass
        finally:
            self._player_proc = None
        
        # ✅ 兜底：只要发生强制停止，也发一次 done
        try:
            if self._speaking:
                self._speaking = False
                self.tts_speaking_pub.publish(Bool(data=False))
            self.tts_done_pub.publish(Bool(data=True))
            self.get_logger().info('✅ /tts/done 已发布（打断触发）')
        except Exception:
            pass
    
    # 11/2  V1.1 新增：打断回调
    def _on_interrupt(self, msg: Bool):
        if not msg or not msg.data:
            return
        self.get_logger().info('⛔ 收到 /tts/interrupt，停止当前播报')
        with self._state_lock:
            self._interrupt_generation += 1
        self._stop_playback()
        
        # ✅ 11/8 新增：关键：立刻复位，避免“下一句也被当成要丢弃”
       # self._interrupt_flag = False

        # 是否清空队列（推荐清空，避免刚被打断后又继续把旧回答播完）
        if self._interrupt_clears_queue:
            try:
                while True:
                    self._q.get_nowait()
                    self._q.task_done()
            except queue.Empty:
                pass

        # 发布不在说话
        if self._speaking:
            self._speaking = False
            self.tts_speaking_pub.publish(Bool(data=False))
        
        self._interrupt_flag = False  # ✅ 关键：防止下一句也被误丢
        # ✅ 新增：等价“播报结束”事件（打断也算结束）
        # self.tts_done_pub.publish(Bool(data=True))

    # ---- 接收文本并分句
    def _on_reply(self, msg: String):
        raw = (msg.data or '').strip()
        if not raw:
            return
        
        
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

        for seg in split_sentences(clean, self.max_sentence_len):
            try:
                self._q.put_nowait(seg)
                self.get_logger().info(f'📥 播放排队: {seg}')
            except queue.Full:
                self.get_logger().warning('⚠️ 队列已满，丢弃一条播报。')

        if not self._speaking:
            self._speaking = True
            self.tts_speaking_pub.publish(Bool(data=True))

    # ---- 播放工作线程
    def _speech_worker(self):
        while self._worker_alive:
            try:
                seg = self._q.get(timeout=0.2)
                if self._interrupt_flag:
                    self._interrupt_flag = False  # 只丢弃本次，恢复等待
                    self.get_logger().info('⏭️ 已丢弃被打断的当前句子')
                    self._q.task_done()
                    continue
            except queue.Empty:
                if self._speaking:
                    time.sleep(0.3)
                    if self._q.empty():
                        self._speaking = False
                        self.tts_speaking_pub.publish(Bool(data=False))
                        # ✅ 新增：自然播报结束 → 发 done
                        self.tts_done_pub.publish(Bool(data=True))
                        self.get_logger().info('✅ /tts/done 已发布（自然结束）')
                continue

            with self._state_lock:
                generation = self._interrupt_generation

            try:
                self._speak_segment(seg, generation)
            except Exception as e:
                self.get_logger().error(f'❌ TTS 播报失败: {e}')
            finally:
                self._q.task_done()

    def _generation_is_current(self, generation: int) -> bool:
        with self._state_lock:
            return generation == self._interrupt_generation

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
                self._play_by_pipe(text)
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
                self._play_wav_file(wav_path)
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

    def _play_wav_file(self, wav_path: str):
        player_argv = normalize_player_for_file(self.player)
        self._player_proc = subprocess.Popen(
            player_argv + [wav_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
            close_fds=True,
        )
        player = self._player_proc
        try:
            player.wait(timeout=60.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning('播放器超时，强制结束')
            self._stop_playback()
        finally:
            if self._player_proc is player:
                self._player_proc = None

    # ---- PIPE 模式
    def _play_by_pipe(self, text: str):
        # ⛔ 开始播放前：若被打断标记已置位，命中即清，直接放弃本句
        if self._interrupt_flag:
            self.get_logger().info('⏭️ 被打断标记已置位，放弃当前句子（PIPE-前置）')
            self._interrupt_flag = False
            return

        # 启动 piper（合成）
        self._piper_proc = subprocess.Popen(
            [self.piper_bin, '-m', self.model_path,
             '--length_scale', str(self.length_scale),
             '--noise-scale', str(self.noise_scale),
             '--noise-w', str(self.noise_w),
             '--sentence-silence', str(self.sentence_silence_s)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
            close_fds=True
        )

        # 启动播放器（aplay/ffplay），接收 piper 的 stdout
        # --- 构造播放器命令，确保 ffplay 从 stdin 读取音频 ---
        player_argv = shlex.split(self.player)
        if player_argv and os.path.basename(player_argv[0]) == 'ffplay':
            # ffplay 默认不会从 stdin 读，必须显式 -i -
            if '-i' not in player_argv:
                player_argv += ['-i', '-']

        self._player_proc = subprocess.Popen(
            player_argv,
            stdin=self._piper_proc.stdout,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
            close_fds=True
        )
        self._piper_proc.stdout.close()  # 重要：父进程关闭写端，避免管道保持为“有人持有”

        # 用局部强引用，避免并发将成员置 None 导致 NoneType.wait
        piper = self._piper_proc
        player = self._player_proc

        try:
            # 合成
            try:
                piper.communicate(input=(text.strip() + '\n').encode('utf-8'), timeout=30.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warning('piper 合成超时，强制打断')
                self._stop_playback()
                return

            # 合成结束后检查是否被打断：命中即清，不再等待播放器
            if self._interrupt_flag:
                self.get_logger().info('⏭️ 被打断（PIPE 合成后），跳过播放器等待')
                self._interrupt_flag = False
                return

            # 等待播放器：判空+超时保护
            if player is not None:
                try:
                    player.wait(timeout=30.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning('播放器超时，强制打断')
                    self._stop_playback()
            else:
                self.get_logger().debug('播放器已被并发清理，跳过 wait()')

        finally:
            # 只在对象未被并发替换时清空成员引用
            if self._piper_proc is piper:
                self._piper_proc = None
            if self._player_proc is player:
                self._player_proc = None


    def destroy_node(self):
        self._worker_alive = False
        with self._state_lock:
            self._interrupt_generation += 1
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
