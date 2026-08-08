#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Network backends shared by the voice-agent and TTS ROS2 nodes.

This module deliberately has no ROS dependency so it can be tested without
starting a ROS graph, microphone, audio player, or robot executor.
"""

from __future__ import annotations

import json
import os
import re
import threading
import wave
from typing import Callable, Dict, Iterable, List, Optional, Tuple

try:
    import requests
except Exception:  # pragma: no cover - handled at runtime by _require_requests
    requests = None


SentenceCallback = Callable[[str], None]
CancelCallback = Callable[[], bool]
_SENTENCE_BOUNDARY_RE = re.compile(r"(?<=[。！？!?；;\n])")


class VoiceBackendError(RuntimeError):
    """A backend failed without exposing credentials or a response body."""


class VoiceBackendCancelled(VoiceBackendError):
    """A backend request was cancelled by the local speech controller."""


def _require_requests():
    if requests is None:
        raise VoiceBackendError("Python requests package is not installed")
    return requests


class SentenceAccumulator:
    """Turn streamed token fragments into complete, speakable sentences."""

    def __init__(self, callback: Optional[SentenceCallback] = None):
        self.callback = callback
        self._buffer = ""

    def feed(self, fragment: str) -> None:
        if not fragment:
            return
        self._buffer += fragment
        parts = _SENTENCE_BOUNDARY_RE.split(self._buffer)
        self._buffer = parts.pop() if parts else ""
        for part in parts:
            sentence = part.strip()
            if sentence and self.callback:
                self.callback(sentence)

    def flush(self) -> None:
        sentence = self._buffer.strip()
        self._buffer = ""
        if sentence and self.callback:
            self.callback(sentence)


def _safe_error(response) -> str:
    request_id = response.headers.get("x-request-id", "") if response is not None else ""
    suffix = f", request_id={request_id}" if request_id else ""
    status = getattr(response, "status_code", "unknown")
    return f"HTTP {status}{suffix}"


def _iter_openai_sse_content(response) -> Iterable[str]:
    for raw_line in response.iter_lines(decode_unicode=False):
        if not raw_line:
            continue
        if isinstance(raw_line, bytes):
            line = raw_line.decode("utf-8", errors="replace").strip()
        else:
            line = str(raw_line).strip()
        if not line.startswith("data:"):
            continue
        payload = line[5:].strip()
        if payload == "[DONE]":
            break
        try:
            data = json.loads(payload)
            choices = data.get("choices") or []
            if not choices:
                continue
            delta = choices[0].get("delta") or {}
            content = delta.get("content")
            if isinstance(content, str) and content:
                yield content
        except (TypeError, ValueError, KeyError):
            continue


def glm_chat(
    *,
    api_base: str,
    api_key: str,
    model: str,
    messages: List[Dict[str, str]],
    temperature: float,
    max_tokens: int,
    timeout: Tuple[float, float],
    thinking: bool = False,
    stream: bool = True,
    on_sentence: Optional[SentenceCallback] = None,
    cancel_check: Optional[CancelCallback] = None,
    cancel_event: Optional[threading.Event] = None,
) -> str:
    """Call the Zhipu GLM OpenAI-compatible chat-completions endpoint.

    ``cancel_check`` mirrors the CosyVoice path: when it returns True the SSE
    stream is aborted with :class:`VoiceBackendCancelled` (a user cancel, not
    an API error), so the caller must not fall back to another model.
    ``cancel_event`` optionally lets the backend close a blocked HTTP response
    immediately instead of waiting for the next SSE chunk/read timeout.
    """
    if not api_key:
        raise VoiceBackendError("GLM API key is missing")

    def raise_if_cancelled() -> None:
        if cancel_check and cancel_check():
            raise VoiceBackendCancelled("GLM request cancelled")

    def guarded_on_sentence(sentence: str) -> None:
        # This is deliberately the last operation before the user callback.
        # Checking only before SentenceAccumulator.feed() leaves a race where
        # an interrupt can arrive while the accumulator is splitting text.
        raise_if_cancelled()
        if on_sentence is not None:
            on_sentence(sentence)

    http = _require_requests()
    url = f"{api_base.rstrip('/')}/chat/completions"
    payload = {
        "model": model,
        "messages": messages,
        "temperature": temperature,
        "max_tokens": max_tokens,
        "thinking": {"type": "enabled" if thinking else "disabled"},
        "stream": bool(stream),
    }
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }

    response = None
    cancel_watcher = None
    cancel_watcher_stop = threading.Event()
    try:
        response = http.post(
            url,
            headers=headers,
            json=payload,
            timeout=timeout,
            stream=bool(stream),
        )
        if response.status_code >= 400:
            raise VoiceBackendError(_safe_error(response))

        if stream and cancel_event is not None:
            # cancel_check alone is only observed between SSE chunks.  Closing
            # the owned HTTP response from this short-lived watcher also wakes
            # a stream blocked waiting for the next network chunk.
            def close_response_on_cancel():
                while not cancel_watcher_stop.wait(0.05):
                    if cancel_event.is_set():
                        try:
                            response.close()
                        except Exception:
                            # The main request path still performs its final
                            # close and reports cancellation, never credentials.
                            pass
                        return

            cancel_watcher = threading.Thread(
                target=close_response_on_cancel,
                name="glm-cancel-watcher",
                daemon=True,
            )
            cancel_watcher.start()

        if not stream:
            raise_if_cancelled()
            data = response.json()
            choices = data.get("choices") or []
            if not choices:
                return ""
            raise_if_cancelled()
            return ((choices[0].get("message") or {}).get("content") or "").strip()

        fragments: List[str] = []
        sentences = SentenceAccumulator(
            guarded_on_sentence if on_sentence is not None else None
        )
        try:
            for content in _iter_openai_sse_content(response):
                raise_if_cancelled()
                fragments.append(content)
                sentences.feed(content)
        except VoiceBackendCancelled:
            # User interrupt: stop streaming immediately. This is not an API
            # error, so the caller must not retry on another backend.
            raise
        except Exception:
            raise_if_cancelled()
            # Keep partial output. Falling back after speech has begun would
            # cause a second model to repeat the answer.
            if not fragments:
                raise

        # Both a complete stream and a partial network stream may have a
        # trailing sentence.  Check immediately before flush; the accumulator's
        # guarded callback checks again immediately before actual publication.
        raise_if_cancelled()
        sentences.flush()
        return "".join(fragments).strip()
    except VoiceBackendError:
        raise
    except Exception as exc:
        raise VoiceBackendError(f"GLM request failed: {type(exc).__name__}") from exc
    finally:
        cancel_watcher_stop.set()
        if response is not None:
            response.close()
        if cancel_watcher is not None:
            cancel_watcher.join(timeout=0.2)


def ollama_chat(
    *,
    base_url: str,
    model: str,
    messages: List[Dict[str, str]],
    temperature: float,
    max_tokens: int,
    num_ctx: int,
    timeout: Tuple[float, float],
) -> str:
    """Call a local Ollama server as a non-streaming fallback."""
    http = _require_requests()
    payload = {
        "model": model,
        "messages": messages,
        "options": {
            "temperature": temperature,
            "num_predict": max_tokens,
            "num_ctx": num_ctx,
        },
        "stream": False,
    }
    response = None
    try:
        response = http.post(
            f"{base_url.rstrip('/')}/api/chat",
            json=payload,
            timeout=timeout,
        )
        if response.status_code >= 400:
            raise VoiceBackendError(_safe_error(response))
        data = response.json()
        return ((data.get("message") or {}).get("content") or "").strip()
    except VoiceBackendError:
        raise
    except Exception as exc:
        raise VoiceBackendError(f"Ollama request failed: {type(exc).__name__}") from exc
    finally:
        if response is not None:
            response.close()


def _cosyvoice_request_parts(
    *,
    mode: str,
    text: str,
    speaker_id: str,
    prompt_text: str,
    prompt_wav: str,
    instruct_text: str,
):
    data = {"tts_text": text}
    files = None
    prompt_handle = None

    if mode == "sft":
        data["spk_id"] = speaker_id
    elif mode == "zero_shot":
        if not prompt_text or not prompt_wav:
            raise VoiceBackendError("CosyVoice zero_shot requires prompt_text and prompt_wav")
        data["prompt_text"] = prompt_text
    elif mode == "cross_lingual":
        if not prompt_wav:
            raise VoiceBackendError("CosyVoice cross_lingual requires prompt_wav")
    elif mode == "instruct":
        data.update({"spk_id": speaker_id, "instruct_text": instruct_text})
    elif mode == "instruct2":
        if not prompt_wav:
            raise VoiceBackendError("CosyVoice instruct2 requires prompt_wav")
        data["instruct_text"] = instruct_text
    else:
        raise VoiceBackendError(f"Unsupported CosyVoice mode: {mode}")

    if mode in {"zero_shot", "cross_lingual", "instruct2"}:
        expanded = os.path.expanduser(prompt_wav)
        if not os.path.isfile(expanded):
            raise VoiceBackendError("CosyVoice prompt_wav does not exist")
        prompt_handle = open(expanded, "rb")
        files = {"prompt_wav": (os.path.basename(expanded), prompt_handle, "audio/wav")}

    return data, files, prompt_handle


def cosyvoice_to_wav(
    *,
    base_url: str,
    mode: str,
    text: str,
    wav_path: str,
    sample_rate: int,
    speaker_id: str = "",
    prompt_text: str = "",
    prompt_wav: str = "",
    instruct_text: str = "",
    timeout: Tuple[float, float] = (5.0, 90.0),
    cancel_check: Optional[CancelCallback] = None,
) -> int:
    """Synthesize with the official CosyVoice FastAPI server into a WAV file.

    The server streams raw mono signed 16-bit PCM. This function adds a WAV
    container and can cancel between received chunks after a TTS interrupt.
    """
    if int(sample_rate) <= 0:
        raise VoiceBackendError("CosyVoice sample_rate must be positive")
    if cancel_check and cancel_check():
        raise VoiceBackendCancelled("CosyVoice request cancelled")

    http = _require_requests()
    data, files, prompt_handle = _cosyvoice_request_parts(
        mode=mode,
        text=text,
        speaker_id=speaker_id,
        prompt_text=prompt_text,
        prompt_wav=prompt_wav,
        instruct_text=instruct_text,
    )
    response = None
    bytes_written = 0
    pending = b""
    try:
        response = http.post(
            f"{base_url.rstrip('/')}/inference_{mode}",
            data=data,
            files=files,
            timeout=timeout,
            stream=True,
        )
        if response.status_code >= 400:
            raise VoiceBackendError(_safe_error(response))
        with wave.open(wav_path, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(int(sample_rate))
            for chunk in response.iter_content(chunk_size=32768):
                if cancel_check and cancel_check():
                    raise VoiceBackendCancelled("CosyVoice request cancelled")
                if not chunk:
                    continue
                pcm = pending + chunk
                even_length = len(pcm) - (len(pcm) % 2)
                if even_length:
                    wav_file.writeframesraw(pcm[:even_length])
                    bytes_written += even_length
                pending = pcm[even_length:]
        if bytes_written == 0:
            raise VoiceBackendError("CosyVoice returned empty audio")
        return bytes_written
    except (VoiceBackendError, VoiceBackendCancelled):
        try:
            os.remove(wav_path)
        except OSError:
            pass
        raise
    except Exception as exc:
        try:
            os.remove(wav_path)
        except OSError:
            pass
        raise VoiceBackendError(f"CosyVoice request failed: {type(exc).__name__}") from exc
    finally:
        if response is not None:
            response.close()
        if prompt_handle is not None:
            prompt_handle.close()
